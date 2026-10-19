"""Airborne Roadster MPC using the repository's SCvx/RK4 foundation.

Convex subproblems optimize six nonnegative fixed-nozzle forces and full
attitude from ramp release to touchdown. Wheels and fan handle the ramp.
"""
from __future__ import annotations

import math
import time
import warnings

import cvxpy as cp
import numpy as np

from .mpc import SixDofMPC, MPCConfig, quaternion_multiply, quaternion_to_matrix


FUEL_WEIGHT = .5

def unit(q):
    """Normalize without changing quaternion hemisphere during a rollout."""
    return np.asarray(q) / max(np.linalg.norm(q), 1e-12)


class FlightMPC(SixDofMPC):
    """Direct fixed-nozzle SCvx; retains SixDofMPC rollout and RK4 propagation."""
    def __init__(self, cfg, on_solve=None):
        from .roadster import NOZZLE_DIR, ALLOCATION, CarMassModel
        self.cfg, self.on_solve = cfg, on_solve
        self.directions, self.wrench = NOZZLE_DIR, ALLOCATION
        self.nu = self.wrench.shape[1]
        self.config = MPCConfig(horizon_steps=10, prediction_dt=.12,
                                dry_mass_kg=1428., initial_mass_kg=1678.,
                                successive_iterations=2, maximum_scaled_defect=.12)
        self.mass_model = CarMassModel()
        self._mass_properties_cache = {}
        self._state_scale = np.array([1678., 15, 5, 10, 12, 5, 6, 1, 1, 1, 1, 3, 3, 3])
        self._state_difference = np.array([.1]+[.002]*6+[.0001]*7)
        self._control_difference = np.full(self.nu, 10.)
        self.control_scale = max(cfg.nozzle_n, 1.)
        self.solves, self.snapshot = [], None
        self.accepted = self.rejected = 0
        self.command = np.zeros(self.nu)
        self.plan = None
        self._build_problem()

    def warm_up(self):
        """Compile the parameterized problem before playback, without a flight solve.

        Placeholder matrices only populate the compiler's parameter cache. No
        controls, warm-start plan, telemetry or simulated time are produced.
        """
        started = time.perf_counter()
        n = self.config.horizon_steps
        state = np.zeros(14)
        state[0], state[3], state[7] = 1428. + self.cfg.gas_kg, 10., 1.
        self.initial.value = state
        self.reference.value = self.nominal.value = np.repeat(state[:, None], n + 1, axis=1)
        self.nominal_u.value = np.zeros((self.nu, n))
        self.previous.value = np.zeros(self.nu)
        self.limit.value = self.cfg.nozzle_n
        for k in range(n):
            self.a[k].value = np.eye(14)
            self.b[k].value = np.zeros((14, self.nu))
            self.c[k].value = np.zeros(14)
        self.problem.get_problem_data(cp.CLARABEL)
        return time.perf_counter() - started

    def continuous_dynamics(self, state, control):
        mass = max(state[0], 1428.)
        q = unit(state[7:11]); omega = state[11:14]
        force = np.clip(control, 0, self.cfg.nozzle_n)
        rotation = quaternion_to_matrix(q)
        inertia = self.mass_properties(mass).inertia_at_com_kgm2
        relative = state[4:7] - np.array([0., self.cfg.wind_mps, 0.])
        drag = -.42*np.linalg.norm(relative)*relative
        derivative = np.zeros(14)
        derivative[0] = -force.sum()/self.cfg.exhaust_mps
        derivative[1:4] = state[4:7]
        derivative[4:7] = (rotation@(self.wrench[:3]@force)+drag)/mass + [0,0,-9.81]
        derivative[7:11] = .5*quaternion_multiply(q, np.r_[0.,omega])
        derivative[11:14] = (self.wrench[3:]@force-np.cross(omega,inertia*omega))/inertia - .025*omega
        return derivative

    def discrete_dynamics(self, state, control):
        result = super().discrete_dynamics(state, control)
        if result[7:11]@state[7:11] < 0: result[7:11] *= -1
        return result

    def _linearize(self, state, control, *, central_differences=False):
        base = self.discrete_dynamics(state, control)
        a, b = np.zeros((14,14)), np.zeros((14,self.nu))
        for i, epsilon in enumerate(self._state_difference):
            plus = state.copy(); plus[i] += epsilon
            a[:,i] = (self.discrete_dynamics(plus,control)-base)/epsilon
        for i, epsilon in enumerate(self._control_difference):
            # Differentiate inward at an upper actuator bound.
            epsilon *= -1 if control[i] > self.cfg.nozzle_n-epsilon else 1
            plus = control.copy(); plus[i] += epsilon
            b[:,i] = (self.discrete_dynamics(state,plus)-base)/epsilon
        return a,b,base-a@state-b@control

    def _build_problem(self):
        n = self.config.horizon_steps
        self.x = cp.multiply(self._state_scale[:,None],cp.Variable((14,n+1)))
        self.u = self.control_scale*cp.Variable((self.nu,n))
        self.virtual = cp.multiply(self._state_scale[:,None],cp.Variable((14,n)))
        self.initial = cp.Parameter(14); self.reference = cp.Parameter((14,n+1))
        self.nominal = cp.Parameter((14,n+1)); self.nominal_u = cp.Parameter((self.nu,n))
        self.previous = cp.Parameter(self.nu); self.limit = cp.Parameter(nonneg=True)
        self.a = [cp.Parameter((14,14)) for _ in range(n)]
        self.b = [cp.Parameter((14,self.nu)) for _ in range(n)]
        self.c = [cp.Parameter(14) for _ in range(n)]
        inv = (1/self._state_scale)[:,None]
        # Track full orientation through inversion, allowing purposeful pitch for braking.
        weights = np.array([0, 2, 4, 5, 3, 3, 5, 15, 15, 15, 15, 4, 4, 4])
        constraints = [cp.multiply(1/self._state_scale,self.x[:,0]-self.initial)==0,
                       self.u/self.control_scale>=0, self.u/self.control_scale<=self.limit/self.control_scale,
                       self.x[0,:]/1678.>=1428./1678., self.x[3,:]/10>=.056,
                       cp.abs(cp.multiply(inv,self.x-self.nominal))<=.65,
                       cp.abs((self.u-self.nominal_u)/self.control_scale)<=.65]
        constraints += [cp.cumsum(cp.sum(self.u,axis=0))*self.config.prediction_dt/self.cfg.exhaust_mps/250 <= (self.initial[0]-1428.)/250.]
        for k in range(n):
            constraints += [self.u[:,k]/self.control_scale <= self.cfg.nozzle_n/12/self.control_scale*(self.x[0,k]-1428.)]
        objective = FUEL_WEIGHT*cp.sum(self.u)/self.control_scale
        objective += .05*cp.sum_squares((self.u-self.nominal_u)/self.control_scale)
        for k in range(n):
            constraints += [cp.multiply(1/self._state_scale,self.x[:,k+1]-self.a[k]@self.x[:,k]-self.b[k]@self.u[:,k]-self.c[k]-self.virtual[:,k])==0]
            objective += cp.sum_squares(cp.multiply(weights/self._state_scale,self.x[:,k+1]-self.reference[:,k+1]))
            objective += .04*cp.sum_squares((self.u[:,k]-(self.previous if k==0 else self.u[:,k-1]))/self.control_scale)
            objective += 30000*cp.norm1(cp.multiply(1/self._state_scale,self.virtual[:,k]))
        objective += 2*cp.sum_squares(cp.multiply(weights/self._state_scale,self.x[:,-1]-self.reference[:,-1]))
        self.problem = cp.Problem(cp.Minimize(objective), constraints)

    def references(self, state, air_t, release_angle, target_x):
        from .roadster import attitude_reference, FLIP_DURATION, COM_HEIGHT
        n, dt = self.config.horizon_steps, self.config.prediction_dt
        ref = np.repeat(state[:,None], n+1,axis=1)
        # Acceleration guidance seeds the optimization; no PD torque is applied
        # when MPC is accepted. Fixed jets must realize all commanded motion.
        h = max(state[3]-COM_HEIGHT,0)
        vz = -min(2., .45+.9*h)
        ay = np.clip(-2*state[5]-state[2], -3., 3.)
        ax = np.clip(-1.1*state[4]+.3*(target_x-state[1]),-3.5,3.5)
        brake_pitch = -math.atan2(ax,9.81)
        for k in range(n+1):
            tau = k*dt; tt = air_t+tau
            blend = np.clip((tt-FLIP_DURATION)/.7,0,1)
            blend = blend*blend*(3-2*blend)
            # Taper tilt near wheels; permit rolling on touchdown, facing -X.
            tilt = brake_pitch * blend * np.clip(h/2.,0,1)
            q = attitude_reference(tt,release_angle)
            q = quaternion_multiply(q,[math.cos(tilt/2),0,math.sin(tilt/2),0])
            # Bank to correct drift without requesting independent side force/yaw.
            side_tilt = math.atan2(ay,9.81) * blend * np.clip(h/3.,0,1)
            q = quaternion_multiply(q,[math.cos(side_tilt/2),math.sin(side_tilt/2),0,0])
            if k==0:
                if q@state[7:11]<0:q=-q
            elif q@ref[7:11,k-1]<0:q=-q
            ref[7:11,k] = q
            ref[1:4,k] = state[1:4]+state[4:7]*tau + np.array([.5*ax*tau*tau,.5*ay*tau*tau,0])
            ref[3,k] = max(.58,state[3]+vz*tau)
            ref[4:7,k] = [state[4]+ax*tau, state[5]+ay*tau, vz]
        for k in range(n):
            q, future = ref[7:11,k], ref[7:11,k+1]
            dq = quaternion_multiply(np.r_[q[0],-q[1:]],future)
            if dq[0]<0:dq=-dq
            ref[11:14,k] = 2*dq[1:]/dt
        ref[11:14,-1] = ref[11:14,-2]
        return ref

    def seed(self, state, refs):
        from .roadster import allocate
        n=self.config.horizon_steps
        controls=np.zeros((self.nu,n)); x=state.copy()
        for k in range(n):
            rotation=quaternion_to_matrix(x[7:11]); inertia=self.mass_properties(x[0]).inertia_at_com_kgm2
            qref=refs[7:11,k+1]
            error=quaternion_multiply(np.r_[x[7],-x[8:11]],qref)
            if error[0]<0:error=-error
            torque=inertia*(18*2*error[1:]+7*(refs[11:14,k+1]-x[11:14]))+np.cross(x[11:14],inertia*x[11:14])
            acceleration=np.clip(3*(refs[4:7,k+1]-x[4:7]),[-4,-3,-6],[4,3,13])
            desired=x[0]*(acceleration+[0,0,9.81])
            controls[:,k],_=allocate(rotation.T@desired,torque,self.cfg.nozzle_n)
            x=self.discrete_dynamics(x,controls[:,k])
        return controls

    def solve_flight(self, state, air_t, release_angle, target_x, sim_time):
        start=time.perf_counter(); refs=self.references(state,air_t,release_angle,target_x)
        n=self.config.horizon_steps
        controls=self.seed(state,refs) if self.plan is None else np.column_stack((self.plan[:,1:],self.plan[:,-1]))
        limit=self.cfg.nozzle_n*min(1.,max(0,state[0]-1428)/12)
        controls=np.clip(controls,0,limit)
        nominal=self.rollout(state,controls)
        # Align reference quaternion signs with the current nonlinear rollout.
        for k in range(n+1):
            if refs[7:11,k]@nominal[7:11,k]<0:refs[7:11,k]*=-1
        status='not_solved'; virtual=math.inf; defect=math.inf; iterations=0
        candidate=nominal.copy(); candidate_u=controls.copy()
        try:
            for iteration in range(self.config.successive_iterations):
                for k in range(n):
                    a,b,c=self._linearize(nominal[:,k],controls[:,k]);self.a[k].value=a;self.b[k].value=b;self.c[k].value=c
                self.initial.value=state;self.reference.value=refs;self.nominal.value=nominal
                self.nominal_u.value=controls;self.previous.value=self.command;self.limit.value=limit
                with warnings.catch_warnings():
                    warnings.simplefilter('ignore',UserWarning)
                    self.problem.solve(solver=cp.CLARABEL,warm_start=True,max_iter=350,tol_gap_abs=1e-5,tol_feas=1e-5)
                status=str(self.problem.status)
                if status not in ('optimal','optimal_inaccurate') or self.u.value is None:break
                candidate=self.x.value.copy();candidate_u=np.clip(self.u.value,0,limit)
                virtual=float(np.max(np.abs(self.virtual.value/self._state_scale[:,None])))
                nonlinear=self.rollout(state,candidate_u)
                defect=float(np.max(np.abs((nonlinear-candidate)/self._state_scale[:,None])))
                controls=candidate_u;nominal=nonlinear
                iterations=iteration+1
        except (cp.error.SolverError,ValueError,FloatingPointError):status='solver_error'
        nonlinear=self.rollout(state,candidate_u)
        # Hard hardware checks on nonlinear rollout, including every gas step.
        gas_ok=all(candidate_u[:,k].sum()*self.config.prediction_dt/self.cfg.exhaust_mps<=max(0,nonlinear[0,k]-1428)+1e-5 for k in range(n))
        success=status in ('optimal','optimal_inaccurate') and defect<=self.config.maximum_scaled_defect and virtual<.01 and gas_ok and np.all(np.isfinite(nonlinear))
        if success:self.plan=candidate_u;self.command=candidate_u[:,0].copy();self.accepted+=1
        else:self.rejected+=1;self.plan=None
        requested=quaternion_to_matrix(state[7:11])@(self.directions.T@candidate_u[:,0]) if success else None
        self.snapshot={'id':len(self.solves)+1,'t':float(sim_time),'success':bool(success),'status':status,
                       'seconds':time.perf_counter()-start,'defect':defect if math.isfinite(defect) else None,
                       'iterations':iterations,'virtual_control':virtual if math.isfinite(virtual) else None,
                       'origin':state[1:4].tolist(),'target':refs[1:4,-1].tolist(),
                       'prediction':nonlinear[1:4].T.tolist(),'prediction_dt':self.config.prediction_dt,
                       'horizon_seconds':n*self.config.prediction_dt,'requested_force':requested.tolist() if requested is not None else None,
                       'nozzle_plan':candidate_u.T.tolist(),'attitude_prediction':nonlinear[7:11].T.tolist(),
                       'phase':'FLIGHT','model':'six fixed jets / full 6-DOF'}
        self.solves.append(self.snapshot)
        if self.on_solve:self.on_solve(self.snapshot)
        return self.command.copy() if success else None
