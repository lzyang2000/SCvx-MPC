"""Roadster demonstrator: constrained wheel contact, 6-DOF flight, local MPC.

World/body axes: +X forward, +Y left, +Z up, scalar-first quaternions.
This is a feasibility model with assumed hardware, not Tesla engineering data.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
from typing import Callable
import time

import numpy as np
from scipy.optimize import lsq_linear

from .mass_properties import MassProperties
from .mpc import normalize_quaternion, quaternion_multiply, quaternion_to_matrix

G = 9.81
WHEELBASE = 2.8
TRACK = 1.70
WHEEL_RADIUS = .37
WHEEL_HUB_Z = -.25
COM_HEIGHT = WHEEL_RADIUS - WHEEL_HUB_Z
FLIP_DURATION = 2.0
WHEELS = np.array([[x, y, -COM_HEIGHT] for x in [-1.4, 1.4] for y in [-.85, .85]])
# User-constrained hardware: four bottom and two front outward-exhaust side jets.
# Side mounts sit on the front body corners beyond the tire envelopes.
# Directions are force on the body, opposite exhaust. No roof or fore/aft jets.
NOZZLE_POS = np.array([[x, y, -.32] for x in [-1.45, 1.45] for y in [-.78, .78]] +
                      [[2., y, -.08] for y in [-.80, .80]])
NOZZLE_DIR = np.array([[0., 0., 1.]] * 4 + [[0., -np.sign(pos[1]), 0.] for pos in NOZZLE_POS[4:]])
NOZZLE_COUNT = len(NOZZLE_POS)
ALLOCATION = np.vstack((NOZZLE_DIR.T, np.cross(NOZZLE_POS, NOZZLE_DIR).T))

@dataclass(frozen=True)
class RoadsterConfig:
    radius: float = 10.
    ramp_degrees: float = 180.
    speed: float = 12.
    gas_kg: float = 250.
    nozzle_n: float = 8500.
    exhaust_mps: float = 650.
    fan_g: float = 2.0
    use_mpc: bool = True
    dt: float = .01
    wind_mps: float = 0.

    def validate(self):
        bounds = {'radius': (8, 16), 'ramp_degrees': (100, 200), 'speed': (9, 16),
                  'gas_kg': (0, 250), 'nozzle_n': (0, 12000), 'exhaust_mps': (300, 700),
                  'fan_g': (0, 4), 'dt': (.005, .02), 'wind_mps': (-8, 8)}
        for name, (lo, hi) in bounds.items():
            value = getattr(self, name)
            if not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError(f'{name} must be between {lo} and {hi}')
        if not isinstance(self.use_mpc, bool):
            raise ValueError('use_mpc must be a boolean')

class CarMassModel:
    """Fixed COM; gas carried centrally; positive inertia scales with total mass."""
    def properties(self, mass_kg):
        return MassProperties(mass_kg, np.zeros(3), np.array([800., 2600., 2900.]) * mass_kg / 1678.)


def attitude_reference(t: float, release_angle: float):
    """Half-roll from inversion to upright, keeping the reverse launch heading.

    This is a guidance target only; the plant integrates nozzle torque.
    Smooth quintics include a spin-up from the ramp's pitch rate separately.
    """
    duration = FLIP_DURATION
    u = np.clip(t / duration, 0., 1.)
    s = 10*u**3 - 15*u**4 + 6*u**5
    pitch = release_angle + (math.pi - release_angle) * s
    roll = math.pi * s
    qp = np.array([math.cos(pitch/2), 0, -math.sin(pitch/2), 0])
    qr = np.array([math.cos(roll/2), math.sin(roll/2), 0, 0])
    return normalize_quaternion(quaternion_multiply(qp, qr))


def allocate(force_body, torque_body, limit):
    if limit <= 0:
        return np.zeros(NOZZLE_COUNT), float(np.linalg.norm(np.r_[force_body, torque_body]))
    # Attitude gets priority. Canted jets couple lateral and vertical force;
    # the exported residual reveals any requested wrench they cannot achieve.
    weights = np.array([1., 1., 1., 3., 3., 3.])
    matrix = ALLOCATION * weights[:, None]
    target = np.r_[force_body, torque_body] * weights
    # A small impulse penalty selects low gas usage among redundant allocations.
    result = lsq_linear(np.vstack((matrix, .035*np.eye(NOZZLE_COUNT))),
                        np.r_[target, np.zeros(NOZZLE_COUNT)], bounds=(0., limit), method='bvls')
    residual = np.linalg.norm(ALLOCATION @ result.x - np.r_[force_body, torque_body])
    return result.x, float(residual)


def simulate(
    cfg: RoadsterConfig | None = None, *,
    on_frame: Callable[[dict], None] | None = None,
    on_solve: Callable[[dict], None] | None = None,
    realtime: bool = False,
):
    cfg = cfg or RoadsterConfig()
    cfg.validate()
    started = time.perf_counter()
    dt = cfg.dt
    dry = 1428.
    mass = dry + cfg.gas_kg
    p = np.array([-20., 0., COM_HEIGHT])
    v = np.array([0., 0., 0.])
    q = np.array([1., 0., 0., 0.])
    omega = np.zeros(3)
    phase = 'APPROACH'
    owner = 'WHEEL DRIVE'
    path_s = -20.
    speed = 0.
    from .roadster_contact import RampContactGeometry
    ramp_contact = RampContactGeometry(cfg.radius, math.radians(cfg.ramp_degrees),
                                       WHEELBASE, WHEEL_RADIUS, -WHEEL_HUB_Z)
    release_t = None
    landing_t = None
    target_x = -16.
    frames, events = [], []
    from .roadster_mpc import FlightMPC
    mpc = FlightMPC(cfg, on_solve=on_solve) if cfg.use_mpc else None
    warmup_seconds = mpc.warm_up() if mpc else 0.
    mpc_force = None
    next_solve = 0.
    contacts = 0
    roll_integral = 0.
    maneuver_roll = 0.
    previous_roll_angle = 0.
    release_angle = math.radians(cfg.ramp_degrees)
    peak_roll = 0.
    max_height = 0.
    max_fan = 0.
    touch = None
    failure = None
    force = np.zeros(NOZZLE_COUNT)
    fan = 0.
    residual = 0.
    prediction = []
    # Setup is outside the wall-clock pacing epoch; frame zero starts the run.
    pace_started = time.perf_counter()
    for step in range(int(24 / dt)):
        t = step * dt
        rotation = quaternion_to_matrix(q)
        inertia = CarMassModel().properties(mass).inertia_at_com_kgm2
        fan = 0.
        force[:] = 0.
        if phase in ('APPROACH', 'RAMP'):
            # Contact generalized coordinate: wheel drive compensates gravity
            # and rolling resistance, limited by available tire normal load.
            _, tangent, phi, _, curvature = ramp_contact.at(path_s)
            desired_accel = np.clip(2.8*(cfg.speed - speed), -5., 5.)
            # Inward normal load uses the COM path curvature and slope.
            fan_needed = max(0., mass * (-speed**2*curvature - G*tangent[0])) + .65*mass*G
            # Fan pulls toward the deck; reaction is inward. The .65g margin
            # improves traction; the configured maximum is enforced.
            fan = min(fan_needed, cfg.fan_g*mass*G) if path_s >= ramp_contact.entry_s else 0.
            # N = m v² curvature + fan + m g cos(path slope).
            normal = mass*speed**2*curvature + fan + mass*G*tangent[0]
            traction = max(normal, 0.) * 1.25
            drive = np.clip(mass*(desired_accel + G*tangent[2]), -traction, traction)
            accel = drive/mass - G*tangent[2]
            speed = max(0., speed + accel*dt)
            path_s += speed*dt
            if path_s < ramp_contact.entry_s:
                p[:] = [path_s, 0, COM_HEIGHT]
                v[:] = [speed, 0, 0]
            else:
                if phase == 'APPROACH':
                    phase = 'RAMP'
                    events.append({'t': t, 'name': 'Fan seal / C-ramp entry'})
                position, tangent, phi, pitch_per_meter, _ = ramp_contact.at(path_s)
                p[:] = position
                v[:] = speed*tangent
                q[:] = [math.cos(phi/2), 0, -math.sin(phi/2), 0]
                omega[:] = [0, -speed*pitch_per_meter, 0]
                owner = 'FAN + WHEEL DRIVE'
                max_fan = max(max_fan, fan / (mass*G))
                # Loss of seal is a genuine detached flight, not forced attachment.
                if normal < 0 or path_s >= ramp_contact.release_s:
                    phase = 'FLIP'
                    release_t = t
                    release_angle = phi
                    target_x = p[0] + v[0]*4.5
                    next_solve = t
                    events.append({'t': t, 'name': 'Ramp release / flight MPC'})
        else:
            air_t = t - release_t
            # Measure recovery roll independently of the integrated body-X rate.
            u = np.clip(air_t/FLIP_DURATION, 0., 1.)
            blend = 10*u**3 - 15*u**4 + 6*u**5
            pitch = release_angle + (math.pi-release_angle)*blend
            qp_inverse = np.array([math.cos(pitch/2), 0, math.sin(pitch/2), 0])
            relative = quaternion_to_matrix(quaternion_multiply(qp_inverse, q))
            roll_angle = math.atan2(relative[2,1], relative[2,2])
            if landing_t is None:
                maneuver_roll += math.atan2(math.sin(roll_angle-previous_roll_angle), math.cos(roll_angle-previous_roll_angle))
                previous_roll_angle = roll_angle
            rotation = quaternion_to_matrix(q)
            if phase != 'SETTLED' and phase != 'CRASH':
                qref = attitude_reference(air_t, release_angle)
                error_q = quaternion_multiply(np.r_[q[0], -q[1:]], qref)
                if error_q[0] < 0:
                    error_q *= -1
                error = 2*error_q[1:]
                # Feedforward from the guidance curve in the current body frame.
                future_q = attitude_reference(air_t+.005, release_angle)
                dq = quaternion_multiply(np.r_[qref[0], -qref[1:]], future_q)
                if dq[0] < 0:
                    dq *= -1
                ref_rate = rotation.T @ quaternion_to_matrix(qref) @ (2*dq[1:]/.005)
                torque = inertia*(22.*error + 8.*(ref_rate - omega)) + np.cross(omega, inertia*omega)
                h = max(0, p[2] - COM_HEIGHT)
                # Ballistic until flip has sufficient clearance, then arrest descent.
                desired_a = np.array([np.clip(-1.2*v[0] + .35*(target_x-p[0]), -4., 4.),
                                      np.clip(-1.5*v[1] - p[1], -3., 3.), 0.])
                if air_t < .15:
                    desired_a[2] = 0.  # no lift except the reaction from attitude jets
                    force_world = np.array([0., 0., 0.])
                    owner = 'ATTITUDE PD / BALLISTIC'
                else:
                    phase = 'FLIP' if air_t < FLIP_DURATION else 'DESCENT'
                    vzref = -min(5., .15 + 1.25*h)
                    desired_a[2] = np.clip(5.*(vzref - v[2]), -6., 13.)
                    force_world = mass*(desired_a + np.array([0, 0, G]))
                    owner = 'DESCENT PD'
                if air_t >= .15 and h < 1.1:
                    # Contact transition uses a high-rate velocity servo, like the
                    # rocket's terminal-PD handoff, avoiding MPC hover above tires.
                    force_world[2] = mass*(G + np.clip(6.*(-.45-v[2]), -6., 13.))
                    owner = 'TERMINAL VELOCITY PD + ATTITUDE PD'
                limit = cfg.nozzle_n * min(1., max(mass-dry, 0.)/12.)
                if mpc and t >= next_solve:
                    mpc_force = mpc.solve_flight(np.r_[mass,p,v,q,omega],air_t,release_angle,target_x,t)
                    prediction = mpc.snapshot['prediction']
                    next_solve = t + mpc.config.prediction_dt
                if mpc_force is not None:
                    force = np.clip(mpc_force,0,limit)
                    owner = 'ROADSTER SCvx MPC / ' + ('HALF-ROLL' if air_t < FLIP_DURATION else 'DESCENT')
                    residual = 0.
                else:
                    force, residual = allocate(rotation.T @ force_world, torque, limit)
                    if mpc:owner = 'FLIGHT MPC REJECTED / PD FALLBACK'

                # Never spend more air than remains in this step.
                usage = force.sum()*dt/cfg.exhaust_mps
                if usage > mass-dry:
                    force *= max(mass-dry, 0)/max(usage, 1e-12)
                if mass-dry < 1e-8:
                    owner = 'AIR DEPLETED / BALLISTIC'
            else:
                owner = 'WHEEL CONTACT' if phase == 'SETTLED' else 'IMPACT'
            force_body = NOZZLE_DIR.T @ force
            torque_body = np.cross(NOZZLE_POS, force[:,None]*NOZZLE_DIR).sum(axis=0)
            # Compliant wheel springs and dampers, equal/opposite ground wrench.
            contacts = 0
            force_ground = np.zeros(3)
            torque_ground = np.zeros(3)
            for wheel in WHEELS:
                r = rotation @ wheel
                wp = p + r
                wv = v + rotation @ np.cross(omega, wheel)
                if wp[2] < 0:
                    contacts += 1
                    n = max(0., -180000.*wp[2] - 11000.*wv[2])
                    tangential = -np.array([wv[0], wv[1], 0.])*mass*2.
                    norm = np.linalg.norm(tangential)
                    tangential *= min(1., .9*n/max(norm, 1e-9))
                    f = tangential + np.array([0, 0, n])
                    force_ground += f
                    torque_ground += np.cross(wheel, rotation.T @ f)
            # Body collision envelope catches inverted/side impacts.
            corners = np.array([[x,y,z] for x in [-2.1,2.1] for y in [-.9,.9] for z in [-.25,.6]])
            lowest = np.min((rotation @ corners.T)[2] + p[2])
            if lowest < -.08 and phase not in ('SETTLED', 'CRASH'):
                failure = 'Body impact'
                phase = 'CRASH'
                events.append({'t':t, 'name':'Body impact — maneuver failed'})
            if contacts and touch is None:
                touch = {'t': t, 'vertical_speed': float(abs(v[2])), 'horizontal_speed': float(np.linalg.norm(v[:2])),
                         'tilt_degrees': math.degrees(math.acos(np.clip(rotation[2,2],-1,1))),
                         'angular_rate': float(np.linalg.norm(omega)), 'roll_degrees': float(peak_roll),
                         'heading_degrees': math.degrees(math.atan2(rotation[1,0], rotation[0,0]))}
                landing_t = t
                events.append({'t':t, 'name':'First wheel contact'})
                if touch['vertical_speed'] > 2 or touch['tilt_degrees'] > 10:
                    failure = 'Hard or tilted touchdown'
                # Cut all jets on wheel contact; suspension settles physically.
                phase = 'SETTLED'
                force[:] = 0.
                force_body[:] = 0.
                torque_body[:] = 0.
            # Aerodynamic drag in world frame, including crosswind.
            rel = v - np.array([0, cfg.wind_mps, 0])
            drag = -.42*np.linalg.norm(rel)*rel
            a = np.array([0.,0.,-G]) + (rotation@force_body + drag + force_ground)/mass
            alpha = (torque_body + torque_ground - np.cross(omega,inertia*omega))/inertia
            # Semi-implicit velocity / quaternion exponential; physical pose only.
            v += a*dt
            p += v*dt
            omega += alpha*dt
            omega *= math.exp(-.025*dt)
            angle = np.linalg.norm(omega)*dt
            dq = np.r_[math.cos(angle/2), omega*dt * (math.sin(angle/2)/angle if angle > 1e-12 else .5)]
            q = normalize_quaternion(quaternion_multiply(q,dq))
            mass = max(dry, mass - force.sum()*dt/cfg.exhaust_mps)
            roll_integral += omega[0]*dt
            peak_roll = max(peak_roll, roll_integral*180/math.pi)
        max_height = max(max_height, p[2])
        end_run = phase == 'CRASH' or (landing_t is not None and t - landing_t > 2.)
        if step % max(1, round(1/(dt*30))) == 0 or end_run:
            frames.append({'t':round(t,4),'p':p.round(6).tolist(),'q':q.round(8).tolist(),
                           'v':v.round(5).tolist(),'omega':omega.round(5).tolist(),
                           'mass':round(mass,5),'gas':round(mass-dry,5),'phase':phase,
                           'controller':owner,'jets':force.round(2).tolist(),'fan_g':round(fan/(mass*G),4),
                           'roll':round(peak_roll,3),'contact':contacts,'residual':round(residual,2),
                           'prediction':prediction if step % 30 == 0 else [],
                           'mpc_id': mpc.snapshot['id'] if mpc and mpc.snapshot else None,
                           'delivered_force': (rotation @ (NOZZLE_DIR.T @ force)).round(3).tolist()})
            if realtime:
                # Pace actual integration, not a previously computed trajectory.
                # A solve may overrun its deadline; report its duration honestly.
                delay = pace_started + t - time.perf_counter()
                if delay > 0:
                    time.sleep(delay)
            if on_frame is not None:
                on_frame(frames[-1])
        if end_run:
            break
        if not np.all(np.isfinite(np.r_[p,v,q,omega])):
            raise RuntimeError('Non-finite rigid-body state')
    success = bool(touch and not failure and touch['vertical_speed'] < 1. and touch['tilt_degrees'] < 5. and touch['horizontal_speed'] < 4. and math.degrees(maneuver_roll) > 170. and np.linalg.norm(v) < .15)
    return {'config':asdict(cfg),'frames':frames,'events':events,'landing_target':[target_x,0.,COM_HEIGHT],
            'nozzles':{'positions':NOZZLE_POS.tolist(),'directions':NOZZLE_DIR.tolist()},
            'summary':{'success':success,'failure':failure or (None if success else 'Acceptance criteria not met'),
                       'touchdown':touch,'max_height':max_height,'roll_degrees':peak_roll,'maneuver_roll_degrees':math.degrees(maneuver_roll),
                       'gas_used':dry+cfg.gas_kg-mass,'gas_remaining':mass-dry,
                       'mpc_accepted':mpc.accepted if mpc else 0,'mpc_rejected':mpc.rejected if mpc else 0,
                       'mpc_solves':mpc.solves if mpc else [],'compute_seconds':time.perf_counter()-started,
                       'mpc_warmup_seconds':warmup_seconds,
                       'settled_speed':float(np.linalg.norm(v)), 'max_fan_g':max_fan},
            'model':'six fixed jets; full-attitude SCvx flight MPC using repo RK4; passive wheel contact'}


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', default='roadster/output/car-python.json')
    parser.add_argument('--no-mpc', action='store_true')
    args = parser.parse_args()
    result = simulate(RoadsterConfig(use_mpc=not args.no_mpc))
    path = Path(args.output)
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(result, separators=(',', ':'), allow_nan=False))
    print(json.dumps(result['summary'], indent=2))

if __name__ == '__main__':
    main()
