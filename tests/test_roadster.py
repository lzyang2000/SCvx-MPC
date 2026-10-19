"""Behavioral checks for gas-limited flight, allocation and truthful failure."""
import json
import math

import numpy as np
import pytest

from rocket_landing.roadster import (
    ALLOCATION, NOZZLE_DIR, NOZZLE_POS, RoadsterConfig, allocate, simulate,
)


def test_six_fixed_jets_lack_longitudinal_force_but_can_hover_and_torque():
    assert ALLOCATION.shape == (6, 6)
    assert np.linalg.matrix_rank(ALLOCATION) == 4
    assert np.all(NOZZLE_DIR[:,0] == 0)
    assert NOZZLE_DIR[:4] == pytest.approx(np.tile([0,0,1.],(4,1)))
    assert np.all(NOZZLE_POS[:4,2] < 0)
    # Side nozzle bells (radius .12 m) clear the tire sweep (radius .37 m).
    assert np.all(np.abs(NOZZLE_POS[4:,0]) - .12 > 1.4 + .37)
    assert np.all(np.abs(NOZZLE_POS[4:,1]) > .75)
    assert np.all(NOZZLE_POS[4:,0] == 2.)
    # With only front side jets, lateral force and yaw torque are coupled.
    assert np.allclose(ALLOCATION[5], 2 * ALLOCATION[1])
    assert np.all(NOZZLE_DIR[4:,2] == 0)
    assert np.all(NOZZLE_DIR[4:,1]*NOZZLE_POS[4:,1] < 0)
    assert np.allclose(np.linalg.norm(NOZZLE_DIR, axis=1), 1.)
    assert np.allclose(ALLOCATION[3:], np.cross(NOZZLE_POS, NOZZLE_DIR).T)
    # A nominal hover wrench must be achievable without unphysical signed jets.
    force, residual = allocate(np.array([0., 0., 1678*9.81]), np.zeros(3), 8500.)
    assert np.all((force >= 0) & (force <= 8500.))
    assert residual < 20.


@pytest.fixture(scope='module')
def nominal_run():
    return simulate()


def test_nominal_roll_and_soft_upright_landing_use_repository_mpc(nominal_run):
    summary = nominal_run['summary']
    assert summary['success'], summary
    assert summary['mpc_accepted'] > 0
    assert any('ROADSTER SCvx MPC' in frame['controller'] for frame in nominal_run['frames'])
    assert summary['mpc_rejected'] == 0
    assert summary['touchdown']['horizontal_speed'] < 4.
    assert 175 < summary['maneuver_roll_degrees'] < 185
    assert summary['roll_degrees'] > 170
    assert summary['max_height'] > 18
    assert summary['touchdown']['vertical_speed'] < 1.
    assert summary['touchdown']['tilt_degrees'] < 5.
    # Upright with the nose opposite the initial +X approach; no airborne turn.
    from rocket_landing.mpc import quaternion_to_matrix
    final_rotation = quaternion_to_matrix(nominal_run['frames'][-1]['q'])
    assert final_rotation[:, 0] == pytest.approx([-1, 0, 0], abs=.01)
    assert final_rotation[:, 2] == pytest.approx([0, 0, 1], abs=.01)
    assert abs(summary['touchdown']['heading_degrees']) == pytest.approx(180, abs=1.)
    airborne = [f for f in nominal_run['frames'] if f['phase'] in ('FLIP', 'DESCENT')]
    assert all(quaternion_to_matrix(f['q'])[0, 0] < -.9 for f in airborne)
    assert summary['gas_remaining'] > 0
    assert summary['settled_speed'] < .15
    assert {frame['phase'] for frame in nominal_run['frames']} >= {'APPROACH','RAMP','FLIP','DESCENT','SETTLED'}


def test_trajectory_conserves_gas_budget_and_normalizes_attitude(nominal_run):
    fs = nominal_run['frames']
    qs = np.array([f['q'] for f in fs])
    gas = np.array([f['gas'] for f in fs])
    jets = np.array([f['jets'] for f in fs])
    assert np.allclose(np.linalg.norm(qs, axis=1), 1., atol=2e-8)
    assert np.all(gas >= 0.)
    assert np.all(np.diff(gas) <= 1e-4)
    assert np.all((jets >= 0.) & (jets <= 8500.))
    # Downsampled impulse agrees with used gas within sampling integration error.
    sample_t = np.array([f['t'] for f in fs])
    integrated = np.sum((jets.sum(axis=1)[1:]+jets.sum(axis=1)[:-1])*.5*np.diff(sample_t))/650.
    assert integrated == pytest.approx(nominal_run['summary']['gas_used'], rel=.02)
    json.dumps(nominal_run, allow_nan=False)


@pytest.mark.parametrize('cfg', [RoadsterConfig(gas_kg=0, use_mpc=False),
                                    RoadsterConfig(nozzle_n=0),
                                    RoadsterConfig(nozzle_n=1500,use_mpc=False)])
def test_insufficient_propulsion_cannot_fake_a_successful_landing(cfg):
    result = simulate(cfg)
    assert not result['summary']['success']
    assert result['summary']['failure']
    if cfg.gas_kg == 0 or cfg.nozzle_n == 0:
        assert result['summary']['gas_used'] == 0
        assert all(sum(frame['jets']) == 0 for frame in result['frames'])
        assert result['summary']['roll_degrees'] < 1


def test_mpc_disabled_is_reported_and_does_not_invoke_optimizer():
    result = simulate(RoadsterConfig(use_mpc=False))
    assert result['summary']['mpc_accepted'] == 0
    assert not result['summary']['mpc_solves']
    assert all('MPC' not in f['controller'] for f in result['frames'])


@pytest.mark.parametrize('kwargs', [{'gas_kg':-1}, {'speed':math.nan}, {'nozzle_n':13000},
                                     {'radius':1}, {'dt':0}, {'use_mpc':'yes'}])
def test_invalid_parameters_are_rejected(kwargs):
    with pytest.raises(ValueError):
        simulate(RoadsterConfig(**kwargs))


def test_full_c_ramp_releases_car_upside_down_at_upper_lip(nominal_run):
    from rocket_landing.mpc import quaternion_to_matrix
    from rocket_landing.roadster import WHEELBASE, WHEEL_RADIUS, WHEEL_HUB_Z
    assert nominal_run['config']['ramp_degrees'] == 180
    release = next(f for f in nominal_run['frames'] if f['phase'] == 'FLIP')
    rotation = quaternion_to_matrix(release['q'])
    assert rotation[:, 2] == pytest.approx([0, 0, -1], abs=1e-6)
    assert rotation[:, 0] == pytest.approx([-1, 0, 0], abs=1e-6)
    radius = nominal_run['config']['radius']
    expected_height = radius + math.sqrt((radius-WHEEL_RADIUS)**2-(WHEELBASE/2)**2) + WHEEL_HUB_Z
    assert release['p'][2] == pytest.approx(expected_height, abs=1e-5)
    assert release['v'][0] < -11
    assert abs(release['v'][2]) < 1e-5


def test_timestamped_mpc_snapshots_link_frames_and_report_rejections(nominal_run):
    solves = nominal_run['summary']['mpc_solves']
    assert len(solves) > 1
    by_id = {s['id']:s for s in solves}
    assert list(by_id) == list(range(1, len(solves)+1))
    assert all(b['t'] > a['t'] for a,b in zip(solves, solves[1:]))
    for solve in solves:
        assert np.array(solve['prediction']).shape == (11, 3)
        assert np.array(solve['nozzle_plan']).shape == (10, 6)
        assert np.array(solve['attitude_prediction']).shape == (11, 4)
        assert np.all(np.asarray(solve['nozzle_plan']) >= 0)
        assert np.all(np.asarray(solve['nozzle_plan']) <= 8500.)
        assert solve['prediction'][0] == pytest.approx(solve['origin'])
        assert solve['horizon_seconds'] == pytest.approx(1.2)
        assert (solve['requested_force'] is not None) == solve['success']
        assert solve['seconds'] >= 0
    for frame in nominal_run['frames']:
        if frame['mpc_id'] is not None:
            assert by_id[frame['mpc_id']]['t'] <= frame['t'] + 1e-6
        # Requested MPC force must not be substituted for actual allocated force.
        assert len(frame['delivered_force']) == 3
        assert np.all(np.isfinite(frame['delivered_force']))
        assert np.linalg.norm(frame['delivered_force']) <= sum(frame['jets']) + .05


def test_stream_callbacks_emit_physical_states_before_completion():
    frames, solves, order = [], [], []
    def frame_callback(frame):
        frames.append(frame)
        order.append(('frame',frame['t']))
    def solve_callback(solve):
        solves.append(solve)
        order.append(('solve',solve['t']))
    result = simulate(on_frame=frame_callback,on_solve=solve_callback)
    assert frames == result['frames']
    assert solves == result['summary']['mpc_solves']
    assert order[0] == ('frame',0.)
    assert all(a[1] <= b[1]+1e-8 for a,b in zip(order,order[1:]))
    # Exception/disconnect stops computation immediately at the emit boundary.
    class Disconnect(Exception): pass
    visited=[]
    def stop(frame):
        visited.append(frame['t'])
        raise Disconnect
    with pytest.raises(Disconnect):simulate(on_frame=stop)
    assert visited == [0.]


def test_mpc_owns_full_airborne_maneuver_and_is_off_on_ramp(nominal_run):
    fs = nominal_run['frames']
    for f in fs:
        assert len(f['jets']) == 6
        if f['phase'] in ('APPROACH','RAMP'):
            assert f['mpc_id'] is None
            assert not any(f['jets'])
        elif f['phase'] in ('FLIP','DESCENT') and f['mpc_id'] is not None:
            assert f['controller'].startswith('ROADSTER SCvx MPC')
    release = next(f for f in fs if f['phase']=='FLIP')
    solves = nominal_run['summary']['mpc_solves']
    assert 0 <= solves[0]['t']-release['t'] < .05
    assert max(s['origin'][2] for s in solves) > 18
    assert min(s['origin'][2] for s in solves) < 1
    # Optimizer predicts rotation through inversion, not a virtual level body.
    from rocket_landing.mpc import quaternion_to_matrix
    attitudes = [q for s in solves for q in s['attitude_prediction']]
    assert min(quaternion_to_matrix(q)[2,2] for q in attitudes) < -.95
    assert max(quaternion_to_matrix(q)[2,2] for q in attitudes) > .95


def test_fixed_nozzle_predictor_reuses_repo_rollout_and_accounts_for_gas():
    from rocket_landing.roadster_mpc import FlightMPC
    from rocket_landing.mpc import SixDofMPC
    controller = FlightMPC(RoadsterConfig())
    assert isinstance(controller,SixDofMPC)
    state = np.r_[1678., [0,0,19.38], [-12,0,0], [0,0,-1,0], [0,-1.2,0]]
    forces = np.zeros((6,4)); forces[0,:]=2000
    prediction = controller.rollout(state,forces)
    assert prediction[0,-1] == pytest.approx(1678.-4*.12*2000/650)
    assert np.allclose(np.linalg.norm(prediction[7:11],axis=0),1)
    assert prediction[6,-1] < 0 # bottom jet points down while inverted
    assert prediction[11,-1] < 0 # right bottom jet rolls about body X


def test_rejected_flight_plan_does_not_become_an_applied_jet_command(monkeypatch):
    from rocket_landing.roadster_mpc import FlightMPC
    controller=FlightMPC(RoadsterConfig())
    state=np.r_[1678.,[0,0,19.38],[-12,0,0],[0,0,-1,0],[0,-1.2,0]]
    import cvxpy as cp
    def fail(*args,**kwargs):raise cp.error.SolverError('test rejection')
    monkeypatch.setattr(controller.problem,'solve',fail)
    assert controller.solve_flight(state,.01,math.pi,-54.,5.) is None
    assert not controller.snapshot['success']
    assert controller.snapshot['requested_force'] is None
    assert controller.rejected == 1


def test_mpc_preparation_compiles_without_solving_or_issuing_commands(monkeypatch):
    from rocket_landing.roadster_mpc import FlightMPC
    emitted = []
    controller = FlightMPC(RoadsterConfig(), on_solve=emitted.append)
    with monkeypatch.context() as patch:
        def unexpected_solve(*args, **kwargs):
            raise AssertionError('Preparation must not optimize a flight plan')
        patch.setattr(controller.problem, 'solve', unexpected_solve)
        assert controller.warm_up() >= 0
    assert controller.plan is None
    assert controller.snapshot is None
    assert controller.accepted == controller.rejected == 0
    assert not controller.command.any()
    assert not emitted
    # The first real plan must still use the measured release state and ID 1.
    state = np.r_[1678., [0,0,19.38], [-12,0,0], [0,0,-1,0], [0,-1.2,0]]
    assert controller.solve_flight(state,.01,math.pi,-54.,5.) is not None
    assert controller.snapshot['id'] == 1
    assert controller.snapshot['origin'] == state[1:4].tolist()
    assert len(emitted) == 1


@pytest.mark.parametrize('radius', [8., 10., 16.])
def test_ramp_tires_are_tangent_and_entry_has_no_pose_jump(radius):
    from rocket_landing.roadster import WHEELBASE, WHEEL_RADIUS, WHEEL_HUB_Z, COM_HEIGHT
    from rocket_landing.roadster_contact import RampContactGeometry
    from rocket_landing.mpc import quaternion_to_matrix
    contact = RampContactGeometry(radius, math.pi, WHEELBASE, WHEEL_RADIUS, -WHEEL_HUB_Z)
    for boundary in [contact.entry_s, contact.entry_s + contact.entry_length]:
        before = contact.at(boundary-1e-6)
        after = contact.at(boundary+1e-6)
        assert np.linalg.norm(before[0]-after[0]) < 3e-6
        assert abs(before[2]-after[2]) < 1e-6
        assert np.linalg.norm(before[1]-after[1]) < .001
    assert contact.at(contact.entry_s)[0][2] == pytest.approx(COM_HEIGHT)
    for s in np.linspace(contact.entry_s-1, contact.release_s, 200):
        p, tangent, pitch, _, _ = contact.at(s)
        rotation = quaternion_to_matrix([math.cos(pitch/2),0,-math.sin(pitch/2),0])
        assert np.linalg.norm(tangent) == pytest.approx(1)
        for x in [-WHEELBASE/2, WHEELBASE/2]:
            hub = p + rotation @ [x,0,WHEEL_HUB_Z]
            # A tire on the approach is on the plane, and a tire on the
            # circular deck fits inside its radius with zero penetration.
            if hub[0] < -1e-6:
                if pitch < math.pi/2:
                    assert hub[2] >= WHEEL_RADIUS-2e-7
            else:
                radial_extent = math.hypot(hub[0],hub[2]-radius) + WHEEL_RADIUS
                assert radial_extent == pytest.approx(radius, abs=2e-6)
