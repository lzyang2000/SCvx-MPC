# SCvx MPC Demos

The site has two Three.js simulations. A six-jet Roadster drives up a C-ramp or performs a barrel roll. A Falcon 9 concept launches, separates and returns its booster. Both compute SCvx plans in a browser worker using Clarabel WASM. Both use the official MuJoCo WASM runtime for physics and separate analytical RK4 models for prediction.

## Run locally

```sh
npm ci --prefix roadster/web
npm --prefix roadster/web run build
python3 -m http.server 8765 --directory roadster/web/dist --bind 127.0.0.1
```

Open http://127.0.0.1:8765. A plain static server is enough. Your browser
computes the simulation locally without a Python API. Node 22 or newer is recommended for the
verification scripts.

The site includes recordings and the original Roadster glTF. No Python setup or
asset generation is needed to open it. Runtime JavaScript and WASM are served
with the site; web fonts have system fallbacks.

The build produces `roadster/web/dist`. Modules, workers and assets use relative
URLs, so the site also works in a repository subdirectory such as `/SCvx-MPC/`.
MuJoCo and Clarabel run in a browser module worker without shared memory,
COOP/COEP headers or server-side computation.

## Controls and live MPC

Choose EN or 中文 in the navigation bar. The choice carries across both demos
and the formulation page. MPC and SI units keep their usual notation.

Sound is enabled at 30% volume by default and starts on your first interaction.
Use Sound on to mute it. Motor pitch follows wheel speed, jet hiss
follows nozzle thrust, and rocket roar follows engine thrust and cutoff. Music
and volume have separate controls. Pause, hidden tabs and a stalled live queue
silence the sound; faster replay keeps the same pitch. Audio uses Web Audio
nodes and a small local music file.

- **Car / Rocket** switches vehicles and stops computation in the inactive tab.
- Under Car, choose **C-ramp recovery** or **360° barrel roll**. Switching
  maneuvers loads its recording; Run live computes it again with the selected settings.
- Click the blue ground strip to choose a landing point along the runway. You can also enter its X coordinate in **Landing position**. **Plan whole flight from target** runs a full airborne trajectory optimization before playback. In the jump scene, **Require a 360° barrel roll** chooses between a full roll and an upright jump. The C-ramp always retains its inverted-to-upright recovery.
- **Run live** advances new physics and solves new plans. **Stop live** terminates
  the worker, including an in-progress solve, and retains a partial recording.
- **Compute flight / mission** runs the same code without real-time pacing and
  records its output. Replay supports pause, restart, scrubbing and speed changes.
- The car has Overview, Follow, Side and MPC close-up cameras. The rocket has
  full-stack, booster, upper-stage, mission-path and landing-pad views. Follow
  cameras keep a fixed world viewing direction. The rocket camera fits the full
  stack and plumes before separation, then closes in on the booster, adjusting
  for the viewport shape.
- Export saves states, applied controls, events and timestamped solver diagnostics.

The car keeps a 500 ms sample queue to smooth playback during solves. Rocket live pace defaults to 7×, with 1×, 5×, 10× and 25× also available. If a device cannot keep up, the simulation slows down. Optimization runs off the rendering thread, and the display waits for computed states during a solver delay.

Each inspector shows the plan available at the displayed time. Blue traces its nonlinear prediction; silver traces the flight so far. The rocket chart compares center of mass positions, accounting for fuel use and separation. Use **Next solve** during replay to pause at each new command. The panels report solve latency, horizon, plan age, scaled defect and virtual control. A rejected plan stops the run and stays visible for inspection. The browser controller has no PD fallback or landing handoff.

## Car model

The car drives up a full 180° C ramp and leaves its upper lip inverted. Wheels
and fan handle the ramp; MPC controls the entire airborne half-roll and descent.
Braking begins at release so the car descends toward the selected landing marker.
The car lands facing back toward the approach and rolls on MuJoCo tire contacts.

Barrel-roll mode drives on flat ground, lifts using the same six jets, rolls
360° over five parked vehicles, and lands facing forward. The wheels brake the
rollout. MPC commands the liftoff, full roll and descent, using one air budget.
Simplified models represent a Model 3, Model Y, trailerless Semi,
Cybertruck and Cybercab. Dimensions and spacing are approximate; conservative
MuJoCo collision boxes stop a run on impact. This recreates the maneuver Tesla showed at its October 15, 2026 reveal using estimated hardware and independent guidance. See [the reveal reporting](https://www.ft.com/content/a43a24b7-2127-400e-8b75-fe970931df51).

The barrel flight predictor allows its terminal COM down to 0.30 m because it
omits suspension contact. This lets it command descent through tire height;
the physical plant cuts jets at actual wheel contact near 0.62 m and then uses
MuJoCo compliant contacts. Predicted points after contact are flight extrapolations.
With whole-flight planning enabled, SCvx adds local separating-plane constraints
around the display vehicles and linearized height constraints for the body and
tires. A finer nonlinear rollout checks clearance before the plan is accepted;
independent overlap checks also run during the flight. Disabling whole-flight
planning uses the guidance reference and actual-flight overlap checks.

The MuJoCo plant has a free chassis and four independent wheel hinges. Wheel
torques and bounded fan suction drive a C-ramp made of 160 collision segments.
Tire contact determines departure; the car pose is never pinned to a ramp curve.
The renderer reads the wheel joint angles. Four bottom nozzles sit at
`(±1.45, ±0.78, −0.32)` m with body +Z force. Two front side nozzles sit at
`(+2.00, ±0.80, −0.08)` m, clear of the tire envelopes, with force inward along
body Y. With both side jets at the front, lateral force also produces yaw torque.
The controller tilts the car to correct sideways drift and brake.

Assumptions: 1,428 kg dry mass, up to 250 kg of gas, 8.5 kN per jet, 650 m/s
exhaust velocity, mass-scaled `[800, 2600, 2900]` kg·m² reference inertia, and a
2 g fan suction limit. The 10-step horizon spans 1.2 s and replans every 0.12 s.
At the 1,678 kg initial mass, each of four level bottom jets needs 4.12 kN
to hover or 6.21 kN for 5 m/s² net takeoff acceleration. The 8.5 kN cap gives
34 kN total lift. Each side jet's 2 m moment arm gives up to 17 kN·m yaw
torque. A two-second quintic 180° yaw reference needs about 6.58 kN peak
side thrust at the reference inertia. Full coupled flight is checked by the
mission tests; the ramp recovery uses nearly the entire side-jet allowance.

Fuel budget, pressure taper below 12 kg, nonnegative thrust and trust regions are
convex constraints. MuJoCo advances at 0.002 s with frictional, compliant tire
contacts and checks chassis and parked-vehicle impacts. Tires are ellipsoids on
rigid axles; suspension travel, sensor noise and pressure dynamics are omitted.
Each jet is applied separately at its mounting point. The analytical RK4
predictor includes wheel spin angular momentum but omits contact forces. Model
parameters remain known; using MuJoCo does not itself test parameter uncertainty.
See [the MuJoCo car plant](web/src/compute/car-mujoco.js) for its generated MJCF
and physical integration.

The glTF is an original model based on reference images. Exact hardware positions and performance are estimates. The
[Car and Driver article](https://www.caranddriver.com/news/a74165059/tesla-roadster-thrusters-fan-system-explained/)
discusses ten perimeter jets. This demo uses six jets.
It cannot establish what a production Tesla could do.

### Whole-flight planning

The goal planner optimizes all airborne nozzle commands together. The objective
penalizes terminal position, velocity, attitude and angular-rate error, plus air
use. Intermediate positions are free. Three orientation gates and a signed
quaternion branch preserve the requested 360° jump roll; clearing the checkbox
removes those gates and targets an upright jump. The C-ramp retains its recovery.

The existing guidance/MPC supplies an initial guess and flight duration. SCvx
then runs up to 35 iterations on a 0.12 s grid, with a nonlinear merit check and
backtracking on control changes. The duration, takeoff state and contact sequence
remain fixed; this is a local trajectory optimizer. An unreachable target or a
failed initialization stops planning rather than switching controllers.

The accepted nonlinear trajectory supplies state references and an initial
control sequence to the 1.2 s feedback MPC. After the planned endpoint at 0.8 m
COM height, the reference lowers at 0.18 m/s until physical wheel contact. An
amber dashed line shows the complete plan; blue shows the current MPC prediction.
Export includes the full plan, terminal error, optimization time and merit history.
Included car recordings use the MuJoCo plant and whole-flight planning. Choose
Compute flight or Run live to apply a new target and generate a full plan.

## Rocket model

Each run builds a new rocket mission from the selected separation altitude and
an upright landing at the launch point with a target descent speed of 0.2 m/s. A feasible MuJoCo/SCvx
flight supplies the initial guess, engine schedule and attitude corridor. A
SCvx mission planner then optimizes seven states—mass, position and velocity—
and three world-force components. Its objective uses the separation altitude,
landing state and propellant use; intermediate seed positions are not tracking
costs. Burn order, engine counts and the initial phase durations remain scheduled.

A six-DoF SCvx MPC tracks the resulting plan in MuJoCo, choosing thrust in powered
flight and RCS moments during coast. The mission planner solves the remaining
trajectory again at boostback, entry, landing ignition and the center-engine
transition, starting from the actual state. Below 300 m, MPC uses a braking
reference based on measured foot clearance and the landing goal. There is no PD
handoff. The translational mission plan omits attitude dynamics; its corridor and
seed attitude approximate aerodynamic area. The tracking MPC includes attitude,
gimbal torque and changing inertia. This uses local planning with scheduled phases, without a global optimality
guarantee. The upper stage follows a
prescribed burn outside the booster MPC.

The green line shows the current mission plan, blue shows the short MPC forecast,
and silver shows the flight so far. Export includes the original plan and all
each later plan. Replay selects the plan that was available at that time.

The model uses the repository's tank depletion, changing COM and inertia, engine
moment arm and MuJoCo free-body conventions. Ascent uses nine engines. Main engine
cutoff precedes separation by 3 s. Opposite axial impulses conserve momentum and
give 3 m/s relative speed. The upper engine waits 8 s after separation and for 5 m axial
clearance. MPC uses RCS to turn the booster while its engines stay off.

Three separate burns follow separation: BOOSTBACK corrects the ballistic return
point, ENTRY slows the descent, and LANDING brakes for touchdown. Engines shut
down between these burns. Landing begins with three engines and reduces to one;
the center engine stays lit through hover and final descent until contact.
Each engine is bounded to 57 to 100% throttle with a maximum of 845 kN. These bounds remain constant
throughout flight. Guidance schedules the burns; SCvx chooses the forces.
Thrust stays inside a 6° cone. Coast pitch/yaw RCS is bounded at an assumed
2 MN·m per axis; roll RCS is bounded at 17.5 kN·m. Powered pitch/yaw RCS is disabled.

The horizon is four seconds in ascent and boostback, 2.4 s in coast, and 1.2 s
in entry. Three-engine landing uses thirty-two steps of at most 0.06 s, with
replanning every 0.1 s. One-engine landing uses sixteen steps of at most 0.25 s
during braking and hover, then at most 0.12 s during final descent. Steps shorten
near contact or fuel exhaustion, down to 0.002 s. Other phases replan every
0.4 s, then every 0.2 s below 300 m, 0.04 s below 30 m and 0.01 s below 5 m.
Warm starts shift by the elapsed fraction of a prediction step.

Guidance for the initial guess and terminal descent numerically predicts braking
with mass depletion and air resistance, then finds a bounded thrust reference
by bisection. It schedules
ignition and the transition to the center engine; SCvx still selects every
applied force. Horizontal references converge position and velocity toward the
launch point over the estimated remaining descent time. Below 80 m, guidance
checks whether the remaining mass can balance minimum engine thrust with a
4,000 kg approach reserve. Lighter boosters land directly. With enough mass,
MPC settles with the feet 2 m above the pad for one second, then lowers at up to
0.7 m/s toward 0.2 m/s before contact. Below 30 m, height uses the lowest physical
foot clearance, including tilt.

The browser's landing pad is centered on the launch point and has a 20 m radius.
Physical foot contact follows the folding-leg model. All engines cut at first foot contact. MuJoCo then
settles the vehicle for 12 s. The original native XML and controller are retained.

Browser actuators respond instantly. Nozzle and flame directions ease visually
over 0.18 s; this rendering filter does not change the forces in the simulation.
An exponential atmosphere uses sea-level density 1.225 kg/m³, scale height 8.5 km
and drag coefficient 0.8. Projected area depends on attitude; both the predictor
and MuJoCo plant apply drag at the center of mass. Wind, lift, aerodynamic
moments, Earth curvature, actuator lag, slosh and structural loads are omitted. Mass, geometry, effective specific impulse and guidance thresholds are estimates. The original native application has separate
PD modes; the browser controller does not invoke them.

## Equations, source and verification

`web/formulation.html` contains the dynamics, scaled SCvx subproblem, per-vehicle
constraints, implementation links and papers. `web/src/compute/scvx.js` builds
multiple-shooting conic problems and independently checks nonlinear rollouts.
Clarabel is vendored from `cvxjs@0.1.4`, with its Apache license and provenance.
MuJoCo 3.16.0 and Three.js 0.180.0 are pinned npm dependencies.

```sh
npm --prefix roadster/web test
PYTHONPATH=src .venv/bin/python -m pytest tests/test_roadster.py -q
npm --prefix roadster/web run build
```

Tests run the WASM optimizer and physics through both full nominal missions. They compare predicted dynamics with MuJoCo and check stage continuity, hardware limits, worker streaming and cancellation. They also check relative links under a repository prefix, model geometry, contact, playback and cameras.

Regenerate the browser recordings with the same modules used by the worker:

```sh
node roadster/scripts/record-browser.mjs car
node roadster/scripts/record-browser.mjs barrel
node roadster/scripts/record-browser.mjs rocket
```

`src/rocket_landing/roadster*.py`, `web/tests/fixtures/car-python.json` and
`scripts/build_model.py` retain the Python car reference and its asset generator.
The native Python car allows an explicitly labeled PD fallback on rejection;
browser runs stop instead. A maneuver can fail when parameters change; its recording retains the failure.

The nominal rocket flight starts with an estimated 407,500 kg of first-stage
propellant and separates at 75 km. The control selects actual separation altitude
from 70 to 80 km; it no longer sets a predicted ballistic apogee. A simplified
ascent profile pitches 20° from vertical. Cutoff anticipates three seconds of
coast before separation, and upper-stage ignition follows eight seconds later.
Fuel use is integrated from thrust and effective specific impulse; residual fuel
is an output rather than a prescribed value.

Full-flight checks cover 70, 75 and 80 km separation, three distinct return burns,
engine bounds, accepted SCvx commands, gentle touchdown and return to the launch
pad. The default flight lands directly because minimum engine thrust exceeds its
final weight. MuJoCo measures foot contact; SCvx supplies every flight command.
Engine bounds remain 57–100% of 845 kN per engine throughout flight. Estimated
dry mass is 25.6 tonnes and effective specific impulse is 282 s. The model uses
constant thrust bounds with altitude.

The calibration range draws on [Fraunhofer's separation observation](https://www.emi.fraunhofer.de/en/news/news-press/fraunhofer-satellite-ernst-monitors-launching-rockets.html)
(about 70 km and 6,000 km/h) and the [CRS-7 planned ascent](https://www.nasa.gov/wp-content/uploads/2018/07/spacex_nasa_crs-7_presskit.pdf)
(80 km at cutoff). The staging delays follow the Falcon user guide's sample LEO
timeline. Mission profiles differ; these values do not reproduce one specific
flight or establish its remaining propellant.

References: [engine data](https://www.spacex.com/assets/media/falcon-users-guide-2025-05-09.pdf),
[RTLS burn sequence](https://www.spacex.com/launches/twilight),
[Roadster reveal](https://www.youtube.com/live/qt5p-_tE8d0),
[Roadster engineering interview](https://www.youtube.com/watch?v=22rvpUecLts).

The exported MP4s can be scored with the original electronic instrumental and
motor, cold-gas and rocket effects that follow the recorded controls:

```sh
python roadster/scripts/score-video.py --source-version v11 --version v12
```

This requires NumPy, FFmpeg and the source MP4s plus their saved shot list and
timing in `output/video/threejs-v11-render-source/`. Keep the nominal recordings
from the same render. The script preserves the video packets, synchronizes
effects to the replay clock, and writes English and Chinese full and landing-only
exports. Music ducks under the effects. The stereo master targets
−16 LUFS with true peaks below −1.5 dBTP. Faster rocket playback changes the
timing of the effects while keeping their pitch. Separate WAV stems and cue,
decode, duration and video-integrity checks are saved beside the exports.
