<p align="center"><img src="design/logos/scvx-mpc.svg" width="420" alt="SCvx MPC Demos logo" /></p>

<p align="center"><a href="https://lzyang2000.github.io/SCvx-MPC/">Open the website</a></p>

# SCvx MPC Demos

Explore model predictive control in two browser simulations, rendered with Three.js and optimized with WebAssembly:

- Car: C-ramp recovery or a 360° barrel roll over five parked vehicles. Six cold-air jets control flight. Wheel contacts govern landing and rollout.
- Rocket: launch, stage separation and first-stage return. SCvx controls booster thrust and attitude through touchdown in MuJoCo WASM.

Replay the included recordings or choose **Run live** to compute physics and new SCvx plans in a background worker. Both demos show predicted and actual trajectories, solve latency and actuator commands. Rejected plans remain visible for inspection. Hardware values are estimates, and guidance sets the flight phases and references.

Choose EN or 中文 in the navigation bar to change language. Motor, jet and rocket audio
starts on your first interaction, with sound enabled at 30% volume by default.
Music, mute and volume have separate controls.
Sound follows live computation and replay, and pauses with the simulation.

## Run locally

```sh
git clone git@github.com:lzyang2000/SCvx-MPC.git
cd SCvx-MPC
npm ci --prefix roadster/web
npm --prefix roadster/web run build
python3 -m http.server 8765 --directory roadster/web/dist --bind 127.0.0.1
```

Open [localhost:8765](http://127.0.0.1:8765). Use Node.js 22 or newer. The browser runs the solver and physics locally, so a plain static server is enough.

## Formulation and implementation

The site's [formulation page](https://lzyang2000.github.io/SCvx-MPC/formulation.html)
explains the equations, constraints, car and rocket implementations, model limits
and original SCvx papers. See the [demo guide](roadster/README.md) for hardware
assumptions, controls and recording commands.

- [SCvx loop](roadster/web/src/compute/scvx.js) and [Clarabel WASM interface](roadster/web/src/compute/conic.js)
- [Car dynamics and maneuver references](roadster/web/src/compute/car-mission.js)
- [Rocket phases and guidance](roadster/web/src/compute/rocket-mission.js), [dynamics](roadster/web/src/compute/rocket-dynamics.js) and [MuJoCo plant](roadster/web/public/assets/rocket.xml)
- [Native Python simulator guide](NATIVE_README.md), [methods](METHODS.md) and [MPC design](MPC_DESIGN.md)

The native Python simulator includes PD modes for comparison. The browser
controllers stop when the solver rejects a plan.

## Verify

```sh
npm --prefix roadster/web test
npm --prefix roadster/web run build
```

Tests run the WASM optimizer and MuJoCo runtime through both car maneuvers and the full rocket mission. They check actuator limits, collision clearance, worker streaming and paths under a repository subdirectory.
The Python setup and tests are documented in [the native guide](NATIVE_README.md).

Three.js, MuJoCo and Clarabel licenses and provenance ship with the built site.
