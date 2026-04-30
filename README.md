# KAYN Controller

Hybrid path-tracking controller for F1TENTH autonomous racing.
Combines **LQR**, **MPC (acados RTI)**, and **Stanley** under a curvature-aware FSM supervisor.

Named after the League of Legends champion.

---

## Architecture

```
Odometry ──┐
           ├──► FSM Supervisor ──► AckermannDriveStamped
Reference  ─┘    │
Trajectory        ├─ WARMUP    → warmup_controller   (default: Stanley)
                  ├─ STRAIGHT  → straight_controller  (default: LQR)
                  ├─ BLEND_OUT → blend(straight→curve)
                  ├─ CURVE     → curve_controller     (default: MPC)
                  ├─ BLEND_IN  → blend(curve→straight)
                  └─ FALLBACK  → fallback_controller  (default: Stanley)
```

### Controllers

| Controller | Algorithm | Use case |
|---|---|---|
| **Stanley** | Heading error + cross-track gain | Warmup, fallback |
| **LQR** | Discrete DARE optimal gain | Straights, low curvature |
| **MPC** | acados RTI, N=20, HPIPM QP | Sharp curves (κ > 0.10 rad/m) |

### FSM Transitions

```
WARMUP  ──(50 steps)──► STRAIGHT
STRAIGHT ──(κ > 0.10 for 3 steps)──► BLEND_OUT
BLEND_OUT ──(5 blend steps)──► CURVE
CURVE ──(κ < 0.06 for 3 steps)──► BLEND_IN
CURVE ──(MPC timeout / infeasible)──► FALLBACK
BLEND_IN ──(5 blend steps)──► STRAIGHT
FALLBACK ──(MPC recovered for 3 steps)──► CURVE or STRAIGHT
```

Blend zones linearly interpolate between the two controllers over 5 steps to prevent steering jumps.

---

## Simulation Results

Run on a straight–curve–straight–curve–straight chicane track (2× 90° turns, R=4m, v=2–3 m/s):

### Trajectory
![Trajectory](simulation/results/trajectory.png)
Actual path (coloured by FSM state) overlaid on the reference. The car tracks the dashed reference closely through both curves.

### Cross-Track Error
![Cross-Track Error](simulation/results/cross_track_error.png)
CTE stays near zero on straights (LQR). Rises to ~0.10m through curves (MPC catching up to the geometry) then snaps back as BLEND_IN hands off to LQR. Vertical lines mark FSM state transitions.

### Control Inputs
![Control Inputs](simulation/results/control_inputs.png)
Steering (blue) and acceleration (orange). No discontinuous jump at transitions — the 5-step blend zone smooths the hand-off between controllers. MPC applies up to ±24° on the tight R=4m turns.

### Curvature + FSM State
![Curvature + FSM](simulation/results/curvature_fsm.png)
Menger curvature (purple) cleanly crosses the 0.10 rad/m enter-threshold at the curve entry and drops below the 0.06 rad/m exit-threshold on the straight. The grey step trace shows the FSM advancing through WARMUP → STRAIGHT → BLEND\_OUT → CURVE → BLEND\_IN → STRAIGHT correctly for both turns.

**Known limitations:**
- LQR shows small steering oscillations on straights (~2–3°) due to frequent gain recomputation at the linearization-change threshold; increase `lqr.gain_cache_tol` if this matters
- MPC steering oscillates on very tight curves (R < 4m); reduce `mpc.q_theta` or increase `mpc.r_delta` to damp it
- FALLBACK recovery probe runs MPC every step — expensive if MPC is consistently slow; consider adding a cooldown

---

## File Structure

```
kayn_controller/
├── kayn_controller/
│   ├── controllers/
│   │   ├── bicycle_model.py    # Kinematic bicycle, RK4, linearisation
│   │   ├── lqr.py              # Discrete DARE LQR with gain caching
│   │   ├── mpc.py              # acados RTI OCP builder + solver
│   │   └── stanley.py          # Stanley controller (left-normal convention)
│   ├── supervisor/
│   │   ├── fsm.py              # 6-state FSM with per-state controller slots
│   │   ├── curvature.py        # Menger curvature estimator
│   │   └── state_handoff.py    # MPC warm-start on entry
│   └── kayn_node.py            # ROS2 node
├── simulation/
│   ├── sim.py                  # Closed-loop sim (no ROS2)
│   ├── track.py                # Waypoint generators
│   └── plot.py                 # 4-panel results plotter
├── tests/                      # 29 pytest tests
├── config/kayn_params.yaml     # All tunable parameters
├── launch/kayn.launch.py       # ROS2 launch file
└── math/                       # Derivation notes (LQR, MPC, Stanley)
```

---

## Configuration

All parameters live in `config/kayn_params.yaml`. The node declares every parameter
with a default so the file can be used as a drop-in override or omitted entirely.

### Vehicle

| Parameter | Default | Description |
|---|---|---|
| `wheelbase` | `0.33` m | Kinematic model parameter; must match the physical car |
| `dt` | `0.005` s | Integration timestep; must equal `1 / control_hz` |
| `control_hz` | `200.0` Hz | Control loop rate |

### FSM — controller slots

Each FSM state runs an independently chosen controller. Valid values: `"stanley"` \| `"lqr"` \| `"mpc"`

| Parameter | Default | State it governs |
|---|---|---|
| `fsm.warmup_controller` | `"stanley"` | WARMUP |
| `fsm.straight_controller` | `"lqr"` | STRAIGHT |
| `fsm.curve_controller` | `"mpc"` | CURVE |
| `fsm.fallback_controller` | `"stanley"` | FALLBACK |

### FSM — timing

| Parameter | Default | Effect |
|---|---|---|
| `fsm.warmup_steps` | `50` | Steps before leaving WARMUP. At 200 Hz, 50 steps = 0.25 s |
| `fsm.confirm_steps` | `3` | Consecutive samples required before any state transition. Raise to reduce false triggers on noisy curvature |
| `fsm.blend_window` | `5` | Steps over which output is linearly interpolated at transitions. Raise for smoother handoff, lower for faster response |

### Curvature detection

Menger curvature κ = 1/R [rad/m] estimated over a lookahead window ahead of the current waypoint.

| Parameter | Default | Effect |
|---|---|---|
| `fsm.enter_threshold` | `0.10` rad/m | κ above which CURVE is entered (R < 10 m). Lower to detect curves earlier |
| `fsm.exit_threshold` | `0.06` rad/m | κ below which STRAIGHT re-enters (R > 16.7 m). Must be strictly less than `enter_threshold` — closing the gap causes rapid oscillation at curve boundaries |
| `fsm.lookahead` | `10` | Waypoints ahead used for the estimate. Raise for earlier detection, lower for a more local view |

### Stanley

| Parameter | Default | Effect |
|---|---|---|
| `stanley.k` | `1.5` | Cross-track gain. Raise for tighter lateral tracking, lower if the car oscillates at speed |

### LQR — weights

`Q = diag([q_px, q_py, q_theta, q_v])` penalises state error.
`R = diag([r_delta, r_a])` penalises control effort.

| Parameter | Default | Effect |
|---|---|---|
| `lqr.q_px` | `5.0` | Position error penalty (longitudinal) |
| `lqr.q_py` | `5.0` | Position error penalty (lateral). Raise to tighten lateral tracking on straights |
| `lqr.q_theta` | `6.0` | Heading error penalty. Raise if the car wanders angularly |
| `lqr.q_v` | `1.0` | Speed error penalty. Raise to follow the reference speed more closely |
| `lqr.r_delta` | `4.0` | Steering effort penalty. Raise to damp oscillations on straights |
| `lqr.r_a` | `0.3` | Acceleration effort penalty. Raise for a smoother acceleration profile |

### MPC — weights

Same Q/R semantics as LQR.

| Parameter | Default | Effect |
|---|---|---|
| `mpc.q_px` | `5.0` | Position error penalty (longitudinal) |
| `mpc.q_py` | `5.0` | Position error penalty (lateral) |
| `mpc.q_theta` | `6.0` | Heading error penalty |
| `mpc.q_v` | `6.0` | Speed error penalty. Set higher than LQR to prevent MPC from overspeeding on curves |
| `mpc.r_delta` | `4.0` | Steering effort penalty. Raise (e.g. `8.0`) to smooth steering on tight curves |
| `mpc.r_a` | `0.3` | Acceleration effort penalty |

### MPC — solver

| Parameter | Default | Effect |
|---|---|---|
| `mpc.horizon_n` | `20` | Prediction horizon steps. Longer horizon sees further ahead but increases solve time |
| `mpc.timeout_ms` | `5.0` ms | Solver budget. If exceeded, FSM transitions to FALLBACK. Raise (e.g. `10.0`) if MPC triggers false fallbacks at high speed |

### Speed / limits

| Parameter | Default | Effect |
|---|---|---|
| `max_speed` | `8.0` m/s | Hard cap applied to the trajectory speed setpoint |
| `max_steering` | `0.4189` rad | Must match `DELTA_MAX` in `bicycle_model.py` (24°) |
| `max_accel` | `5.0` m/s² | Hard acceleration cap |

### Common recipes

| Goal | Change |
|---|---|
| Pure LQR everywhere | `fsm.curve_controller: "lqr"` |
| Pure Stanley | `fsm.straight_controller: "stanley"`, `fsm.curve_controller: "stanley"` |
| Skip warmup | `fsm.warmup_steps: 0` |
| Detect curves earlier | lower `fsm.enter_threshold` (e.g. `0.07`) |
| Smoother MPC steering | raise `mpc.r_delta` (e.g. `8.0`) |
| Tighter straight tracking | raise `lqr.q_py` (e.g. `10.0`) |
| Prevent MPC fallback | raise `mpc.timeout_ms` (e.g. `10.0`) |

---

## Running Tests

```bash
# From the kayn_controller directory
ACADOS_SOURCE_DIR=/path/to/acados \
LD_LIBRARY_PATH=/path/to/acados/lib \
python -m pytest tests/ -v
```

Expected: **29 passed**

| Test file | What it covers |
|---|---|
| `test_bicycle_model.py` | RK4 integration, linearisation shapes and values, front-axle geometry |
| `test_lqr.py` | DARE convergence, gain shape, straight-line convergence, gain caching |
| `test_mpc.py` | OCP setup, feasibility on curves, solve time budget, steering limits |
| `test_stanley.py` | Sign convention (left/right), zero-error, CTE convergence |
| `test_curvature.py` | Circle κ accuracy, straight near-zero, hysteresis enter/exit |
| `test_fsm.py` | Warmup→straight transition, state machine transitions, blending, fallback, controller slots |

---

## Running the Simulation

No ROS2 required:

```bash
ACADOS_SOURCE_DIR=/path/to/acados \
LD_LIBRARY_PATH=/path/to/acados/lib \
python simulation/sim.py
# Output: simulation/results/kayn_sim.png
```

---

## ROS2 Interface

**Subscriptions:**

| Topic | Type | Description |
|---|---|---|
| `/odom` | `nav_msgs/Odometry` | Vehicle pose and velocity |
| `/horizon_mapper/reference_trajectory` | `giu_f1t_interfaces/VehicleStateArray` | Reference waypoints `[x, y, θ, v]` |
| `/horizon_mapper/path_ready` | `std_msgs/Bool` | Gate to start control |

**Publications:**

| Topic | Type | Description |
|---|---|---|
| `/kayn/drive` | `ackermann_msgs/AckermannDriveStamped` | Steering angle + acceleration + speed |
| `/kayn/mode` | `std_msgs/String` | Active FSM state name |
| `/kayn/cross_track_error` | `std_msgs/Float32` | Signed lateral error [m] |
| `/kayn/curvature` | `std_msgs/Float32` | Estimated κ ahead [rad/m] |
| `/kayn/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Mode, path_ready, traj_len, iter count |

**Launch:**

```bash
ros2 launch kayn_controller kayn.launch.py
# or with a custom config:
ros2 launch kayn_controller kayn.launch.py params_file:=/path/to/my_params.yaml
```

---

## Dependencies

| Dependency | Version | Notes |
|---|---|---|
| ROS2 | Humble | ament_python build |
| Python | ≥ 3.10 | |
| numpy | any | |
| scipy | any | DARE solver for LQR |
| acados | ≥ 0.3 | Must be built from source on arm64 |
| casadi | any | Pulled in by acados Python interface |
| ackermann_msgs | ROS2 | |
| giu_f1t_interfaces | internal | VehicleStateArray message |

**acados setup (arm64):**

```bash
git clone https://github.com/acados/acados.git /tmp/acados --depth=1
cd /tmp/acados
git submodule update --init external/hpipm external/blasfeo
mkdir build && cd build
cmake .. -DACADOS_WITH_QPOASES=OFF
make -j$(nproc) && make install
pip install -e /tmp/acados/interfaces/acados_template

# Download arm64 tera renderer
wget https://github.com/acados/tera_renderer/releases/download/v0.0.34/t_renderer-v0.0.34-linux-aarch64 \
  -O /tmp/acados/bin/t_renderer && chmod +x /tmp/acados/bin/t_renderer

export ACADOS_SOURCE_DIR=/tmp/acados
export LD_LIBRARY_PATH=/tmp/acados/lib:$LD_LIBRARY_PATH
```
