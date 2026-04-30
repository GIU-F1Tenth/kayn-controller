# KAYN Controller Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix dead-param wiring, clean up four code-quality issues, and rewrite the README into a complete tuning reference.

**Architecture:** Six targeted file changes. No new files, no restructuring. Dependency order: curvature.py → fsm.py → kayn_node.py (wiring chain), then state_handoff.py, kayn_params.yaml, and README independently.

**Tech Stack:** Python 3.10, ROS2 Humble, pytest

---

## File Map

| File | Change |
|---|---|
| `kayn_controller/supervisor/curvature.py` | Add `enter_threshold`, `exit_threshold` constructor args |
| `kayn_controller/supervisor/fsm.py` | Add `confirm_steps`, `blend_window`, `log_fn` args; read thresholds from `curv_est` |
| `kayn_controller/kayn_node.py` | Wire 4 dead params; replace `g = lambda` with `_p()` method |
| `kayn_controller/supervisor/state_handoff.py` | `isinstance` instead of type-name string |
| `config/kayn_params.yaml` | Restore correct defaults |
| `README.md` | Full parameter reference replacing shallow config section |

---

## Task 1: CurvatureEstimator — add threshold constructor args

**Files:**
- Modify: `kayn_controller/supervisor/curvature.py`

- [ ] **Step 1: Open the file and read the current state**

Confirm `ENTER_THRESHOLD = 0.10` and `EXIT_THRESHOLD = 0.06` are module-level constants and that `__init__` only takes `lookahead`.

- [ ] **Step 2: Add threshold args to `__init__`**

Replace the `__init__` method (lines 36–37):

```python
class CurvatureEstimator:
    def __init__(self, lookahead: int = 10,
                 enter_threshold: float = ENTER_THRESHOLD,
                 exit_threshold: float = EXIT_THRESHOLD):
        self.lookahead = lookahead
        self.enter_threshold = enter_threshold
        self.exit_threshold = exit_threshold
```

Module-level `ENTER_THRESHOLD` and `EXIT_THRESHOLD` constants stay — they serve as defaults and are still imported by fsm.py as fallbacks.

- [ ] **Step 3: Run existing curvature tests**

```bash
cd /home/ubuntu/sim/racing_playground/master/kayn_controller
python -m pytest tests/test_curvature.py -v
```

Expected: all tests pass (constructor signature is backwards-compatible — all new args have defaults).

- [ ] **Step 4: Commit**

```bash
git add kayn_controller/supervisor/curvature.py
git commit -m "feat: add enter/exit threshold args to CurvatureEstimator"
```

---

## Task 2: FSM — add confirm_steps, blend_window, log_fn; read thresholds from curv_est

**Files:**
- Modify: `kayn_controller/supervisor/fsm.py`

- [ ] **Step 1: Update imports and module constants**

The module-level constants `BLEND_WINDOW` and `CONFIRM_STEPS` stay (they are still the defaults and imported by tests). `ENTER_THRESHOLD` and `EXIT_THRESHOLD` are no longer imported — the FSM will read them from `self.curv_est`. Remove them from the import line:

Replace:
```python
from .curvature import CurvatureEstimator, ENTER_THRESHOLD, EXIT_THRESHOLD
```

With:
```python
from .curvature import CurvatureEstimator
```

- [ ] **Step 2: Update `FSM.__init__` signature and body**

Replace the full `__init__` method:

```python
def __init__(self, lqr, mpc, stanley, curvature_estimator: CurvatureEstimator,
             warmup_steps: int = WARMUP_STEPS,
             warmup_ctrl: str = 'stanley',
             straight_ctrl: str = 'lqr',
             curve_ctrl: str = 'mpc',
             fallback_ctrl: str = 'stanley',
             confirm_steps: int = CONFIRM_STEPS,
             blend_window: int = BLEND_WINDOW,
             log_fn=print):
    for slot, val in [('warmup', warmup_ctrl), ('straight', straight_ctrl),
                      ('curve', curve_ctrl),   ('fallback', fallback_ctrl)]:
        if val not in _VALID_CTRLS:
            raise ValueError(f"fsm.{slot}_controller={val!r} — must be one of {_VALID_CTRLS}")

    self.lqr      = lqr
    self.mpc      = mpc
    self.stanley  = stanley
    self.curv_est = curvature_estimator
    self._warmup_steps   = warmup_steps
    self._warmup_ctrl    = warmup_ctrl
    self._straight_ctrl  = straight_ctrl
    self._curve_ctrl     = curve_ctrl
    self._fallback_ctrl  = fallback_ctrl
    self._confirm_steps  = confirm_steps
    self._blend_window   = blend_window
    self._log_fn         = log_fn

    self.state = KAYNState.WARMUP
    self._warmup_count   = 0
    self._confirm_count  = 0
    self._blend_step     = 0
    self._recovery_count = 0
```

- [ ] **Step 3: Replace CONFIRM_STEPS references**

In `_step_straight`, replace:
```python
if self._confirm_count >= CONFIRM_STEPS:
```
With:
```python
if self._confirm_count >= self._confirm_steps:
```

In `_step_curve`, replace:
```python
if self._confirm_count >= CONFIRM_STEPS:
```
With:
```python
if self._confirm_count >= self._confirm_steps:
```

In `_step_fallback`, replace:
```python
if self._recovery_count >= CONFIRM_STEPS:
```
With:
```python
if self._recovery_count >= self._confirm_steps:
```

- [ ] **Step 4: Replace BLEND_WINDOW references**

In `_step_blend_out`, replace:
```python
alpha = self._blend_step / BLEND_WINDOW
...
if self._blend_step >= BLEND_WINDOW:
```
With:
```python
alpha = self._blend_step / self._blend_window
...
if self._blend_step >= self._blend_window:
```

In `_step_blend_in`, replace:
```python
alpha = self._blend_step / BLEND_WINDOW
...
if self._blend_step >= BLEND_WINDOW:
```
With:
```python
alpha = self._blend_step / self._blend_window
...
if self._blend_step >= self._blend_window:
```

- [ ] **Step 5: Replace ENTER_THRESHOLD / EXIT_THRESHOLD references**

In `_step_straight`, replace:
```python
if kappa > ENTER_THRESHOLD:
```
With:
```python
if kappa > self.curv_est.enter_threshold:
```

In `_step_curve`, replace:
```python
if kappa < EXIT_THRESHOLD:
```
With:
```python
if kappa < self.curv_est.exit_threshold:
```

In `_step_fallback`, replace:
```python
target = KAYNState.CURVE if kappa > ENTER_THRESHOLD else KAYNState.STRAIGHT
```
With:
```python
target = KAYNState.CURVE if kappa > self.curv_est.enter_threshold else KAYNState.STRAIGHT
```

- [ ] **Step 6: Replace print() with log_fn**

In `_transition`, replace:
```python
print(f"[KAYN] {self.state.name} → {new_state.name} | {reason} | idx={ref_idx}")
```
With:
```python
self._log_fn(f"[KAYN] {self.state.name} → {new_state.name} | {reason} | idx={ref_idx}")
```

- [ ] **Step 7: Run existing FSM tests**

```bash
cd /home/ubuntu/sim/racing_playground/master/kayn_controller
python -m pytest tests/test_fsm.py -v
```

Expected: all tests pass. All new args have defaults matching the old constants — no test call sites need to change.

- [ ] **Step 8: Commit**

```bash
git add kayn_controller/supervisor/fsm.py
git commit -m "feat: accept confirm_steps, blend_window, log_fn in FSM; read thresholds from curv_est"
```

---

## Task 3: kayn_node — wire dead params, remove lambda

**Files:**
- Modify: `kayn_controller/kayn_node.py`

- [ ] **Step 1: Add `_p` helper method**

Add this private method to `KAYNNode`, just before `_declare_params`:

```python
def _p(self, name: str):
    return self.get_parameter(name).value
```

- [ ] **Step 2: Replace `_load_params` body**

Replace the entire `_load_params` method:

```python
def _load_params(self):
    self.wheelbase     = self._p('wheelbase')
    self.dt            = self._p('dt')
    self.mpc_n         = self._p('mpc.horizon_n')
    self.max_speed     = self._p('max_speed')
    self.max_steering  = self._p('max_steering')
    self.max_accel     = self._p('max_accel')
    self.control_hz    = self._p('control_hz')
    self.odom_topic    = self._p('odom_topic')
    self.traj_topic    = self._p('trajectory_topic')
    self.ready_topic   = self._p('path_ready_topic')
    self.drive_topic   = self._p('drive_topic')
    self.debug         = self._p('debug')
    self.log_every_n   = self._p('log_every_n')

    self.lqr_Q = np.diag([self._p('lqr.q_px'), self._p('lqr.q_py'),
                           self._p('lqr.q_theta'), self._p('lqr.q_v')])
    self.lqr_R = np.diag([self._p('lqr.r_delta'), self._p('lqr.r_a')])
    self.mpc_Q = np.diag([self._p('mpc.q_px'), self._p('mpc.q_py'),
                           self._p('mpc.q_theta'), self._p('mpc.q_v')])
    self.mpc_R = np.diag([self._p('mpc.r_delta'), self._p('mpc.r_a')])

    self.stanley_k      = self._p('stanley.k')
    self.warmup_ctrl    = self._p('fsm.warmup_controller')
    self.straight_ctrl  = self._p('fsm.straight_controller')
    self.curve_ctrl     = self._p('fsm.curve_controller')
    self.fallback_ctrl  = self._p('fsm.fallback_controller')
    self.warmup_steps   = self._p('fsm.warmup_steps')
    self.confirm_steps  = self._p('fsm.confirm_steps')
    self.blend_window   = self._p('fsm.blend_window')
    self.curv_lookahead = self._p('fsm.lookahead')
    self.enter_threshold = self._p('fsm.enter_threshold')
    self.exit_threshold  = self._p('fsm.exit_threshold')
```

- [ ] **Step 3: Pass new params in `_build_controllers`**

Replace the entire `_build_controllers` method:

```python
def _build_controllers(self):
    model = BicycleModel(L=self.wheelbase, dt=self.dt)
    curv_est = CurvatureEstimator(
        lookahead=self.curv_lookahead,
        enter_threshold=self.enter_threshold,
        exit_threshold=self.exit_threshold,
    )
    self.fsm = FSM(
        lqr=LQRController(model, Q=self.lqr_Q, R=self.lqr_R),
        mpc=MPCController(model, N=self.mpc_n, Q=self.mpc_Q, R=self.mpc_R),
        stanley=StanleyController(k=self.stanley_k, model=model),
        curvature_estimator=curv_est,
        warmup_steps=self.warmup_steps,
        warmup_ctrl=self.warmup_ctrl,
        straight_ctrl=self.straight_ctrl,
        curve_ctrl=self.curve_ctrl,
        fallback_ctrl=self.fallback_ctrl,
        confirm_steps=self.confirm_steps,
        blend_window=self.blend_window,
        log_fn=self.get_logger().info,
    )
```

- [ ] **Step 4: Verify import of CurvatureEstimator in kayn_node.py**

`kayn_node.py` already imports from `.supervisor.curvature` — confirm `CurvatureEstimator` is in that import. If not, add it:

```python
from .supervisor.curvature import CurvatureEstimator
```

- [ ] **Step 5: Commit**

```bash
git add kayn_controller/kayn_node.py
git commit -m "fix: wire fsm.enter_threshold, exit_threshold, confirm_steps, blend_window from params"
```

---

## Task 4: state_handoff — isinstance fix

**Files:**
- Modify: `kayn_controller/supervisor/state_handoff.py`

- [ ] **Step 1: Add MPCController import**

Add at the top of the file, after the existing imports:

```python
from ..controllers.mpc import MPCController
```

- [ ] **Step 2: Replace type-name string check**

In `handoff()`, replace:
```python
if type(incoming_controller).__name__ == 'MPCController':
```
With:
```python
if isinstance(incoming_controller, MPCController):
```

- [ ] **Step 3: Run full test suite**

```bash
cd /home/ubuntu/sim/racing_playground/master/kayn_controller
python -m pytest tests/ -v
```

Expected: all tests pass.

- [ ] **Step 4: Commit**

```bash
git add kayn_controller/supervisor/state_handoff.py
git commit -m "fix: use isinstance instead of type name string in state_handoff"
```

---

## Task 5: kayn_params.yaml — restore correct defaults

**Files:**
- Modify: `config/kayn_params.yaml`

- [ ] **Step 1: Replace the file contents**

Write the full corrected file:

```yaml
kayn_controller_node:
  ros__parameters:

    # ── Vehicle ──────────────────────────────────────────────────────────────
    wheelbase: 0.33        # [m]  kinematic model parameter; must match physical car
    dt: 0.005              # [s]  integration timestep; must equal 1 / control_hz
    control_hz: 200.0      # [Hz] control loop rate

    # ── LQR ──────────────────────────────────────────────────────────────────
    # Q = diag([q_px, q_py, q_theta, q_v])  state error penalty
    # R = diag([r_delta, r_a])               control effort penalty
    lqr.q_px:    5.0   # raise to track lateral position more aggressively
    lqr.q_py:    5.0   # raise to penalise lateral drift on straights
    lqr.q_theta: 6.0   # raise if car wanders angularly on straights
    lqr.q_v:     1.0   # raise to track reference speed more closely
    lqr.r_delta: 4.0   # raise to damp steering oscillations
    lqr.r_a:     0.3   # raise for smoother acceleration profile

    # ── MPC ───────────────────────────────────────────────────────────────────
    # Same Q/R semantics as LQR.
    mpc.horizon_n:  20     # prediction steps; longer = better lookahead, slower solve
    mpc.q_px:    5.0
    mpc.q_py:    5.0
    mpc.q_theta: 6.0
    mpc.q_v:     6.0   # strong velocity tracking; prevents overspeeding on curves
    mpc.r_delta: 4.0   # raise (e.g. 8.0) to smooth MPC steering on tight curves
    mpc.r_a:     0.3
    mpc.timeout_ms: 5.0  # solver budget [ms]; exceeded → FSM enters FALLBACK

    # ── Stanley ───────────────────────────────────────────────────────────────
    stanley.k: 1.5       # cross-track gain; raise for tighter tracking, lower if oscillating

    # ── FSM controller slots ──────────────────────────────────────────────────
    # Each FSM state runs an independently chosen controller.
    # Valid values: "stanley" | "lqr" | "mpc"
    fsm.warmup_controller:   "stanley"
    fsm.straight_controller: "lqr"
    fsm.curve_controller:    "mpc"
    fsm.fallback_controller: "stanley"

    # ── FSM timing ────────────────────────────────────────────────────────────
    fsm.warmup_steps:  50  # steps before leaving WARMUP (50 steps = 1 s at 200 Hz)
    fsm.confirm_steps:  3  # consecutive samples required before any state transition
    fsm.blend_window:   5  # steps to interpolate between controllers at transitions

    # ── Curvature detection ───────────────────────────────────────────────────
    # Menger curvature kappa = 1/R [rad/m] estimated over lookahead waypoints.
    # Hysteresis: exit_threshold must be strictly less than enter_threshold.
    fsm.enter_threshold: 0.10  # kappa above which CURVE is entered  (R < 10 m)
    fsm.exit_threshold:  0.06  # kappa below which STRAIGHT re-enters (R > 16.7 m)
    fsm.lookahead:       10    # waypoints ahead used for curvature estimate

    # ── Speed / limits ────────────────────────────────────────────────────────
    max_speed:    8.0     # [m/s]   hard cap on trajectory speed setpoint
    max_steering: 0.4189  # [rad]   must match DELTA_MAX in bicycle_model.py (24°)
    max_accel:    5.0     # [m/s²]  hard acceleration cap

    # ── Topics ────────────────────────────────────────────────────────────────
    odom_topic:       "/ego_racecar/odom"
    trajectory_topic: "/horizon_mapper/reference_trajectory"
    path_ready_topic: "/horizon_mapper/path_ready"
    drive_topic:      "/drive"

    # ── Logging ───────────────────────────────────────────────────────────────
    debug:       false
    log_every_n: 25
```

- [ ] **Step 2: Commit**

```bash
git add config/kayn_params.yaml
git commit -m "fix: restore correct FSM defaults and add per-param tuning comments"
```

---

## Task 6: README — full parameter reference

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Replace the Configuration section**

Locate the `## Configuration` heading and replace everything from that heading down to (but not including) `## Running Tests` with:

```markdown
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
```

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: replace shallow config section with full parameter reference"
```

---

## Task 7: Final verification

- [ ] **Step 1: Run full test suite**

```bash
cd /home/ubuntu/sim/racing_playground/master/kayn_controller
python -m pytest tests/ -v
```

Expected output — all tests pass:
```
tests/test_bicycle_model.py  ...  PASSED
tests/test_curvature.py      ...  PASSED
tests/test_fsm.py            ...  PASSED
tests/test_lqr.py            ...  PASSED
tests/test_stanley.py        ...  PASSED
```

(test_mpc.py requires acados — skip if not installed: `python -m pytest tests/ -v --ignore=tests/test_mpc.py`)

- [ ] **Step 2: Verify yaml default correctness**

Manually confirm in `config/kayn_params.yaml`:
- `fsm.straight_controller: "lqr"`
- `fsm.curve_controller: "mpc"`
- `max_steering: 0.4189`
- `dt * control_hz == 1.0` (0.005 × 200 = 1.0)

- [ ] **Step 3: Final commit if any loose files**

```bash
git status
# If anything unstaged:
git add -p
git commit -m "chore: final cleanup"
```
