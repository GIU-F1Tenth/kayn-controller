"""
Vehicle parameters

Physical constants only: geometry, actuator limits, speed envelope..
"""

import math

# ── Geometry ──────────────────────────────────────────────────────────────────
WHEELBASE = 0.33                  # [m]    front-to-rear axle distance (F1TENTH)

# ── Actuator limits ───────────────────────────────────────────────────────────
DELTA_MAX = math.radians(24.0)   # [rad]  maximum steering angle (24°)
A_MAX     = 5.0                  # [m/s²] maximum acceleration magnitude
V_MAX     = 8.0                  # [m/s]  maximum longitudinal speed
