"""
parking/config.py - Tunable constants for standalone parking exit.
Team Yo_Error_404 - WRO 2026 Future Engineers (V2 Bot)
"""

# Hardware limits (calibrated from live vehicle)
SERVO_CENTER = 87            # True straight ahead is 87, not 90 (-3 deg trim)
SERVO_MIN = 30               # 2 deg inside mechanical stop (28)
SERVO_MAX = 140              # 2 deg inside mechanical stop (142)
MAX_PWM = 110                # Team cap on motor speed
TICKS_PER_CM = 20.78         # 2078 ticks/m, measured

# Networking & Hub
HUB_PORT = 5005              # UDP relay port used by dashboard.py
TELEMETRY_MAX_AGE_S = 0.5    # Freshness limit for telemetry
LOG_DIR = "/home/pi/parking/logs"

def load_params():
    return {}

# Parking exit tuning
EXIT_SPEED = 75              # Motor PWM during parking exit (out of 255)
STEER_LOCK_DEG = 50          # Increased turn lock (87+50=137 Right, 87-50=37 Left, safely within [30, 140])

TURN_OUT_CM = 25.0           # Stage 1: distance of initial turn-out arc (cm)
STRAIGHT_CM = 55.0           # Stage 2: forward displacement across lane (55 cm)
TURN_IN_CM = 25.0            # Stage 3: distance of turn-back arc (cm)
FORWARD_CM = 20.0            # Stage 4: final forward alignment run (20 cm)

HOLD_KP = 1.5                # Proportional gain for heading hold
STAGE_TIMEOUT_S = 14.0       # Failsafe stage timeout in seconds
