'''
before running need to set the masskg of the gripper finger
and the center of mass distance 

check the original moteus example: 
The example doesn't explain whether kLinkLength = 0.15 represents the full physical link length 
or its effective centre-of-mass lever arm.

'''

import asyncio
import math
import time
import moteus

# Gripper parameters
MASS_KG = 0.08                         # also definetly larger but should start with 0 first ie no force applied and then slowly increase it! 
COM_DISTANCE_M = 0.087                  # or 3 cm the gripper finger may be sligghtly larger
GRAVITY = 9.81  

# Original position bounds (revolutions)
POS_MIN = 0.03
POS_MAX = 0.195

TARGET_POSITION = 0.15

# Safety limits
MAX_TORQUE = 0.1       # Nm
MAX_VELOCITY = 0.75     # rev/s
MAX_ACCEL = 0.20 
CONTROL_PERIOD = 0.01   # 100 Hz
WATCHDOG_TIMEOUT = 0.1

# Gravity model
GRAVITY_ZERO = 0.0      # Calibrate this!
GRAVITY_SIGN = 1.0      # Verify torque direction

PRINT_PERIOD = 0.1


def gravity_torque(position):
    angle = (position - GRAVITY_ZERO) * 2.0 * math.pi

    return (
        GRAVITY_SIGN
        * MASS_KG
        * COM_DISTANCE_M
        * GRAVITY
        * math.sin(angle)
    )


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


async def main():
    c = moteus.Controller(id=1)

    await c.set_stop()

    try:
        if not POS_MIN <= TARGET_POSITION <= POS_MAX:
            raise ValueError("Target outside allowed bounds")

        state = await c.query()

        position = state.values[moteus.Register.POSITION]
        velocity = state.values[moteus.Register.VELOCITY]

        if not all(math.isfinite(x) for x in (position, velocity)):
            raise RuntimeError("Invalid initial feedback")

        if not POS_MIN <= position <= POS_MAX:
            raise RuntimeError("Initial position outside bounds")

        if abs(velocity) > 0.05:
            raise RuntimeError("Motor moving at startup")

        last_print = 0.0

        while True:

            gravity_ff = clamp(
                gravity_torque(position),
                -MAX_TORQUE,
                MAX_TORQUE
            )

            result = await c.set_position(
                position=TARGET_POSITION,
                velocity=0.0,

                # Position controller
                # kp_scale=0.5,
                # kd_scale=0.5,
                # ilimit_scale=0.0,

                # Gravity compensation
                feedforward_torque=0.5*gravity_ff,

                # Safety
                maximum_torque=MAX_TORQUE,
                velocity_limit=MAX_VELOCITY,
                accel_limit=MAX_ACCEL,
                watchdog_timeout=WATCHDOG_TIMEOUT,

                query=True,
            )

            if result is None:
                raise RuntimeError("No moteus response")

            position = result.values[moteus.Register.POSITION]
            velocity = result.values[moteus.Register.VELOCITY]
            measured_torque = result.values[moteus.Register.TORQUE]

            if not all(math.isfinite(x) for x in (
                position, velocity, measured_torque
            )):
                raise RuntimeError("Invalid feedback")

            position_error = TARGET_POSITION - position

            if not POS_MIN <= position <= POS_MAX:
                raise RuntimeError("Position limit exceeded")

            if abs(velocity) > MAX_VELOCITY + 0.02:
                raise RuntimeError("Velocity limit exceeded")

            now = time.monotonic()

            if now - last_print >= PRINT_PERIOD:
                print(
                    f"cmd={TARGET_POSITION:+.4f} rev | "
                    f"actual={position:+.4f} rev | "
                    f"error={position_error:+.5f} rev | "
                    f"error_deg={position_error * 360:+.2f}° | "
                    f"gravity_ff={gravity_ff:+.5f} Nm | "
                    f"torque={measured_torque:+.5f} Nm"
                )

                last_print = now

            await asyncio.sleep(CONTROL_PERIOD)

    finally:
        await c.set_stop()


if __name__ == "__main__":
    asyncio.run(main())