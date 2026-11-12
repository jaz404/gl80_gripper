import asyncio
import math
import struct
import time

import moteus
import serial

SERIAL_PORT = "/dev/ttyACM1"
BAUD_RATE = 115200

RAW_MIN = 0
RAW_MAX = 10000

POS_MIN = 0.03
POS_MAX = 0.199
SERIAL_TIMEOUT = 0.1

MASS_KG = 0.08
COM_DISTANCE_M = 0.087  # Joint axis to the finger's center of mass.
GRAVITY = 9.81
GRAVITY_ZERO = 0.0  # rev
GRAVITY_SIGN = 1.0

# MAX_TORQUE = 0.1  # Nm
# MAX_VELOCITY = 0.75  # rev/s
# MAX_ACCEL = 0.20  # rev/s²
# CONTROL_PERIOD = 0.01
# WATCHDOG_TIMEOUT = 0.1
# PRINT_PERIOD = 0.1

# Safety limits (tuned limits!)
MAX_TORQUE = 0.1       # Nm
MAX_VELOCITY = 2.00  # rev/s
MAX_ACCEL = 0.20 
CONTROL_PERIOD = 0.01   # 100 Hz
WATCHDOG_TIMEOUT = 0.1
PRINT_PERIOD = 0.1


FREE_TORQUE_MIN = -0.045
FREE_TORQUE_MAX = 0.125


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


def map_position(raw):
    raw = clamp(raw, RAW_MIN, RAW_MAX)
    position = POS_MIN + (
        (raw - RAW_MIN) / (RAW_MAX - RAW_MIN)
    ) * (POS_MAX - POS_MIN)
    return clamp(position, POS_MIN, POS_MAX)


async def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
    c = moteus.Controller(id=1)

    try:
        await c.set_stop()
        state = await c.query()
        if state is None:
            raise RuntimeError("No moteus response")

        position = state.values[moteus.Register.POSITION]
        velocity = state.values[moteus.Register.VELOCITY]

        if not all(math.isfinite(x) for x in (position, velocity)):
            raise RuntimeError("Invalid initial feedback")
        if not POS_MIN <= position <= POS_MAX:
            raise RuntimeError("Initial position outside bounds")
        if abs(velocity) > 0.05:
            raise RuntimeError("Motor moving at startup")

        rx = bytearray()
        latest_position = None
        last_packet_time = 0.0
        last_print = 0.0

        while True:
            if ser.in_waiting:
                rx.extend(ser.read(ser.in_waiting))

            # 'T' + uint16 little endian
            while len(rx) >= 3:
                if rx[0] != ord("T"):
                    del rx[0]
                    continue

                raw = rx[1] | (rx[2] << 8)
                del rx[:3]

                if raw > RAW_MAX:
                    continue

                latest_position = map_position(raw)
                last_packet_time = time.monotonic()

            if (
                latest_position is not None
                and time.monotonic() - last_packet_time < SERIAL_TIMEOUT
            ):
                gravity_ff = clamp(
                    gravity_torque(position), -MAX_TORQUE, MAX_TORQUE
                )
                result = await c.set_position(
                    position=latest_position,
                    velocity=0.0,
                    feedforward_torque=0.5 * gravity_ff,
                    # maximum_torque=MAX_TORQUE,
                    # velocity_limit=MAX_VELOCITY,
                    # accel_limit=MAX_ACCEL,
                    maximum_torque=3.0, # 10.0 worked fine but be careful
                    velocity_limit=12.0,
                    accel_limit=15.0,
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

                position_error = latest_position - position

                torque_residual = measured_torque - 0.5 * gravity_ff

                if not POS_MIN <= position <= POS_MAX + 0.02:
                    raise RuntimeError("Position limit exceeded")
                if abs(velocity) > MAX_VELOCITY + 0.02:
                    raise RuntimeError("Velocity limit exceeded")

                # if FREE_TORQUE_MIN <= measured_torque <= FREE_TORQUE_MAX:
                #     external_torque = 0.0
                # elif measured_torque > FREE_TORQUE_MAX:
                #     external_torque = measured_torque - FREE_TORQUE_MAX
                # else:
                #     external_torque = measured_torque - FREE_TORQUE_MIN

                feedback_packet = struct.pack(
                    "<cff",
                    b"G",
                    position_error,
                    torque_residual, # added torque residual instead of the measured torque
                )

                ser.write(feedback_packet)

                now = time.monotonic()
                if now - last_print >= PRINT_PERIOD:
                    print(
                        f"cmd={latest_position:+.4f} rev | "
                        f"actual={position:+.4f} rev | "
                        f"error={position_error:+.5f} rev | "
                        f"error_deg={position_error * 360:+.2f}° | "
                        f"gravity_ff={gravity_ff:+.5f} Nm | "
                        f"torque={torque_residual:+.5f} Nm"
                    )
                    last_print = now

            await asyncio.sleep(CONTROL_PERIOD)

    finally:
        try:
            await c.set_stop()
        finally:
            ser.close()


if __name__ == "__main__":
    asyncio.run(main())