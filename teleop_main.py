import asyncio
import math
import struct
import time

import moteus
import serial

# ---------------------------------------------------------------
# Serial (single Teensy)
# ---------------------------------------------------------------
SERIAL_PORT = "/dev/serial/by-id/usb-Teensyduino_Dual_Serial_20440910-if00"   # <-- check: gripper script used ACM1, roll used ACM2
BAUD_RATE = 115200
SERIAL_TIMEOUT = 0.1           # s without a packet -> stop commanding

PACKET_SIZE = 6

# ---------------------------------------------------------------
# Loop
# ---------------------------------------------------------------
CONTROL_PERIOD = 1 / 400           # 400 Hz
WATCHDOG_TIMEOUT = 0.1
PRINT_PERIOD = 0.1

# ---------------------------------------------------------------
# Gripper (moteus id 1)
# ---------------------------------------------------------------
GRIPPER_ID = 1

RAW_MIN = 0
RAW_MAX = 10000

POS_MIN = 0.03
POS_MAX = 0.199

MASS_KG = 0.08
COM_DISTANCE_M = 0.087  # Joint axis to the finger's center of mass.
GRAVITY = 9.81
GRAVITY_ZERO = 0.0  # rev
GRAVITY_SIGN = 1.0

# MAX_TORQUE = 0.1  # Nm
# MAX_VELOCITY = 0.75  # rev/s
# MAX_ACCEL = 0.20  # rev/s²

# Safety limits (tuned limits!)
MAX_TORQUE = 0.1       # Nm
MAX_VELOCITY = 2.00    # rev/s
MAX_ACCEL = 0.20

FREE_TORQUE_MIN = -0.045
FREE_TORQUE_MAX = 0.125

# ---------------------------------------------------------------
# Roll motor (moteus id 3)
# ---------------------------------------------------------------
ROLL_ID = 3
 
ROLL_POS_MIN = -0.26        # rev   <-- SET YOUR BOUNDS
ROLL_POS_MAX = 0.25         # rev   <-- SET YOUR BOUNDS
ROLL_LIMIT_MARGIN = 0.03    # rev of overshoot tolerated before e-stop
 
ROLL_MAX_RATE = 0.5         # rev/s at full stick
ROLL_VEL_TRIP_MARGIN = 0.25 # rev/s above ROLL_MAX_RATE before e-stop
ROLL_VEL_TRIP_TIME = 0.03   # s the overspeed must last (ignores 1-sample noise spikes)
ROLL_MAX_TORQUE = 0.5       # Nm
ROLL_ACCEL_LIMIT = 4.0      # rev/s^2
# ---------------------------------------------------------------
# Joystick X needs tuning!
# ---------------------------------------------------------------
JOY_MIN = 225                 # raw at full left  (Teensy 10-bit ADC)
JOY_MAX = 842                # raw at full right

JOY_CENTER_SAMPLES = 50     # packets averaged to find centre (~0.1 s)
JOY_SIGN = -1.0              # flip to -1.0 if direction is reversed

JOY_CENTER = 528            # middle of observed rest band (485..571)
JOY_DEADBAND_COUNTS = 55    # half the band (43) + margin

# ---------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------
def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def gravity_torque(position):
    angle = (position - GRAVITY_ZERO) * 2.0 * math.pi
    return (
        GRAVITY_SIGN
        * MASS_KG
        * COM_DISTANCE_M
        * GRAVITY
        * math.sin(angle)
    )


def map_position(raw):
    raw = clamp(raw, RAW_MIN, RAW_MAX)
    position = POS_MIN + (
        (raw - RAW_MIN) / (RAW_MAX - RAW_MIN)
    ) * (POS_MAX - POS_MIN)
    return clamp(position, POS_MIN, POS_MAX)


def joystick_to_rate(raw, center):
    """Map raw joystick X to a signed roll rate in rev/s."""
    half_range = (JOY_MAX - JOY_MIN) / 2.0
    deviation = raw - center

    if abs(deviation) <= JOY_DEADBAND_COUNTS:
        return 0.0

    norm = clamp(deviation / half_range, -1.0, 1.0)
    deadband = JOY_DEADBAND_COUNTS / half_range

    # Rescale so output starts at 0 just outside the deadband
    scaled = max(abs(norm) - deadband, 0.0) / (1.0 - deadband)
    return JOY_SIGN * math.copysign(scaled, norm) * ROLL_MAX_RATE


def parse_packets(rx):
    """
    Consume complete packets from rx.
    Returns (latest_trigger, latest_joy_x); either may be None.
    Trigger values > RAW_MAX are discarded (as in the gripper script),
    but the joystick X from that packet is still used.
    """
    latest_trigger = None
    latest_joy_x = None

    while len(rx) >= PACKET_SIZE:
        if rx[0] != ord("T"):
            del rx[0]
            continue

        checksum = 0
        for b in rx[1:PACKET_SIZE - 1]:
            checksum ^= b
        if checksum != rx[PACKET_SIZE - 1]:
            del rx[0]          # misaligned/corrupt: resync on next 'T'
            continue

        trigger, joy_x = struct.unpack_from("<HH", rx, 1)
        del rx[:PACKET_SIZE]

        if trigger <= RAW_MAX:
            latest_trigger = trigger
        latest_joy_x = joy_x

    return latest_trigger, latest_joy_x


# ---------------------------------------------------------------
# Startup checks
# ---------------------------------------------------------------
async def check_gripper(c):
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

    return position


async def check_roll(c):
    state = await c.query()
    if state is None:
        raise RuntimeError("No response from roll motor (id 3)")

    position = state.values[moteus.Register.POSITION]
    velocity = state.values[moteus.Register.VELOCITY]

    if not all(math.isfinite(x) for x in (position, velocity)):
        raise RuntimeError("Invalid initial feedback")
    if not ROLL_POS_MIN <= position <= ROLL_POS_MAX:
        raise RuntimeError(
            f"Initial position {position:+.4f} outside "
            f"[{ROLL_POS_MIN}, {ROLL_POS_MAX}]"
        )
    if abs(velocity) > 0.05:
        raise RuntimeError("Roll motor moving at startup")

    return position


# ---------------------------------------------------------------
# Main
# ---------------------------------------------------------------
async def main():
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0)
    gripper = moteus.Controller(id=GRIPPER_ID)
    roll = moteus.Controller(id=ROLL_ID)

    try:
        await gripper.set_stop()
        await roll.set_stop()

        gripper_position = await check_gripper(gripper)
        roll_position = await check_roll(roll)

        # Roll: hold wherever it currently is - no jump on start
        roll_target = roll_position

        rx = bytearray()

        # Gripper state
        latest_position = None
        gripper_last_packet_time = 0.0
        gripper_last_print = 0.0

        # Roll state
        joy_x = None
        joy_center = JOY_CENTER
        center_samples = []
        roll_last_packet_time = 0.0
        roll_last_print = 0.0
        roll_overspeed_since = None
        last_loop = time.monotonic()

        while True:
            now = time.monotonic()
            dt = now - last_loop
            last_loop = now

            if ser.in_waiting:
                rx.extend(ser.read(ser.in_waiting))

            new_trigger, new_joy_x = parse_packets(rx)

            if new_trigger is not None:
                latest_position = map_position(new_trigger)
                gripper_last_packet_time = time.monotonic()

            if new_joy_x is not None:
                joy_x = new_joy_x
                roll_last_packet_time = now

                if joy_center is None:
                    center_samples.append(joy_x)
                    if len(center_samples) >= JOY_CENTER_SAMPLES:
                        joy_center = sum(center_samples) / len(center_samples)
                        print(
                            f"Joystick X centre captured: {joy_center:.1f} "
                            f"(min {min(center_samples)}, max {max(center_samples)})"
                        )

            # =======================================================
            # Gripper (id 1)
            # =======================================================
            if (
                latest_position is not None
                and time.monotonic() - gripper_last_packet_time < SERIAL_TIMEOUT
            ):
                gravity_ff = clamp(
                    gravity_torque(gripper_position), -MAX_TORQUE, MAX_TORQUE
                )
                result = await gripper.set_position(
                    position=latest_position,
                    velocity=0.0,
                    feedforward_torque=0.5 * gravity_ff,
                    # maximum_torque=MAX_TORQUE,
                    # velocity_limit=MAX_VELOCITY,
                    # accel_limit=MAX_ACCEL,
                    maximum_torque=3.0,  # 10.0 worked fine but be careful
                    velocity_limit=12.0,
                    accel_limit=15.0,
                    watchdog_timeout=WATCHDOG_TIMEOUT,
                    query=True,
                )

                if result is None:
                    raise RuntimeError("No moteus response")

                gripper_position = result.values[moteus.Register.POSITION]
                gripper_velocity = result.values[moteus.Register.VELOCITY]
                measured_torque = result.values[moteus.Register.TORQUE]

                if not all(math.isfinite(x) for x in (
                    gripper_position, gripper_velocity, measured_torque
                )):
                    raise RuntimeError("Invalid feedback")

                position_error = latest_position - gripper_position

                torque_residual = measured_torque - 0.5 * gravity_ff

                if not POS_MIN <= gripper_position <= POS_MAX + 0.02:
                    raise RuntimeError("Position limit exceeded")
                if abs(gripper_velocity) > MAX_VELOCITY + 0.02:
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
                    torque_residual,  # torque residual instead of measured torque
                )

                ser.write(feedback_packet)

                t = time.monotonic()
                if t - gripper_last_print >= PRINT_PERIOD:
                    print(
                        f"[GRIP] cmd={latest_position:+.4f} rev | "
                        f"actual={gripper_position:+.4f} rev | "
                        f"error={position_error:+.5f} rev | "
                        f"error_deg={position_error * 360:+.2f}° | "
                        f"gravity_ff={gravity_ff:+.5f} Nm | "
                        f"torque={torque_residual:+.5f} Nm"
                    )
                    gripper_last_print = t

            # =======================================================
            # Roll (id 3)
            # =======================================================
            roll_link_ok = (
                joy_x is not None
                and joy_center is not None
                and now - roll_last_packet_time < SERIAL_TIMEOUT
            )

            if not roll_link_ok:
                # No fresh input: don't command; watchdog will stop the motor.
                # Re-sync target so it doesn't jump when the link returns.
                roll_target = clamp(roll_position, ROLL_POS_MIN, ROLL_POS_MAX)
                roll_overspeed_since = None
            else:
                rate = joystick_to_rate(joy_x, joy_center)
                roll_target += rate * dt

                if roll_target >= ROLL_POS_MAX:
                    roll_target = ROLL_POS_MAX
                    rate = min(rate, 0.0)
                elif roll_target <= ROLL_POS_MIN:
                    roll_target = ROLL_POS_MIN
                    rate = max(rate, 0.0)

                result = await roll.set_position(
                    position=roll_target,
                    velocity=rate,              # feedforward; 0 when stick centred
                    maximum_torque=ROLL_MAX_TORQUE,
                    velocity_limit=ROLL_MAX_RATE,
                    accel_limit=ROLL_ACCEL_LIMIT,
                    watchdog_timeout=WATCHDOG_TIMEOUT,
                    query=True,
                )

                if result is None:
                    raise RuntimeError("No response from roll motor (id 3)")

                roll_position = result.values[moteus.Register.POSITION]
                roll_velocity = result.values[moteus.Register.VELOCITY]
                roll_torque = result.values[moteus.Register.TORQUE]

                if not all(math.isfinite(x) for x in (
                    roll_position, roll_velocity, roll_torque
                )):
                    raise RuntimeError("Invalid feedback")
                if not (
                    ROLL_POS_MIN - ROLL_LIMIT_MARGIN
                    <= roll_position
                    <= ROLL_POS_MAX + ROLL_LIMIT_MARGIN
                ):
                    raise RuntimeError("Roll position limit exceeded")
                if abs(roll_velocity) > ROLL_MAX_RATE + ROLL_VEL_TRIP_MARGIN:
                    if roll_overspeed_since is None:
                        roll_overspeed_since = now
                    elif now - roll_overspeed_since >= ROLL_VEL_TRIP_TIME:
                        raise RuntimeError(
                            f"Roll velocity limit exceeded: {roll_velocity:+.3f} rev/s"
                        )
                else:
                    roll_overspeed_since = None

                if now - roll_last_print >= PRINT_PERIOD:
                    print(
                        f"[ROLL] joyX={joy_x:5d} | rate={rate:+.3f} rev/s | "
                        f"target={roll_target:+.4f} | actual={roll_position:+.4f} rev | "
                        f"err={(roll_target - roll_position) * 360:+.2f}° | "
                        f"torque={roll_torque:+.4f} Nm"
                    )
                    roll_last_print = now

            await asyncio.sleep(CONTROL_PERIOD)

    finally:
        try:
            await gripper.set_stop()
        finally:
            try:
                await roll.set_stop()
            finally:
                ser.close()


if __name__ == "__main__":
    asyncio.run(main())