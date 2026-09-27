import asyncio
import moteus
import serial
import time
import struct




SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

RAW_MIN = 0
RAW_MAX = 10000

POS_MIN = -0.02
POS_MAX = 0.18

# SERIAL_TIMEOUT = 0.1      # stop motor if no valid packet for 100 ms
# CONTROL_PERIOD = 0.002    # 500 Hz
# CONTROL_PERIOD = 0.1   # 200 Hz

CONTROL_PERIOD = 0.01   # 100 Hz
SERIAL_TIMEOUT = 0.1

def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def map_position(raw):
    raw = clamp(raw, RAW_MIN, RAW_MAX)

    position = POS_MIN + (
        (raw - RAW_MIN) / (RAW_MAX - RAW_MIN)
    ) * (POS_MAX - POS_MIN)

    # Hard safety clamp
    return clamp(position, POS_MIN, POS_MAX)


async def main():
    ser = serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=0
    )

    c = moteus.Controller(id=1)

    await c.set_stop()

    rx = bytearray()
    latest_position = None
    last_packet_time = 0.0

    try:
        while True:

            # Read everything currently available
            if ser.in_waiting:
                rx.extend(ser.read(ser.in_waiting))

            # Parse:
            # 'T' + uint16 little endian
            while len(rx) >= 3:

                # Find packet start
                if rx[0] != ord('T'):
                    del rx[0]
                    continue

                raw = rx[1] | (rx[2] << 8)
                del rx[:3]

                if raw > 10000:
                    continue

                latest_position = map_position(raw)
                last_packet_time = time.monotonic()

                print(
                    f"raw={raw:5d}  "
                    f"position={latest_position:+.4f}"
                )

            # Only command motor if serial data is fresh
            if (
                latest_position is not None
                and time.monotonic() - last_packet_time < SERIAL_TIMEOUT
            ):
                result = await c.set_position(
                    position=latest_position,
                    velocity=0,
                    maximum_torque=0.3,
                    velocity_limit=12.0,
                    accel_limit=15.0,
                    # maximum_torque=0.1,
                    # velocity_limit=5.0,
                    # accel_limit=5.0,
                    watchdog_timeout=0.3,
                    query=True, 
                )
                measured_position = result.values[moteus.Register.POSITION]
                measured_torque = result.values[moteus.Register.TORQUE]

                position_error = latest_position - measured_position

                feedback_packet = struct.pack(
                    "<cff",
                    b"G",
                    position_error,
                    measured_torque,
                )

                ser.write(feedback_packet)

            await asyncio.sleep(CONTROL_PERIOD)

    finally:
        await c.set_stop()
        ser.close()


asyncio.run(main())