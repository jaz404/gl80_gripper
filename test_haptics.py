import asyncio
import moteus
import serial


SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200


POS_MIN = -0.02
POS_MAX = 0.18

CENTER_POSITION = (POS_MIN + POS_MAX) / 2.0

CONTROL_PERIOD = 0.01  # 100 Hz


async def main():
    ser = serial.Serial(
        SERIAL_PORT,
        BAUD_RATE,
        timeout=0
    )

    c = moteus.Controller(id=1)

    await c.set_stop()

    print(f"Moving to center position: {CENTER_POSITION:.4f} rev")

    try:
        while True:
            result = await c.set_position(
                position=POS_MAX,
                velocity=0.0,
                maximum_torque=0.3,
                velocity_limit=1.0,
                accel_limit=0.5,
                watchdog_timeout=0.3,
                query=True,
            )

            position = result.values[moteus.Register.POSITION]
            torque = result.values[moteus.Register.TORQUE]

            position_error = POS_MAX - position

            print(
                f"position={position:+.4f}  "
                f"error={position_error:+.4f}  "
                f"torque={torque:+.4f} Nm"
            )
            packet = f"H,{torque:.4f},{position_error:.5f}\n"
            ser.write(packet.encode())
            await asyncio.sleep(CONTROL_PERIOD)

    finally:
        await c.set_stop()


asyncio.run(main())