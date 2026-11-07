import asyncio
import moteus
import serial
import time
import struct

import matplotlib.pyplot as plt
from collections import deque

SERIAL_PORT = "/dev/ttyACM1"
BAUD_RATE = 115200

RAW_MIN = 0
RAW_MAX = 10000

POS_MIN = 0.03
POS_MAX = 0.195

CONTROL_PERIOD = 0.01   # 10 ms sleep between iterations
SERIAL_TIMEOUT = 0.1

FREE_TORQUE_MIN = -0.045
FREE_TORQUE_MAX = 0.125


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
    await c.set_stop()

    rx = bytearray()
    latest_position = None
    last_packet_time = 0.0

    # Live plot setup
    plt.ion()
    fig, ax = plt.subplots(figsize=(10, 5))
    fig.subplots_adjust(left=0.12, bottom=0.15)
    fig.patch.set_facecolor("#f5f7fa")
    ax.set_facecolor("#ffffff")

    line, = ax.plot([], [], color="#0089a7", linewidth=1.8)
    ax.axhline(0, color="gray", linestyle="--", linewidth=0.8)
    ax.set_title("Gripper Position Error", fontsize=15, fontweight="bold")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Position error (revolutions)")
    ax.grid(True, alpha=0.25)
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)

    times = deque()
    errors = deque()
    plot_start = time.monotonic()
    last_plot_time = 0.0
    PLOT_PERIOD = 0.05       # Refresh graph at approximately 20 Hz
    WINDOW_SECONDS = 10.0
    plt.show(block=False)

    try:
        while True:
            # Read everything currently available
            if ser.in_waiting:
                rx.extend(ser.read(ser.in_waiting))

            # Parse: 'T' + uint16 little endian
            while len(rx) >= 3:
                if rx[0] != ord('T'):
                    del rx[0]
                    continue

                raw = rx[1] | (rx[2] << 8)
                del rx[:3]
                if raw > 10000:
                    continue

                latest_position = map_position(raw)
                last_packet_time = time.monotonic()

            # Only command motor if serial data is fresh
            if (
                latest_position is not None
                and time.monotonic() - last_packet_time < SERIAL_TIMEOUT
            ):
                result = await c.set_position(
                    position=latest_position,
                    velocity=0,
                    maximum_torque=3.0,
                    velocity_limit=12.0,
                    accel_limit=15.0,
                    watchdog_timeout=0.3,
                    query=True,
                )
                measured_position = result.values[moteus.Register.POSITION]
                measured_torque = result.values[moteus.Register.TORQUE]
                position_error = latest_position - measured_position

                # Store the newly calculated error for plotting
                elapsed = time.monotonic() - plot_start
                times.append(elapsed)
                errors.append(position_error)

                if FREE_TORQUE_MIN <= measured_torque <= FREE_TORQUE_MAX:
                    external_torque = 0.0
                elif measured_torque > FREE_TORQUE_MAX:
                    external_torque = measured_torque - FREE_TORQUE_MAX
                else:
                    external_torque = measured_torque - FREE_TORQUE_MIN

                feedback_packet = struct.pack(
                    "<cff", b"G", position_error, external_torque,
                )
                ser.write(feedback_packet)

            # Refresh independently of whether serial data is available
            now = time.monotonic()
            if now - last_plot_time >= PLOT_PERIOD:
                if not plt.fignum_exists(fig.number):
                    break  # Closing the graph stops the loop

                elapsed = now - plot_start
                while times and times[0] < elapsed - WINDOW_SECONDS:
                    times.popleft()
                    errors.popleft()

                line.set_data(list(times), list(errors))
                ax.set_xlim(
                    max(0.0, elapsed - WINDOW_SECONDS),
                    max(WINDOW_SECONDS, elapsed),
                )

                # Automatically scale Y-axis with 20% headroom
                peak_error = max((abs(e) for e in errors), default=0.0)
                y_limit = max(0.001, peak_error * 1.2)
                ax.set_ylim(-y_limit, y_limit)

                fig.canvas.draw_idle()
                plt.pause(0.001)
                last_plot_time = now

            await asyncio.sleep(CONTROL_PERIOD)

    finally:
        try:
            await c.set_stop()
        finally:
            ser.close()
            plt.close(fig)


if __name__ == "__main__":
    asyncio.run(main())
