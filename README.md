# GL80 Gripper & Gravity Compensation

## Test Scripts

- **`test_haptics.py`** — Tests vibration-based haptic feedback using motor position error.
- **`testgl_80.py`** — Tests the CubeMars GL80 motor with a moteus r4.11 controller, configured as a temporary gripper for testing TSHOR haptics.
- **`test_gravity_comp.py`** — Implements gravity compensation using feedforward torque to improve position control.

## Gravity Compensation

Gravity compensation applies an additional feedforward torque based on the joint angle:


tau_g = m*g*L*sin(theta)


**Main parameters to configure:**

- **Mass (`m`)** — Mass of the supported load (kg).
- **COM distance (`L`)** — Distance from the joint axis to the load's center of mass (m).
- **Angle offset** — Aligns the measured joint angle with the gravity model.
- **Gravity torque sign** — Sets the direction of the compensating torque.
- **Target position** — Desired motor position (revolutions).

Adjust these parameters according to the mechanical setup and load.

## Tuned PID Gains

The following gains provided good results with gravity compensation enabled:

```text
servo.pid_position.kp = 10.0
servo.pid_position.ki = 0.0
servo.pid_position.kd = 0.1
servo.pid_position.iratelimit = -1.0
servo.pid_position.ilimit = 0.0
servo.pid_position.max_desired_rate = 0.0
servo.pid_position.sign = -1
```

## Open moteus tview

```bash
python3 -m moteus_gui.tview
```