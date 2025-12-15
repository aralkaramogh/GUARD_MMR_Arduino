GUARD_MMR_ARDUINO — Quick Reference

Pinout

| Motor | Side        | PWM pin | DIR pin | Code constant |
|-------|-------------|---------|---------|---------------|
| M1    | Right Front | 3       | 4       | MOTOR_PWM_1 / MOTOR_DIR_1 |
| M2    | Left Front  | 5       | 7       | MOTOR_PWM_2 / MOTOR_DIR_2 |
| M3    | Right Back  | 6       | 8       | MOTOR_PWM_3 / MOTOR_DIR_3 |
| M4    | Left Back   | 9       | 12      | MOTOR_PWM_4 / MOTOR_DIR_4 |

Operating Keys (Serial)

| Key         | Action / Notes |
|-------------|----------------|
| W / w / F   | Move forward at `forwardBackwardSpeed`%. Press once to start; `X`/space to stop.
| S / s / B   | Move backward at `forwardBackwardSpeed`%. Press once to start; `X`/space to stop.
| A / a / L   | Turn left at `turningSpeed`%.
| D / d / R   | Turn right at `turningSpeed`%.
| X / (space) | Stop all motors immediately.
| Q / q       | Increase forward/back speed by +10 (clamped to `FORWARD_BACKWARD_CAP`).
| Z / z       | Decrease forward/back speed by -10 (min 10).
| E / e       | Increase turning speed by +10 (clamped to `TURNING_CAP`).
| C / c       | Decrease turning speed by -10 (min 10).
| H / h       | Reset: stop + set `forwardBackwardSpeed = 10`, `turningSpeed = 10`.

Recommended presets (edit in `src/main_v2.cpp`)

- `forwardBackwardSpeed` = 10  (startup user speed)
- `turningSpeed` = 10          (startup turning speed)
- `FORWARD_BACKWARD_CAP` = 30–40 (non-serial max for linear motion; safety limit)
- `TURNING_CAP` = 100          (non-serial max for turning)
- `MOTOR_INVERT[]` = {0,1,0,1} (set 1 for motors that must be inverted; adjust if a motor runs opposite)

Notes

- Change caps only in code when you want a permanent safety limit.
- After editing presets, upload the firmware and test on the bench before deployment.
- If a motor spins opposite of expected, flip its value in `MOTOR_INVERT`.
