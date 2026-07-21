# Race-Proven Firmware Notes (vehicle-tested)

This branch mirrors the `Desktop/user` firmware that completed a full track run on the physical car.

## Verified on hardware

- **Platform**: TC264 (Seekfree-style project), MT9V03X 188×120 camera, IPS200 menu display
- **Motor driver**: 4-PWM H-bridge (`IN1` = duty, `IN2` = 0), not PWM+DIR on P02.x
- **Completion**: Full lap / race finish achieved with this build
- **Startup**: Motors run immediately on power-up (no Arm/Disarm step)

## Architecture (what runs)

```
camera frame → eight-neighbor track + cross fill → weighted midline error
             → PD steering + duty caps → motor_apply
```

Removed vs earlier repo: element FSM (cross/ring/ramp states), display chirps, perf profiler, Arm gate, `DEBUG_NO_DRIVE`.

## Speed scheduling (duty scale 0–10000 = 0–100%)

| Parameter | Value | Effect |
|-----------|-------|--------|
| `STRAIGHT_DUTY` | 2000 | Target cruise **20%** on straight |
| `DUTY_HARD_CAP` | 6000 | Absolute max **60%** |
| Row / curvature / steer LUTs | see `config.h` | Caps exist but **do not reduce below 2000** while `STRAIGHT_DUTY=2000` |
| `DUTY_SLEW_UP/DOWN` | 10000 | Effectively instant duty tracking |

## Steering

- Adaptive Kp: `Kp Min` / `Kp Max` / `Kp E Sat` (menu)
- `Kd` + `D Filt Alpha` (menu)
- `SERVO_DIR = -1`, center 770, range 685–850
- `STEER_W_BOTH_LOST_PCT = 40` (keep steer weight when both edges lost on curves)

## Image

- Otsu threshold when menu `Threshold = 0`
- Eight-neighbor bilateral track
- **Cross fill retained** (line extension only; no FSM cross mode)

## Failsafe

- `valid_rows < 8` or both-lost ≥ 70% for 10 frames → motor stop, auto-resume when track valid
- Severe (≥90% both-lost or zero rows) for 2 frames → fast stop
- Brief stops on bumps/curves possible (failsafe flicker)

## Menu items

Kp Min, Kp Max, Kp E Sat, Kd, D Filt Alpha, Threshold, Save, Load, Restore Def

## Pin map (`pins.h`) — must match wiring

| Function | Pin / channel |
|----------|----------------|
| Servo PWM | ATOM1_CH1_P33_9 |
| Left motor IN1 / IN2 | ATOM0_CH2_P21_4 / ATOM0_CH3_P21_5 |
| Right motor IN1 / IN2 | ATOM0_CH1_P21_3 / ATOM0_CH0_P21_2 |
| Buzzer | P33_10 |

## ADS project cleanup

Remove from build if still listed: `fsm.c`, `display.c`, `perf.c`.

## Known non-issues on this branch

- Workspace `test_CarRun` main branch used **PWM+DIR on P02.x** and `DEBUG_NO_DRIVE=1` / `armed=0` — motors would not move on this car even with correct logic.
