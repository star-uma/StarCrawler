# La v1: Arduino MKR + control desde el PC por UDP

Referencia de la arquitectura anterior, la del robot de 2025: un Arduino MKR
WiFi 1010 con shield CAN para la tracción y un script en el PC que mandaba las
consignas por UDP. Ya no es el camino del proyecto, pero el firmware sigue en
`firmware/starcrawler_mkr_traccion/` y el MKR se usa todavía como sonda de
banco para el bus CAN.

## Control por PC

```powershell
cd control
python StarCrawlerXbox.py --test
```

Manda datagramas UDP al firmware de la v1.

## Protocolo UDP

| Index | Field | Description |
|---|---|---|
| 0 | `id` | Always 1 (PC origin) |
| 1 | `action_left_train` | Left traction speed (dps × 100) |
| 2 | `action_right_train` | Right traction speed (dps × 100) |
| 3–6 | `o0..o3` | Crawler target angles / directions (modes 2–4) |
| 7 | `mode` | Active control mode (1..5) |
| 8 | `code_error` | 0 = no error |

## Modos de control

| Mode | Name | Description |
|---|---|---|
| 1 | Traction | Differential drive — left stick (speed) + right stick (turn) |
| 2 | Absolute position | Set all crawlers to a fixed angle via A/B/X/Y buttons |
| 3 | Incremental × 4 | Tilt all crawlers together via D-pad |
| 4 | Incremental × 2 | Control crawler pairs independently via D-pad + triggers |
| 5 | Auto-levelling | IMU-based automatic horizontal levelling (hold Start) |

## Mando

| Input | Action |
|---|---|
| Left stick (vertical) | Forward / backward |
| Right stick (horizontal) | Turn |
| RB | Cycle mode: 1 → 2 → 3 → 4 → 5 → 1 |
| A / B / X / Y | Absolute position (Mode 2): 225° / 180° / 135° / 90° |
| D-pad + triggers | Crawler control (Modes 3 / 4) |
| Start (hold) | Auto-levelling (Mode 5) |

## Seguridad

The Arduino firmware includes a **500 ms watchdog**: if no UDP packet is
received within that window (connection lost, script stopped), all traction
motors are released immediately.
