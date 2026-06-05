# manus_inspire — Architecture

Source: `src/core/manus_inspire/manus_inspire/manus_inspire.py`

## Data flow

```
/manus_glove_0, /manus_glove_1  (manus_ros2_msgs/ManusGlove)
    → manus_inspire
        → /rt/inspire_hand/ctrl/l
        → /rt/inspire_hand/ctrl/r
```

Hand side determined by `msg.side` ('Left'/'Right'), **not by topic number**. Both topics use the same callback.

## Finger flex mapping

```
flex = 0.25*MCP + 0.55*PIP + 0.20*DIP   # fingers (PIP dominates)
thumb_flex = MCP only                     # PIP/DIP contaminated by finger motion
spread = ThumbMCPSpread
```

Flex-to-Inspire range: fingers are **inverted** (closed=max inspire value), thumb is not. Output range: `[0, MAX_INSPIRE=1000]`.

`InspireHandCtrl.mode = 0b1101` (angle + force + speed). Forces=800, speeds=1000 — hardcoded.

## Calibration

4 phases × 4s each (`CALIB_DURATION = 4.0`). Saves to `~/.ros/manus_inspire_calib.yaml`. Auto-starts if file missing.

| Phase | Pose | Captures |
|---|---|---|
| 0 | Open hands | min flex, min spread |
| 1 | Thumbs up (fist) | max finger flex, max thumb MCPStretch |
| 2 | Thumb to index side | max spread |
| 3 | Open fingers, bend thumb | min thumb MCPStretch |

Default ranges (fallback if no calib): index/middle/ring 0–75°, pinky 0–65°, thumb 0–55°, spread −30–+30°.

Service: `~/calibrate` (Trigger) — triggers recalibration.

## Mirror mode

Subscribes to `/teleop/mirror_mode` (String: `'mirror'`/`'normal'`). In mirror mode: left glove → right hand ctrl, right glove → left hand ctrl.

## Gotchas

- `CALIB_DURATION = 4.0` is hardcoded here **and** in `scm_gui_node.py` — change both together.
- Calibration auto-triggers on startup if no YAML file found — can block first launch.
- `msg.side` string is the authoritative hand selector, not the topic. If SDK sends wrong side string, mapping breaks silently.
