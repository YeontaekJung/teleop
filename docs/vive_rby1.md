# vive_rby1_node — Architecture

Source: `src/core/vive_rby1/src/vive_rby1_node.cpp` (~1100 lines). Production node only — `vive_rby1_debug_node` (Python) is dev-only, not in default launch.

## Coordinate frames

- **`v2r_R_`** = `[[0,1,0],[-1,0,0],[0,0,1]]` — -90° Z rotation, Vive world → robot frame
- Tracker delta: `v2r_R_ * (tracker.t - ref.t)`
- Rotation delta: `dR_robot = v2r_R_ * (tracker.R * ref.R.T) * v2r_R_.T`

## Engage / disengage

**Engage** (pedal A falling edge) captures:
- Tracker poses → `ref_l_`, `ref_r_`, `ref_body_`
- Pinocchio FK → `ee_l_0_`, `ee_r_0_`, `torso5_0_`
- SDK FK with Z-offset compensation → `sdk_ee_l_0_` (+3.9 cm), `sdk_ee_r_0_` (+2.6 cm)

Commands built as `ee_*_0_ + delta` — relative to robot pose at engage, not absolute.

**Disengage**: all captures cleared, `engaged_=false`.

Special cases:
- Body tracker arrives while engaged → auto-captures `ref_body_` + `torso5_0_`
- Mirror mode toggled while engaged → recaptures `ref_l_`, `ref_r_`, `ee_l_0_`, `ee_r_0_` (prevents discontinuity)

## Mirror mode

`mirror_flip = diag(1, -1, 1)` — Y-axis negation for facing-each-other operation.

```
# position
target = ee_0_.t + pos_scale * (mirror_flip * v2r_R_ * delta)

# rotation
dR_robot = v2r_R_ * dR * v2r_R_.T
dR_mirrored = mirror_flip * dR_robot * mirror_flip
```

Left/right EEs are also **swapped**: left tracker → right EE, right tracker → left EE.

## Per-frame step clamp

Applied to every command before publish:
- Position: if `‖delta‖ > sdk_max_delta_pos` (default 0.03 m), clamp along direction
- Rotation: if angle exceeds `sdk_max_delta_rot_deg`, slerp to limit

## Warmup hold

On stream start: publishes current `/rby1/state/ee_pose` for `publish_rate` ticks (1s at 100 Hz) before sending real targets. Prevents snap-to-target on first frame.

## Body tracker → torso

- Sub: `/teleop/tracker/body`, smoothed with α=`tracker_smooth_alpha` (default 0.9)
- Published as third `/rby1/cmd/pose` entry: `child_frame_id="link_torso_5"`
- Only active when `use_torso_=true` (GUI checkbox → `/vive_rby1/set_use_torso` or `/teleop/use_torso`)
- hw-core ignores `link_torso_5` in `CartesianPosition`; only consumed in `CartesianImpedance`

## Key parameters

| Param | Default | Notes |
|---|---|---|
| `pos_scale` | 0.5 | Hand tracker position scale |
| `torso_pos_scale` | 1.0 | Body tracker position scale |
| `sdk_max_delta_pos` | 0.03 m | Per-frame step clamp |
| `tracker_smooth_alpha` | 0.9 | EMA smoothing |
| `pedal_engage_idx` | 0 | Pedal A |
| `pedal_discard_idx` | 1 | Pedal B |
| `pedal_episode_idx` | 2 | Pedal C |

`max_teleop_dq` (1.5 rad/s) is hardcoded, not a parameter.
