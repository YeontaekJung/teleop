# Input Nodes — Architecture

## vive_ros2 — Vive Tracker → ROS

Source: `src/input/vive_ros2/vive_ros2/vive_tracker_node.py`

### Coordinate transform (OpenVR → ROS)

OpenVR is Y-up right-handed; ROS is Z-up right-handed.

```
pos_ros = [-vr_z, -vr_x, vr_y]
```

Rotation: column-wise axis transform applied to each basis vector.

### Station alignment

On first initialization (once only): computes yaw rotation that aligns the `station_left → station_right` vector to the Y-axis. Applied to all subsequent tracker poses. **Never recalibrated at runtime.**

Both base stations + both hand trackers must be alive before initialization completes.

### Device index mapping

| Index | Device |
|---|---|
| 0 | station_left |
| 1 | station_right |
| 2 | tracker_left |
| 3 | tracker_right |
| 4 | tracker_body (optional) |

Body tracker: only published if serial configured and device alive. Missing serial = silently skipped.

### Published topics

`/teleop/tracker/{left,right,body}` — `geometry_msgs/PoseStamped`, `frame_id='world'`, 100 Hz.

### Parameters

`serial_station_{left,right}`, `serial_tracker_{left,right,body}`, `publish_rate` (100.0 Hz).

### Gotchas

- `bPoseIsValid` checked per frame — invalid poses not published (no stale data)
- Unknown serials throttle-logged every 10s
- `openvr.TrackingUniverseStanding` (not Sitting)
- Scipy `as_quat()` returns `[x,y,z,w]`

---

## pedal_ros2 — PCsensor FootSwitch → Joy

Source: `src/input/pedal_ros2/pedal_ros2/pedal_node.py`

### Hardware mapping

| Pedal | evdev code | `Joy.buttons[]` |
|---|---|---|
| A (left) | `KEY_A` (30) | `[0]` |
| B (middle) | `KEY_B` (48) | `[1]` |
| C (right) | `KEY_C` (46) | `[2]` |

Event values: 1=press, 0=release, 2=hold → normalized to 1 if ≥1. Publishes on every state change.

Device is **exclusively grabbed** (`.grab()`) to prevent key leakage to OS. Requires `input` group membership.

Publishes: `/teleop/pedal` (`sensor_msgs/Joy`), `buttons[0:3]`, `axes=[]`.

### Gotchas

- Hard fails startup (`RuntimeError`) if device not found — SteamVR or other USB HID device may claim the name first.
- Background thread is daemon — killed with main process.
- `sudo usermod -aG input $USER` + **full re-login** required (closing terminal not enough).

---

## manus_ros2 — Manus SDK → ROS

Source: `src/input/manus_ros2/src/manus_data_publisher.cpp`

### Architecture

`ManusDataPublisher` inherits both `SDKClientPlatformSpecific` and `rclcpp::Node`. SDK callbacks are static (singleton pattern via `s_Instance`).

### Data pipeline

SDK fires async callbacks → data buffered in mutexed maps → `PublishCallback()` (ROS timer) reads maps and publishes.

| Callback | Data stored |
|---|---|
| `OnRawSkeletonStreamCallback` | `m_GloveDataMap[glove_id]` (SkeletonNode[]) |
| `OnErgonomicsStreamCallback` | `m_ErgonomicsDataMap[glove_id]` (ErgonomicsData) |
| `OnRawDeviceDataStreamCallback` | `m_RawSensorDataMap[glove_id]` |

Publishes: `manus_ros2_msgs/ManusGlove` on `m_GlovePublisher[glove_id]`, one topic per connected glove.

### Coordinate system

Default: `{AxisView_XFromViewer, AxisPolarity_PositiveZ, Side_Right, scale=1.0}`, `WorldSpace=true`. Must be configured **before** `Initialize()` — no effect after.

### Gotchas

- Singleton (`s_Instance`) — only one publisher per process.
- `PublishCallback()` must lock all mutexes — SDK writes from callback threads concurrently.
- Coordinate system is set-once at init.
