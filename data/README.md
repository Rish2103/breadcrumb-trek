# Raw Sensor Data & Replay Storage

## CSV Ingestion & Recording Schema

To support offline testing, replayability, and deterministic validation, raw sensor logs recorded by Breadcrumb Trek adhere to the following CSV structure:

```csv
timestamp_ns,sensor_type,x,y,z
```

### Fields
- `timestamp_ns`: Sensor hardware event timestamp in nanoseconds (`SensorEvent.timestamp`).
- `sensor_type`: Integer sensor identifier:
  - `1`: Accelerometer (`m/s²`)
  - `4`: Gyroscope (`rad/s`)
  - `2`: Magnetometer (`µT`)
  - `6`: Barometer (`hPa`, value in `x`, `0.0` for `y` and `z`)
- `x, y, z`: Calibrated sensor axis readings in standard Android body frame.

## Replay Invariant
The native navigation engine receives sensor events through the identical `pushAccel`, `pushGyro`, `pushMag`, and `pushBaro` interface regardless of whether samples originate from live Android `SensorEventListener` callbacks or from replayed CSV logs.
