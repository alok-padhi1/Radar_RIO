# RIO Stack Sensor Contracts

This document defines the strict data contracts for all sensors entering the RIO stack.
Violating these contracts is the most common cause of integration failure.

## 1. Radar (Linpowave U300)
- **Coordinate Frame**: Native radar frame (X=forward, Y=left, Z=up or as defined by vendor).
- **Angles**: Must be computed using `atan2`, NEVER `asin`. (Blueprint §5.2)
- **Doppler Sign**: Corrected exactly ONCE in `drivers/u300/adapter.py`.
- **Timestamps**: Preserve the firmware timestamp if available. Do not overwrite with `time.monotonic()` unless required.

## 2. IMU (Cube Orange+)
- **Message**: `RAW_IMU` or `SCALED_IMU` only.
- **Content**: Raw gyroscope (rad/s) and accelerometer (m/s²).
- **Prohibited**: Do NOT use `LOCAL_POSITION_NED` or `ATTITUDE` as the primary input to the estimator.
- **Feedback Loop**: Never feed autopilot-fused data into RIO, which then feeds back into the autopilot.

## 3. Altimeter (Linpowave U200A)
- **Measurement**: Slant range along the beam axis.
- **Correction**: Must be corrected for vehicle attitude: `h_AGL = r * |e_z^T R_WA a_A|`
- **Staleness**: Do not hold the last valid measurement indefinitely. Reject after `max_age_s`.
- **Invalid State**: When out of range, set `valid=false`. Do not output 0 or maximum range as a valid measurement.
