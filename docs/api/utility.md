# Utility API

## `Stop()`

Brakes both motors for 40 ms, then holds them in place.

## `DegToMM(deg)`

Converts motor encoder degrees to millimeters of travel using the configured wheel diameter.

| Parameter | Type | Description |
|-----------|------|-------------|
| `deg` | float | Motor encoder value in degrees |

**Returns:** Distance in millimeters (float).

## `ThetaErr(angle)`

Returns the signed heading error in degrees (range −180° to 180°) between the current IMU heading and a target angle.

| Parameter | Type | Description |
|-----------|------|-------------|
| `angle` | float | Target angle in degrees |

**Returns:** Signed heading error (float), range −180° to 180°.
