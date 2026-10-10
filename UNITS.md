## One-Page Unit Reference
This page is a comprehensive documentation of the units of values reported in the Sensor Logger app. Much effort has been spent to ensure correctness and consistency across iOS and Android. Should you notice any error, please report by making an Issue or make a pull request directly. Additional information about sensors and measurements is available in-app by tapping the "eye" icon next to each sensor.

For full documentation, please visit [Units](https://www.tszheichoi.com/sensorloggerhelp)

## Time
time is in nanoseconds since epoch. For more information about conversion, use tools like https://www.epochconverter.com/.
seconds_elapsed is in seconds since the start of the recording.
## Accelerometer
x, y and z in meters per second squared (ms-2); Assumes g to be 9.80665ms-2.
Android Only: On Android, due to varying device capabilities, there are two separate acceleration files:

Accelerometer.csv: measures acceleration without gravity (user-induced motion only). Note that some Android devices may not have the capability to isolate acceleration without gravity, and only TotalAcceleration is available.
TotalAcceleration.csv: measures acceleration with gravity included (raw accelerometer readings).
On iOS, only Accelerometer.csv is logged. You may create acceleration with gravity by combining the Accelerometer and Gravity sensor readings.

See the Coordinates Reference for definitions of axes.

## Accelerometer Uncalibrated
x, y and z:
On iOS, this is in standard gravity (g), when "Standardise Units & Frames" is off (default). The unit is ms-2 when "Standardise Units & Frames" is on.
On Android, this is in meters per second squared (ms-2).
💡: Toggling Standardise Units & Frames under Settings > Sensor Configuration removes all platform-dependent units and coordinate systems differences.

## Gyroscope
x, y and z, in radians per second (rad/s).
## Gyroscope Uncalibrated
x, y and z, in radians per second (rad/s).


## Location
** altitude, in meters as a height above the WGS84 ellipsoid.

** altitudeAboveMeanSeaLevel, in meters above mean sea level. On Android this requires API 34+ (Android 14), and is absent on older devices.

** bearing, in degrees from 0 to 360 relative to due north. Negative values indicate it is invalid.

** bearingAccuracy, in degrees. Negative values indicate it is invalid.

**horizontalAccuracy, in meters as the radius of uncertainty. It is approximately the unit standard deviation around the reported position. Negative values indicate it is invalid.

**verticalAccuracy, in meters. It is approximately the unit standard deviation around the altitude. Negative values indicate it is invalid.

**speed, in meters per second. Negative values indicate it is invalid. Note this is derived from the location, so may be unreliable if the speed changes between location updates.

**speedAccuracy, in meters per second. Negative values indicate it is invalid.

**latitude, in degrees. Positive values are north of the equator (-90 to 90)

**longitude, in degrees. Positive values are east of the meridian line (-180 to 180).

**x, y and z, in radians per second (rad/s).


