# Bash Example

The bash example shows the low level access to the driver API. All sensors
and motors are exposed via the linux sysfs. To use this script, no setup is
needed.

## The example robot

The `flee.sh` shell script expects a robot with the following configuration:

* Heavy tacho-motor connected to A
* Heavy tacho-motor connected to B
* Ultrasonic sensor connected to 1, heading forward

If you have setup your robot like that, you can execute the `flee.sh` script.
The motors get activated and move the robot backwards, if the ultrasonic sensor
detects something in front of the robot.
