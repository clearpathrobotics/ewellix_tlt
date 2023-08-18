# The ewellix TLT ROS package

This ROS package allows you to control or simulate an Ewellix TLT lifting column.
It is compatible with the Kinetic and melodic versions.

## Use in simulation

```bash
roslaunch ewellix_tlt ewellix_simulated.launch
```
Once the program is launched you can move the column joint via the joint_state_publisher_gui interface.

## Use in real conditions
Connect the column to your computer and modify the launcher to set the port. By default it is ttyUSB0.

Then use:

```bash
roslaunch ewellix_tlt ewellix_driver.launch
```

## Initialization
The lift kit's motors use a it's encoder limits to determine it's current locaiton. Therefore, if the lift is powered off when it is not at its base position then you will need to move it to the base position before it can be sent position commands.

To facilitate this, you can call the intialization service:
```bash
rossrv call /ewellix_tlt/actions/init_sequence {}
```

## Movement
The lift can be stopped at any time by publishing to the stop topic:
```bash
rostopic pub /ewellix_tlt/actions/stop
```

The lift can be moved in three different ways:
1. Setting it's extension in meters by publishing to the `/ewellix_tlt/actions/position` topic
2. Setting the motor ticks for each motor using the `/ewellix_tlt/actions/motor1_ticks` and `/ewellix_tlt/actions/motor2_ticks`
3. Setting a duration to move the arm up or down using the `/ewellix_tlt/actions/duration_up` and `/ewellix_tlt/actions/duration_down`, specified in seconds.

> Motor ticks for each motor range from 10 to 850.
