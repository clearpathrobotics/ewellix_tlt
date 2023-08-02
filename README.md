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
