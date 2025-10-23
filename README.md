# miniROS

mini Robot Operation System.


## Structure

The system is similar to ROS1. There is one server for communication management. All robot/camera/teleoperation/display devices are clients. 

Modularized design. The key idea is for easily plug in & out.

## Develop plan

- Finish checking the locking issue. Tune it out. [Done]
- Add autosaver for robot side.
- Connect it with current commander and test.
- Rewrite the commander logic.
- Joint test.
- Add gripper calibration.