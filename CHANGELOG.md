# CHANGELOG

## 2021-04-14
- **[Changed]** Remove screen pipe from the launch. It needs to be started by the roceptionist components.

## 2021-03-20
- **[Added]** launch file that spawn an empty world with baxter.
- **[Fixed]** HeadController now returns whether the movement actually successed instead of whether the actionlib completed.
- **[Fixed]** bug where head controller commands the robot to go to 0 when there is no message.


## 2021-03-18
- **[Changed]** Baxter now publish the `world` tf frame that is colocated with `base`. This matches the code on the actual robot.

## 2021-03-04
- **[Fixed]** Bug where face relay doesn't work in gazebo.

## 2021-03-03
- **[Fixed]** Fixed robocept face piping package path.

## 2021-02-18
- **[Added]** code to pipe robocept's face to baxter's screen. 
- **[Changed]** structure & launch files.

## 2021-02-12
- **[Changed]** Reorganized head controller into a different ROS Package.
- **[Changed]** Updated code to follow ROS Noetic formats.
- **[Added]** ROS Messages in the same meta package.