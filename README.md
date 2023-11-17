### Use AprilTags with Franka

Ensure you have:

[RealSense ROS2](https://github.com/IntelRealSense/realsense-ros.git)

Launch with:

   ```
   ros2 launch apriltags get_apriltags.launch.xml
   ```

### Import Mover API
`${WorkSpace}` is your work space directory and `${Repo_Dir}` is the directory name for this repository

Import the **Mover API** via 
```
vcs import ${WorkSpace}/src < ${Repo_Dir}/mover.repos
```
# Launch Instructions
First, connect to `https://panda0.robot`, and activated the FCI

**Copy SSH ID**

If you have not already done so, copy your ssh key to station via
```
ssh-copy-id -i ~/.ssh/id_ed25519.pub student@station
```

**Connect the station**

Connect to the station via
```
ssh student@station
```

On station run 
```
ros2 launch franka_moveit_config moveit.launch.py use_rviz:=false robot_ip:=panda0.robot
```

On laptop run
```
ros2 launch polyglotbot polyglotbot.launch.xml hardware_type:=real
```


### Use AprilTags with Franka

Ensure you have:

[RealSense ROS2](https://github.com/IntelRealSense/realsense-ros.git)

Launch with:

   ```
   ros2 launch apriltags get_apriltags.launch.xml
   ```

### Import Mover API
`${WorkSpace}` is your work space directory and `${Repo_Dir}` is the directory name for this repository

Import the **Mover API** via 
```
vcs import ${WorkSpace}/src < ${Repo_Dir}/mover.repos
```
