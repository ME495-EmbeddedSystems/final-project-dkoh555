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