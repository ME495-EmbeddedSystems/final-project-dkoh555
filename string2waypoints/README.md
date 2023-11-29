Author: Kassidy Shedd

The ROS2 string2waypoint Node package has the ability to take in a string in any 
language except the following: arabic, hindi, persian, thai and create waypoints
that create these letters.

This package will not account for language that read right to left, so user
must be careful to input strings in the order of desired output waypoints.

To launch this Node package run the following:
`ros2 launch string2waypoints create_waypoint.launch.xml`

To get the waypoints from a desired string, call 
`ros2 service call /`....

The waypoints will outpoint as a list of point messages. 