# Mavros Primitive
ROS package to demonstrate primitive functionality of ros services with an autopilot using mavros.


How To Run
==================

Open a terminal and run ```roscore```

Open a second terminal run our rosnode via ```mavros_primitive mavros_primitive_node```
If this command fails to find our node you can navigate to the directory```catkin_ws/devel/lib/mavros_primitive``` and run the node manually by ```./mavros_primitive_node```

The ```mavros_primitive_node``` should now be running.

Press '''w''' to send takeoff request.

Press '''s''' to send land request.
