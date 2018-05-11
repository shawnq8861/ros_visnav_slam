# ros-visnav-slam-cnn
ROS package holding nodes that demonstrate:

1. Basic ROS publish/subscribe
2. ROS integration with real hardware, i.e. motors, cameras, and IMUs.
3. High level computer vision and robot control functionality:
      a. Optical flow camera egomotion.
      b. Monocular visual odometry.
      c. Brushed DC motor control.
      d. Rotary encoder interface.
      e. Extended Kalman Filter fused visual navigation with single camera and IMU.
      f. Visual simultaneous localization and mapping (vSLAM) using a single camera and IMU.
      g. Object detection on live camera images using pre-trained convolutional neural networks (CNN).
      
4. Install ROS Kinetic:  http://wiki.ros.org/kinetic/Installation/Ubuntu
5. Configure a Catkin workspace and environment:  http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
6. Clone into the catkin workspace and run catkin_make.
