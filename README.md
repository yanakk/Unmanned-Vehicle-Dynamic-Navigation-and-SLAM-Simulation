# Unmanned-Vehicle-Dynamic-Navigation-and-SLAM-Simulation
One of the projects of the "Robot Design and Practice" course for the 2019-2020 academic year. Simulate in RViz and Gazebo.

The content includes:
- A-star route plan, DWA dynamic obstacle avoidance, and dynamic re-plan [[code]](https://github.com/yanakk/Unmanned-Vehicle-Dynamic-Navigation-and-SLAM-Simulation/tree/master/src/c5/course_agv_nav/scripts)
- Iterative Closest Point (ICP) localization [[code]](https://github.com/yanakk/Unmanned-Vehicle-Dynamic-Navigation-and-SLAM-Simulation/blob/master/src/course_agv_slam_task/src/icp_lm.cpp)
- Laser-based road sign extraction [[code]](https://github.com/yanakk/Unmanned-Vehicle-Dynamic-Navigation-and-SLAM-Simulation/blob/master/src/course_agv_slam_task/src/extraction.cpp)
- Extended Kalman Filter (EKF) SLAM [[code]](https://github.com/yanakk/Unmanned-Vehicle-Dynamic-Navigation-and-SLAM-Simulation/blob/master/src/course_agv_slam_task/src/ekf.cpp)
- Monte Carlo method for particle filtering [[code]](https://github.com/yanakk/Unmanned-Vehicle-Dynamic-Navigation-and-SLAM-Simulation/blob/master/src/course_agv_slam_task/src/particle_filter.cpp)
- Dynamic real-time mapping [[code]](https://github.com/yanakk/Unmanned-Vehicle-Dynamic-Navigation-and-SLAM-Simulation/blob/master/src/course_agv_slam_task/src/bayes_mapping.cpp)


Launch commands:

- Route plan and navigation (also launch RViz and Gazebo)

  `roslaunch course_agv_nav nav.launch`

- Start laser-based road sign extraction

  `roslaunch course_agv_slam_task extraction.launch`

- ICP localization

  `roslaunch course_agv_slam_task icp_lm.launch`

- EKF-SLAM

  `roslaunch course_agv_slam_task ekf.launch`

- Real-time mapping

  `roslaunch course_agv_slam_task bayes_mapping.launch`

- Particle filtering localization

  `roslaunch course_agv_slam_task particle_filter.launch`
