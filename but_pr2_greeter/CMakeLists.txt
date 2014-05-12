cmake_minimum_required(VERSION 2.8.3)
project(but_pr2_greeter)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  moveit_commander
  moveit_msgs
  rospy
  tf
#  roslint
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS src/face_detector.py src/greeter.py src/fake_face_detector.py src/people_to_point.py src/speech_synthesis.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}/src)
