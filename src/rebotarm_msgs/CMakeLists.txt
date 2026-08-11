cmake_minimum_required(VERSION 3.8)
project(rebotarm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/JointMitCmd.msg"
  "msg/JointPosVelCmd.msg"
  "msg/JointMotorState.msg"
  "msg/ArmStatus.msg"
  "msg/TimingStatus.msg"
  "msg/DigitalIO.msg"
  "msg/DigitalIOState.msg"
  "msg/SegmentedObject.msg"
  "msg/SegmentedObjectArray.msg"
  "srv/SetZero.srv"
  "srv/MoveToPoseIK.srv"
  "srv/SetGripper.srv"
  "srv/GripperCommand.srv"
  "srv/SetTorqueProfile.srv"
  "srv/SetSafetyStep.srv"
  "srv/CheckTorqueProfile.srv"
  "srv/CheckTrajectoryClearance.srv"
  "srv/SetDigitalOutput.srv"
  "srv/SetDigitalIOConfig.srv"
  "srv/CalibrateVision.srv"
  "action/MoveToPose.action"
  DEPENDENCIES builtin_interfaces geometry_msgs sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
