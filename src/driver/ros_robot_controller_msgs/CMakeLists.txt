cmake_minimum_required(VERSION 3.8)
project(ros_robot_controller_msgs)

if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(MSG_FILES
  "msg/ButtonState.msg"
  "msg/BuzzerState.msg"
  "msg/OLEDState.msg"
  "msg/MotorState.msg"
  "msg/MotorsState.msg"
  "msg/MotorSpeedControl.msg"
  "msg/MotorsSpeedControl.msg"
  "msg/MecanumState.msg"
  "msg/TankState.msg"
  "msg/ConveyorState.msg"
  "msg/StepperRun.msg"
  "msg/ArmCoords.msg"
  "msg/ArmMoveInc.msg"
  "msg/ArmServoSingle.msg"
  "msg/ServoPosition.msg"
  "msg/ServosPosition.msg"
  "msg/EspnowState.msg"
  "msg/ArmFullState.msg"
  "msg/Sbus.msg"
  "msg/StepperSubdivision.msg"
  "msg/ServoId.msg"
  "msg/ServoMode.msg"
  "msg/ServoOffset.msg"
  "msg/ServoPID.msg"
  "msg/ServoOverload.msg"
  "msg/ServoBaudRate.msg"
  "msg/ServoMaxTorque.msg"
  "msg/ServoAngleLimit.msg"
  "msg/CoordLimits.msg"
  "msg/ChassisConfig.msg"
  "msg/KinematicsParam.msg"
  "msg/SyncWriteServos.msg"
  "msg/MacAddress.msg"
)

set(SRV_FILES
  "srv/BusServoCtrl.srv"
  "srv/GetArmCoords.srv"
  "srv/GetArmIK.srv"
  "srv/GetArmFK.srv"
  "srv/GetArmFullState.srv"
  "srv/GetFirmwareVersion.srv"
  "srv/GetServoOffset.srv"
  "srv/GetServoPID.srv"
  "srv/GetServoOverload.srv"
  "srv/GetServoBaud.srv"
  "srv/GetServoMaxTorque.srv"
  "srv/GetServoAngleLimit.srv"
  "srv/GetCoordLimits.srv"
  "srv/GetChassisConfig.srv"
  "srv/GetKinematicsParam.srv"
  "srv/SetKinematicsParam.srv"
  "srv/SetStepperPosition.srv"
  "srv/MoveStepper.srv"
  "srv/ScanWifiChannels.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${MSG_FILES}
  ${SRV_FILES}
  DEPENDENCIES std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
