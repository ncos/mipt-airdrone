# Install script for directory: /home/ncos/mipt-airdrone/src/action_client

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "/home/ncos/mipt-airdrone/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Debug")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

# Install shared libraries without execute permission?
IF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  SET(CMAKE_INSTALL_SO_NO_EXE "1")
ENDIF(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_client/action" TYPE FILE FILES
    "/home/ncos/mipt-airdrone/src/action_client/action/MoveAlong.action"
    "/home/ncos/mipt-airdrone/src/action_client/action/Rotation.action"
    "/home/ncos/mipt-airdrone/src/action_client/action/ApproachDoor.action"
    "/home/ncos/mipt-airdrone/src/action_client/action/SwitchWall.action"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_client/msg" TYPE FILE FILES
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongAction.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionGoal.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionResult.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongActionFeedback.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongGoal.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongResult.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/MoveAlongFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_client/msg" TYPE FILE FILES
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationAction.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionGoal.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionResult.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationActionFeedback.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationGoal.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationResult.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/RotationFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_client/msg" TYPE FILE FILES
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachDoorAction.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachDoorActionGoal.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachDoorActionResult.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachDoorActionFeedback.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachDoorGoal.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachDoorResult.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/ApproachDoorFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_client/msg" TYPE FILE FILES
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallAction.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionGoal.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionResult.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallActionFeedback.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallGoal.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallResult.msg"
    "/home/ncos/mipt-airdrone/devel/share/action_client/msg/SwitchWallFeedback.msg"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_client/cmake" TYPE FILE FILES "/home/ncos/mipt-airdrone/build/action_client/catkin_generated/installspace/action_client-msg-paths.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/ncos/mipt-airdrone/devel/include/action_client")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/common-lisp/ros" TYPE DIRECTORY FILES "/home/ncos/mipt-airdrone/devel/share/common-lisp/ros/action_client")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  execute_process(COMMAND "/usr/bin/python" -m compileall "/home/ncos/mipt-airdrone/devel/lib/python2.7/dist-packages/action_client")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python2.7/dist-packages" TYPE DIRECTORY FILES "/home/ncos/mipt-airdrone/devel/lib/python2.7/dist-packages/action_client")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ncos/mipt-airdrone/build/action_client/catkin_generated/installspace/action_client.pc")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_client/cmake" TYPE FILE FILES "/home/ncos/mipt-airdrone/build/action_client/catkin_generated/installspace/action_client-msg-extras.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_client/cmake" TYPE FILE FILES
    "/home/ncos/mipt-airdrone/build/action_client/catkin_generated/installspace/action_clientConfig.cmake"
    "/home/ncos/mipt-airdrone/build/action_client/catkin_generated/installspace/action_clientConfig-version.cmake"
    )
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/action_client" TYPE FILE FILES "/home/ncos/mipt-airdrone/src/action_client/package.xml")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

