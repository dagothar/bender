## gripper test sandbox
ADD_EXECUTABLE(gripper_test gripper_test.cpp)
ADD_DEPENDENCIES(gripper_test evg55)
TARGET_LINK_LIBRARIES(gripper_test evg55)

## packet test sandbox
ADD_EXECUTABLE(packet_test packet_test.cpp)
ADD_DEPENDENCIES(packet_test evg55)
TARGET_LINK_LIBRARIES(packet_test evg55)
