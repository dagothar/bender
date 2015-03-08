## gripper test sandbox
ADD_EXECUTABLE(gripper_test gripper_test.cpp)
ADD_DEPENDENCIES(gripper_test evg55)
TARGET_LINK_LIBRARIES(gripper_test evg55)

## packet test sandbox
ADD_EXECUTABLE(packet_test packet_test.cpp)
ADD_DEPENDENCIES(packet_test evg55)
TARGET_LINK_LIBRARIES(packet_test evg55)

## send ack command
ADD_EXECUTABLE(send_ack send_ack.cpp)
ADD_DEPENDENCIES(send_ack evg55)
TARGET_LINK_LIBRARIES(send_ack evg55)

## send ref command
ADD_EXECUTABLE(send_ref send_ref.cpp)
ADD_DEPENDENCIES(send_ref evg55)
TARGET_LINK_LIBRARIES(send_ref evg55)

# send move to 20mm
ADD_EXECUTABLE(send_move send_move.cpp)
ADD_DEPENDENCIES(send_move evg55)
TARGET_LINK_LIBRARIES(send_move evg55)
