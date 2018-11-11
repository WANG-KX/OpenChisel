cmake_minimum_required(VERSION 2.8.3)
project(chisel_msgs)
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs message_generation)

add_message_files(FILES
  ChunkMessage.msg
  ChunkListMessage.msg
)

add_service_files(FILES
  GetAllChunksService.srv
  PauseService.srv
  ResetService.srv
  SaveMeshService.srv
  UpdateAllMeshService.srv
)

generate_messages(DEPENDENCIES std_msgs sensor_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS  message_runtime std_msgs sensor_msgs geometry_msgs)
