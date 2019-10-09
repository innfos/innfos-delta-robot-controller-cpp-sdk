cmake_minimum_required(VERSION 3.5)
project(DeltaController)


add_compile_options(-std=c++14)
add_compile_options(-fPIC)
add_compile_options(-DQT_NO_VERSION_TAGGING)
add_compile_options(-pthread)
add_compile_options(-DASIO_STANDALONE)
add_compile_options(-DASIO_HAS_STD_CHRONO)


include_directories(
        include
        include/${PROJECT_NAME}
        ${CMAKE_CURRENT_SOURCE_DIR}/../innfos-cpp-sdk/sdk/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../delta_sdk/include
        ${catkin_INCLUDE_DIRS}
)


add_executable(mode1 src/advanced/mode1.cpp)
target_link_libraries(mode1 pthread)
target_link_libraries(mode1 ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/../innfos-cpp-sdk/sdk/lib/linux_x86_64/libActuatorController.so)
target_link_libraries(mode1 ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/../delta_sdk/lib/libDeltaController.so)


add_executable(mode2 src/advanced/mode2.cpp)
target_link_libraries(mode2 pthread)
target_link_libraries(mode2 ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/../innfos-cpp-sdk/sdk/lib/linux_x86_64/libActuatorController.so)
target_link_libraries(mode2 ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/../delta_sdk/lib/libDeltaController.so)




