set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE BOOL "No dev warnings")
set(CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake/Qt5" ${CMAKE_PREFIX_PATH})
# Qt5 查找与链接
find_package(Qt5 REQUIRED COMPONENTS Core Gui Widgets)


cmake_minimum_required(VERSION 3.25)
project(Dart)

aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_NAME}_src)

add_executable(${PROJECT_NAME} ${${PROJECT_NAME}_src})
target_link_libraries(${PROJECT_NAME} PRIVATE Qt5::Core Qt5::Gui Qt5::Widgets)
target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS})
target_include_directories(${PROJECT_NAME} PRIVATE ${OpenCV_INCLUDE_DIRS})

add_subdirectory(src/utility)
add_subdirectory(src/capture)
add_subdirectory(src/sensors)
add_subdirectory(src/detect)
add_subdirectory(src/track)
add_subdirectory(src/mouse)


target_link_libraries(${PROJECT_NAME} PRIVATE utility)
target_include_directories(${PROJECT_NAME} PRIVATE src/utility/inc)

target_link_libraries(${PROJECT_NAME} PRIVATE capture)
target_include_directories(${PROJECT_NAME} PRIVATE src/capture/inc)

target_include_directories(${PROJECT_NAME} PRIVATE src/threads/inc)

target_link_libraries(${PROJECT_NAME} PRIVATE sensors)
target_include_directories(${PROJECT_NAME} PRIVATE src/sensors/inc)

target_link_libraries(${PROJECT_NAME} PRIVATE detect)
target_include_directories(${PROJECT_NAME} PRIVATE src/detect/inc)

target_link_libraries(${PROJECT_NAME} PRIVATE track)
target_include_directories(${PROJECT_NAME} PRIVATE src/track/inc)

target_link_libraries(${PROJECT_NAME} PRIVATE mouse)
target_include_directories(${PROJECT_NAME} PRIVATE src/mouse/inc)


target_link_libraries(${PROJECT_NAME} PRIVATE pthread)
