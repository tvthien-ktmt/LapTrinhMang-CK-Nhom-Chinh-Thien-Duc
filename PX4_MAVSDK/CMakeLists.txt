cmake_minimum_required(VERSION 3.10)
project(px4_offboard_square)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(px4_offboard_square main.cpp)

# Include path MAVSDK
target_include_directories(px4_offboard_square PUBLIC
    /usr/local/include/mavsdk
)

# Link duy nhất libmavsdk.so
target_link_libraries(px4_offboard_square
    /usr/local/lib/libmavsdk.so
)
