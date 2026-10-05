cmake_minimum_required(VERSION 3.22.1)
project("breadcrumb_native" CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(ahrs pdr trail guidance)

add_library(breadcrumb_native SHARED
    ahrs/NavigationEngine.cpp
    guidance/ReverseNavigator.cpp
    jni/breadcrumb_native.cpp
)

find_library(log-lib log)

target_link_libraries(breadcrumb_native
    ${log-lib}
)
