find_package(Threads)
find_package(Boost)

include_directories(
        inc
        ${Boost_INCLUDE_DIRS}
)

add_library(
        serialPort
        src/serialPort.cpp
)
target_link_libraries(
        serialPort
        -lboost_system
        -lpthread
)
