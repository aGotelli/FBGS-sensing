

#include directories
include_directories(${PROJECT_SOURCE_DIR}/src/include)




add_executable(benchmark_reading
    benchmark_reading.cpp
)
target_link_libraries(benchmark_reading
    PUBLIC
        ${PROJECT_NAME}
        benchmark::benchmark
)
