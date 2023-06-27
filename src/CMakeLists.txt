





#include directories
include_directories(include)



add_library(${PROJECT_NAME}
    include/${PROJECT_NAME}/illumisense_interface.h
    include/${PROJECT_NAME}/shape_sensing_interface.h
    ${PROJECT_NAME}/illumisense_interface.cpp
    ${PROJECT_NAME}/shape_sensing_interface.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        ${Boost_LIBRARIES}
)

add_executable(main
    main.cpp
)
target_link_libraries(main
    PUBLIC
        ${PROJECT_NAME}
)

