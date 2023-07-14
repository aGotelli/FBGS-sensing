





#include directories
include_directories(include)



add_library(${PROJECT_NAME} SHARED
    include/${PROJECT_NAME}/illumisense_interface.h
    include/${PROJECT_NAME}/shape_sensing_interface.h
    ${PROJECT_NAME}/illumisense_interface.cpp
    ${PROJECT_NAME}/shape_sensing_interface.cpp
)
target_link_libraries(${PROJECT_NAME}
    PUBLIC
        Eigen3::Eigen
        ${Boost_LIBRARIES}
        real_time_tools::real_time_tools
)



install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)






add_executable(main
    main.cpp
)

target_link_libraries(main
    PUBLIC
        ${PROJECT_NAME}
)

