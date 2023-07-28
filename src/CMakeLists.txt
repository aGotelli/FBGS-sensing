





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
        debug utilities-debug
        optimized utilities
        yaml-cpp
)

add_executable(test_tcpip_connection
    test_tcpip_connection.cpp
)

target_link_libraries(test_tcpip_connection
    PUBLIC
    Eigen3::Eigen
    ${Boost_LIBRARIES}
    real_time_tools::real_time_tools
    utilities
    Qt5::Widgets
    Qt5::Charts
)


add_executable(read_and_save_data
    read_and_save_data.cpp
)

target_link_libraries(read_and_save_data
    PUBLIC
        ${PROJECT_NAME}
        debug utilities-debug
        optimized utilities
        yaml-cpp
)


add_executable(save_with_mutex
    save_with_mutex.cpp
)

target_link_libraries(save_with_mutex
    PUBLIC
        ${PROJECT_NAME}
        debug utilities-debug
        optimized utilities
        yaml-cpp
)
