





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



install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)




find_package(PythonLibs)


add_executable(main
    main.cpp
    matplotlibcpp.h
)
target_include_directories(main PRIVATE ${PYTHON_INCLUDE_DIRS})

target_link_libraries(main
    PUBLIC
        ${PROJECT_NAME}
        ${PYTHON_LIBRARIES}
)

