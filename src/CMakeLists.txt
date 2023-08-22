


include_directories(include)

add_library(${PROJECT_NAME}
    SHARED
        include/${PROJECT_NAME}/generic_torque_control.hpp
        ${PROJECT_NAME}/generic_torque_control.cpp
        include/${PROJECT_NAME}/actuators_interface.hpp
        ${PROJECT_NAME}/actuators_interface.cpp
)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    blmc_drivers::blmc_drivers
    ${VTK_LIBRARIES}
    yaml-cpp
    Eigen3::Eigen
)



install(TARGETS ${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_LIBDIR}
)
install(DIRECTORY include/${PROJECT_NAME}
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)


add_executable(custom_law custom_law.cpp)
target_link_libraries(custom_law
    ${PROJECT_NAME}
    utilities
)
