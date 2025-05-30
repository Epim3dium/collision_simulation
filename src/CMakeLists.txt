set(SOURCE_FILES
    main.cpp
    scene.cpp
    types.cpp
    physics/col_utils.cpp
    physics/physics_manager.cpp
    physics/restraint.cpp
    physics/rigidbody.cpp
    physics/solver.cpp
)
set(HEADER_FILES
    io_manager.hpp
    scene.hpp
    types.hpp
    physics/col_utils.hpp
    physics/collider.hpp
    physics/material.hpp
    physics/transform.hpp
    physics/physics_manager.hpp
    physics/restraint.hpp
    physics/rigidbody.hpp
    physics/solver.hpp
)
add_compile_options(
  #-Wall
  #-Wextra
  #-Wconversion
  #-Wsign-conversion
  $<$<CONFIG:DEBUG>:-glldb>
  $<$<CONFIG:DEBUG>:-Og>
  $<$<CONFIG:RELEASE>:-O3>
)
add_compile_definitions(
$<$<CONFIG:DEBUG>:EPI_DEBUG>
)

add_library(EpiSim
    ${SOURCE_FILES} ${HEADER_FILES}
)

target_include_directories(EpiSim PUBLIC ./physics . ./../vendor)
# Yep, that's it!
target_link_libraries(EpiSim
  PUBLIC ImGui-SFML::ImGui-SFML
)

include(GNUInstallDirs)

install(TARGETS EpiSim
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Installing is not easy, when we're dealing with shared libs
if(NOT LINK_DEPS_STATIC)
  set_target_properties(EpiSim PROPERTIES
    INSTALL_RPATH $ORIGIN/../${CMAKE_INSTALL_LIBDIR}
  )

  set_target_properties(
    ImGui-SFML
    sfml-graphics sfml-system sfml-window
    PROPERTIES
    INSTALL_RPATH $ORIGIN
  )

  install(TARGETS
    ImGui-SFML
    sfml-graphics sfml-system sfml-window
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    NAMELINK_SKIP # don't need versionless .so's
  )
endif()
if (WIN32 AND BUILD_SHARED_LIBS)
    add_custom_command(TARGET CMakeSFMLProject POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_RUNTIME_DLLS:CMakeSFMLProject> $<TARGET_FILE_DIR:CMakeSFMLProject> COMMAND_EXPAND_LISTS)
endif()

#install(SCRIPT PostInstall.cmake)

include(CPack)
