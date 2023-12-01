cmake_minimum_required(VERSION 3.0)
project(lis_vr_app)

if (POLICY CMP0072)
  cmake_policy (SET CMP0072 NEW)
endif(POLICY CMP0072)

find_package(X11 REQUIRED)
find_package(OpenGL REQUIRED)

INCLUDE(FindPkgConfig)
PKG_SEARCH_MODULE(SDL2 REQUIRED sdl2)

add_executable(lis_vr_app main.c)

include(FindPkgConfig)
pkg_search_module(OPENXR openxr)
if (OPENXR_FOUND)
  MESSAGE("OpenXR found with pkg-config")
  target_link_libraries(lis_vr_app PRIVATE ${OPENXR_LIBRARIES})
else()
  MESSAGE("OpenXR not found with pkg-config, trying cmake script")
  # current issue in upstream OpenXR cmake files requires us to find Threads on our own
  find_package(Threads REQUIRED)
  find_package(OpenXR REQUIRED)
  if (NOT OpenXR_FOUND)
     MESSAGE(FATAL_ERROR "OpenXR not found!")
  endif()

  target_include_directories(lis_vr_app PRIVATE ${OpenXR_INCLUDE_DIR})
  target_link_libraries(lis_vr_app PRIVATE OpenXR::openxr_loader)
endif()

target_link_libraries(lis_vr_app PRIVATE Xrandr ${X11_LIBRARIES} ${OPENGL_LIBRARIES} ${SDL2_LIBRARIES} m)
target_include_directories(lis_vr_app PRIVATE ${SDL2_INCLUDE_DIRS})

if(MSVC)
  target_compile_options(lis_vr_app PRIVATE /W4 /WX)
else(MSVC)
  target_compile_options(lis_vr_app PRIVATE -pedantic -Wall -Wextra -Wno-unused-parameter)
endif(MSVC)


install(TARGETS lis_vr_app RUNTIME DESTINATION bin)
