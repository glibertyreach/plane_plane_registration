cmake_minimum_required(VERSION 3.20)
project(lri_plane_registration LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LRI_BUILD_TESTS "Build the unit tests" ON)

find_package(Eigen3 3.3 REQUIRED)

# Warnings are errors on this project's own targets only (not on fetched third-party code).
set(LRI_WARNING_FLAGS -Wall -Wextra -Wpedantic -Werror)

add_library(lri_plane_registration STATIC src/PlaneRegistration.cpp)
target_include_directories(lri_plane_registration PUBLIC include)
target_link_libraries(lri_plane_registration PUBLIC Eigen3::Eigen)
target_compile_options(lri_plane_registration PRIVATE ${LRI_WARNING_FLAGS})

if(LRI_BUILD_TESTS)
  enable_testing()

  # Prefer a system GoogleTest; fall back to downloading v1.14.0.
  find_package(GTest QUIET)
  if(NOT GTest_FOUND)
    include(FetchContent)
    FetchContent_Declare(
      googletest
      GIT_REPOSITORY https://github.com/google/googletest.git
      GIT_TAG v1.14.0)
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
  endif()

  include(GoogleTest)
  add_executable(plane_registration_tests tests/PlaneRegistrationTest.cpp)
  target_link_libraries(plane_registration_tests PRIVATE
    lri_plane_registration GTest::gtest GTest::gtest_main)
  target_compile_options(plane_registration_tests PRIVATE ${LRI_WARNING_FLAGS})
  gtest_discover_tests(plane_registration_tests)
endif()
