include(GoogleTest)

add_subdirectory(googletest)
include_directories(googletest)

add_executable(unittests given_a_peak_finder.cpp)
target_link_libraries(unittests iasdk_network iasdk_utility iasdk_protobuf iasdk_navigation gtest_main gmock)
gtest_discover_tests(unittests)
