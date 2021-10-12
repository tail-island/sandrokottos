cmake_minimum_required(VERSION 3.18)
project(sandrokottos LANGUAGES CXX)

find_package(Boost REQUIRED)

include(cmake/nlohmann_json.cmake)

add_executable(sandrokottos
    IO.h
    Main.cpp
    Model.h
    OptimizeOrderSize.h
    OptimizePickupAndDeliveryDuration.h
    SolveCVRPPDTW.h
)

target_compile_features(sandrokottos PRIVATE
    cxx_std_20
)

target_compile_options(sandrokottos PRIVATE
    /Zc:__cplusplus
    /arch:AVX2
)

target_include_directories(sandrokottos PRIVATE
    or-tools-v9.0/include
    ${Boost_INCLUDE_DIRS}
)

target_link_directories(sandrokottos PRIVATE
    or-tools-v9.0/lib
    ${Boost_LIBRARY_DIRS}
)

target_link_libraries(sandrokottos PRIVATE
    ortools
    nlohmann_json::nlohmann_json
)
