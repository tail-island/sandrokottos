cmake_minimum_required(VERSION 3.20)
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
    cxx_std_23  # コードはcxx_std_20相当なのですけど、Visual Studio 2022だとcxx_std_20では<ranges>が使えなかった……。→ https://github.com/microsoft/STL/issues/1814
)

target_compile_options(sandrokottos PRIVATE
    /Zc:__cplusplus
    /arch:AVX2
)

target_include_directories(sandrokottos PRIVATE
    or-tools-v9.0/include  # 9.1より9.0の方が結果が良かった。。。
    ${Boost_INCLUDE_DIRS}
)

target_link_directories(sandrokottos PRIVATE
    or-tools-v9.0/lib  # 9.1より9.0の方が結果が良かった。。。
    ${Boost_LIBRARY_DIRS}
)

target_link_libraries(sandrokottos
    ortools
    nlohmann_json::nlohmann_json
)
