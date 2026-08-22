cmake_minimum_required(VERSION 3.22)

project(balloon_common_tests C)

enable_testing()

add_executable(balloon_radio_protocol_test
    balloon_radio_protocol_test.c
    ../Src/balloon_radio_protocol.c
)
target_include_directories(balloon_radio_protocol_test PRIVATE ../Inc)
target_compile_options(balloon_radio_protocol_test PRIVATE
    -Wall
    -Wextra
    -Werror
)

add_test(NAME balloon_radio_protocol_test COMMAND balloon_radio_protocol_test)
