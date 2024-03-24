#
# @file       ctre.cmake
# @brief
# @date       2024-03-14
# @author     Wentsing Nee
# @copyright
#      Wentsing Nee of China Agricultural University
#   all rights reserved
#

message(STATUS "Finding ctre")
find_package(ctre REQUIRED)
message(STATUS "Finding ctre -- Found")

target_link_libraries(dksave PRIVATE ctre::ctre)
