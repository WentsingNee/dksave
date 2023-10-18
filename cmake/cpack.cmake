#
# @file       cpack.cmake
# @brief
# @date       2023-10-18
# @author     Wentsing Nee
# @copyright
#      Wentsing Nee of China Agricultural University
#   all rights reserved
#

set(CPACK_PACKAGE_NAME ${CMAKE_PROJECT_NAME})
set(CPACK_PACKAGE_VERSION ${CMAKE_PROJECT_VERSION})
set(CPACK_PACKAGE_CONTACT "Wentsing Nee")
set(CPACK_RESOURCE_FILE_LICENSE ${PROJECT_SOURCE_DIR}/LICENSE)
set(CPACK_GENERATOR "NSIS64")


include(CPack)