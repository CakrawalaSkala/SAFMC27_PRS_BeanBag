cmake_minimum_required(VERSION 3.16)

# Tambahkan jalur ke folder Components luar
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../Components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Main_PRS_BeanBag)