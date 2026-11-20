cmake_minimum_required(VERSION 3.16)

# Daftarkan folder Components agar semua library di dalamnya terdeteksi otomatis
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/Components")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Main_PRS_BeanBag)