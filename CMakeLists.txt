cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS
    ./components/bsp_extra
)

add_compile_options(-Wno-format)
add_compile_options(-Wno-ignored-qualifiers)
add_compile_options(-Wno-attributes)
add_compile_options(-Wno-unused-function)

project(esp32_p4_modbus_display)
