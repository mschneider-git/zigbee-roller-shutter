# ESP32-H2 Zigbee roller shutter controller (Window Covering for Home Assistant ZHA)
cmake_minimum_required(VERSION 3.16)

# Firmware version, increase it for every OTA update. It yields the OTA file
# version (1.2.3 -> 0x01020300) and the version in the Basic cluster.
set(PROJECT_VER "1.0.0")

if(NOT PROJECT_VER MATCHES "^([0-9]+)\\.([0-9]+)\\.([0-9]+)$")
    message(FATAL_ERROR "PROJECT_VER must have the form x.y.z, is: ${PROJECT_VER}")
endif()
math(EXPR SHUTTER_OTA_FILE_VERSION "(${CMAKE_MATCH_1} << 24) | (${CMAKE_MATCH_2} << 16) | (${CMAKE_MATCH_3} << 8)"
     OUTPUT_FORMAT HEXADECIMAL)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(COMPILE_DEFINITIONS "SHUTTER_OTA_FILE_VERSION=${SHUTTER_OTA_FILE_VERSION}" APPEND)

project(zigbee_roller_shutter)

# Zigbee OTA file for ZHA after every build:
# build/<manufacturer code>-<image type>-<file version>-ota-file.zigbee
idf_build_get_property(build_dir BUILD_DIR)
idf_build_get_property(python PYTHON)
add_custom_target(ota_file ALL
    COMMAND ${python} ${CMAKE_CURRENT_LIST_DIR}/tools/image_builder_tool.py
            --manuf-id ${CONFIG_SHUTTER_OTA_MANUF_CODE}
            --image-type ${CONFIG_SHUTTER_OTA_IMAGE_TYPE}
            --file-version ${SHUTTER_OTA_FILE_VERSION}
            --header-string "Roller shutter ${PROJECT_VER}"
            --tag 0x0000 ${build_dir}/${CMAKE_PROJECT_NAME}.bin
    WORKING_DIRECTORY ${build_dir}
    DEPENDS gen_project_binary
    COMMENT "Generating Zigbee OTA file (version ${PROJECT_VER})"
    VERBATIM)
