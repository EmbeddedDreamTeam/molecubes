# Seleziona sorgenti in base al target
# NON ELIMINARE NON È AUTOGENERATO DA PLATFORMIO
if(CONFIG_IDF_TARGET_ESP32C3)

    if(TEST_CINEMATICS)
        message(STATUS "Building TEST_CINEMATICS for ESP32-C3")

        FILE(GLOB_RECURSE app_sources 
            ${CMAKE_SOURCE_DIR}/tests/esp32-c3/cinematics/*.*
            ${CMAKE_SOURCE_DIR}/lib/*.*)
    elseif(TEST_PROTOCOL_MANAGER)
        message(STATUS "Building TEST_PROTOCOL_MANAGER for ESP32-C3")

        FILE(GLOB_RECURSE app_sources 
            ${CMAKE_SOURCE_DIR}/tests/esp32-c3/protocol_manager/*.*
            ${CMAKE_SOURCE_DIR}/lib/*.*)
    elseif(C3-ROOT)
        FILE(GLOB_RECURSE app_sources 
            ${CMAKE_SOURCE_DIR}/src/esp32-s3/*.* 
            ${CMAKE_SOURCE_DIR}/lib/*.*)
    else()
        message(STATUS "Building NORMAL ESP32-C3")

        FILE(GLOB_RECURSE app_sources 
            ${CMAKE_SOURCE_DIR}/src/esp32-c3/*.*
            ${CMAKE_SOURCE_DIR}/lib/*.*)

    endif()
elseif(CONFIG_IDF_TARGET_ESP32S3)
    FILE(GLOB_RECURSE app_sources 
        ${CMAKE_SOURCE_DIR}/src/esp32-s3/*.* 
        ${CMAKE_SOURCE_DIR}/lib/*.*)
endif()

idf_component_register(SRCS ${app_sources}
                       INCLUDE_DIRS "../include")