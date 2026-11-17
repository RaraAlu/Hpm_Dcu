add_custom_command(TARGET ${APP_ELF_NAME} POST_BUILD
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/image_guard.py
        --elf $<TARGET_FILE:${APP_ELF_NAME}>
        --report ${CMAKE_CURRENT_BINARY_DIR}/image-check.json
    COMMENT "Checking Flash load/erase span against retained EEPROM"
    VERBATIM)

add_custom_target(check-image
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/image_guard.py
        --elf $<TARGET_FILE:${APP_ELF_NAME}>
        --report ${CMAKE_CURRENT_BINARY_DIR}/image-check.json
    DEPENDS ${APP_ELF_NAME}
    VERBATIM)

enable_testing()
add_custom_target(check-protocol
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/tools/protocol/generate.py
    COMMENT "Checking generated protocol files against their source definitions"
    VERBATIM)
add_test(NAME devtools-offline
    COMMAND ${PYTHON_EXECUTABLE} -m unittest discover -s tests/host -v)
set_tests_properties(devtools-offline PROPERTIES WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})

get_filename_component(DCU_PRESET ${CMAKE_CURRENT_BINARY_DIR} NAME)
foreach(action IN ITEMS flash verify debug-server)
    add_custom_target(${action}
        COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/dcu.py
            ${action} --preset ${DCU_PRESET} --skip-build
        DEPENDS ${APP_ELF_NAME}
        USES_TERMINAL VERBATIM)
endforeach()
