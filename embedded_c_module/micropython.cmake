add_library(usermod_mpu6050 INTERFACE)

target_sources(usermod_mpu6050 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/mpu6050.c
)

target_include_directories(usermod_mpu6050 INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(usermod INTERFACE usermod_mpu6050)
