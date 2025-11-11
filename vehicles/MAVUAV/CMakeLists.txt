project(MAVUAV ASM C CXX)

# 明确列出所有任务源文件
set(TASK_SOURCES
        task/Src/communication_task.cc
        task/Src/imu_task.cc
        task/Src/led_task.cc
        task/Src/selftest_task.cc
        task/Src/suction_task.cc
        task/Src/system_state_task.cc
        task/Src/transform_task.cc
        task/Src/vehicle_task.cc
)

file(GLOB_RECURSE COMMON_SOURCES
        "mode_config/*.cc"
)

# 设置包含目录
set(INCLUDE_DIRS
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/task/Inc
        ${CMAKE_CURRENT_SOURCE_DIR}/mode_config
)

# 主执行文件 - drone
irm_add_arm_executable(MAVUAV
        TARGET DJI_Board_TypeC
        SOURCES
        main.cc
        ${TASK_SOURCES}
        ${COMMON_SOURCES}
        INCLUDES ${INCLUDE_DIRS}
)