#pragma once

#include <cstdint>
#include "cmsis_os.h"

//==================================================================================================
// Task Configuration Constants
//==================================================================================================

// Task delay times (milliseconds)
static constexpr uint32_t VEHICLE_TASK_DELAY_MS = 2;
static constexpr uint32_t TRANSFORM_TASK_DELAY_MS = 10;
static constexpr uint32_t SUCTION_TASK_DELAY_MS = 20;
static constexpr uint32_t IMU_TASK_DELAY_MS = 1;
static constexpr uint32_t SELFTEST_TASK_DELAY_MS = 100;
static constexpr uint32_t LED_TASK_DELAY_MS = 1;
static constexpr uint32_t COMMUNICATION_TASK_DELAY_MS = 50;
static constexpr uint32_t DEFAULT_TASK_DELAY_MS = 100;

// Task stack sizes
static constexpr uint32_t TASK_STACK_SIZE = 256 * 4;

// Task attributes template
#define DECLARE_TASK_ATTRIBUTES(name) \
    extern const osThreadAttr_t name##TaskAttribute; \
    extern osThreadId_t name##TaskHandle;

// Task function declarations

    void vehicleTask(void* arg);
    void transformTask(void* arg);
    void suctionTask(void* arg);
    void imuTask(void* arg);
    void selfTestTask(void* arg);
    void communicationTask(void* arg);
    void ledTask(void* arg);


// Declare all task attributes and handles
DECLARE_TASK_ATTRIBUTES(vehicle)
DECLARE_TASK_ATTRIBUTES(transform)
DECLARE_TASK_ATTRIBUTES(suction)
DECLARE_TASK_ATTRIBUTES(imu)
DECLARE_TASK_ATTRIBUTES(selfTest)
DECLARE_TASK_ATTRIBUTES(communication)
DECLARE_TASK_ATTRIBUTES(led)