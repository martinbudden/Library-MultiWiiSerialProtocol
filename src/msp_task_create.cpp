/*
 * This file is part of the MultiWiiSerialProtocol library.
 *
 * The MultiWiiSerialProtocol library is free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * The MultiWiiSerialProtocol library is distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include "msp_task.h"

#include <array>
#include <cassert>
#include <cstring>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif
#endif


MspTask* MspTask::create_task(MspSerial& msp_serial, msp_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds) // NOLINT(readability-convert-member-functions-to-static)
{
    task_info_t task_info {}; // NOLINT(cppcoreguidelines-init-variables) false positive
    return create_task(task_info, msp_serial, context, priority, core, task_interval_microseconds);
}

MspTask* MspTask::create_task(task_info_t& task_info, MspSerial& msp_serial, msp_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds) // NOLINT(readability-convert-member-functions-to-static)
{
    static MspTask msp_task(task_interval_microseconds, msp_serial, context);

    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t task_parameters { // NOLINT(misc-const-correctness) false positive
        .task = &msp_task
    };
#if !defined(MSP_TASK_STACK_DEPTH_BYTES)
    enum { MSP_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32) || !defined(FRAMEWORK_USE_FREERTOS)
    static std::array <uint8_t, MSP_TASK_STACK_DEPTH_BYTES> stack;
#else
    static std::array<StackType_t, MSP_TASK_STACK_DEPTH_BYTES / sizeof(StackType_t)> stack;
#endif
    task_info = {
        .task_handle = nullptr,
        .name = "MspTask",
        .stack_depth_bytes = MSP_TASK_STACK_DEPTH_BYTES,
        .stack_buffer = reinterpret_cast<uint8_t*>(&stack[0]), // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        .priority = priority,
        .core = core,
        .task_interval_microseconds = task_interval_microseconds,
    };

#if defined(FRAMEWORK_USE_FREERTOS)
    assert(std::strlen(task_info.name) < configMAX_TASK_NAME_LEN);
    assert(task_info.priority < configMAX_PRIORITIES);

    static StaticTask_t task_buffer;
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
    task_info.task_handle = xTaskCreateStaticPinnedToCore(
        MspTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create MSP task");
#elif defined(FRAMEWORK_RPI_PICO) || defined(FRAMEWORK_ARDUINO_RPI_PICO)
    task_info.task_handle = xTaskCreateStaticAffinitySet(
        MspTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer,
        task_info.core
    );
    assert(task_info.task_handle != nullptr && "Unable to create MSP task");
#else
    task_info.task_handle = xTaskCreateStatic(
        MspTask::task_static,
        task_info.name,
        task_info.stack_depth_bytes / sizeof(StackType_t),
        &task_parameters,
        task_info.priority,
        &stack[0],
        &task_buffer
    );
    assert(task_info.task_handle != nullptr && "Unable to create MSP task");
    // vTaskCoreAffinitySet(task_info.task_handle, task_info.core);
#endif
#else
    (void)task_parameters;
#endif // FRAMEWORK_USE_FREERTOS

    return &msp_task;
}
