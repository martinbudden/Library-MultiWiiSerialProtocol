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

#include "MSP_Task.h"

#if defined(USE_DEBUG_PRINTF_TASK_INFORMATION)
#if defined(USE_ESPNOW)
#include <HardwareSerial.h>
#endif
#endif

#include <array>
#include <cstring>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#endif


MSP_Task* MSP_Task::createTask(task_info_t& taskInfo, MSP_SerialBase& mspSerial, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds) // NOLINT(readability-convert-member-functions-to-static)
{
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static MSP_Task mspTask(taskIntervalMicroSeconds, mspSerial);

#if defined(USE_FREERTOS)
    // Note that task parameters must not be on the stack, since they are used when the task is started, which is after this function returns.
    static TaskBase::parameters_t taskParameters { // NOLINT(misc-const-correctness) false positive
        .task = &mspTask
    };
#if !defined(MSP_TASK_STACK_DEPTH_BYTES)
    enum { MSP_TASK_STACK_DEPTH_BYTES = 4096 };
#endif
    static std::array <StackType_t, MSP_TASK_STACK_DEPTH_BYTES> stack;
    static StaticTask_t taskBuffer;
    taskInfo = {
        .taskHandle = nullptr,
        .name = "MSP_Task",
        .stackDepth = MSP_TASK_STACK_DEPTH_BYTES,
        .stackBuffer = &stack[0],
        .priority = priority,
        .coreID = coreID,
    };
    const TaskHandle_t taskHandle = xTaskCreateStaticPinnedToCore(
        MSP_Task::Task,
        taskInfo.name,
        taskInfo.stackDepth,
        &taskParameters,
        taskInfo.priority,
        taskInfo.stackBuffer,
        &taskBuffer,
        taskInfo.coreID
    );
    assert(taskHandle != nullptr && "Unable to create MSP task.");
#if defined(USE_DEBUG_PRINTF_TASK_INFORMATION)
#if !defined(FRAMEWORK_ESPIDF)
    std::array<char, 128> buf;
    sprintf(&buf[0], "**** MSP_Task,      core:%u, priority:%u, task interval:%ums\r\n", coreID, priority, taskIntervalMicroSeconds / 1000);
    Serial.print(&buf[0]);
#endif
#endif
#else
    (void)taskInfo;
    (void)priority;
    (void)coreID;
#endif // USE_FREERTOS
    return &mspTask;
}

MSP_Task* MSP_Task::createTask(MSP_SerialBase& mspSerial, uint8_t priority, uint8_t coreID, uint32_t taskIntervalMicroSeconds) // NOLINT(readability-convert-member-functions-to-static)
{
    task_info_t taskInfo {}; // NOLINT(cppcoreguidelines-init-variables) false positive
    return createTask(taskInfo, mspSerial, priority, coreID, taskIntervalMicroSeconds);
}
