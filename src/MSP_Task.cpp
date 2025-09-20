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

#include "MSP_SerialBase.h"
#include "MSP_Task.h"

#include <TimeMicroseconds.h>
#include <cassert>

#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <task.h>
#endif
#endif


MSP_Task::MSP_Task(uint32_t taskIntervalMicroseconds, MSP_SerialBase& mspSerial) :
    TaskBase(taskIntervalMicroseconds),
    _taskIntervalMilliseconds(taskIntervalMicroseconds/1000), // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    _mspSerial(mspSerial)
{
}

/*!
loop() function for when not using FREERTOS
*/
void MSP_Task::loop()
{
    const uint32_t tickCount = timeMs();
    _tickCountDelta = tickCount - _tickCountPrevious;

    if (_tickCountDelta >= _taskIntervalMilliseconds) { // if _taskIntervalMicroseconds has passed, then run the update
        _tickCountPrevious = tickCount;
        _mspSerial.processInput();
    }
}

/*!
Task function for the MSP. Sets up and runs the task loop() function.
*/
[[noreturn]] void MSP_Task::task() // NOLINT(readability-convert-member-functions-to-static)
{
#if defined(FRAMEWORK_USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    const uint32_t taskIntervalTicks = pdMS_TO_TICKS(_taskIntervalMicroseconds / 1000);
    assert(taskIntervalTicks > 0 && "MSP taskIntervalTicks is zero.");

    _previousWakeTimeTicks = xTaskGetTickCount();
    while (true) {
        // delay until the end of the next taskIntervalTicks
        vTaskDelayUntil(&_previousWakeTimeTicks, taskIntervalTicks);
        // calculate _tickCountDelta to get actual deltaT value, since we may have been delayed for more than taskIntervalTicks
        const TickType_t tickCount = xTaskGetTickCount();
        _tickCountDelta = tickCount - _tickCountPrevious;
        _tickCountPrevious = tickCount;

        if (_tickCountDelta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            _mspSerial.processInput();
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for MSP_Task::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void MSP_Task::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg); // NOLINT(cppcoreguidelines-init-variables) false positive

    static_cast<MSP_Task*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
