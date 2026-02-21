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

#include "msp_serial.h"
#include "msp_task.h"

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

#include <time_microseconds.h>


MspTask::MspTask(uint32_t task_interval_microseconds, MspSerial& msp_serial, msp_parameter_group_t& parameter_group) :
    TaskBase(task_interval_microseconds),
    _task_interval_milliseconds(task_interval_microseconds/1000), // NOLINT(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    _msp_serial(msp_serial),
    _parameter_group(parameter_group)
{
}

/*!
loop() function for when not using FREERTOS
*/
void MspTask::loop()
{
    const uint32_t tick_count = time_ms();
    _tick_count_delta = tick_count - _tick_count_previous;

    if (_tick_count_delta >= _task_interval_milliseconds) { // if _task_interval_microseconds has passed, then run the update
        _tick_count_previous = tick_count;
        _msp_serial.process_input(_parameter_group);
    }
}

/*!
Task function for the MSP. Sets up and runs the task loop() function.
*/
[[noreturn]] void MspTask::task() // NOLINT(readability-convert-member-functions-to-static)
{
#if defined(FRAMEWORK_USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    const uint32_t task_interval_ticks = pdMS_TO_TICKS(_taskIntervalMicroseconds / 1000);
    assert(task_interval_ticks > 0 && "MSP task_interval_ticks is zero.");

    _previous_wake_time_ticks = xTaskGetTickCount();
    while (true) {
        // delay until the end of the next task_interval_ticks
#if (tskKERNEL_VERSION_MAJOR > 10) || ((tskKERNEL_VERSION_MAJOR == 10) && (tskKERNEL_VERSION_MINOR >= 5))
            const BaseType_t was_delayed = xTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
            if (was_delayed) {
                _was_delayed = true;
            }
#else
            vTaskDelayUntil(&_previous_wake_time_ticks, task_interval_ticks);
#endif
        // calculate _tick_count_delta to get actual deltaT value, since we may have been delayed for more than task_interval_ticks
        const TickType_t tick_count = xTaskGetTickCount();
        _tick_count_delta = tick_count - _tick_count_previous;
        _tick_count_previous = tick_count;

        if (_tick_count_delta > 0) { // guard against the case of this while loop executing twice on the same tick interval
            _msp_serial.process_input(_parameter_group);
        }
    }
#else
    while (true) {}
#endif // FRAMEWORK_USE_FREERTOS
}

/*!
Wrapper function for MspTask::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void MspTask::task_static(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg); // NOLINT(cppcoreguidelines-init-variables) false positive

    static_cast<MspTask*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
