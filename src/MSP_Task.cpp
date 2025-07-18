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

#include <TimeMicroSeconds.h>

#if defined(USE_FREERTOS)
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#endif

/*!
loop() function for when not using FREERTOS
*/
void MSP_Task::loop()
{
    const uint32_t timeMicroSeconds = timeUs();
    _timeMicroSecondsDelta = timeMicroSeconds - _timeMicroSecondsPrevious;

    if (_timeMicroSecondsDelta >= _taskIntervalMicroSeconds) { // if _taskIntervalMicroSeconds has passed, then run the update
        _timeMicroSecondsPrevious = timeMicroSeconds;
        _mspSerial.processInput();
    }
}

/*!
Task function for the MSP. Sets up and runs the task loop() function.
*/
[[noreturn]] void MSP_Task::task() // NOLINT(readability-convert-member-functions-to-static)
{
#if defined(USE_FREERTOS)
    // pdMS_TO_TICKS Converts a time in milliseconds to a time in ticks.
    const uint32_t taskIntervalTicks = pdMS_TO_TICKS(_taskIntervalMicroSeconds / 1000);
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
#endif // USE_FREERTOS
}

/*!
Wrapper function for MSP_Task::Task with the correct signature to be used in xTaskCreate.
*/
[[noreturn]] void MSP_Task::Task(void* arg)
{
    const TaskBase::parameters_t* parameters = static_cast<TaskBase::parameters_t*>(arg); // NOLINT(cppcoreguidelines-init-variables) false positive

    static_cast<MSP_Task*>(parameters->task)->task(); // NOLINT(cppcoreguidelines-pro-type-static-cast-downcast)
}
