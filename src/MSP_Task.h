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

#pragma once

#include <TaskBase.h>

class MSP_Serial;


class MSP_Task : public TaskBase {
public:
    MSP_Task(uint32_t taskIntervalMicroseconds, MSP_Serial& mspSerial);
public:
    static MSP_Task* createTask(task_info_t& taskInfo, MSP_Serial& mspSerial, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
    static MSP_Task* createTask(MSP_Serial& mspSerial, uint8_t priority, uint32_t core, uint32_t taskIntervalMicroseconds);
private:
    // class is not copyable or moveable
    MSP_Task(const MSP_Task&) = delete;
    MSP_Task& operator=(const MSP_Task&) = delete;
    MSP_Task(MSP_Task&&) = delete;
    MSP_Task& operator=(MSP_Task&&) = delete;
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    uint32_t _taskIntervalMilliseconds;
    MSP_Serial& _mspSerial;
};
