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

class MspSerial;

struct msp_parameter_group_t;


class MspTask : public TaskBase {
public:
    MspTask(uint32_t task_interval_microseconds, MspSerial& msp_serial, msp_parameter_group_t& parameter_group);
public:
    static MspTask* create_task(task_info_t& taskInfo, MspSerial& msp_serial, msp_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static MspTask* create_task(MspSerial& msp_serial, msp_parameter_group_t& parameter_group, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
private:
    // class is not copyable or moveable
    MspTask(const MspTask&) = delete;
    MspTask& operator=(const MspTask&) = delete;
    MspTask(MspTask&&) = delete;
    MspTask& operator=(MspTask&&) = delete;
public:
    [[noreturn]] static void Task(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    uint32_t _taskIntervalMilliseconds;
    MspSerial& _msp_serial;
    msp_parameter_group_t& _parameter_group;
};
