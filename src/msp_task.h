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

#include <task_base.h>

class MspSerial;

struct msp_context_t;


class MspTask : public TaskBase {
public:
    MspTask(uint32_t task_interval_microseconds, MspSerial& msp_serial, msp_context_t& context);
public:
    static MspTask* create_task(task_info_t& task_info, MspSerial& msp_serial, msp_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
    static MspTask* create_task(MspSerial& msp_serial, msp_context_t& context, uint8_t priority, uint32_t core, uint32_t task_interval_microseconds);
private:
    // class is not copyable or moveable
    MspTask(const MspTask&) = delete;
    MspTask& operator=(const MspTask&) = delete;
    MspTask(MspTask&&) = delete;
    MspTask& operator=(MspTask&&) = delete;
public:
    [[noreturn]] static void task_static(void* arg);
    void loop();
private:
    [[noreturn]] void task();
private:
    uint32_t _task_interval_milliseconds;
    MspSerial& _msp_serial;
    msp_context_t& _context;
};
