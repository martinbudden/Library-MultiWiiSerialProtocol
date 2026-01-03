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
 *
 * The MultiWiiSerialProtocol library is a port (modification) of the MultiWii Serial Protocol
 * implementation in Betaflight (which itself was a port of the Cleanflight implementation).
 *
 * The original Betaflight copyright notice is included below, as per the GNU GPL
 * "keep intact all notices” requirement.
 */

/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
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

#include <array>
#include <bitset>
#include <cstdint>

class StreamBuf;

class MSP_Box {
public:
    MSP_Box() = default;
public:
    struct box_t {
        const uint8_t id;
        const uint8_t permanentId; // permanent ID used to identify BOX. This ID is unique for one function, DO NOT REUSE IT
        const char *name;       // GUI-readable box name
    };
    // Each page contains at most 32 boxes
    enum { MAX_BOXES_PER_PAGE = 32 };
    enum { PERMANENT_ID_NONE = 255 };
    enum  id_e {
        // ARM flag
        BOX_ARM = 0,
        // Flight modes
        BOX_ANGLE,
        BOX_HORIZON,
        BOX_MAG,
        BOX_ALTITUDE_HOLD,
        BOX_POSITION_HOLD,
        BOX_HEADFREE,
        BOX_CHIRP,
        BOX_PASSTHRU,
        BOX_FAILSAFE,
        BOX_GPS_RESCUE,
        BOX_ID_FLIGHTMODE_COUNT,

    // When new flight modes are added, the parameter group version for 'modeActivationConditions' in src/main/fc/rc_modes.c has to be incremented to ensure that the RC modes configuration is reset.

        // RC_Mode flags
        BOX_ANTIGRAVITY = BOX_ID_FLIGHTMODE_COUNT,
        BOX_HEADADJ,
        BOX_CAMSTAB,
        BOX_BEEPER_ON,
        BOX_LED_LOW,
        BOX_CALIBRATE,
        BOX_OSD,
        BOX_TELEMETRY,
        BOX_SERVO1,
        BOX_SERVO2,
        BOX_SERVO3,
        BOX_BLACKBOX,
        BOX_AIRMODE,
        BOX_3D,
        BOX_FPV_ANGLE_MIX,
        BOX_BLACKBOX_ERASE,
        BOX_CAMERA1,
        BOX_CAMERA2,
        BOX_CAMERA3,
        BOX_CRASH_FLIP,
        BOX_PREARM,
        BOX_BEEP_GPS_COUNT,
        BOX_VTX_PIT_MODE,
        BOX_PARALYZE,
        BOX_USER1,
        BOX_USER2,
        BOX_USER3,
        BOX_USER4,
        BOX_PID_AUDIO,
        BOX_ACRO_TRAINER,
        BOX_VTX_CONTROL_DISABLE,
        BOX_LAUNCH_CONTROL,
        BOX_MSP_OVERRIDE,
        BOX_STICK_COMMAND_DISABLE,
        BOX_BEEPER_MUTE,
        BOX_READY,
        BOX_LAP_TIMER_RESET,
        BOX_COUNT // number of boxes
    };
    typedef std::bitset<BOX_COUNT> bitset_t;
    typedef int serializeBoxFn(StreamBuf& dst, const box_t* box);
public:
    static const box_t* findBoxByBoxId(id_e boxId);
    static const box_t* findBoxByPermanentId(uint8_t permanentId);

    static int serializeBoxName(StreamBuf& dst, const box_t* box);
    void serializeBoxReplyBoxName(StreamBuf& dst, size_t page) const;
    void serializeBoxReplyPermanentId(StreamBuf& dst, size_t page) const;
    bool getActiveBoxId(id_e boxId) const;
    void setActiveBoxId(id_e boxId);
    void resetActiveBoxId(id_e boxId);
protected:
    std::bitset<BOX_COUNT> _activeBoxIds {};
    // permanent IDs must uniquely identify BOX meaning, DO NOT REUSE THEM!
    static constexpr std::array<box_t, BOX_COUNT> boxes = {{
        { .id = BOX_ARM,         .permanentId = 0,  .name = "ARM" },
        { .id = BOX_ANGLE,       .permanentId = 1,  .name = "ANGLE" },
        { .id = BOX_HORIZON,     .permanentId = 2,  .name = "HORIZON" },
        { .id = BOX_ALTITUDE_HOLD, .permanentId = 3, .name = "ALTHOLD" },
        { .id = BOX_ANTIGRAVITY, .permanentId = 4,  .name = "ANTI GRAVITY" },
        { .id = BOX_MAG,         .permanentId = 5,  .name = "MAG" },
        { .id = BOX_HEADFREE,    .permanentId = 6,  .name = "HEADFREE" },
        { .id = BOX_HEADADJ,     .permanentId = 7,  .name = "HEADADJ" },
        { .id = BOX_CAMSTAB,     .permanentId = 8,  .name = "CAMSTAB" },
//      { .id = BOX_CAM_TRIG,    .permanentId = 9,  .name = "CAM_TRIG", },
//      { .id = BOX_GPS_HOME,    .permanentId = 10, .name = "GPS HOME" },
        { .id = BOX_POSITION_HOLD, .permanentId = 11, .name = "POS HOLD" },
        { .id = BOX_PASSTHRU,    .permanentId = 12, .name = "PASSTHRU" },
        { .id = BOX_BEEPER_ON,   .permanentId = 13, .name = "BEEPER" },
//      { .id = BOX_LED_MAX,     .permanentId = 14, .name = "LEDMAX" }, (removed)
        { .id = BOX_LED_LOW,     .permanentId = 15, .name = "LEDLOW" },
//      { .id = BOX_LLIGHTS,     .permanentId = 16, .name = "LLIGHTS" }, (removed)
        { .id = BOX_CALIBRATE,   .permanentId = 17, .name = "CALIBRATE" },
//      { .id = BOX_GOVERNOR,    .permanentId = 18, .name = "GOVERNOR" }, (removed)
        { .id = BOX_OSD,         .permanentId = 19, .name = "OSD DISABLE" },
        { .id = BOX_TELEMETRY,   .permanentId = 20, .name = "TELEMETRY" },
//      { .id = BOX_GTUNE,       .permanentId = 21, .name = "GTUNE" }, (removed)
//      { .id = BOX_RANGEFINDER, .permanentId = 22, .name = "RANGEFINDER" }, (removed)
        { .id = BOX_SERVO1,      .permanentId = 23, .name = "SERVO1" },
        { .id = BOX_SERVO2,      .permanentId = 24, .name = "SERVO2" },
        { .id = BOX_SERVO3,      .permanentId = 25, .name = "SERVO3" },
        { .id = BOX_BLACKBOX,    .permanentId = 26, .name = "BLACKBOX_" },
        { .id = BOX_FAILSAFE,    .permanentId = 27, .name = "FAILSAFE" },
        { .id = BOX_AIRMODE,     .permanentId = 28, .name = "AIR MODE" },
        { .id = BOX_3D,          .permanentId = 29, .name = "3D DISABLE / SWITCH" },
        { .id = BOX_FPV_ANGLE_MIX, .permanentId = 30, .name = "FPV ANGLE MIX" },
        { .id = BOX_BLACKBOX_ERASE, .permanentId = 31, .name = "BLACKBOX_ ERASE" },
        { .id = BOX_CAMERA1,     .permanentId = 32, .name = "CAMERA CONTROL 1" },
        { .id = BOX_CAMERA2,     .permanentId = 33, .name = "CAMERA CONTROL 2" },
        { .id = BOX_CAMERA3,     .permanentId = 34, .name = "CAMERA CONTROL 3" },
        { .id = BOX_CRASH_FLIP,  .permanentId = 35, .name = "FLIP OVER AFTER CRASH" },
        { .id = BOX_PREARM,      .permanentId = 36, .name = "PREARM" },
        { .id = BOX_BEEP_GPS_COUNT, .permanentId = 37, .name = "GPS BEEP SATELLITE COUNT" },
//      { .id = BOX3D_ON_A_SWITCH,.permanentId= 38, .name = "3D ON A SWITCH", }, (removed)
        { .id = BOX_VTX_PIT_MODE, .permanentId = 39, .name = "VTX PIT MODE" },
        { .id = BOX_USER1,       .permanentId = 40, .name = "USER1" }, // may be overridden
        { .id = BOX_USER2,       .permanentId = 41, .name = "USER2" },
        { .id = BOX_USER3,       .permanentId = 42, .name = "USER3" },
        { .id = BOX_USER4,       .permanentId = 43, .name = "USER4" },
        { .id = BOX_PID_AUDIO,   .permanentId = 44, .name = "PID AUDIO" },
        { .id = BOX_PARALYZE,    .permanentId = 45, .name = "PARALYZE" },
        { .id = BOX_GPS_RESCUE,  .permanentId = 46, .name = "GPS RESCUE" },
        { .id = BOX_ACRO_TRAINER, .permanentId = 47, .name = "ACRO TRAINER" },
        { .id = BOX_VTX_CONTROL_DISABLE, .permanentId = 48, .name = "VTX CONTROL DISABLE" },
        { .id = BOX_LAUNCH_CONTROL, .permanentId = 49, .name = "LAUNCH CONTROL" },
        { .id = BOX_MSP_OVERRIDE, .permanentId = 50, .name = "MSP OVERRIDE" },
        { .id = BOX_STICK_COMMAND_DISABLE, .permanentId = 51, .name = "STICK COMMANDS DISABLE" },
        { .id = BOX_BEEPER_MUTE, .permanentId = 52, .name = "BEEPER MUTE" },
        { .id = BOX_READY,       .permanentId = 53, .name = "READY" },
        { .id = BOX_LAP_TIMER_RESET, .permanentId = 54, .name = "LAP TIMER RESET" },
    }};
};
