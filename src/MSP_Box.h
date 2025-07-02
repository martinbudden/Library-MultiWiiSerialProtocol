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
    void init(
        bool accelerometerAvailable,
        bool inflightAccCalibrationEnabled,
        bool mspOverrideEnabled,
        bool airModeEnabled,
        bool antiGravityEnabled
    );
public:
    struct msp_box_t {
        const uint8_t boxId;
        const uint8_t permanentId; // permanent ID used to identify BOX. This ID is unique for one function, DO NOT REUSE IT
        const char *boxName;       // GUI-readable box name
    };
    // Each page contains at most 32 boxes
    enum { MAX_BOXES_PER_PAGE = 32 };
    enum { PERMANENT_ID_NONE = 255 };
    enum  boxId_e {
        // ARM flag
        BOX_ARM = 0,
        // Flight modes
        BOX_ANGLE,
        BOX_HORIZON,
        BOX_MAG,
        BOX_ALTHOLD,
        BOX_HEADFREE,
        BOX_PASSTHRU,
        BOX_FAILSAFE,
        BOX_GPS_RESCUE,
        BOX_ID_FLIGHTMODE_LAST = BOX_GPS_RESCUE,

    // When new flight modes are added, the parameter group version for 'modeActivationConditions' in src/main/fc/rc_modes.c has to be incremented to ensure that the RC modes configuration is reset.

        // RCMODE flags
        BOX_ANTIGRAVITY,
        BOX_HEADADJ,
        BOX_CAMSTAB,
        BOX_BEEPER_ON,
        BOX_LED_LOW,
        BOX_CALIB,
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

public:
    typedef int serializeBoxFn(StreamBuf& dst, const msp_box_t* box);
public:
    static const msp_box_t* findBoxByBoxId(boxId_e boxId);
    static const msp_box_t* findBoxByPermanentId(uint8_t permanentId);

    static int serializeBoxName(StreamBuf& dst, const msp_box_t* box);
    void serializeBoxReplyBoxName(StreamBuf& dst, size_t page) const;
    void serializeBoxReplyPermanentId(StreamBuf& dst, size_t page) const;
protected:
    bool getActiveBoxId(boxId_e boxId) const;
    std::bitset<BOX_COUNT> _activeBoxIds {};
    static const std::array<MSP_Box::msp_box_t, MSP_Box::BOX_COUNT> boxes;
};
