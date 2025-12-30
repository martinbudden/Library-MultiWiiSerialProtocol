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

#include "MSP_Box.h"

#include <StreamBuf.h>
#include <cstring>

//!!std::bitset<MSP_Box::BOX_COUNT> rcModeActivationMask {};

// permanent IDs must uniquely identify BOX meaning, DO NOT REUSE THEM!
const std::array<MSP_Box::box_t, MSP_Box::BOX_COUNT> MSP_Box::boxes = {{
    { .id = BOX_ARM,         .permanentId = 0, .name = "ARM" },
    { .id = BOX_ANGLE,       .permanentId = 1, .name = "ANGLE" },
    { .id = BOX_HORIZON,     .permanentId = 2, .name = "HORIZON" },
    { .id = BOX_ALTHOLD,     .permanentId = 3, .name = "ALTHOLD" },
    { .id = BOX_ANTIGRAVITY, .permanentId = 4, .name = "ANTI GRAVITY" },
    { .id = BOX_MAG,         .permanentId = 5, .name = "MAG" },
    { .id = BOX_HEADFREE,    .permanentId = 6, .name = "HEADFREE" },
    { .id = BOX_HEADADJ,     .permanentId = 7, .name = "HEADADJ" },
    { .id = BOX_CAMSTAB,     .permanentId = 8, .name = "CAMSTAB" },
    { .id = BOX_PASSTHRU,    .permanentId = 12, .name = "PASSTHRU" },
    { .id = BOX_BEEPER_ON,   .permanentId = 13, .name = "BEEPER" },
    { .id = BOX_LED_LOW,     .permanentId = 15, .name = "LEDLOW" },
    { .id = BOX_CALIBRATE,   .permanentId = 17, .name = "CALIBRATE" },
    { .id = BOX_OSD,         .permanentId = 19, .name = "OSD DISABLE" },
    { .id = BOX_TELEMETRY,   .permanentId = 20, .name = "TELEMETRY" },
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

const MSP_Box::box_t* MSP_Box::findBoxByBoxId(id_e boxId)
{
    for (const box_t& box : boxes) {
        // cppcheck-suppress useStlAlgorithm
        if (box.id == boxId) {
            return &box;
        }
    }
    return nullptr;
}

const MSP_Box::box_t* MSP_Box::findBoxByPermanentId(uint8_t permanentId)
{
    for (const box_t& box : boxes) {
        // cppcheck-suppress useStlAlgorithm
        if (box.permanentId == permanentId) {
            return &box;
        }
    }
    return nullptr;
}

bool MSP_Box::getActiveBoxId(id_e boxId) const
{
    return _activeBoxIds[boxId];
}

int MSP_Box::serializeBoxName(StreamBuf& dst, const box_t* box) // box may be nullptr
{
    if (box == nullptr) {
        return -1;
    }
#if defined(LIBRARY_MULTI_WII_SERIAL_PROTOCOL_USE_CUSTOM_BOX_NAMES)
    const char* name = nullptr;
    size_t len {};
    if (box->id >= BOX_USER1 && box->id <= BOX_USER4) {
        const int n = box->id - BOX_USER1;
        name = modeActivationConfig()->box_user_names[n];
        // possibly there is no '\0' in boxname
        if (*name) {
            len = strnlen(name, sizeof(modeActivationConfig()->box_user_names[n]));
        } else {
            name = nullptr;
        }
    }
    if (name == nullptr) {
        name = box->name;
        len = strlen(name);
    }
#else
    const char* name = box->name;
    size_t len = strlen(name);
#endif
    if (dst.bytesRemaining() < len + 1) {
        // boxname or separator won't fit
        return -1;
    }
    dst.writeData(name, len);
    dst.writeU8(';');
    return static_cast<int>(len) + 1;
}

void MSP_Box::serializeBoxReplyBoxName(StreamBuf& dst, size_t page) const
{
    size_t boxIndex = 0;
    const size_t pageStart = page * MAX_BOXES_PER_PAGE;
    const size_t pageEnd = pageStart + MAX_BOXES_PER_PAGE;
    for (size_t id = 0; id < BOX_COUNT; ++id) {
        const auto boxId = static_cast<id_e>(id);
        if (getActiveBoxId(boxId)) {
            if (boxIndex >= pageStart && boxIndex < pageEnd) {
                if (serializeBoxName(dst, findBoxByBoxId(boxId)) < 0) {
                    // failed to serialize, abort
                    return;
                }
            }
        ++boxIndex; // count active boxes
        }
    }
}
void MSP_Box::serializeBoxReplyPermanentId(StreamBuf& dst, size_t page) const
{
    size_t boxIndex = 0;
    const size_t pageStart = page * MAX_BOXES_PER_PAGE;
    const size_t pageEnd = pageStart + MAX_BOXES_PER_PAGE;
    for (size_t id = 0; id < BOX_COUNT; ++id) {
        const auto boxId = static_cast<id_e>(id);
        if (getActiveBoxId(boxId)) {
            if (boxIndex >= pageStart && boxIndex < pageEnd) {
                const box_t* box = findBoxByBoxId(boxId);
                if (box == nullptr || dst.bytesRemaining() < 1) {
                    // failed to serialize, abort
                    return;
                }
                dst.writeU8(box->permanentId);
            }
        ++boxIndex; // count active boxes
        }
    }
}

void MSP_Box::init(bool accelerometerAvailable, bool inflightAccCalibrationEnabled, bool mspOverrideEnabled, bool airModeEnabled, bool antiGravityEnabled) // NOLINT(readability-convert-member-functions-to-static) false positive
{
    bitset_t enabled {};

    enabled.set(BOX_ARM);
    enabled.set(BOX_PREARM);
    if (!airModeEnabled) {
        enabled.set(BOX_AIRMODE);
    }

    if (!antiGravityEnabled) {
        enabled.set(BOX_ANTIGRAVITY);
    }

    if (accelerometerAvailable) {
        enabled.set(BOX_ANGLE);
        enabled.set(BOX_HORIZON);
        enabled.set(BOX_ALTHOLD);
        enabled.set(BOX_HEADFREE);
        enabled.set(BOX_HEADADJ);
        enabled.set(BOX_FPV_ANGLE_MIX);
        if (inflightAccCalibrationEnabled) {
            enabled.set(BOX_CALIBRATE);
        }
        enabled.set(BOX_ACRO_TRAINER);
    }

    enabled.set(BOX_FAILSAFE);

    enabled.set(BOX_BEEPER_ON);
    enabled.set(BOX_BEEPER_MUTE);

    enabled.set(BOX_PARALYZE);

    if (mspOverrideEnabled) {
        enabled.set(BOX_MSP_OVERRIDE);
    }

    enabled.set(BOX_STICK_COMMAND_DISABLE);
    enabled.set(BOX_READY);

    // check that all enabled IDs are in boxes array (check may be skipped when using findBoxById() functions)
    for (size_t boxId = 0;  boxId < BOX_COUNT; ++boxId) {
        if (enabled[boxId] && findBoxByBoxId(static_cast<id_e>(boxId)) == nullptr) {
            enabled.reset(boxId); // this should not happen, but handle it gracefully
        }
    }
    _activeBoxIds = enabled;
}
