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
const std::array<MSP_Box::msp_box_t, MSP_Box::BOX_COUNT> MSP_Box::boxes = {{
    { .boxId = BOX_ARM,         .permanentId = 0, .boxName = "ARM" },
    { .boxId = BOX_ANGLE,       .permanentId = 1, .boxName = "ANGLE" },
    { .boxId = BOX_HORIZON,     .permanentId = 2, .boxName = "HORIZON" },
    { .boxId = BOX_ALTHOLD,     .permanentId = 3, .boxName = "ALTHOLD" },
    { .boxId = BOX_ANTIGRAVITY, .permanentId = 4, .boxName = "ANTI GRAVITY" },
    { .boxId = BOX_MAG,         .permanentId = 5, .boxName = "MAG" },
    { .boxId = BOX_HEADFREE,    .permanentId = 6, .boxName = "HEADFREE" },
    { .boxId = BOX_HEADADJ,     .permanentId = 7, .boxName = "HEADADJ" },
    { .boxId = BOX_CAMSTAB,     .permanentId = 8, .boxName = "CAMSTAB" },
    { .boxId = BOX_PASSTHRU,    .permanentId = 12, .boxName = "PASSTHRU" },
    { .boxId = BOX_BEEPER_ON,   .permanentId = 13, .boxName = "BEEPER" },
    { .boxId = BOX_LED_LOW,     .permanentId = 15, .boxName = "LEDLOW" },
    { .boxId = BOX_CALIBRATE,   .permanentId = 17, .boxName = "CALIBRATE" },
    { .boxId = BOX_OSD,         .permanentId = 19, .boxName = "OSD DISABLE" },
    { .boxId = BOX_TELEMETRY,   .permanentId = 20, .boxName = "TELEMETRY" },
    { .boxId = BOX_SERVO1,      .permanentId = 23, .boxName = "SERVO1" },
    { .boxId = BOX_SERVO2,      .permanentId = 24, .boxName = "SERVO2" },
    { .boxId = BOX_SERVO3,      .permanentId = 25, .boxName = "SERVO3" },
    { .boxId = BOX_BLACKBOX,    .permanentId = 26, .boxName = "BLACKBOX_" },
    { .boxId = BOX_FAILSAFE,    .permanentId = 27, .boxName = "FAILSAFE" },
    { .boxId = BOX_AIRMODE,     .permanentId = 28, .boxName = "AIR MODE" },
    { .boxId = BOX_3D,          .permanentId = 29, .boxName = "3D DISABLE / SWITCH" },
    { .boxId = BOX_FPV_ANGLE_MIX, .permanentId = 30, .boxName = "FPV ANGLE MIX" },
    { .boxId = BOX_BLACKBOX_ERASE, .permanentId = 31, .boxName = "BLACKBOX_ ERASE" },
    { .boxId = BOX_CAMERA1,     .permanentId = 32, .boxName = "CAMERA CONTROL 1" },
    { .boxId = BOX_CAMERA2,     .permanentId = 33, .boxName = "CAMERA CONTROL 2" },
    { .boxId = BOX_CAMERA3,     .permanentId = 34, .boxName = "CAMERA CONTROL 3" },
    { .boxId = BOX_CRASH_FLIP,  .permanentId = 35, .boxName = "FLIP OVER AFTER CRASH" },
    { .boxId = BOX_PREARM,      .permanentId = 36, .boxName = "PREARM" },
    { .boxId = BOX_BEEP_GPS_COUNT, .permanentId = 37, .boxName = "GPS BEEP SATELLITE COUNT" },
    { .boxId = BOX_VTX_PIT_MODE, .permanentId = 39, .boxName = "VTX PIT MODE" },
    { .boxId = BOX_USER1,       .permanentId = 40, .boxName = "USER1" }, // may be overridden
    { .boxId = BOX_USER2,       .permanentId = 41, .boxName = "USER2" },
    { .boxId = BOX_USER3,       .permanentId = 42, .boxName = "USER3" },
    { .boxId = BOX_USER4,       .permanentId = 43, .boxName = "USER4" },
    { .boxId = BOX_PID_AUDIO,   .permanentId = 44, .boxName = "PID AUDIO" },
    { .boxId = BOX_PARALYZE,    .permanentId = 45, .boxName = "PARALYZE" },
    { .boxId = BOX_GPS_RESCUE,  .permanentId = 46, .boxName = "GPS RESCUE" },
    { .boxId = BOX_ACRO_TRAINER, .permanentId = 47, .boxName = "ACRO TRAINER" },
    { .boxId = BOX_VTX_CONTROL_DISABLE, .permanentId = 48, .boxName = "VTX CONTROL DISABLE" },
    { .boxId = BOX_LAUNCH_CONTROL, .permanentId = 49, .boxName = "LAUNCH CONTROL" },
    { .boxId = BOX_MSP_OVERRIDE, .permanentId = 50, .boxName = "MSP OVERRIDE" },
    { .boxId = BOX_STICK_COMMAND_DISABLE, .permanentId = 51, .boxName = "STICK COMMANDS DISABLE" },
    { .boxId = BOX_BEEPER_MUTE, .permanentId = 52, .boxName = "BEEPER MUTE" },
    { .boxId = BOX_READY,       .permanentId = 53, .boxName = "READY" },
    { .boxId = BOX_LAP_TIMER_RESET, .permanentId = 54, .boxName = "LAP TIMER RESET" },
}};

const MSP_Box::msp_box_t* MSP_Box::findBoxByBoxId(box_id_e boxId)
{
    for (const msp_box_t& box : boxes) {
        // cppcheck-suppress useStlAlgorithm
        if (box.boxId == boxId) {
            return &box;
        }
    }
    return nullptr;
}

const MSP_Box::msp_box_t* MSP_Box::findBoxByPermanentId(uint8_t permanentId)
{
    for (const msp_box_t& box : boxes) {
        // cppcheck-suppress useStlAlgorithm
        if (box.permanentId == permanentId) {
            return &box;
        }
    }
    return nullptr;
}

bool MSP_Box::getActiveBoxId(box_id_e boxId) const
{
    return _activeBoxIds[boxId];
}

int MSP_Box::serializeBoxName(StreamBuf& dst, const msp_box_t* box) // box may be nullptr
{
    if (box == nullptr) {
        return -1;
    }
#if defined(LIBRARY_MULTI_WII_SERIAL_PROTOCOL_USE_CUSTOM_BOX_NAMES)
    const char* name = nullptr;
    size_t len {};
    if (box->boxId >= BOXUSER1 && box->boxId <= BOXUSER4) {
        const int n = box->boxId - BOXUSER1;
        name = modeActivationConfig()->box_user_names[n];
        // possibly there is no '\0' in boxname
        if (*name) {
            len = strnlen(name, sizeof(modeActivationConfig()->box_user_names[n]));
        } else {
            name = nullptr;
        }
    }
    if (name == nullptr) {
        name = box->boxName;
        len = strlen(name);
    }
#else
    const char* name = box->boxName;
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
        const auto boxId = static_cast<box_id_e>(id);
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
        const auto boxId = static_cast<box_id_e>(id);
        if (getActiveBoxId(boxId)) {
            if (boxIndex >= pageStart && boxIndex < pageEnd) {
                const msp_box_t* box = findBoxByBoxId(boxId);
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
        if (enabled[boxId] && findBoxByBoxId(static_cast<box_id_e>(boxId)) == nullptr) {
            enabled.reset(boxId); // this should not happen, but handle it gracefully
        }
    }
    _activeBoxIds = enabled;
}
