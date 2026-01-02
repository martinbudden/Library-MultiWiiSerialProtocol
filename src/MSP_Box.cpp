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
