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

void MSP_Box::setActiveBoxId(id_e boxId) // NOLINT(readability-convert-member-functions-to-static)
{
    if (findBoxByBoxId(boxId) != nullptr) {
        _activeBoxIds.set(boxId);
    }
}

void MSP_Box::resetActiveBoxId(id_e boxId)
{
    _activeBoxIds.reset(boxId);
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
