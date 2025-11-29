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

#include <StreamBufReader.h>

struct serialPort_t;

enum {
    PROTOCOL_SIMONK = 0,
    PROTOCOL_BLHELI = 1,
    PROTOCOL_KISS = 2,
    PROTOCOL_KISSALL = 3,
    PROTOCOL_CASTLE = 4,
    PROTOCOL_COUNT
};

class MSP_Base {
public:
    enum { V2_FRAME_ID = 255 };

    enum version_e {
        V1          = 0,
        V2_OVER_V1  = 1,
        V2_NATIVE   = 2,
        VERSION_COUNT
    };

    // return positive for ACK, negative on error, zero for no reply
    enum result_e {
        RESULT_ACK = 1,
        RESULT_ERROR = -1,
        RESULT_NO_REPLY = 0,
        RESULT_CMD_UNKNOWN = -2,   // don't know how to process command, try next handler
    };

    enum direction_e {
        DIRECTION_REPLY = 0,
        DIRECTION_REQUEST = 1
    };

    enum {
        REBOOT_FIRMWARE = 0,
        REBOOT_BOOTLOADER_ROM,
        REBOOT_MSC,
        REBOOT_MSC_UTC,
        REBOOT_BOOTLOADER_FLASH,
        REBOOT_COUNT,
    };

    enum SDCardState_e {
        SDCARD_STATE_NOT_PRESENT = 0,
        SDCARD_STATE_FATAL       = 1,
        SDCARD_STATE_CARD_INIT   = 2,
        SDCARD_STATE_FS_INIT     = 3,
        SDCARD_STATE_READY       = 4
    };

    enum SDCardFlags_e {
        SDCARD_FLAG_SUPPORTED   = 1
    };

    enum flashFsFlags_e {
        FLASHFS_FLAG_READY       = 1,
        FLASHFS_FLAG_SUPPORTED  = 2
    };

    enum  passthroughType_e {
        PASSTHROUGH_ESC_SIMONK = PROTOCOL_SIMONK,
        PASSTHROUGH_ESC_BLHELI = PROTOCOL_BLHELI,
        PASSTHROUGH_ESC_KISS = PROTOCOL_KISS,
        PASSTHROUGH_ESC_KISSALL = PROTOCOL_KISSALL,
        PASSTHROUGH_ESC_CASTLE = PROTOCOL_CASTLE,

        PASSTHROUGH_SERIAL_ID = 0xFD,
        PASSTHROUGH_SERIAL_FUNCTION_ID = 0xFE,

        PASSTHROUGH_ESC_4WAY = 0xFF,
    };

    struct packet_t {
        StreamBuf payload;  // payload only, ie no header or crc
        int16_t cmd;
        int16_t result;
        uint8_t flags;      // MSPv2 flags byte
        uint8_t direction;  // Currently unused
    };

    struct const_packet_t {
        StreamBufReader payload;  // payload only, ie no header or crc
        int16_t cmd;
        int16_t result;
        uint8_t flags;      // MSPv2 flags byte
        uint8_t direction;  // Currently unused
    };
    typedef int descriptor_t;

public:
    // post process function pointer
    typedef void (MSP_Base::*postProcessFnPtr)(serialPort_t* port); // msp post process function, used for gracefully handling reboots, etc.

public:
    MSP_Base() = default;
    virtual ~MSP_Base() = default;

    virtual void rebootFn(serialPort_t* serialPort);

    virtual result_e setPassthroughCommand(StreamBuf& dst, StreamBufReader& src, postProcessFnPtr* postProcessFn);

    virtual result_e processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn);
    virtual result_e processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn, StreamBufReader& src);

    virtual result_e processInCommand(int16_t cmdMSP, StreamBufReader& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn);

    virtual void processReply(const packet_t& reply);

    virtual result_e processCommand(const const_packet_t& cmd, packet_t& reply, descriptor_t srcDesc, postProcessFnPtr* postProcessFn);
protected:
    uint8_t _passthroughMode {};
    uint8_t _passthroughArgument {};
    uint8_t _rebootMode {};
};
