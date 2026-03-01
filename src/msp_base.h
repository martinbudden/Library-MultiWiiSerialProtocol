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

#include <stream_buf_reader.h>

struct msp_context_t;
struct serialPort_t;

enum {
    PROTOCOL_SIMONK = 0,
    PROTOCOL_BLHELI = 1,
    PROTOCOL_KISS = 2,
    PROTOCOL_KISSALL = 3,
    PROTOCOL_CASTLE = 4,
    PROTOCOL_COUNT
};

enum msp_version_e {
    MSP_V1          = 0,
    MSP_V2_OVER_V1  = 1,
    MSP_V2_NATIVE   = 2,
    MSP_VERSION_COUNT
};

// return positive for ACK, negative on error, zero for no reply
enum msp_result_e {
    MSP_RESULT_ACK = 1,
    MSP_RESULT_ERROR = -1,
    MSP_RESULT_NO_REPLY = 0,
    MSP_RESULT_CMD_UNKNOWN = -2,   // don't know how to process command, try next handler
};

struct msp_packet_t {
    StreamBufWriter payload;  // payload only, ie no header or crc
    int16_t cmd;
    int16_t result;
    uint8_t flags;      // MSPv2 flags byte
    uint8_t direction;  // Currently unused
};

struct msp_const_packet_t {
    StreamBufReader payload;  // payload only, ie no header or crc
    int16_t cmd;
    int16_t result;
    uint8_t flags;      // MSPv2 flags byte
    uint8_t direction;  // Currently unused
};

class MspBase {
public:
    static constexpr uint8_t V2_FRAME_ID = 255;

    static constexpr uint8_t DIRECTION_REPLY = 0;
    static constexpr uint8_t DIRECTION_REQUEST = 1;

    static constexpr uint8_t REBOOT_FIRMWARE = 0;
    static constexpr uint8_t REBOOT_BOOTLOADER_ROM = 1;
    static constexpr uint8_t REBOOT_MSC = 2;
    static constexpr uint8_t REBOOT_MSC_UTC= 3;
    static constexpr uint8_t REBOOT_BOOTLOADER_FLASH = 4;
    static constexpr uint8_t REBOOT_COUNT = 5;

    static constexpr uint8_t SDCARD_STATE_NOT_PRESENT = 0;
    static constexpr uint8_t SDCARD_STATE_FATAL       = 1;
    static constexpr uint8_t SDCARD_STATE_CARD_INIT   = 2;
    static constexpr uint8_t SDCARD_STATE_FS_INIT     = 3;
    static constexpr uint8_t SDCARD_STATE_READY       = 4;

    static constexpr uint8_t SDCARD_FLAG_SUPPORTED   = 1;

    static constexpr uint8_t FLASHFS_FLAG_READY       = 1;
    static constexpr uint8_t FLASHFS_FLAG_SUPPORTED  = 2;

    static constexpr uint8_t PASSTHROUGH_ESC_SIMONK = PROTOCOL_SIMONK;
    static constexpr uint8_t PASSTHROUGH_ESC_BLHELI = PROTOCOL_BLHELI;
    static constexpr uint8_t PASSTHROUGH_ESC_KISS = PROTOCOL_KISS;
    static constexpr uint8_t PASSTHROUGH_ESC_KISSALL = PROTOCOL_KISSALL;
    static constexpr uint8_t PASSTHROUGH_ESC_CASTLE = PROTOCOL_CASTLE;

    static constexpr uint8_t PASSTHROUGH_SERIAL_ID = 0xFD;
    static constexpr uint8_t PASSTHROUGH_SERIAL_FUNCTION_ID = 0xFE;

    static constexpr uint8_t PASSTHROUGH_ESC_4WAY = 0xFF;

public:
    MspBase() = default;
    virtual ~MspBase() = default;

    virtual msp_result_e process_write_command(msp_context_t& pg, int16_t cmd_msp, StreamBufWriter& dst, StreamBufReader& src);

    virtual msp_result_e process_read_command(msp_context_t& pg, int16_t cmd_msp, StreamBufReader& src);

    virtual void process_reply(msp_context_t& pg, const msp_packet_t& reply);

    virtual msp_result_e process_command(msp_context_t& pg, const msp_const_packet_t& cmd, msp_packet_t& reply);
};
