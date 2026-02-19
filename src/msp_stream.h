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

#include "msp_base.h"
#include <array>

class MspSerial;
struct msp_parameter_group_t;

enum msp_stream_state_e {
    STREAM_IDLE,
    STREAM_MSP_PACKET,
    STREAM_CLI_ACTIVE,
    STREAM_CLI_CMD
};

enum msp_packet_state_e {
    MSP_IDLE,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER_X,

    MSP_HEADER_V1,
    MSP_PAYLOAD_V1,
    MSP_CHECKSUM_V1,

    MSP_HEADER_V2_OVER_V1,
    MSP_PAYLOAD_V2_OVER_V1,
    MSP_CHECKSUM_V2_OVER_V1,

    MSP_HEADER_V2_NATIVE,
    MSP_PAYLOAD_V2_NATIVE,
    MSP_CHECKSUM_V2_NATIVE,

    MSP_COMMAND_RECEIVED
};

enum msp_packet_type_e {
    MSP_PACKET_COMMAND,
    MSP_PACKET_REPLY
};

enum msp_pending_system_request_e {
    MSP_PENDING_NONE,
    MSP_PENDING_BOOTLOADER_ROM,
    MSP_PENDING_CLI,
    MSP_PENDING_BOOTLOADER_FLASH,
};

#pragma pack(push, 1)
struct  msp_stream_header_v1_t {
    uint8_t size;
    uint8_t cmd;
};
struct msp_stream_header_jumbo_t {
    uint16_t size;
};
struct msp_stream_header_v2_t {
    uint8_t  flags;
    uint16_t cmd;
    uint16_t size;
};
#pragma pack(pop)

struct msp_stream_packet_with_header_t {
    std::array<uint8_t, 16> hdr_buf;
    std::array<uint8_t, 2> crc_buf;
    const uint8_t* data_ptr;
    uint16_t data_len;
    uint16_t hdr_len = 3;
    uint16_t crc_len = 0;
    uint8_t checksum;
};

class MspStream {
public:
    static constexpr size_t JUMBO_FRAME_SIZE_LIMIT = 255;

    static constexpr uint8_t MSP_EVALUATE_NON_MSP_DATA = 0;
    static constexpr uint8_t MSP_SKIP_NON_MSP_DATA = 1;
    static constexpr size_t MSP_HEADER_LENGTH = 3;
    static constexpr size_t MSP_STREAM_INBUF_SIZE = 192;
    static constexpr size_t MSP_STREAM_OUTBUF_SIZE_MIN = 512; // As of 2021/08/10 MSP_BOXNAMES generates a 307 byte response for page 1. There has been overflow issues with 320 byte buffer.
#ifdef USE_FLASHFS
    static constexpr size_t MSP_STREAM_DATAFLASH_BUFFER_SIZE = 4096;
    static constexpr size_t MSP_STREAM_DATAFLASH_INFO_SIZE = 16;
    static constexpr size_t MSP_STREAM_OUTBUF_SIZE = MSP_STREAM_DATAFLASH_BUFFER_SIZE + MSP_STREAM_DATAFLASH_INFO_SIZE;
#else
    static constexpr size_t MSP_STREAM_OUTBUF_SIZE = MSP_STREAM_OUTBUF_SIZE_MIN; // As of 2021/08/10 MSP_BOXNAMES generates a 307 byte response for page 1.
#endif

    static constexpr size_t MSP_MAX_HEADER_SIZE = 9;
public:
    //MspStream(MspBase& msp_base, MspSerial* msp_serial);
    explicit MspStream(MspBase& msp_base);
    void set_msp_serial(MspSerial* msp_serial) { _msp_serial = msp_serial; }

    void set_stream_state(msp_stream_state_e streamState) { _stream_state = streamState; }

    msp_packet_state_e get_packet_state() const { return _packet_state; }
    void set_packet_state(msp_packet_state_e packetState) { _packet_state = packetState; }

    msp_packet_type_e get_packet_type() const { return _packet_type; }

    void process_received_packet_data(uint8_t c);
    MspBase::postProcessFnPtr process_received_command(msp_parameter_group_t& pg, msp_stream_packet_with_header_t* pwh);
    void process_received_reply();
    void process_pending_request();
    msp_stream_packet_with_header_t serial_encode(const msp_const_packet_t& packet, msp_version_e mspVersion);
    msp_stream_packet_with_header_t serial_encode_msp_v1(uint8_t command, const uint8_t* buf, uint8_t len);
    //bool put_char(uint8_t c, MspBase::process_commandFnPtr process_commandFn, MspBase::process_replyFnPtr process_replyFn, packet_with_header_t& pwh);
    bool put_char(msp_parameter_group_t& pg, uint8_t c, msp_stream_packet_with_header_t* pwh);

public: // for testing
    msp_const_packet_t process_in_buf(msp_parameter_group_t& pg);
    msp_stream_state_e get_stream_state() const { return _stream_state; }
    uint16_t get_data_size() const { return _data_size; }
    uint8_t get_in_buf(size_t index) { return _in_buf[index]; }
    uint8_t get_checksum1() const { return _checksum1; }
    uint8_t get_checksum2() const { return _checksum2; }
public: // made public for testing
    static uint8_t checksum_xor(uint8_t checksum, const uint8_t* data, size_t len);
    static uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);
    static uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly);
    static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) { return crc8_calc(crc, a, 0xD5); }
    static uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length) { return crc8_update(crc, data, length, 0xD5); }
private:
    MspBase& _msp_base;
    MspSerial* _msp_serial {};
    msp_pending_system_request_e _pending_request {};
    msp_stream_state_e _stream_state {};
    msp_packet_state_e _packet_state {};
    msp_packet_type_e _packet_type {};
    msp_version_e _msp_version {};
    MspBase::descriptor_t _descriptor {};
    uint16_t _cmd_msp {};
    uint16_t _offset {};
    uint16_t _data_size {};
    uint8_t _cmd_flags {};
    uint8_t _checksum1 {};
    uint8_t _checksum2 {};
    std::array<uint8_t, MSP_STREAM_INBUF_SIZE> _in_buf {};
    std::array<uint8_t, MSP_STREAM_OUTBUF_SIZE> _out_buf {};
};
