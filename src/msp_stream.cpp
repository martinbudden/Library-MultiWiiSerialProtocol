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

#include "msp_serial.h"
#include "msp_stream.h"
#include <cassert>


MspStream::MspStream(MspBase& msp_base) :
    _msp_base(msp_base)
{
}

uint8_t MspStream::checksum_xor(uint8_t checksum, const uint8_t* data, size_t len)
{
    while (len-- > 0) {
        checksum ^= *data++; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    return checksum;
}

uint8_t MspStream::crc8_calc(uint8_t crc, unsigned char a, uint8_t poly)
{
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80U) {
            crc = (crc << 1U) ^ poly; // NOLINT(hicpp-signed-bitwise)
        } else {
            crc = crc << 1U;
        }
    }
    return crc;
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,readability-magic-numbers)
}

uint8_t MspStream::crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly)
{
    const auto* p = static_cast<const uint8_t*>(data);
    const uint8_t* pend = p + length; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

    for (; p != pend; p++) { // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        crc = crc8_calc(crc, *p, poly);
    }
    return crc;
}

/*!
State machine to build up MSP packet from individual incoming characters.
*/
void MspStream::process_received_packet_data(uint8_t c) // NOLINT(readability-function-cognitive-complexity)
{
    switch (_packet_state) {
    default:
        [[fallthrough]];
    case MSP_IDLE:
        [[fallthrough]];
    case MSP_HEADER_START:  // Waiting for 'M' (MSPv1 or MSPv2 over MSPv1) or 'X' (MSPv2 native)
        _offset = 0;
        _checksum1 = 0;
        _checksum2 = 0;
        switch (c) {
        case 'M':
            _packet_state = MSP_HEADER_M;
            _msp_version = MSP_V1;
            break;
        case 'X':
            _packet_state = MSP_HEADER_X;
            _msp_version = MSP_V2_NATIVE;
            break;
        default:
            _packet_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_M:      // Waiting for '<' or '>'
        _packet_state = MSP_HEADER_V1;
        switch (c) {
        case '<':
            _packet_type = MSP_PACKET_COMMAND;
            break;
        case '>':
            _packet_type = MSP_PACKET_REPLY;
            break;
        default:
            _packet_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_X:
        _packet_state = MSP_HEADER_V2_NATIVE;
        switch (c) {
        case '<':
            _packet_type = MSP_PACKET_COMMAND;
            break;
        case '>':
            _packet_type = MSP_PACKET_REPLY;
            break;
        default:
            _packet_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_V1:     // Now receive v1 header (size/cmd), this is already checksummable
        _in_buf[_offset++] = c;
        _checksum1 ^= c;
        if (_offset == sizeof(msp_stream_header_v1_t)) {
            const auto* hdr = reinterpret_cast<msp_stream_header_v1_t*>(&_in_buf[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-init-variables)
            // Check incoming buffer size limit
            if (hdr->size > MSP_STREAM_INBUF_SIZE) {
                _packet_state = MSP_IDLE;
            }
            else if (hdr->cmd == MspBase::V2_FRAME_ID) {
                // MSPv1 payload must be big enough to hold V2 header + extra checksum
                if (hdr->size >= sizeof(msp_stream_header_v2_t) + 1) {
                    _msp_version = MSP_V2_OVER_V1;
                    _packet_state = MSP_HEADER_V2_OVER_V1;
                } else {
                    _packet_state = MSP_IDLE;
                }
            } else {
                _data_size = hdr->size;
                _cmd_msp = hdr->cmd;
                _cmd_flags = 0;
                _offset = 0;                // re-use buffer
                _packet_state = _data_size > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
            }
        }
        break;

    case MSP_PAYLOAD_V1:
        _in_buf[_offset++] = c;
        _checksum1 ^= c;
        if (_offset == _data_size) {
            _packet_state = MSP_CHECKSUM_V1;
        }
        break;

    case MSP_CHECKSUM_V1:
        if (_checksum1 == c) {
            _packet_state = MSP_COMMAND_RECEIVED;
        } else {
            _packet_state = MSP_IDLE;
        }
        break;

    case MSP_HEADER_V2_OVER_V1:     // V2 header is part of V1 payload - we need to calculate both checksums now
        _in_buf[_offset++] = c;
        _checksum1 ^= c;
        _checksum2 = crc8_dvb_s2(_checksum2, c);
        if (_offset == (sizeof(msp_stream_header_v2_t) + sizeof(msp_stream_header_v1_t))) {
            const msp_stream_header_v2_t* hdrv2 = reinterpret_cast<msp_stream_header_v2_t*>(&_in_buf[sizeof(msp_stream_header_v1_t)]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-init-variables)
            if (hdrv2->size > MSP_STREAM_INBUF_SIZE) {
                _packet_state = MSP_IDLE;
            } else {
                _data_size = hdrv2->size;
                _cmd_msp = hdrv2->cmd;
                _cmd_flags = hdrv2->flags;
                _offset = 0;                // re-use buffer
                _packet_state = _data_size > 0 ? MSP_PAYLOAD_V2_OVER_V1 : MSP_CHECKSUM_V2_OVER_V1;
            }
        }
        break;

    case MSP_PAYLOAD_V2_OVER_V1:
        _checksum2 = crc8_dvb_s2(_checksum2, c);
        _checksum1 ^= c;
        _in_buf[_offset++] = c;

        if (_offset == _data_size) {
            _packet_state = MSP_CHECKSUM_V2_OVER_V1;
        }
        break;

    case MSP_CHECKSUM_V2_OVER_V1:
        _checksum1 ^= c;
        if (_checksum2 == c) {
            _packet_state = MSP_CHECKSUM_V1; // Checksum 2 correct - verify v1 checksum
        } else {
            _packet_state = MSP_IDLE;
        }
        break;

    case MSP_HEADER_V2_NATIVE:
        _in_buf[_offset++] = c;
        _checksum2 = crc8_dvb_s2(_checksum2, c);
        if (_offset == sizeof(msp_stream_header_v2_t)) {
            const msp_stream_header_v2_t* hdrv2 = reinterpret_cast<msp_stream_header_v2_t*>(&_in_buf[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-init-variables)
            _data_size = hdrv2->size;
            _cmd_msp = hdrv2->cmd;
            _cmd_flags = hdrv2->flags;
            _offset = 0;                // re-use buffer
            _packet_state = _data_size > 0 ? MSP_PAYLOAD_V2_NATIVE : MSP_CHECKSUM_V2_NATIVE;
        }
        break;

    case MSP_PAYLOAD_V2_NATIVE:
        _checksum2 = crc8_dvb_s2(_checksum2, c);
        _in_buf[_offset++] = c;

        if (_offset == _data_size) {
            _packet_state = MSP_CHECKSUM_V2_NATIVE;
        }
        break;

    case MSP_CHECKSUM_V2_NATIVE:
        if (_checksum2 == c) {
            _packet_state = MSP_COMMAND_RECEIVED;
        } else {
            _packet_state = MSP_IDLE;
        }
        break;
    }
}

void MspStream::process_pending_request(msp_parameter_group_t& pg)
{
    // If no request is pending or 100ms guard time has not elapsed - do nothing
    //if ((_pending_request == MSP_PENDING_NONE) || (cmp32(millis(), _lastActivityMs) < 100)) {
    //    return;
    //}
    (void)pg;

    switch(_pending_request) {
    case MSP_PENDING_BOOTLOADER_ROM:
        //systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
        break;

#if defined(LIBRARY_MULTI_WII_SERIAL_PROTOCOL_USE_FLASH_BOOT_LOADER)
    case MSP_PENDING_BOOTLOADER_FLASH:
        systemResetToBootloader(BOOTLOADER_REQUEST_FLASH);
        break;
#endif

#if defined(LIBRARY_MULTI_WII_SERIAL_PROTOCOL_USE_CLI)
    case MSP_PENDING_CLI:
        _pending_request = MSP_PENDING_NONE;
        _stream_state = STREAM_CLI_ACTIVE;
        cliEnter(_port, true);
        break;
#endif

    default:
        break;
    }
}

/*!
Creates an msp_stream_packet_with_header_t from the MspBase::packet
and sends the stream packet to the serial device.

MSP V1 stream packet is of the form:

3 bytes header: two start bytes $M followed by message direction (< or >) or the error message indicator (!).
< - from the flight controller (FC →)
> - to the flight controller (→ FC)
! - error message.

one byte payload length
one byte message type
payload
checksum - XOR of the size, type, and payload bytes.

The checksum of a request (ie a message with no payload) equals the type.
*/
msp_stream_packet_with_header_t MspStream::serial_encode(const msp_const_packet_t& packet, msp_version_e mspVersion)
{
    static constexpr std::array<uint8_t, MSP_VERSION_COUNT> mspMagic = { 'M', 'M', 'X' };

    msp_stream_packet_with_header_t ret {
        .hdr_buf = {
            '$',
            mspMagic[mspVersion],
            packet.result == MSP_RESULT_ERROR ? static_cast<uint8_t>('!') : static_cast<uint8_t>('>')
        },
        .crc_buf = { 0, 0 },
        .data_ptr = packet.payload.ptr(),
        .data_len = static_cast<uint16_t>(packet.payload.bytes_remaining()),

        .hdr_len = MSP_HEADER_LENGTH,
        .crc_len = 0,
        .checksum = 0,
    };

    enum { V1_CHECKSUM_STARTPOS = 3 };

    if (mspVersion == MSP_V1) {
        auto hdrV1 = reinterpret_cast<msp_stream_header_v1_t*>(&ret.hdr_buf[ret.hdr_len]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.hdr_len += sizeof(msp_stream_header_v1_t);
        hdrV1->cmd = static_cast<uint8_t>(packet.cmd);

        // Add JUMBO-frame header if necessary
        if (ret.data_len >= JUMBO_FRAME_SIZE_LIMIT) {
            auto hdrJUMBO = reinterpret_cast<msp_stream_header_jumbo_t*>(&ret.hdr_buf[ret.hdr_len]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            ret.hdr_len += sizeof(msp_stream_header_jumbo_t);

            hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdrJUMBO->size = static_cast<uint8_t>(ret.data_len);
        } else {
            hdrV1->size = static_cast<uint8_t>(ret.data_len);
        }

        // Pre-calculate CRC
        ret.checksum = checksum_xor(0, &ret.hdr_buf[V1_CHECKSUM_STARTPOS], ret.hdr_len - V1_CHECKSUM_STARTPOS); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ret.checksum = checksum_xor(ret.checksum, packet.payload.ptr(), ret.data_len);
        ret.crc_buf[ret.crc_len++] = ret.checksum;
    } else if (mspVersion == MSP_V2_OVER_V1) {
        auto hdrV1 = reinterpret_cast<msp_stream_header_v1_t*>(&ret.hdr_buf[ret.hdr_len]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

        ret.hdr_len += sizeof(msp_stream_header_v1_t);

        auto hdrV2 = reinterpret_cast<msp_stream_header_v2_t*>(&ret.hdr_buf[ret.hdr_len]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.hdr_len += sizeof(msp_stream_header_v2_t);

        const uint16_t v1PayloadSize = sizeof(msp_stream_header_v2_t) + ret.data_len + 1;  // MSPv2 header + data payload + MSPv2 checksum
        hdrV1->cmd = MspBase::V2_FRAME_ID;

        // Add JUMBO-frame header if necessary
        if (v1PayloadSize >= JUMBO_FRAME_SIZE_LIMIT) {
            auto hdrJUMBO = reinterpret_cast<msp_stream_header_jumbo_t*>(&ret.hdr_buf[ret.hdr_len]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            ret.hdr_len += sizeof(msp_stream_header_jumbo_t);

            hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdrJUMBO->size = v1PayloadSize;
        } else {
            hdrV1->size = static_cast<uint8_t>(v1PayloadSize);
        }

        // Fill V2 header
        hdrV2->flags = packet.flags;
        hdrV2->cmd = static_cast<uint16_t>(packet.cmd);
        hdrV2->size = ret.data_len;

        // V2 CRC: only V2 header + data payload
        ret.checksum = crc8_dvb_s2_update(0, reinterpret_cast<uint8_t*>(hdrV2), sizeof(msp_stream_header_v2_t)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.checksum = crc8_dvb_s2_update(ret.checksum, packet.payload.ptr(), ret.data_len);
        ret.crc_buf[ret.crc_len++] = ret.checksum;

        // V1 CRC: All headers + data payload + V2 CRC byte
        ret.checksum = checksum_xor(0, &ret.hdr_buf[V1_CHECKSUM_STARTPOS], ret.hdr_len - V1_CHECKSUM_STARTPOS); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ret.checksum = checksum_xor(ret.checksum, packet.payload.ptr(), ret.data_len);
        ret.checksum = checksum_xor(ret.checksum, &ret.crc_buf[0], ret.crc_len);
        ret.crc_buf[ret.crc_len++] = ret.checksum; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    } else if (mspVersion == MSP_V2_NATIVE) {
        auto hdrV2 = reinterpret_cast<msp_stream_header_v2_t*>(&ret.hdr_buf[ret.hdr_len]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.hdr_len += sizeof(msp_stream_header_v2_t);

        hdrV2->flags = packet.flags;
        hdrV2->cmd = static_cast<uint16_t>(packet.cmd);
        hdrV2->size = ret.data_len;

        ret.checksum = crc8_dvb_s2_update(0, reinterpret_cast<uint8_t*>(hdrV2), sizeof(msp_stream_header_v2_t)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.checksum = crc8_dvb_s2_update(ret.checksum, packet.payload.ptr(), ret.data_len);
        ret.crc_buf[ret.crc_len++] = ret.checksum;
    } else {
        // Shouldn't get here
        assert(false);
    }

    // Send the frame
    if (_msp_serial) {
        _msp_serial->send_frame(&ret.hdr_buf[0], ret.hdr_len, ret.data_ptr, ret.data_len, &ret.crc_buf[0], ret.crc_len);
    }
    return ret;
}

msp_stream_packet_with_header_t MspStream::serial_encode_msp_v1(uint8_t command, const uint8_t* buf, uint8_t len)
{
    msp_const_packet_t packet = {
        .payload = StreamBufReader(buf, len),
        .cmd = command,
        .result = 0,
        .flags = 0,
        .direction = MspBase::DIRECTION_REPLY
    };

    packet.payload.switch_to_reader(); // change streambuf direction
    set_packet_state(MSP_IDLE);
    return serial_encode(packet, _msp_version);
}

/*!
For test code
Called when the state machine has assembled a packet into _in_buf.
*/
msp_const_packet_t MspStream::process_in_buf(msp_parameter_group_t& pg)
{
    msp_const_packet_t command = {
        .payload = StreamBufReader(&_in_buf[0], _data_size),
        .cmd = static_cast<int16_t>(_cmd_msp),
        .result = MSP_RESULT_NO_REPLY,
        .flags = _cmd_flags,
        .direction = MspBase::DIRECTION_REQUEST
    };

    msp_packet_t reply = {
        .payload = StreamBufWriter(&_out_buf[0], _out_buf.size()),
        .cmd = -1, // set to command.cmd in process_command
        .result = MSP_RESULT_NO_REPLY,
        .flags = 0,
        .direction = MspBase::DIRECTION_REPLY
    };

    const msp_result_e status = _msp_base.process_command(pg, command, reply);
    (void)status;
    reply.payload.switch_to_reader(); // change streambuf direction

    const msp_const_packet_t replyConst = {
        .payload = StreamBufReader(reply.payload),
        .cmd = reply.cmd,
        .result = reply.result,
        .flags = reply.flags,
        .direction = reply.direction
    };
    return replyConst;
}

/*!
Called when the state machine has assembled a packet into _in_buf.

pwh is optional parameter for use by test code.
*/
void MspStream::process_received_command(msp_parameter_group_t& pg, msp_stream_packet_with_header_t* pwh)
{
    const msp_const_packet_t command = {
        .payload = StreamBufReader(&_in_buf[0], _data_size),
        .cmd = static_cast<int16_t>(_cmd_msp),
        .result = MSP_RESULT_NO_REPLY,
        .flags = _cmd_flags,
        .direction = MspBase::DIRECTION_REQUEST
    };

    msp_packet_t reply = {
        .payload = StreamBufWriter(&_out_buf[0], _out_buf.size()),
        .cmd = -1, // set to command.cmd by process_command
        .result = MSP_RESULT_NO_REPLY,
        .flags = 0,
        .direction = MspBase::DIRECTION_REPLY
    };

    //!!const msp_result_e status = _msp_base.*mspProcessCommandFn(command, reply, _descriptor, &mspPostProcessFn);
    //(void)mspProcessCommandFn;
    const msp_result_e status = _msp_base.process_command(pg, command, reply);

    msp_const_packet_t replyConst = {
        .payload = StreamBufReader(reply.payload),
        .cmd = reply.cmd,
        .result = reply.result,
        .flags = reply.flags,
        .direction = reply.direction
    };
    if (status != MSP_RESULT_NO_REPLY) {
        replyConst.payload.switch_to_reader(); // change streambuf direction
        if (pwh) {
            *pwh = serial_encode(replyConst, _msp_version);
        } else {
            serial_encode(replyConst, _msp_version);
        }

    }
}

void MspStream::process_received_reply(msp_parameter_group_t& pg)
{
    const msp_packet_t reply = {
        .payload = StreamBufWriter(&_in_buf[0], _data_size),
        .cmd = static_cast<int16_t>(_cmd_msp),
        .result = 0,
        .flags = 0,
        .direction = 0
    };

    //!!_msp_base.*process_replyFn(reply);
    //(void)process_replyFn;
    _msp_base.process_reply(pg, reply);
}

/*!
pwh is optional return value for use by test code.
*/
bool MspStream::put_char(msp_parameter_group_t& pg, uint8_t c, msp_stream_packet_with_header_t* pwh)
{
    bool ret = false;

    // Run state machine on incoming character
    process_received_packet_data(c);

    if (_packet_state == MSP_COMMAND_RECEIVED) {
        ret = true;
        if (_packet_type == MSP_PACKET_COMMAND) {
            process_received_command(pg, pwh); // eventually calls processWriteCommand or processReadCommand
        } else if (_packet_type == MSP_PACKET_REPLY) {
            process_received_reply(pg); // by default does nothing
        }

        // we've processed the command, so return to idle state
        _packet_state = MSP_IDLE;
    }
    if (_packet_state == MSP_IDLE) {
        _stream_state = STREAM_IDLE;
    }
    return ret;
}
