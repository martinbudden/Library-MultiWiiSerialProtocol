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

#include "MSP_SerialBase.h"
#include "MSP_Stream.h"
#include <cassert>




MSP_Stream::MSP_Stream(MSP_Base& mspBase, MSP_SerialBase* mspSerialBase) :
    _mspBase(mspBase),
    _mspSerialBase(mspSerialBase)
{
}

MSP_Stream::MSP_Stream(MSP_Base& mspBase) :
    MSP_Stream(mspBase, nullptr)
{
}

uint8_t MSP_Stream::checksumXOR(uint8_t checksum, const uint8_t* data, size_t len)
{
    while (len-- > 0) {
        checksum ^= *data++; // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
    }
    return checksum;
}

uint8_t MSP_Stream::crc8_calc(uint8_t crc, unsigned char a, uint8_t poly)
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

uint8_t MSP_Stream::crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly)
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
void MSP_Stream::processReceivedPacketData(uint8_t c) // NOLINT(readability-function-cognitive-complexity)
{
    switch (_packetState) {
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
            _packetState = MSP_HEADER_M;
            _mspVersion = MSP_Base::V1;
            break;
        case 'X':
            _packetState = MSP_HEADER_X;
            _mspVersion = MSP_Base::V2_NATIVE;
            break;
        default:
            _packetState = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_M:      // Waiting for '<' or '>'
        _packetState = MSP_HEADER_V1;
        switch (c) {
        case '<':
            _packetType = MSP_PACKET_COMMAND;
            break;
        case '>':
            _packetType = MSP_PACKET_REPLY;
            break;
        default:
            _packetState = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_X:
        _packetState = MSP_HEADER_V2_NATIVE;
        switch (c) {
        case '<':
            _packetType = MSP_PACKET_COMMAND;
            break;
        case '>':
            _packetType = MSP_PACKET_REPLY;
            break;
        default:
            _packetState = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_V1:     // Now receive v1 header (size/cmd), this is already checksummable
        _inBuf[_offset++] = c;
        _checksum1 ^= c;
        if (_offset == sizeof(mspHeaderV1_t)) {
            const auto* hdr = reinterpret_cast<mspHeaderV1_t*>(&_inBuf[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-init-variables)
            // Check incoming buffer size limit
            if (hdr->size > MSP_Stream_INBUF_SIZE) {
                _packetState = MSP_IDLE;
            }
            else if (hdr->cmd == MSP_Base::V2_FRAME_ID) {
                // MSPv1 payload must be big enough to hold V2 header + extra checksum
                if (hdr->size >= sizeof(mspHeaderV2_t) + 1) {
                    _mspVersion = MSP_Base::V2_OVER_V1;
                    _packetState = MSP_HEADER_V2_OVER_V1;
                } else {
                    _packetState = MSP_IDLE;
                }
            } else {
                _dataSize = hdr->size;
                _cmdMSP = hdr->cmd;
                _cmdFlags = 0;
                _offset = 0;                // re-use buffer
                _packetState = _dataSize > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
            }
        }
        break;

    case MSP_PAYLOAD_V1:
        _inBuf[_offset++] = c;
        _checksum1 ^= c;
        if (_offset == _dataSize) {
            _packetState = MSP_CHECKSUM_V1;
        }
        break;

    case MSP_CHECKSUM_V1:
        if (_checksum1 == c) {
            _packetState = MSP_COMMAND_RECEIVED;
        } else {
            _packetState = MSP_IDLE;
        }
        break;

    case MSP_HEADER_V2_OVER_V1:     // V2 header is part of V1 payload - we need to calculate both checksums now
        _inBuf[_offset++] = c;
        _checksum1 ^= c;
        _checksum2 = crc8_dvb_s2(_checksum2, c);
        if (_offset == (sizeof(mspHeaderV2_t) + sizeof(mspHeaderV1_t))) {
            const mspHeaderV2_t* hdrv2 = reinterpret_cast<mspHeaderV2_t*>(&_inBuf[sizeof(mspHeaderV1_t)]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-init-variables)
            if (hdrv2->size > MSP_Stream_INBUF_SIZE) {
                _packetState = MSP_IDLE;
            } else {
                _dataSize = hdrv2->size;
                _cmdMSP = hdrv2->cmd;
                _cmdFlags = hdrv2->flags;
                _offset = 0;                // re-use buffer
                _packetState = _dataSize > 0 ? MSP_PAYLOAD_V2_OVER_V1 : MSP_CHECKSUM_V2_OVER_V1;
            }
        }
        break;

    case MSP_PAYLOAD_V2_OVER_V1:
        _checksum2 = crc8_dvb_s2(_checksum2, c);
        _checksum1 ^= c;
        _inBuf[_offset++] = c;

        if (_offset == _dataSize) {
            _packetState = MSP_CHECKSUM_V2_OVER_V1;
        }
        break;

    case MSP_CHECKSUM_V2_OVER_V1:
        _checksum1 ^= c;
        if (_checksum2 == c) {
            _packetState = MSP_CHECKSUM_V1; // Checksum 2 correct - verify v1 checksum
        } else {
            _packetState = MSP_IDLE;
        }
        break;

    case MSP_HEADER_V2_NATIVE:
        _inBuf[_offset++] = c;
        _checksum2 = crc8_dvb_s2(_checksum2, c);
        if (_offset == sizeof(mspHeaderV2_t)) {
            const mspHeaderV2_t* hdrv2 = reinterpret_cast<mspHeaderV2_t*>(&_inBuf[0]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast,cppcoreguidelines-init-variables)
            _dataSize = hdrv2->size;
            _cmdMSP = hdrv2->cmd;
            _cmdFlags = hdrv2->flags;
            _offset = 0;                // re-use buffer
            _packetState = _dataSize > 0 ? MSP_PAYLOAD_V2_NATIVE : MSP_CHECKSUM_V2_NATIVE;
        }
        break;

    case MSP_PAYLOAD_V2_NATIVE:
        _checksum2 = crc8_dvb_s2(_checksum2, c);
        _inBuf[_offset++] = c;

        if (_offset == _dataSize) {
            _packetState = MSP_CHECKSUM_V2_NATIVE;
        }
        break;

    case MSP_CHECKSUM_V2_NATIVE:
        if (_checksum2 == c) {
            _packetState = MSP_COMMAND_RECEIVED;
        } else {
            _packetState = MSP_IDLE;
        }
        break;
    }
}

void MSP_Stream::processPendingRequest()
{
    // If no request is pending or 100ms guard time has not elapsed - do nothing
    //if ((_pendingRequest == MSP_PENDING_NONE) || (cmp32(millis(), _lastActivityMs) < 100)) {
    //    return;
    //}

    switch(_pendingRequest) {
    case MSP_PENDING_BOOTLOADER_ROM:
        //systemResetToBootloader(BOOTLOADER_REQUEST_ROM);
        break;

#if defined(USE_FLASH_BOOT_LOADER)
    case MSP_PENDING_BOOTLOADER_FLASH:
        systemResetToBootloader(BOOTLOADER_REQUEST_FLASH);
        break;
#endif

#ifdef USE_CLI
    case MSP_PENDING_CLI:
        _pendingRequest = MSP_PENDING_NONE;
        _streamState = STREAM_CLI_ACTIVE;
        cliEnter(_port, true);
        break;
#endif

    default:
        break;
    }
}

/*!
Creates an MSP_Stream::packet_with_header_t from the MSP_Base::packet
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
MSP_Stream::packet_with_header_t MSP_Stream::serialEncode(MSP_Base::packet_t& packet, MSP_Base::version_e mspVersion)
{
    static constexpr std::array<uint8_t, MSP_Base::VERSION_COUNT> mspMagic = { 'M', 'M', 'X' };

    packet_with_header_t ret;
    ret.hdrBuf = {
        '$',
        mspMagic[mspVersion],
        packet.result == MSP_Base::RESULT_ERROR ? static_cast<uint8_t>('!') : static_cast<uint8_t>('>')
    };

    ret.dataPtr = packet.payload.ptr();
    ret.dataLen = static_cast<uint16_t>(packet.payload.bytesRemaining());

    enum { V1_CHECKSUM_STARTPOS = 3 };

    if (mspVersion == MSP_Base::V1) {
        auto hdrV1 = reinterpret_cast<mspHeaderV1_t*>(&ret.hdrBuf[ret.hdrLen]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.hdrLen += sizeof(mspHeaderV1_t);
        hdrV1->cmd = static_cast<uint8_t>(packet.cmd);

        // Add JUMBO-frame header if necessary
        if (ret.dataLen >= JUMBO_FRAME_SIZE_LIMIT) {
            auto hdrJUMBO = reinterpret_cast<mspHeaderJUMBO_t*>(&ret.hdrBuf[ret.hdrLen]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            ret.hdrLen += sizeof(mspHeaderJUMBO_t);

            hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdrJUMBO->size = static_cast<uint8_t>(ret.dataLen);
        } else {
            hdrV1->size = static_cast<uint8_t>(ret.dataLen);
        }

        // Pre-calculate CRC
        ret.checksum = checksumXOR(0, &ret.hdrBuf[V1_CHECKSUM_STARTPOS], ret.hdrLen - V1_CHECKSUM_STARTPOS); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ret.checksum = checksumXOR(ret.checksum, packet.payload.ptr(), ret.dataLen);
        ret.crcBuf[ret.crcLen++] = ret.checksum;
    } else if (mspVersion == MSP_Base::V2_OVER_V1) {
        auto hdrV1 = reinterpret_cast<mspHeaderV1_t*>(&ret.hdrBuf[ret.hdrLen]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)

        ret.hdrLen += sizeof(mspHeaderV1_t);

        auto hdrV2 = reinterpret_cast<mspHeaderV2_t*>(&ret.hdrBuf[ret.hdrLen]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.hdrLen += sizeof(mspHeaderV2_t);

        const uint16_t v1PayloadSize = sizeof(mspHeaderV2_t) + ret.dataLen + 1;  // MSPv2 header + data payload + MSPv2 checksum
        hdrV1->cmd = MSP_Base::V2_FRAME_ID;

        // Add JUMBO-frame header if necessary
        if (v1PayloadSize >= JUMBO_FRAME_SIZE_LIMIT) {
            auto hdrJUMBO = reinterpret_cast<mspHeaderJUMBO_t*>(&ret.hdrBuf[ret.hdrLen]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
            ret.hdrLen += sizeof(mspHeaderJUMBO_t);

            hdrV1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdrJUMBO->size = v1PayloadSize;
        } else {
            hdrV1->size = static_cast<uint8_t>(v1PayloadSize);
        }

        // Fill V2 header
        hdrV2->flags = packet.flags;
        hdrV2->cmd = static_cast<uint16_t>(packet.cmd);
        hdrV2->size = ret.dataLen;

        // V2 CRC: only V2 header + data payload
        ret.checksum = crc8_dvb_s2_update(0, reinterpret_cast<uint8_t*>(hdrV2), sizeof(mspHeaderV2_t)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.checksum = crc8_dvb_s2_update(ret.checksum, packet.payload.ptr(), ret.dataLen);
        ret.crcBuf[ret.crcLen++] = ret.checksum;

        // V1 CRC: All headers + data payload + V2 CRC byte
        ret.checksum = checksumXOR(0, &ret.hdrBuf[V1_CHECKSUM_STARTPOS], ret.hdrLen - V1_CHECKSUM_STARTPOS); // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)
        ret.checksum = checksumXOR(ret.checksum, packet.payload.ptr(), ret.dataLen);
        ret.checksum = checksumXOR(ret.checksum, &ret.crcBuf[0], ret.crcLen);
        ret.crcBuf[ret.crcLen++] = ret.checksum; // NOLINT(cppcoreguidelines-pro-bounds-constant-array-index)
    } else if (mspVersion == MSP_Base::V2_NATIVE) {
        auto hdrV2 = reinterpret_cast<mspHeaderV2_t*>(&ret.hdrBuf[ret.hdrLen]); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.hdrLen += sizeof(mspHeaderV2_t);

        hdrV2->flags = packet.flags;
        hdrV2->cmd = static_cast<uint16_t>(packet.cmd);
        hdrV2->size = ret.dataLen;

        ret.checksum = crc8_dvb_s2_update(0, reinterpret_cast<uint8_t*>(hdrV2), sizeof(mspHeaderV2_t)); // NOLINT(cppcoreguidelines-pro-type-reinterpret-cast)
        ret.checksum = crc8_dvb_s2_update(ret.checksum, packet.payload.ptr(), ret.dataLen);
        ret.crcBuf[ret.crcLen++] = ret.checksum;
    } else {
        // Shouldn't get here
        assert(false);
    }

    // Send the frame
    if (_mspSerialBase) {
        _mspSerialBase->sendFrame(&ret.hdrBuf[0], ret.hdrLen, ret.dataPtr, ret.dataLen, &ret.crcBuf[0], ret.crcLen);
    }
    return ret;
}

/*!
For test code
Called when the state machine has assembled a packet into _inBuf.
*/
MSP_Base::packet_t MSP_Stream::processInbuf()
{
    MSP_Base::packet_t command = {
        .payload = StreamBuf(&_inBuf[0], _dataSize),
        .cmd = static_cast<int16_t>(_cmdMSP),
        .result = 0,
        .flags = _cmdFlags,
        .direction = MSP_Base::DIRECTION_REQUEST
    };

    MSP_Base::packet_t reply = {
        .payload = StreamBuf(&_outBuf[0], _outBuf.size()),
        .cmd = -1,
        .result = 0,
        .flags = 0,
        .direction = MSP_Base::DIRECTION_REPLY
    };

    MSP_Base::postProcessFnPtr mspPostProcessFn = nullptr;
    const MSP_Base::result_e status = _mspBase.processCommand(command, reply, _descriptor, &mspPostProcessFn);
    (void)status;
    reply.payload.switchToReader(); // change streambuf direction

    return reply;
}

/*!
Called when the state machine has assembled a packet into _inBuf.

pwh is optional parameter for use by test code.
*/
MSP_Base::postProcessFnPtr MSP_Stream::processReceivedCommand(packet_with_header_t* pwh)
{
    MSP_Base::packet_t command = {
        .payload = StreamBuf(&_inBuf[0], _dataSize),
        .cmd = static_cast<int16_t>(_cmdMSP),
        .result = 0,
        .flags = _cmdFlags,
        .direction = MSP_Base::DIRECTION_REQUEST
    };

    MSP_Base::packet_t reply = {
        .payload = StreamBuf(&_outBuf[0], _outBuf.size()),
        .cmd = -1,
        .result = 0,
        .flags = 0,
        .direction = MSP_Base::DIRECTION_REPLY
    };

    MSP_Base::postProcessFnPtr mspPostProcessFn = nullptr;

    //!!const MSP_Base::result_e status = _mspBase.*mspProcessCommandFn(command, reply, _descriptor, &mspPostProcessFn);
    //(void)mspProcessCommandFn;
    const MSP_Base::result_e status = _mspBase.processCommand(command, reply, _descriptor, &mspPostProcessFn);

    if (status != MSP_Base::RESULT_NO_REPLY) {
        reply.payload.switchToReader(); // change streambuf direction
        if (pwh) {
            *pwh = serialEncode(reply, _mspVersion);
        } else {
            serialEncode(reply, _mspVersion);
        }

    }

    return mspPostProcessFn;
}

void MSP_Stream::processReceivedReply()
{
    const MSP_Base::packet_t reply = {
        .payload = StreamBuf(&_inBuf[0], _dataSize),
        .cmd = static_cast<int16_t>(_cmdMSP),
        .result = 0,
        .flags = 0,
        .direction = 0
    };

    //!!_mspBase.*processReplyFn(reply);
    //(void)processReplyFn;
    _mspBase.processReply(reply);
}

/*!
pwh is optional parameter for use by test code.
*/
bool MSP_Stream::putChar(uint8_t c, packet_with_header_t* pwh)
{
    bool ret = false;

    // Run state machine on incoming character
    processReceivedPacketData(c);

    if (_packetState == MSP_COMMAND_RECEIVED) {
        ret = true;
        if (_packetType == MSP_PACKET_COMMAND) {
            processReceivedCommand(pwh); // eventually calls processOutCommand or processInCommand
        } else if (_packetType == MSP_PACKET_REPLY) {
            processReceivedReply(); // by default does nothing
        }

        // we've processed the command, so return to idle state
        _packetState = MSP_IDLE;
    }
    if (_packetState == MSP_IDLE) {
        _streamState = STREAM_IDLE;
    }
    return ret;
}
