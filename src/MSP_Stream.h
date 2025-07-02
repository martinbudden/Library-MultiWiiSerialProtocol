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

#include "MSP_Base.h"
#include <array>

class MSP_SerialBase;

class MSP_Stream {
public:
    enum { JUMBO_FRAME_SIZE_LIMIT = 255 };
    enum  streamState_e {
        STREAM_IDLE,
        STREAM_MSP_PACKET,
        STREAM_CLI_ACTIVE,
        STREAM_CLI_CMD
    };
    enum packetState_e {
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
    enum packetType_e {
        MSP_PACKET_COMMAND,
        MSP_PACKET_REPLY
    };
    enum  evaluateNonMspData_e {
        MSP_EVALUATE_NON_MSP_DATA,
        MSP_SKIP_NON_MSP_DATA
    };
    enum pendingSystemRequest_e {
        MSP_PENDING_NONE,
        MSP_PENDING_BOOTLOADER_ROM,
        MSP_PENDING_CLI,
        MSP_PENDING_BOOTLOADER_FLASH,
    };
    enum { MSP_Stream_INBUF_SIZE = 192 };
    enum { MSP_Stream_OUTBUF_SIZE_MIN = 512 }; // As of 2021/08/10 MSP_BOXNAMES generates a 307 byte response for page 1. There has been overflow issues with 320 byte buffer.
#ifdef USE_FLASHFS
    enum { MSP_Stream_DATAFLASH_BUFFER_SIZE = 4096 };
    enum { MSP_Stream_DATAFLASH_INFO_SIZE = 16 };
    enum { MSP_Stream_OUTBUF_SIZE = MSP_Stream_DATAFLASH_BUFFER_SIZE + MSP_Stream_DATAFLASH_INFO_SIZE };
#else
    enum { MSP_Stream_OUTBUF_SIZE = MSP_Stream_OUTBUF_SIZE_MIN }; // As of 2021/08/10 MSP_BOXNAMES generates a 307 byte response for page 1.
#endif
#pragma pack(push, 1)
    struct  mspHeaderV1_t {
        uint8_t size;
        uint8_t cmd;
    };
    struct mspHeaderJUMBO_t {
        uint16_t size;
    };
    struct mspHeaderV2_t {
        uint8_t  flags;
        uint16_t cmd;
        uint16_t size;
    };
#pragma pack(pop)
    struct packet_with_header_t {
        std::array<uint8_t, 16> hdrBuf;
        std::array<uint8_t, 2> crcBuf;
        uint8_t* dataPtr;
        uint16_t dataLen;
        uint16_t hdrLen = 3;
        uint16_t crcLen = 0;
        uint8_t checksum;
    };

    enum { MSP_MAX_HEADER_SIZE = 9 };
public:
    MSP_Stream(MSP_Base& mspBase, MSP_SerialBase* mspSerialBase);
    explicit MSP_Stream(MSP_Base& mspBase);

    void setStreamState(streamState_e streamState) { _streamState = streamState; }

    packetState_e getPacketState() const { return _packetState; }
    void setPacketState(packetState_e packetState) { _packetState = packetState; }

    packetType_e getPacketType() const { return _packetType; }

    void processReceivedPacketData(uint8_t c);
    MSP_Base::postProcessFnPtr processReceivedCommand(packet_with_header_t* pwh);
    void processReceivedReply();
    void processPendingRequest();
    packet_with_header_t serialEncode(MSP_Base::packet_t& packet, MSP_Base::version_e mspVersion);
    //bool putChar(uint8_t c, MSP_Base::processCommandFnPtr processCommandFn, MSP_Base::processReplyFnPtr processReplyFn, packet_with_header_t& pwh);
    bool putChar(uint8_t c, packet_with_header_t* pwh);

public: // for testing
    MSP_Base::packet_t processInbuf();
    streamState_e getStreamState() const { return _streamState; }
    uint16_t getDataSize() const { return _dataSize; }
    uint8_t getInbuf(size_t index) { return _inBuf[index]; }
    uint8_t getCheckSum1() const { return _checksum1; }
    uint8_t getCheckSum2() const { return _checksum2; }
private:
    static uint8_t checksumXOR(uint8_t checksum, const uint8_t* data, size_t len);
    static uint8_t crc8_calc(uint8_t crc, unsigned char a, uint8_t poly);
    static uint8_t crc8_update(uint8_t crc, const void *data, uint32_t length, uint8_t poly);
    static uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a) { return crc8_calc(crc, a, 0xD5); }
    static uint8_t crc8_dvb_s2_update(uint8_t crc, const void *data, uint32_t length) { return crc8_update(crc, data, length, 0xD5); }
private:
    MSP_Base& _mspBase;
    MSP_SerialBase* _mspSerialBase;
    pendingSystemRequest_e _pendingRequest {};
    streamState_e _streamState {};
    packetState_e _packetState {};
    packetType_e _packetType {};
    MSP_Base::version_e _mspVersion {};
    MSP_Base::descriptor_t _descriptor {};
    uint16_t _cmdMSP {};
    uint16_t _offset {};
    uint16_t _dataSize {};
    uint8_t _cmdFlags {};
    uint8_t _checksum1 {};
    uint8_t _checksum2 {};
    std::array<uint8_t, MSP_Stream_INBUF_SIZE> _inBuf {};
    std::array<uint8_t, MSP_Stream_OUTBUF_SIZE> _outBuf {};
};
