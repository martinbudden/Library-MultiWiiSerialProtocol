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

#include "MSP_Serial.h"
#include "MSP_SerialPortBase.h"
#include "MSP_Stream.h"

#include <StreamBuf.h>


MSP_Serial::MSP_Serial(MSP_Stream& mspStream, MSP_SerialPortBase& mspSerialPort) :
    _mspStream(mspStream),
    _mspSerialPort(mspSerialPort)
{
}

/*!
Called from MSP_Task::loop()
*/
void MSP_Serial::processInput()
{
    while (_mspSerialPort.isDataAvailable()) {
        const uint8_t inChar = _mspSerialPort.readByte();
        _mspStream.putChar(inChar, nullptr); // This will invoke MSP_Serial::sendFrame(), when a completed frame is received
    }
}

/*!
Called from  MSP_Stream::serialEncode() which is called from MSP_Stream::processReceivedCommand() which is called from MSP_Stream::putChar()
*/
size_t MSP_Serial::sendFrame(const uint8_t* hdr, size_t hdrLen, const uint8_t* data, size_t dataLen, const uint8_t* crc, size_t crcLen)
{
    const size_t totalFrameLength = hdrLen + dataLen + crcLen;

    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    //if (!isSerialTransmitBufferEmpty(_serialPort) && ((int)serialTxBytesFree(_serialPort) < totalFrameLength)) {
    //    return 0;
    //}
    // buffer size is 64 bytes on arduino
    // buffer empty if Serial.availableForWrite() >= SERIAL_TX_BUFFER_SIZE - 1
    // if (totalFrameLength <= Serial.availableForWrite())

    StreamBuf sbuf(&_buffer[0], sizeof(_buffer));

    // copy the frame into a StreamBuf
    sbuf.writeData(hdr, hdrLen);
    sbuf.writeData(data, dataLen);
    sbuf.writeData(crc, crcLen);
    sbuf.switchToReader();

    while (sbuf.bytesRemaining() > 0) {
        const size_t available = _mspSerialPort.availableForWrite();
        const size_t writeLen = std::min(available, sbuf.bytesRemaining());
        _mspSerialPort.write(sbuf.ptr(), writeLen);
        sbuf.advance(writeLen);
        //!!delayMs(1);
    }

    return totalFrameLength;
}
