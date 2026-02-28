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
#include "msp_serial_port_base.h"
#include "msp_stream.h"

#include <stream_buf_writer.h>

static void yield();


#if defined(FRAMEWORK_USE_FREERTOS)
#if defined(FRAMEWORK_ESPIDF) || defined(FRAMEWORK_ARDUINO_ESP32)
#include <freertos/FreeRTOS.h>
#include <freertos/FreeRTOSConfig.h>
#include <freertos/task.h>
#else
#if defined(FRAMEWORK_ARDUINO_STM32)
#include <STM32FreeRTOS.h>
#endif
#include <FreeRTOS.h>
#include <FreeRTOSConfig.h>
#include <task.h>
#endif

void yield() { taskYIELD(); }

#else

#if defined(FRAMEWORK_RPI_PICO)
#include <pico/time.h>
static void yield() { sleep_ms(1); }
#else
static void yield() {}
#endif
#endif


MspSerial::MspSerial(MspStream& msp_stream, MspSerialPortBase& msp_serial_port) :
    _msp_stream(msp_stream),
    _msp_serial_port(msp_serial_port)
{
    msp_stream.set_msp_serial(this);
}

/*!
Called from MspTask::loop()
*/
void MspSerial::process_input(msp_parameter_group_t& pg)
{
    while (_msp_serial_port.is_data_available()) {
        const uint8_t in_char = _msp_serial_port.read_byte();
        _msp_stream.put_char(pg, in_char, nullptr); // This will invoke MspSerial::send_frame(), when a completed frame is received
    }
}

/*!
Called from  MspStream::serial_encode() which is called from MspStream::process_received_command() which is called from MspStream::put_char()
*/
size_t MspSerial::send_frame(const uint8_t* header, size_t header_len, const uint8_t* data, size_t data_len, const uint8_t* crc, size_t crc_len)
{
    const size_t total_frame_length = header_len + data_len + crc_len;

    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    //if (!isSerialTransmitBufferEmpty(_serialPort) && ((int)serialTxBytesFree(_serialPort) < total_frame_length)) {
    //    return 0;
    //}
    // buffer size is 64 bytes on arduino
    // buffer empty if Serial.available_for_write() >= SERIAL_TX_BUFFER_SIZE - 1
    // if (total_frame_length <= Serial.available_for_write())


    // write the header
    while (_msp_serial_port.available_for_write() < header_len) {
        yield();
    }
    _msp_serial_port.write(header, header_len);

    // write the data

    StreamBufReader sbuf(data, data_len);
    while (sbuf.bytes_remaining() > 0) {
        const size_t available = _msp_serial_port.available_for_write();
        const size_t writeLen = std::min(available, static_cast<size_t>(sbuf.bytes_remaining()));
        _msp_serial_port.write(sbuf.ptr(), writeLen);
        sbuf.advance(writeLen);
        yield();
    }

    // write the crc
    while (_msp_serial_port.available_for_write() < crc_len) {
        yield();
    }
    _msp_serial_port.write(crc, crc_len);

    return total_frame_length;
}
