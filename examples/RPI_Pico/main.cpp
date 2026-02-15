#include <Arduino.h>
#include <msp_protocol_base.h>
#include <msp_serial.h>
#include <msp_stream.h>
#include <SoftwareSerial.h>
#include <boards/pico.h>

/*
See https://gist.github.com/Xorgon/d933ea8cc7d657ac309d6fc75374f836 for example basic Arduino sketch
*/

enum { SERIAL_INVERTED = false, SERIAL_NOT_INVERTED = true };
enum { SERIAL_RX = 11, SERIAL_TX = 12 };

static MspStream* msp_stream_ptr {};
static MspSerial* msp_serial_ptr {};

class MspSerial : public MspSerial {
public:
    MspSerial(MspStream& mspStream, SoftwareSerial& msp_serial) : _msp_stream(mspStream), _msp_serial(msp_serial) {}
    virtual size_t send_frame(const uint8_t* hdr, size_t hdr_len, const uint8_t* data, size_t data_len, const uint8_t* crc, size_t crc_len) override;
    virtual void process_input() override;
private:
    MspStream& _msp_stream;
    SoftwareSerial& _msp_serial;
    std::array<uint8_t, 256> _buffer {};
};


/*!
Note:
MSP_SET_* commands set configuration data on the flight controller
MSP_* commands get data from the flight controller
*/
void setup()
{
    Serial.begin(9600);

    static SoftwareSerial softwareSerial(SERIAL_RX, SERIAL_TX, SERIAL_NOT_INVERTED);
    softwareSerial.begin(9600);

    static MspBase msp;
    static MspStream mspStream(msp);
    static MspSerial msp_serial(mspStream, softwareSerial);

    msp_stream_ptr = &mspStream;

    // create and send a packet to get the BASE API VERSION
    const uint8_t payloadSize = 0;
    const uint8_t checksum = 1; // for 0-byte payload, checksum is 1
    const std::array<uint8_t, 6> inStream = {
        '$', 'M', '<', payloadSize, MSP_BASE_API_VERSION, checksum,
    };
    for (uint8_t inChar : inStream) {
        mspStream.put_char(inChar, nullptr);
    }


}

void loop()
{
    msp_serial_ptr->process_input();
}


void MspSerial::process_input()
{
    while (_msp_serial.available() > 0) {
        const uint8_t inChar = _msp_serial.read();
        if (inChar < 0x10) { Serial.print("0"); }
        Serial.print(inChar, HEX);
        Serial.print(" ");
        Serial.print(inChar);
        Serial.println();

        Serial.print(inChar);
        //_msp_stream.put_char(inChar, nullptr); // This will invoke MspSerial::send_frame(), when a completed frame is received
        // interpre data back from the flight controller
    }
}

/*!
Sends the packet to the flight controller
Called from  MspStreamserial_encode() which is called from MspStreamprocess_received_command() which is called from MspStreamput_char()
*/
size_t MspSerial::send_frame(const uint8_t* hdr, size_t hdr_len, const uint8_t* data, size_t data_len, const uint8_t* crc, size_t crc_len)
{
    const int total_frame_length = hdr_len + data_len + crc_len;

    StreamBufWriter sbuf(&_buffer[0], sizeof(_buffer));

    // copy the frame into a StreamBuf
    sbuf.write_data(hdr, hdr_len);
    sbuf.write_data(data, data_len);
    sbuf.write_data(crc, crc_len);
    sbuf.switch_to_reader();

    while (sbuf.bytes_remaining() > 0) {
        const size_t available = _msp_serial.available_for_write();
        const size_t writeLen = std::min(available, static_cast<size_t>(sbuf.bytes_remaining()));
        _msp_serial.write(sbuf.ptr(), writeLen);
        sbuf.advance(writeLen);
        delay(1);
    }

    return total_frame_length;
}
