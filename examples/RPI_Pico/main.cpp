#include <Arduino.h>
#include <MSP_Protocol_Base.h>
#include <MSP_SerialBase.h>
#include <MSP_Stream.h>
#include <SoftwareSerial.h>
#include <boards/pico.h>

/*
See https://gist.github.com/Xorgon/d933ea8cc7d657ac309d6fc75374f836 for example basic Arduino sketch
*/

enum { SERIAL_INVERTED = false, SERIAL_NOT_INVERTED = true };
enum { SERIAL_RX = 11, SERIAL_TX = 12 };

static MSP_Stream* mspStreamPtr {};
static MSP_Serial* mspSerialPtr {};

class MSP_Serial : public MSP_SerialBase {
public:
    MSP_Serial(MSP_Stream& mspStream, SoftwareSerial& mspSerial) : _mspStream(mspStream), _mspSerial(mspSerial) {}
    virtual size_t sendFrame(const uint8_t* hdr, size_t hdrLen, const uint8_t* data, size_t dataLen, const uint8_t* crc, size_t crcLen) override;
    virtual void processInput() override;
private:
    MSP_Stream& _mspStream;
    SoftwareSerial& _mspSerial;
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

    static MSP_Base msp;
    static MSP_Stream mspStream(msp);
    static MSP_Serial mspSerial(mspStream, softwareSerial);

    mspStreamPtr = &mspStream;

    // create and send a packet to get the BASE API VERSION
    const uint8_t payloadSize = 0;
    const uint8_t checksum = 1; // for 0-byte payload, checksum is 1
    const std::array<uint8_t, 6> inStream = {
        '$', 'M', '<', payloadSize, MSP_BASE_API_VERSION, checksum,
    };
    for (uint8_t inChar : inStream) {
        mspStream.putChar(inChar, nullptr);
    }


}

void loop()
{
    mspSerialPtr->processInput();
}


void MSP_Serial::processInput()
{
    while (_mspSerial.available() > 0) {
        const uint8_t inChar = _mspSerial.read();
        if (inChar < 0x10) { Serial.print("0"); }
        Serial.print(inChar, HEX);
        Serial.print(" ");
        Serial.print(inChar);
        Serial.println();

        Serial.print(inChar);
        //_mspStream.putChar(inChar, nullptr); // This will invoke MSP_Serial::sendFrame(), when a completed frame is received
        // interpre data back from the flight controller
    }
}

/*!
Sends the packet to the flight controller
Called from  MSP_Stream::serialEncode() which is called from MSP_Stream::processReceivedCommand() which is called from MSP_Stream::putChar()
*/
size_t MSP_Serial::sendFrame(const uint8_t* hdr, size_t hdrLen, const uint8_t* data, size_t dataLen, const uint8_t* crc, size_t crcLen)
{
    const int totalFrameLength = hdrLen + dataLen + crcLen;

    StreamBuf sbuf(&_buffer[0], sizeof(_buffer));

    // copy the frame into a StreamBuf
    sbuf.writeData(hdr, hdrLen);
    sbuf.writeData(data, dataLen);
    sbuf.writeData(crc, crcLen);
    sbuf.switchToReader();

    while (sbuf.bytesRemaining() > 0) {
        const size_t available = _mspSerial.availableForWrite();
        const size_t writeLen = std::min(available, sbuf.bytesRemaining());
        _mspSerial.write(sbuf.ptr(), writeLen);
        sbuf.advance(writeLen);
        delay(1);
    }

    return totalFrameLength;
}
