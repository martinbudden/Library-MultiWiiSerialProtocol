#include <MSP_Protocol_Base.h>
#include <MSP_Serial.h>
#include <MSP_SerialPortBase.h>
#include <MSP_Stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}
// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-equals-delete,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-equals-delete,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)
class MSP_Test : public MSP_Base {
public:
    enum { MSP_ATTITUDE = 108 };
public:
    virtual result_e processGetCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
};

MSP_Base::result_e MSP_Test::processGetCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn)
{
    (void)srcDesc;
    (void)postProcessFn;

    switch (cmdMSP) {
    case MSP_BASE_API_VERSION:
        dst.writeU8(MSP_BASE_PROTOCOL_VERSION);
        dst.writeU8(MSP_BASE_API_VERSION_MAJOR);
        dst.writeU8(MSP_BASE_API_VERSION_MINOR);
        break;

    case MSP_ATTITUDE:
        dst.writeU16(100);
        dst.writeU16(200);
        dst.writeU16(300);
        break;
    default:
        return RESULT_CMD_UNKNOWN;
    }

    return RESULT_ACK;
}

class MSP_SerialPortTest : public MSP_SerialPortBase
{
public:
    virtual ~MSP_SerialPortTest() = default;
    MSP_SerialPortTest() = default;
private:
    // MSP_SerialPortTest is not copyable or moveable
    MSP_SerialPortTest(const MSP_SerialPortTest&) = delete;
    MSP_SerialPortTest& operator=(const MSP_SerialPortTest&) = delete;
    MSP_SerialPortTest(MSP_SerialPortTest&&) = delete;
    MSP_SerialPortTest& operator=(MSP_SerialPortTest&&) = delete;
public:
    bool isDataAvailable() const override { return true; }
    uint8_t readByte() override { return 0; }
    size_t availableForWrite() const override { return 100; }
    size_t write(uint8_t* buf, size_t len) override { (void)buf; return len; }
};

class MSP_SerialTest : public MSP_Serial {
public:
    virtual ~MSP_SerialTest() = default;
    MSP_SerialTest(MSP_Stream& mspStream, MSP_SerialPortBase& mspSerialPort) : MSP_Serial(mspStream, mspSerialPort) {}
public:
    // MSP_SerialTest is not copyable or moveable
    MSP_SerialTest(const MSP_SerialTest&) = delete;
    MSP_SerialTest& operator=(const MSP_SerialTest&) = delete;
    MSP_SerialTest(MSP_SerialTest&&) = delete;
    MSP_SerialTest& operator=(MSP_SerialTest&&) = delete;
private:
    virtual size_t sendFrame(const uint8_t* hdr, size_t hdrLen, const uint8_t* data, size_t dataLen, const uint8_t* crc, size_t crcLen) override;
    virtual void processInput() override;
};

/*!
Sends the packet to the flight controller
Called from  MSP_Stream::serialEncode() which is called from MSP_Stream::processReceivedCommand() which is called from MSP_Stream::putChar()

In "normal" implementations writes the frame to the serial port. 
Here we just check that the correct value has been received.
*/
size_t MSP_SerialTest::sendFrame(const uint8_t* hdr, size_t hdrLen, const uint8_t* data, size_t dataLen, const uint8_t* crc, size_t crcLen)
{
    (void)crc;
    TEST_ASSERT_EQUAL('$', hdr[0]);
    TEST_ASSERT_EQUAL('M', hdr[1]);
    TEST_ASSERT_EQUAL('>', hdr[2]);
    TEST_ASSERT_EQUAL(5, hdrLen);
    if (hdr[4] == MSP_BASE_API_VERSION) {
        TEST_ASSERT_EQUAL(3, dataLen); 
        TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, data[0]);
        TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, data[1]);
        TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, data[2]);
        TEST_ASSERT_EQUAL(1, crcLen);
        static constexpr uint8_t replyChecksum = 44;
        TEST_ASSERT_EQUAL(replyChecksum, *crc);
    } else if (hdr[4] == MSP_Test::MSP_ATTITUDE) {
        TEST_ASSERT_EQUAL(6, dataLen); 
        TEST_ASSERT_EQUAL(100, data[0]);
        TEST_ASSERT_EQUAL(0, data[1]);
        TEST_ASSERT_EQUAL(200, data[2]);
        TEST_ASSERT_EQUAL(0, data[3]);
        TEST_ASSERT_EQUAL(44, data[4]);
        TEST_ASSERT_EQUAL(1, data[5]);
        TEST_ASSERT_EQUAL(0, data[6]);
        TEST_ASSERT_EQUAL(1, crcLen);
        static constexpr uint8_t replyChecksum = 235;
        TEST_ASSERT_EQUAL(replyChecksum, *crc);
    } else {
        TEST_ASSERT_EQUAL(0, hdr[4]);
    }

    return 0;
}

void MSP_SerialTest::processInput()
{
}

void test_msp_out()
{
    static MSP_Base msp;
    static const MSP_Stream mspStream(msp);

    std::array<uint8_t, 128> buf;
    StreamBuf sbuf(&buf[0], sizeof(buf)); // NOLINT(cppcoreguidelines-init-variables)

    msp.processGetCommand(MSP_BASE_API_VERSION, sbuf, 0, nullptr);
    TEST_ASSERT_EQUAL(sizeof(buf) - 3, sbuf.bytesRemaining());
    sbuf.switchToReader();
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, sbuf.readU8());
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, sbuf.readU8());
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, sbuf.readU8());
}

void test_putchar()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    bool complete = mspStream.putChar('M', &pwh);
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());

    mspStream.putChar('<', &pwh); // command packet
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());

    mspStream.putChar(1, &pwh); // size
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());

    mspStream.putChar(MSP_BASE_API_VERSION, &pwh); // command
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    const uint8_t payload = 19;
    complete = mspStream.putChar(19, &pwh); // arbitrary 1-byte payload
    TEST_ASSERT_EQUAL(payload, mspStream.getCheckSum1()); // after first put, checksum is payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());

    const uint8_t checkSum = 19;
    complete = mspStream.putChar(checkSum, &pwh);
    TEST_ASSERT_EQUAL(checkSum, mspStream.getCheckSum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(3, pwh.hdrBuf[3]);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION, pwh.hdrBuf[4]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    static constexpr uint8_t replyChecksum = 44;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

void test_putchar_array_stream()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 1;
    const uint8_t payload = 19;
    const uint8_t checksum = payload; // for 1-byte payload, checksum is payload
    const std::array<uint8_t, 6> inStream = {
        'M', '<', payloadSize, MSP_BASE_API_VERSION, payload, checksum,
    };

    bool complete = mspStream.putChar(inStream[0], &pwh);
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.putChar(inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.putChar(inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());

    mspStream.putChar(inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    complete = mspStream.putChar(inStream[4], &pwh); // 1-byte payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());
    const uint8_t checkSum = mspStream.getCheckSum1();
    TEST_ASSERT_EQUAL(inStream[4], checkSum); // after first put, checksum is payload

    complete = mspStream.putChar(inStream[5], &pwh); // checksum
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE
    TEST_ASSERT_EQUAL(inStream[5], mspStream.getCheckSum1());


    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    static constexpr uint8_t replyChecksum = 44;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

void test_putchar_array_stream_no_payload()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    const uint8_t payloadSize = 0;
    const uint8_t type = MSP_BASE_API_VERSION;
    const uint8_t checksum = type; // for 0-byte payload, checksum is type
    const std::array<uint8_t, 5> inStream = {
        'M', '<', payloadSize, type, checksum,
    };

    MSP_Stream::packet_with_header_t pwh;

    bool complete = mspStream.putChar(inStream[0], &pwh);
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());

    mspStream.putChar(inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());

    mspStream.putChar(inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());

    complete = mspStream.putChar(inStream[4], &pwh); // checksum
    TEST_ASSERT_EQUAL(inStream[4], mspStream.getCheckSum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    static constexpr uint8_t replyChecksum = 44;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

void test_putchar_array_stream_loop()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 0;
    const uint8_t type = MSP_BASE_API_VERSION;
    const uint8_t checksum = type; // for 0-byte payload, checksum is type
    const std::array<uint8_t, 6> inStream = {
        '$', 'M', '<', payloadSize, type, checksum,
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.putChar(inChar, &pwh);
        if (eof) {
            break;
        }
    }

    TEST_ASSERT_EQUAL(inStream[4], mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(3, pwh.hdrBuf[3]);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION, pwh.hdrBuf[4]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    static constexpr uint8_t replyChecksum = 44;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

void test_msp_attitude()
{
    static MSP_Test msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 0;
    const uint8_t type = MSP_Test::MSP_ATTITUDE; // 108
    const uint8_t checksum = type; // for 0-byte payload, checksum is type
    const std::array<uint8_t, 6> inStream = {
        '$', 'M', '<', payloadSize, type, checksum,
    };

    const uint8_t computedChecksum = MSP_Stream::checksumXOR(0, &inStream[3], 2);
    TEST_ASSERT_EQUAL(108, computedChecksum);

    // simulate reading from serial port
    //for (uint8_t inChar : inStream) {
    //    mspStream.putChar(inChar, &pwh);
    //}

    bool complete = mspStream.putChar(inStream[0], &pwh);
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState());

    complete = mspStream.putChar(inStream[1], &pwh);
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());

    mspStream.putChar(inStream[2], &pwh); // command packet
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());

    mspStream.putChar(inStream[3], &pwh); // size
    TEST_ASSERT_EQUAL(0, pwh.checksum);
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[4], &pwh); // type
    TEST_ASSERT_EQUAL(0, pwh.checksum);
    TEST_ASSERT_EQUAL(checksum, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());

//#if false
    mspStream.putChar(inStream[5], &pwh); // checksum
    //TEST_ASSERT_EQUAL(235, pwh.checksum);
    //TEST_ASSERT_EQUAL(checksum, mspStream.getCheckSum1());
    //TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState());
#if false

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(6, pwh.hdrBuf[3]);
    TEST_ASSERT_EQUAL(type, pwh.hdrBuf[4]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(1, pwh.crcLen);
    static constexpr uint8_t replyChecksum = 235;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.crcBuf[0]);
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(100, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(0, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(200, *(pwh.dataPtr+2));
    TEST_ASSERT_EQUAL(0, *(pwh.dataPtr+3));
    TEST_ASSERT_EQUAL(44, *(pwh.dataPtr+4));
    TEST_ASSERT_EQUAL(1, *(pwh.dataPtr+5));
    TEST_ASSERT_EQUAL(0, *(pwh.dataPtr+6));
#endif
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-equals-delete,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-equals-delete,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_msp_out);
    RUN_TEST(test_putchar);
    RUN_TEST(test_putchar_array_stream);
    RUN_TEST(test_putchar_array_stream_no_payload);
    RUN_TEST(test_putchar_array_stream_loop);
    RUN_TEST(test_msp_attitude);

    UNITY_END();
}
