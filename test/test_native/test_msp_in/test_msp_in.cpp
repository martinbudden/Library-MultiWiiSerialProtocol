#include <MSP_Protocol_Base.h>
#include <MSP_Serial.h>
#include <MSP_Stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)
class MSP_Test : public MSP_Base {
public:
    enum { MSP_SET_NAME = 11 };
public:
    virtual result_e processInCommand(int16_t cmdMSP, StreamBuf& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
public:
    std::array<uint8_t, 8> _name;
};

/*
MSP_SET_* commands handled in processInCommand
*/
MSP_Base::result_e MSP_Test::processInCommand(int16_t cmdMSP, StreamBuf& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)srcDesc;
    (void)postProcessFn;

    switch (cmdMSP) { // NOLINT(hicpp-multiway-paths-covered)
    case MSP_SET_NAME: {
        _name.fill(0xFF);
        size_t ii = 0;
        while (src.bytesRemaining()) {
            _name[ii++] = src.readU8();
        }
        _name[ii] = 0; // zero terminate
        break;
    }
    default:
        return RESULT_ERROR;
    }
    return RESULT_ACK;
}

void test_msp_set_name()
{
    static MSP_Test msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 6;
    const uint8_t checksum = 30;
    const std::array<uint8_t, 14> inStream = {
        '$', 'M', '<', payloadSize, MSP_Test::MSP_SET_NAME,
        'M', 'y', 'N', 'a', 'm', 'e',
        checksum
    };
#if false
    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.putChar(inChar, &pwh);
        if (eof) {
            break;
        }
    }
#endif

    bool complete = mspStream.putChar(inStream[0], &pwh); // $
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState());

    complete = mspStream.putChar(inStream[1], &pwh); // M
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());

    mspStream.putChar(inStream[2], &pwh); // <
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());

    mspStream.putChar(inStream[3], &pwh); // size = 6
    TEST_ASSERT_EQUAL(6, mspStream.getCheckSum1());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[4], &pwh); // P1
    TEST_ASSERT_EQUAL(13, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[5], &pwh);
    TEST_ASSERT_EQUAL(64, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[6], &pwh);
    TEST_ASSERT_EQUAL(57, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[7], &pwh);
    TEST_ASSERT_EQUAL(119, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[8], &pwh);
    TEST_ASSERT_EQUAL(22, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[9], &pwh);
    TEST_ASSERT_EQUAL(123, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());

    mspStream.putChar(inStream[10], &pwh);
    TEST_ASSERT_EQUAL(30, mspStream.getCheckSum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());

    complete = mspStream.putChar(inStream[11], &pwh); // checksum
    TEST_ASSERT_EQUAL(30, mspStream.getCheckSum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState()); // putChar sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(11, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.dataLen);

    TEST_ASSERT_EQUAL(msp._name[0], 'M');
    TEST_ASSERT_EQUAL(msp._name[1], 'y');
    TEST_ASSERT_EQUAL(msp._name[2], 'N');
    TEST_ASSERT_EQUAL(msp._name[3], 'a');
    TEST_ASSERT_EQUAL(msp._name[4], 'm');
    TEST_ASSERT_EQUAL(msp._name[5], 'e');
    TEST_ASSERT_EQUAL(msp._name[6], 0);
    TEST_ASSERT_EQUAL(msp._name[7], 0xFF);
}

void test_msp_set_name_loop()
{
    static MSP_Test msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    MSP_Stream::packet_with_header_t pwh;

    const uint8_t payloadSize = 6;
    const uint8_t checksum = 30;
    const std::array<uint8_t, 13> inStream = {
        'M', '<', payloadSize, MSP_Test::MSP_SET_NAME,
        'M', 'y', 'N', 'a', 'm', 'e',
        checksum
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.putChar(inChar, &pwh);
        (void)eof;
        //if (eof) { break; }
    }

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(11, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.dataLen);

    TEST_ASSERT_EQUAL(msp._name[0], 'M');
    TEST_ASSERT_EQUAL(msp._name[1], 'y');
    TEST_ASSERT_EQUAL(msp._name[2], 'N');
    TEST_ASSERT_EQUAL(msp._name[3], 'a');
    TEST_ASSERT_EQUAL(msp._name[4], 'm');
    TEST_ASSERT_EQUAL(msp._name[5], 'e');
    TEST_ASSERT_EQUAL(msp._name[6], 0);
    TEST_ASSERT_EQUAL(msp._name[7], 0xFF);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_msp_set_name);
    RUN_TEST(test_msp_set_name_loop);

    UNITY_END();
}
