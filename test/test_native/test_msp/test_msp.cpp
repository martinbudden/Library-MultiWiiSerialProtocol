#include <MSP_Protocol_Base.h>
#include <MSP_SerialBase.h>
#include <MSP_Stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)
void test_msp_state()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    mspStream.processReceivedPacketData('M');
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData('>'); // reply packet
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_REPLY, mspStream.getPacketType());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(3); // size
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(3, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(MSP_BASE_API_VERSION); // command
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(2, mspStream.getCheckSum1());

    // 3 bytes of data
    mspStream.processReceivedPacketData(1);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(3, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(2);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(3);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(2, mspStream.getCheckSum1());

    // checksum
    mspStream.processReceivedPacketData(2);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_COMMAND_RECEIVED, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(2, mspStream.getCheckSum1());

    // next byte puts port into idle
    mspStream.processReceivedPacketData(0);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_IDLE, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());
}

void test_get_msp_base_api_version()
{
    static MSP_Base msp;
    static MSP_Stream mspStream(msp);

    mspStream.setPacketState(MSP_Stream::MSP_IDLE);

    mspStream.processReceivedPacketData('M');
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_M, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData('<'); // command packet
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PACKET_COMMAND, mspStream.getPacketType());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(1); // size
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_HEADER_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(1, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(MSP_BASE_API_VERSION); // command
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_PAYLOAD_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(0, mspStream.getCheckSum1());

    // checksum
    mspStream.processReceivedPacketData(3);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_CHECKSUM_V1, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(3, mspStream.getCheckSum1());

    mspStream.processReceivedPacketData(3);
    TEST_ASSERT_EQUAL(MSP_Stream::MSP_COMMAND_RECEIVED, mspStream.getPacketState());
    TEST_ASSERT_EQUAL(3, mspStream.getCheckSum1());

    MSP_Base::packet_t reply = mspStream.processInbuf();

    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION, reply.cmd);
    const uint8_t b0 = reply.payload.readU8();
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, b0);
    const uint8_t b1 = reply.payload.readU8();
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, b1);
    const uint8_t b2 = reply.payload.readU8();
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, b2);

    reply.payload.switchToReader(); // change streambuf direction
    const MSP_Stream::packet_with_header_t pwh = mspStream.serialEncode(reply, MSP_Base::V1); // encode with MSP version 1

    TEST_ASSERT_EQUAL('$', pwh.hdrBuf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdrBuf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdrBuf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdrLen);
    TEST_ASSERT_EQUAL(44, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.dataLen);
    TEST_ASSERT_EQUAL(MSP_BASE_PROTOCOL_VERSION, *pwh.dataPtr);
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MAJOR, *(pwh.dataPtr+1));
    TEST_ASSERT_EQUAL(MSP_BASE_API_VERSION_MINOR, *(pwh.dataPtr+2));
}

// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_msp_state);
    RUN_TEST(test_get_msp_base_api_version);

    UNITY_END();
}
