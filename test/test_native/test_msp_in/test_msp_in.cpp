#include <msp_protocol_base.h>
#include <msp_serial.h>
#include <msp_stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)
class MSP_Test : public MspBase {
public:
    enum { MSP_SET_NAME = 11 };
public:
    virtual msp_result_e process_set_command(int16_t cmdMSP, StreamBufReader& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) override;
public:
    std::array<uint8_t, 8> _name;
};

/*
MSP_SET_* commands handled in process_set_command
*/
msp_result_e MSP_Test::process_set_command(int16_t cmdMSP, StreamBufReader& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)srcDesc;
    (void)postProcessFn;

    switch (cmdMSP) { // NOLINT(hicpp-multiway-paths-covered)
    case MSP_SET_NAME: {
        _name.fill(0xFF);
        size_t ii = 0;
        while (src.bytes_remaining()) {
            _name[ii++] = src.read_u8();
        }
        _name[ii] = 0; // zero terminate
        break;
    }
    default:
        return MSP_RESULT_ERROR;
    }
    return MSP_RESULT_ACK;
}

void test_msp_set_name()
{
    static MSP_Test msp;
    static MspStream mspStream(msp);

    mspStream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh {};

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
        const bool eof = mspStream.put_char(inChar, &pwh);
        if (eof) {
            break;
        }
    }
#endif

    bool complete = mspStream.put_char(inStream[0], &pwh); // $
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, mspStream.get_packet_state());

    complete = mspStream.put_char(inStream[1], &pwh); // M
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, mspStream.get_packet_state());

    mspStream.put_char(inStream[2], &pwh); // <
    TEST_ASSERT_EQUAL(0, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, mspStream.get_packet_type());

    mspStream.put_char(inStream[3], &pwh); // size = 6
    TEST_ASSERT_EQUAL(6, mspStream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, mspStream.get_packet_state());

    mspStream.put_char(inStream[4], &pwh); // P1
    TEST_ASSERT_EQUAL(13, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(inStream[5], &pwh);
    TEST_ASSERT_EQUAL(64, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(inStream[6], &pwh);
    TEST_ASSERT_EQUAL(57, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(inStream[7], &pwh);
    TEST_ASSERT_EQUAL(119, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(inStream[8], &pwh);
    TEST_ASSERT_EQUAL(22, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(inStream[9], &pwh);
    TEST_ASSERT_EQUAL(123, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, mspStream.get_packet_state());

    mspStream.put_char(inStream[10], &pwh);
    TEST_ASSERT_EQUAL(30, mspStream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, mspStream.get_packet_state());

    complete = mspStream.put_char(inStream[11], &pwh); // checksum
    TEST_ASSERT_EQUAL(30, mspStream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, mspStream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(11, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.data_len);

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
    static MspStream mspStream(msp);

    mspStream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payloadSize = 6;
    const uint8_t checksum = 30;
    const std::array<uint8_t, 13> inStream = {
        'M', '<', payloadSize, MSP_Test::MSP_SET_NAME,
        'M', 'y', 'N', 'a', 'm', 'e',
        checksum
    };

    // simulate reading from serial port
    for (uint8_t inChar : inStream) {
        const bool eof = mspStream.put_char(inChar, &pwh);
        (void)eof;
        //if (eof) { break; }
    }

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(11, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.data_len);

    TEST_ASSERT_EQUAL('M', msp._name[0]);
    TEST_ASSERT_EQUAL('y', msp._name[1]);
    TEST_ASSERT_EQUAL('N', msp._name[2]);
    TEST_ASSERT_EQUAL('a', msp._name[3]);
    TEST_ASSERT_EQUAL('m', msp._name[4]);
    TEST_ASSERT_EQUAL('e', msp._name[5]);
    TEST_ASSERT_EQUAL(0, msp._name[6]);
    TEST_ASSERT_EQUAL(0xFF, msp._name[7]);
}

void test_msp_set_name_serial_encode_v1()
{
    static MSP_Test msp;
    static MspStream mspStream(msp);

    const std::array<uint8_t, 6> buf = {
        'M', 'y', 'N', 'a', 'm', 'e',
    };
    TEST_ASSERT_EQUAL(6, buf.size());

    const msp_stream_packet_with_header_t pwh = mspStream.serial_encode_msp_v1(MSP_Test::MSP_SET_NAME, &buf[0], buf.size());

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]); // '>' for success '!' for error
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(11, pwh.checksum);
    TEST_ASSERT_EQUAL(0, pwh.data_len);

    TEST_ASSERT_EQUAL('M', pwh.data_ptr[0]);
    TEST_ASSERT_EQUAL('y', pwh.data_ptr[1]);
    TEST_ASSERT_EQUAL('N', pwh.data_ptr[2]);
    TEST_ASSERT_EQUAL('a', pwh.data_ptr[3]);
    TEST_ASSERT_EQUAL('m', pwh.data_ptr[4]);
    TEST_ASSERT_EQUAL('e', pwh.data_ptr[5]);
}
// NOLINTEND(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    UNITY_BEGIN();

    RUN_TEST(test_msp_set_name);
    RUN_TEST(test_msp_set_name_loop);
    RUN_TEST(test_msp_set_name_serial_encode_v1);

    UNITY_END();
}
