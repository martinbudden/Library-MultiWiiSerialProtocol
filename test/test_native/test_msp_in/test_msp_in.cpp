#include <msp_protocol.h>
#include <msp_serial.h>
#include <msp_stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

struct msp_parameter_group_t {
};

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)
class MspTest : public MspBase {
public:
    enum { MSP_SET_NAME = 11 };
public:
    virtual msp_result_e process_set_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufReader& src) override;
public:
    std::array<uint8_t, 8> _name;
};

/*
MSP_SET_* commands handled in process_set_command
*/
msp_result_e MspTest::process_set_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufReader& src) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)pg;

    switch (cmd_msp) { // NOLINT(hicpp-multiway-paths-covered)
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
    static MspTest msp;
    static MspStream msp_stream(msp);
    static msp_parameter_group_t pg;

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh {};

    const uint8_t payloadSize = 6;
    const uint8_t checksum = 30;
    const std::array<uint8_t, 14> inStream = {
        '$', 'M', '<', payloadSize, MspTest::MSP_SET_NAME,
        'M', 'y', 'N', 'a', 'm', 'e',
        checksum
    };
#if false
    // simulate reading from serial port
    for (uint8_t in_char : inStream) {
        const bool eof = msp_stream.put_char(pg, in_char, &pwh);
        if (eof) {
            break;
        }
    }
#endif

    bool complete = msp_stream.put_char(pg, inStream[0], &pwh); // $
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state());

    complete = msp_stream.put_char(pg, inStream[1], &pwh); // M
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[2], &pwh); // <
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());

    msp_stream.put_char(pg, inStream[3], &pwh); // size = 6
    TEST_ASSERT_EQUAL(6, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[4], &pwh); // P1
    TEST_ASSERT_EQUAL(13, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[5], &pwh);
    TEST_ASSERT_EQUAL(64, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[6], &pwh);
    TEST_ASSERT_EQUAL(57, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[7], &pwh);
    TEST_ASSERT_EQUAL(119, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[8], &pwh);
    TEST_ASSERT_EQUAL(22, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[9], &pwh);
    TEST_ASSERT_EQUAL(123, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[10], &pwh);
    TEST_ASSERT_EQUAL(30, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());

    complete = msp_stream.put_char(pg, inStream[11], &pwh); // checksum
    TEST_ASSERT_EQUAL(30, msp_stream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

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
    static MspTest msp;
    static MspStream msp_stream(msp);
    static msp_parameter_group_t pg;

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payloadSize = 6;
    const uint8_t checksum = 30;
    const std::array<uint8_t, 13> inStream = {
        'M', '<', payloadSize, MspTest::MSP_SET_NAME,
        'M', 'y', 'N', 'a', 'm', 'e',
        checksum
    };

    // simulate reading from serial port
    for (uint8_t in_char : inStream) {
        const bool eof = msp_stream.put_char(pg, in_char, &pwh);
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
    static MspTest msp;
    static MspStream msp_stream(msp);

    const std::array<uint8_t, 6> buf = {
        'M', 'y', 'N', 'a', 'm', 'e',
    };
    TEST_ASSERT_EQUAL(6, buf.size());

    const msp_stream_packet_with_header_t pwh = msp_stream.serial_encode_msp_v1(MspTest::MSP_SET_NAME, &buf[0], buf.size());

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
