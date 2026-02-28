#include <msp_protocol.h>
#include <msp_serial.h>
#include <msp_serial_port_base.h>
#include <msp_stream.h>

#include <unity.h>

void setUp() {
}

void tearDown() {
}

// NOLINTBEGIN(cppcoreguidelines-avoid-magic-numbers,cppcoreguidelines-explicit-virtual-functions,cppcoreguidelines-pro-bounds-pointer-arithmetic,hicpp-use-equals-delete,hicpp-use-override,misc-const-correctness,misc-non-private-member-variables-in-classes,modernize-use-equals-delete,modernize-use-override,readability-magic-numbers,readability-redundant-access-specifiers)
struct msp_parameter_group_t {
};

class MspTest : public MspBase {
public:
    enum { MSP_ATTITUDE = 108 };
public:
    virtual msp_result_e process_get_set_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufWriter& dst, StreamBufReader& src) override;
};

msp_result_e MspTest::process_get_set_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufWriter& dst, StreamBufReader& src)
{
    (void)pg;
    (void)src;

    switch (cmd_msp) {
    case MSP_API_VERSION:
        dst.write_u8(MSP_PROTOCOL_VERSION);
        dst.write_u8(MSP_API_VERSION_MAJOR);
        dst.write_u8(MSP_API_VERSION_MINOR);
        break;

    case MSP_ATTITUDE:
        dst.write_u16(100);
        dst.write_u16(200);
        dst.write_u16(300);
        break;
    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }

    return MSP_RESULT_ACK;
}

class MspSerialPortTest : public MspSerialPortBase
{
public:
    virtual ~MspSerialPortTest() = default;
    MspSerialPortTest() = default;
private:
    // MspSerialPortTest is not copyable or moveable
    MspSerialPortTest(const MspSerialPortTest&) = delete;
    MspSerialPortTest& operator=(const MspSerialPortTest&) = delete;
    MspSerialPortTest(MspSerialPortTest&&) = delete;
    MspSerialPortTest& operator=(MspSerialPortTest&&) = delete;
public:
    bool is_data_available() const override { return true; }
    uint8_t read_byte() override { return 0; }
    size_t available_for_write() const override { return 100; }
    size_t write(const uint8_t* buf, size_t len) override { (void)buf; return len; }
};

class MspSerialTest : public MspSerial {
public:
    virtual ~MspSerialTest() = default;
    MspSerialTest(MspStream& msp_stream, MspSerialPortBase& msp_serialPort) : MspSerial(msp_stream, msp_serialPort) {}
public:
    // MspSerialTest is not copyable or moveable
    MspSerialTest(const MspSerialTest&) = delete;
    MspSerialTest& operator=(const MspSerialTest&) = delete;
    MspSerialTest(MspSerialTest&&) = delete;
    MspSerialTest& operator=(MspSerialTest&&) = delete;
private:
    virtual size_t send_frame(const uint8_t* hdr, size_t hdr_len, const uint8_t* data, size_t data_len, const uint8_t* crc, size_t crc_len) override;
    virtual void process_input(msp_parameter_group_t& pg) override;
};

/*!
Sends the packet to the flight controller
Called from  MspStream::serial_encode() which is called from MspStream::process_received_command() which is called from MspStream::put_char()

In "normal" implementations writes the frame to the serial port.
Here we just check that the correct value has been received.
*/
size_t MspSerialTest::send_frame(const uint8_t* hdr, size_t hdr_len, const uint8_t* data, size_t data_len, const uint8_t* crc, size_t crc_len)
{
    (void)crc;
    TEST_ASSERT_EQUAL('$', hdr[0]);
    TEST_ASSERT_EQUAL('M', hdr[1]);
    TEST_ASSERT_EQUAL('>', hdr[2]);
    TEST_ASSERT_EQUAL(5, hdr_len);
    if (hdr[4] == MSP_API_VERSION) {
        TEST_ASSERT_EQUAL(3, data_len);
        TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, data[0]);
        TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, data[1]);
        TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, data[2]);
        TEST_ASSERT_EQUAL(1, crc_len);
        static constexpr uint8_t replyChecksum = 44;
        TEST_ASSERT_EQUAL(replyChecksum, *crc);
    } else if (hdr[4] == MspTest::MSP_ATTITUDE) {
        TEST_ASSERT_EQUAL(6, data_len);
        TEST_ASSERT_EQUAL(100, data[0]);
        TEST_ASSERT_EQUAL(0, data[1]);
        TEST_ASSERT_EQUAL(200, data[2]);
        TEST_ASSERT_EQUAL(0, data[3]);
        TEST_ASSERT_EQUAL(44, data[4]);
        TEST_ASSERT_EQUAL(1, data[5]);
        TEST_ASSERT_EQUAL(0, data[6]);
        TEST_ASSERT_EQUAL(1, crc_len);
        static constexpr uint8_t replyChecksum = 235;
        TEST_ASSERT_EQUAL(replyChecksum, *crc);
    } else {
        TEST_ASSERT_EQUAL(0, hdr[4]);
    }

    return 0;
}

void MspSerialTest::process_input(msp_parameter_group_t& pg)
{
    (void)pg;
}

void test_msp_out()
{
    static MspBase msp;
    static const MspStream msp_stream(msp);
    static msp_parameter_group_t pg;

    std::array<uint8_t, 128> dst_buf;
    StreamBufWriter dst(&dst_buf[0], sizeof(dst_buf)); // NOLINT(cppcoreguidelines-init-variables)
    std::array<uint8_t, 16> src_buf;
    StreamBufReader src(&src_buf[0], sizeof(src_buf)); // NOLINT(cppcoreguidelines-init-variables)

    msp.process_get_set_command(pg, MSP_API_VERSION, dst, src);
    TEST_ASSERT_EQUAL(sizeof(dst_buf) - 3, dst.bytes_remaining());
    dst.switch_to_reader();
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, dst.read_u8());
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, dst.read_u8());
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, dst.read_u8());
}

void test_putchar()
{
    static MspBase msp;
    static MspStream msp_stream(msp);
    static msp_parameter_group_t pg;

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    bool complete = msp_stream.put_char(pg, 'M', &pwh);
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());

    msp_stream.put_char(pg, '<', &pwh); // command packet
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());

    msp_stream.put_char(pg, 1, &pwh); // size
    TEST_ASSERT_EQUAL(1, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, MSP_API_VERSION, &pwh); // command
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());

    const uint8_t payload = 19;
    complete = msp_stream.put_char(pg, 19, &pwh); // arbitrary 1-byte payload
    TEST_ASSERT_EQUAL(payload, msp_stream.get_checksum1()); // after first put, checksum is payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());

    const uint8_t checksum = 19;
    complete = msp_stream.put_char(pg, checksum, &pwh);
    TEST_ASSERT_EQUAL(checksum, msp_stream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(3, pwh.hdr_buf[3]);
    TEST_ASSERT_EQUAL(MSP_API_VERSION, pwh.hdr_buf[4]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    static constexpr uint8_t replyChecksum = 44;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.data_len);
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, *(pwh.data_ptr+2));
}

void test_putchar_array_stream()
{
    static MspBase msp;
    static MspStream msp_stream(msp);
    static msp_parameter_group_t pg;

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payloadSize = 1;
    const uint8_t payload = 19;
    const uint8_t checksum = payload; // for 1-byte payload, checksum is payload
    const std::array<uint8_t, 6> inStream = {
        'M', '<', payloadSize, MSP_API_VERSION, payload, checksum,
    };

    bool complete = msp_stream.put_char(pg, inStream[0], &pwh);
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    msp_stream.put_char(pg, inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    msp_stream.put_char(pg, inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(1, msp_stream.get_checksum1());

    msp_stream.put_char(pg, inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(MSP_PAYLOAD_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());

    complete = msp_stream.put_char(pg, inStream[4], &pwh); // 1-byte payload
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());
    const uint8_t stream_checksum = msp_stream.get_checksum1();
    TEST_ASSERT_EQUAL(inStream[4], stream_checksum); // after first put, checksum is payload

    complete = msp_stream.put_char(pg, inStream[5], &pwh); // checksum
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE
    TEST_ASSERT_EQUAL(inStream[5], msp_stream.get_checksum1());


    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    static constexpr uint8_t replyChecksum = 44;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.data_len);
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, *(pwh.data_ptr+2));
}

void test_putchar_array_stream_no_payload()
{
    static MspBase msp;
    static MspStream msp_stream(msp);
    static msp_parameter_group_t pg;

    msp_stream.set_packet_state(MSP_IDLE);

    const uint8_t payloadSize = 0;
    const uint8_t type = MSP_API_VERSION;
    const uint8_t checksum = type; // for 0-byte payload, checksum is type
    const std::array<uint8_t, 5> inStream = {
        'M', '<', payloadSize, type, checksum,
    };

    msp_stream_packet_with_header_t pwh;

    bool complete = msp_stream.put_char(pg, inStream[0], &pwh);
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[1], &pwh); // command packet
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());

    msp_stream.put_char(pg, inStream[2], &pwh); // size = 1
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[3], &pwh); // command
    TEST_ASSERT_EQUAL(1, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());

    complete = msp_stream.put_char(pg, inStream[4], &pwh); // checksum
    TEST_ASSERT_EQUAL(inStream[4], msp_stream.get_checksum1());
    TEST_ASSERT_TRUE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    static constexpr uint8_t replyChecksum = 44;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.data_len);
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, *(pwh.data_ptr+2));
}

void test_putchar_array_stream_loop()
{
    static MspBase msp;
    static MspStream msp_stream(msp);
    static msp_parameter_group_t pg;

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payloadSize = 0;
    const uint8_t type = MSP_API_VERSION;
    const uint8_t checksum = type; // for 0-byte payload, checksum is type
    const std::array<uint8_t, 6> inStream = {
        '$', 'M', '<', payloadSize, type, checksum,
    };

    // simulate reading from serial port
    for (uint8_t in_char : inStream) {
        const bool eof = msp_stream.put_char(pg, in_char, &pwh);
        if (eof) {
            break;
        }
    }

    TEST_ASSERT_EQUAL(inStream[4], msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state()); // put_char sets from MSP_COMMAND_RECEIVED to MSP_IDLE

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(3, pwh.hdr_buf[3]);
    TEST_ASSERT_EQUAL(MSP_API_VERSION, pwh.hdr_buf[4]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    static constexpr uint8_t replyChecksum = 44;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(3, pwh.data_len);
    TEST_ASSERT_EQUAL(MSP_PROTOCOL_VERSION, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MAJOR, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(MSP_API_VERSION_MINOR, *(pwh.data_ptr+2));
}

void test_msp_attitude()
{
    static MspTest msp;
    static MspStream msp_stream(msp);
    static msp_parameter_group_t pg;

    msp_stream.set_packet_state(MSP_IDLE);

    msp_stream_packet_with_header_t pwh;

    const uint8_t payloadSize = 0;
    const uint8_t type = MspTest::MSP_ATTITUDE; // 108
    const uint8_t checksum = type; // for 0-byte payload, checksum is type
    const std::array<uint8_t, 6> inStream = {
        '$', 'M', '<', payloadSize, type, checksum,
    };

    const uint8_t computedChecksum = MspStream::checksum_xor(0, &inStream[3], 2);
    TEST_ASSERT_EQUAL(108, computedChecksum);

    // simulate reading from serial port
    //for (uint8_t in_char : inStream) {
    //    msp_stream.put_char(pg, in_char, &pwh);
    //}

    bool complete = msp_stream.put_char(pg, inStream[0], &pwh);
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state());

    complete = msp_stream.put_char(pg, inStream[1], &pwh);
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_FALSE(complete);
    TEST_ASSERT_EQUAL(MSP_HEADER_M, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[2], &pwh); // command packet
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());
    TEST_ASSERT_EQUAL(MSP_PACKET_COMMAND, msp_stream.get_packet_type());

    msp_stream.put_char(pg, inStream[3], &pwh); // size
    TEST_ASSERT_EQUAL(0, pwh.checksum);
    TEST_ASSERT_EQUAL(0, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_HEADER_V1, msp_stream.get_packet_state());

    msp_stream.put_char(pg, inStream[4], &pwh); // type
    TEST_ASSERT_EQUAL(0, pwh.checksum);
    TEST_ASSERT_EQUAL(checksum, msp_stream.get_checksum1());
    TEST_ASSERT_EQUAL(MSP_CHECKSUM_V1, msp_stream.get_packet_state());

//#if false
    msp_stream.put_char(pg, inStream[5], &pwh); // checksum
    //TEST_ASSERT_EQUAL(235, pwh.checksum);
    //TEST_ASSERT_EQUAL(checksum, msp_stream.get_checksum1());
    //TEST_ASSERT_EQUAL(MSP_IDLE, msp_stream.get_packet_state());
#if false

    TEST_ASSERT_EQUAL('$', pwh.hdr_buf[0]);
    TEST_ASSERT_EQUAL('M', pwh.hdr_buf[1]);
    TEST_ASSERT_EQUAL('>', pwh.hdr_buf[2]);
    TEST_ASSERT_EQUAL(6, pwh.hdr_buf[3]);
    TEST_ASSERT_EQUAL(type, pwh.hdr_buf[4]);
    TEST_ASSERT_EQUAL(5, pwh.hdr_len);
    TEST_ASSERT_EQUAL(1, pwh.crc_len);
    static constexpr uint8_t replyChecksum = 235;
    TEST_ASSERT_EQUAL(replyChecksum, pwh.crc_buf[0]);
    TEST_ASSERT_EQUAL(replyChecksum, pwh.checksum);
    TEST_ASSERT_EQUAL(100, *pwh.data_ptr);
    TEST_ASSERT_EQUAL(0, *(pwh.data_ptr+1));
    TEST_ASSERT_EQUAL(200, *(pwh.data_ptr+2));
    TEST_ASSERT_EQUAL(0, *(pwh.data_ptr+3));
    TEST_ASSERT_EQUAL(44, *(pwh.data_ptr+4));
    TEST_ASSERT_EQUAL(1, *(pwh.data_ptr+5));
    TEST_ASSERT_EQUAL(0, *(pwh.data_ptr+6));
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
