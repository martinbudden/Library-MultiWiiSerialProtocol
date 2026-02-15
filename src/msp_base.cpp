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

#include "msp_base.h"
#include "msp_protocol_base.h"

#if false
enum defaultsType_e {
    DEFAULTS_TYPE_BASE = 0,
    DEFAULTS_TYPE_CUSTOM,
};
#endif

void MspBase::reboot_fn(serialPort_t* serialPort)
{
    (void)serialPort;
}

msp_result_e MspBase::set_passthrough_command(StreamBufWriter& dst, StreamBufReader& src, postProcessFnPtr* postProcessFn) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)postProcessFn;

    const size_t dataSize = src.bytes_remaining();
    if (dataSize == 0) {
        // Legacy format
        _passthrough_mode = PASSTHROUGH_ESC_4WAY;
    } else {
        _passthrough_mode = src.read_u8();
        _passthrough_argument = src.read_u8();
    }

    switch (_passthrough_mode) {
    case PASSTHROUGH_SERIAL_ID:
    case PASSTHROUGH_SERIAL_FUNCTION_ID:
#if false
        if (findPassthroughSerialPort()) {
            if (postProcessFn) {
                *postProcessFn = serialPassthroughFn;
            }
            dst.write_u8(1);
        } else {
            dst.write_u8(0);
        }
#endif
        break;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    case MSP_PASSTHROUGH_ESC_4WAY:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        dst.write_u8(esc4wayInit());

        if (mspPostProcessFn) {
            *mspPostProcessFn = esc4wayProcess;
        }
        break;

#ifdef USE_ESCSERIAL
    case MSP_PASSTHROUGH_ESC_SIMONK:
    case MSP_PASSTHROUGH_ESC_BLHELI:
    case MSP_PASSTHROUGH_ESC_KISS:
    case MSP_PASSTHROUGH_ESC_KISSALL:
    case MSP_PASSTHROUGH_ESC_CASTLE:
        if (mspPassthroughArgument < getMotorCount() || (mspPassthroughMode == MSP_PASSTHROUGH_ESC_KISS && mspPassthroughArgument == ALL_MOTORS)) {
            dst.write_u8(1);

            if (mspPostProcessFn) {
                *mspPostProcessFn = mspEscPassthroughFn;
            }

            break;
        }
        FALLTHROUGH;
#endif // USE_ESCSERIAL
#endif // USE_SERIAL_4WAY_BLHELI_INTERFACE
    default:
        dst.write_u8(0);
    }
    return MSP_RESULT_ACK;
}

msp_result_e MspBase::process_get_command(int16_t cmdMSP, StreamBufWriter& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)srcDesc;
    (void)postProcessFn;

    switch (cmdMSP) { // NOLINT(hicpp-multiway-paths-covered)
    case MSP_BASE_API_VERSION:
        dst.write_u8(MSP_BASE_PROTOCOL_VERSION);
        dst.write_u8(MSP_BASE_API_VERSION_MAJOR);
        dst.write_u8(MSP_BASE_API_VERSION_MINOR);
        break;
    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
    return MSP_RESULT_ACK;
}

msp_result_e MspBase::process_get_command(int16_t cmdMSP, StreamBufWriter& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn, StreamBufReader& src) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)src;
    return process_get_command(cmdMSP, dst, srcDesc, postProcessFn);
}

msp_result_e MspBase::process_set_command(int16_t cmdMSP, StreamBufReader& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)cmdMSP;
    (void)src;
    (void)srcDesc;
    (void)postProcessFn;
    return MSP_RESULT_CMD_UNKNOWN;
}
/*
Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
*/
msp_result_e MspBase::process_command(const msp_const_packet_t& cmd, msp_packet_t& reply, descriptor_t srcDesc, postProcessFnPtr* postProcessFn)
{
    StreamBufWriter& dst = reply.payload;
    StreamBufReader src(cmd.payload);
    const int16_t cmdMSP = cmd.cmd; // NOLINT(cppcoreguidelines-init-variables) false positive
    // initialize reply by default
    reply.cmd = cmd.cmd;

    msp_result_e ret = process_get_command(cmdMSP, dst, srcDesc, postProcessFn, src); // NOLINT(cppcoreguidelines-init-variables) false positive
    if (ret == MSP_RESULT_CMD_UNKNOWN) {
        if (cmdMSP == MSP_BASE_SET_PASSTHROUGH) {
            ret = set_passthrough_command(dst, src, postProcessFn);
#ifdef USE_FLASHFS
        } else if (cmdMSP == MSP_DATAFLASH_READ) {
            ret = mspFcDataFlashReadCommand(dst, src);
#endif
        } else {
            ret = process_set_command(cmdMSP, src, srcDesc, postProcessFn); // chains to processReadCommand
        }
    }
    reply.result = ret;
    return ret;
}

void MspBase::process_reply(const msp_packet_t& reply)
{
    (void)reply;
}
