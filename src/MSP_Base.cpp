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

#include "MSP_Base.h"
#include "MSP_Protocol_Base.h"

#if false
enum defaultsType_e {
    DEFAULTS_TYPE_BASE = 0,
    DEFAULTS_TYPE_CUSTOM,
};
#endif

void MSP_Base::rebootFn(serialPort_t* serialPort)
{
    (void)serialPort;
}

MSP_Base::result_e  MSP_Base::setPassthroughCommand(StreamBuf& dst, StreamBuf& src, postProcessFnPtr* postProcessFn) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)postProcessFn;

    const size_t dataSize = src.bytesRemaining();
    if (dataSize == 0) {
        // Legacy format
        _passthroughMode = PASSTHROUGH_ESC_4WAY;
    } else {
        _passthroughMode = src.readU8();
        _passthroughArgument = src.readU8();
    }

    switch (_passthroughMode) {
    case PASSTHROUGH_SERIAL_ID:
    case PASSTHROUGH_SERIAL_FUNCTION_ID:
#if false
        if (findPassthroughSerialPort()) {
            if (postProcessFn) {
                *postProcessFn = serialPassthroughFn;
            }
            dst.writeU8(1);
        } else {
            dst.writeU8(0);
        }
#endif
        break;
#ifdef USE_SERIAL_4WAY_BLHELI_INTERFACE
    case MSP_PASSTHROUGH_ESC_4WAY:
        // get channel number
        // switch all motor lines HI
        // reply with the count of ESC found
        dst.writeU8(esc4wayInit());

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
            dst.writeU8(1);

            if (mspPostProcessFn) {
                *mspPostProcessFn = mspEscPassthroughFn;
            }

            break;
        }
        FALLTHROUGH;
#endif // USE_ESCSERIAL
#endif // USE_SERIAL_4WAY_BLHELI_INTERFACE
    default:
        dst.writeU8(0);
    }
    return RESULT_ACK;
}

MSP_Base::result_e MSP_Base::processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)srcDesc;
    (void)postProcessFn;

    switch (cmdMSP) { // NOLINT(hicpp-multiway-paths-covered)
    case MSP_BASE_API_VERSION:
        dst.writeU8(MSP_BASE_PROTOCOL_VERSION);
        dst.writeU8(MSP_BASE_API_VERSION_MAJOR);
        dst.writeU8(MSP_BASE_API_VERSION_MINOR);
        break;
    default:
        return RESULT_CMD_UNKNOWN;
    }
    return RESULT_ACK;
}

MSP_Base::result_e MSP_Base::processOutCommand(int16_t cmdMSP, StreamBuf& dst, descriptor_t srcDesc, postProcessFnPtr* postProcessFn, StreamBuf& src) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)src;
    return processOutCommand(cmdMSP, dst, srcDesc, postProcessFn);
}

MSP_Base::result_e MSP_Base::processInCommand(int16_t cmdMSP, StreamBuf& src, descriptor_t srcDesc, postProcessFnPtr* postProcessFn) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)cmdMSP;
    (void)src;
    (void)srcDesc;
    (void)postProcessFn;
    return RESULT_CMD_UNKNOWN;
}
/*
Returns RESULT_ACK, RESULT_ERROR or RESULT_NO_REPLY
*/
MSP_Base::result_e MSP_Base::processCommand(packet_t& cmd, packet_t& reply, descriptor_t srcDesc, postProcessFnPtr* postProcessFn)
{
    StreamBuf& dst = reply.payload;
    StreamBuf& src = cmd.payload;
    const int16_t cmdMSP = cmd.cmd; // NOLINT(cppcoreguidelines-init-variables) false positive
    // initialize reply by default
    reply.cmd = cmd.cmd;

    MSP_Base::result_e ret = processOutCommand(cmdMSP, dst, srcDesc, postProcessFn, src); // NOLINT(cppcoreguidelines-init-variables) false positive
    if (ret == RESULT_CMD_UNKNOWN) {
        if (cmdMSP == MSP_BASE_SET_PASSTHROUGH) {
            ret = setPassthroughCommand(dst, src, postProcessFn);
#ifdef USE_FLASHFS
        } else if (cmdMSP == MSP_DATAFLASH_READ) {
            ret = mspFcDataFlashReadCommand(dst, src);
#endif
        } else {
            ret = processInCommand(cmdMSP, src, srcDesc, postProcessFn); // chains to processInCommand
        }
    }
    reply.result = ret;
    return ret;
}

void MSP_Base::processReply(const packet_t& reply)
{
    (void)reply;
}
