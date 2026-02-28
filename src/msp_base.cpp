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
#include "msp_protocol.h"

#if false
enum defaultsType_e {
    DEFAULTS_TYPE_BASE = 0,
    DEFAULTS_TYPE_CUSTOM,
};
#endif

msp_result_e MspBase::process_write_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufWriter& dst, StreamBufReader& src) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)pg;
    (void)src;

    switch (cmd_msp) { // NOLINT(hicpp-multiway-paths-covered)
    case MSP_API_VERSION:
        dst.write_u8(MSP_PROTOCOL_VERSION);
        dst.write_u8(MSP_API_VERSION_MAJOR);
        dst.write_u8(MSP_API_VERSION_MINOR);
        return MSP_RESULT_ACK;
    default:
        return MSP_RESULT_CMD_UNKNOWN;
    }
}

msp_result_e MspBase::process_read_command(msp_parameter_group_t& pg, int16_t cmd_msp, StreamBufReader& src) // NOLINT(readability-convert-member-functions-to-static)
{
    (void)pg;
    (void)cmd_msp;
    (void)src;

    return MSP_RESULT_CMD_UNKNOWN;
}
/*
Returns MSP_RESULT_ACK, MSP_RESULT_ERROR or MSP_RESULT_NO_REPLY
*/
msp_result_e MspBase::process_command(msp_parameter_group_t& pg, const msp_const_packet_t& cmd, msp_packet_t& reply)
{
    StreamBufWriter& dst = reply.payload;
    StreamBufReader src(cmd.payload);
    // initialize reply by default
    reply.cmd = cmd.cmd;

    msp_result_e ret = process_write_command(pg, cmd.cmd, dst, src); // NOLINT(cppcoreguidelines-init-variables) false positive
    if (ret == MSP_RESULT_CMD_UNKNOWN) {
        ret = process_read_command(pg, cmd.cmd, src); // chains to processReadCommand
    }
    reply.result = ret;
    return ret;
}

void MspBase::process_reply(msp_parameter_group_t& pg, const msp_packet_t& reply)
{
    (void)pg;
    (void)reply;
}
