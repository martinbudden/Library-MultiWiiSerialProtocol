Notes.mb

enum msp_result_e {
    MSP_RESULT_ACK = 1,
    MSP_RESULT_ERROR = -1,
    MSP_RESULT_NO_REPLY = 0,
    MSP_RESULT_CMD_UNKNOWN = -2,   // don't know how to process command, try next handler
};

MSP task calls _msp_serial.process_input()
    calls _msp_stream.put_char
        calls process_received_command
            calls _msp_base.process_command
                calls process_get_set_command (gets parameters and writes dst) returns MSP_RESULT_ACK or MSP_RESULT_CMD_UNKNOWN
                or process_set_command (reads src and sets parameters) returns MSP_RESULT_ERROR or MSP_RESULT_ACK
            serial encodes frame into MSP format
                calls _msp_serial->send_frame
                    calls _msp_serial_port.write