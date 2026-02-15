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

/**
 * MSP Guidelines, emphasis is used to clarify.
 *
 * Each FlightController (FC, Server) MUST change the API version when any MSP command is added, deleted, or changed.
 *
 * If you fork the FC source code and release your own version, you MUST change the Flight Controller Identifier.
 *
 * NEVER release a modified copy of this code that shares the same Flight controller IDENT and API version
 * if the API doesn't match EXACTLY.
 *
 * Consumers of the API (API clients) SHOULD first attempt to get a response from the MSP_API_VERSION command.
 * If no response is obtained then client MAY try the legacy MSP_IDENT command.
 *
 * API consumers should ALWAYS handle communication failures gracefully and attempt to continue
 * without the information if possible.  Clients MAY log/display a suitable message.
 *
 * API clients should NOT attempt any communication if they can't handle the returned API MAJOR VERSION.
 *
 * API clients SHOULD attempt communication if the API MINOR VERSION has increased from the time
 * the API client was written and handle command failures gracefully.  Clients MAY disable
 * functionality that depends on the commands while still leaving other functionality intact.
 * that the newer API version may cause problems before using API commands that change FC state.
 *
 * It is for this reason that each MSP command should be specific as possible, such that changes
 * to commands break as little functionality as possible.
 *
 * API client authors MAY use a compatibility matrix/table when determining if they can support
 * a given command from a given flight controller at a given api version level.
 *
 * Developers MUST NOT create new MSP commands that do more than one thing.
 *
 * Failure to follow these guidelines will likely invoke the wrath of developers trying to write tools
 * that use the API and the users of those tools.
 */

#pragma once

#include "msp_protocol_base.h"

/* Protocol numbers used both by the wire format, config system, and
   field setters.
*/

//NOLINTBEGIN(modernize-macro-to-enum)

static constexpr uint8_t MSP_PROTOCOL_VERSION                = 0;

static constexpr uint8_t MSP_API_VERSION_MAJOR               = 1;  // increment when major changes are made
static constexpr uint8_t MSP_API_VERSION_MINOR               = 47; // increment after a release, to set the version for all changes to go into the following release (if no changes to MSP are made between the releases, this can be reverted before the release)

static constexpr uint8_t MSP_API_VERSION_LENGTH              = 2;

#define MULTIWII_IDENTIFIER "MWII";
#define BASEFLIGHT_IDENTIFIER "BAFL";
//#define BETAFLIGHT_IDENTIFIER "BTFL" Actual value stored in FC_FIRMWARE_IDENTIFIER in build/version.h
#define CLEANFLIGHT_IDENTIFIER "CLFL"
#define INAV_IDENTIFIER "INAV"
#define RACEFLIGHT_IDENTIFIER "RCFL"
#define PROTOFLIGHT_IDENTIFIER "PTFL"

#define FLIGHT_CONTROLLER_IDENTIFIER_LENGTH 4
#define FLIGHT_CONTROLLER_VERSION_LENGTH    3
#define FLIGHT_CONTROLLER_VERSION_MASK      0xFFF

#define BOARD_IDENTIFIER_LENGTH             4 // 4 UPPER CASE alpha numeric characters that identify the board being used.
#define BOARD_HARDWARE_REVISION_LENGTH      2

// These are baseflight specific flags but they are useless now since MW 2.3 uses the upper 4 bits for the navigation version.
#define CAP_PLATFORM_32BIT          ((uint32_t)1 << 31)
#define CAP_BASEFLIGHT_CONFIG       ((uint32_t)1 << 30)

// MW 2.3 stores NAVI_VERSION in the top 4 bits of the capability mask.
#define CAP_NAVI_VERSION_BIT_4_MSB  ((uint32_t)1 << 31)
#define CAP_NAVI_VERSION_BIT_3      ((uint32_t)1 << 30)
#define CAP_NAVI_VERSION_BIT_2      ((uint32_t)1 << 29)
#define CAP_NAVI_VERSION_BIT_1_LSB  ((uint32_t)1 << 28)

#define CAP_DYNBALANCE              ((uint32_t)1 << 2)
#define CAP_FLAPS                   ((uint32_t)1 << 3)
#define CAP_NAVCAP                  ((uint32_t)1 << 4)
#define CAP_EXTAUX                  ((uint32_t)1 << 5)

static constexpr uint8_t MSP_API_VERSION                 = 1;    //out message
static constexpr uint8_t MSP_FC_VARIANT                  = 2;    //out message
static constexpr uint8_t MSP_FC_VERSION                  = 3;    //out message
static constexpr uint8_t MSP_BOARD_INFO                  = 4;    //out message
static constexpr uint8_t MSP_BUILD_INFO                  = 5;    //out message

static constexpr uint8_t MSP_NAME                        = 10;   //out message          Returns user set board name - betaflight
static constexpr uint8_t MSP_SET_NAME                    = 11;   //in message           Sets board name - betaflight

//
// MSP commands for Cleanflight original features
//
static constexpr uint8_t MSP_BATTERY_CONFIG              = 32;
static constexpr uint8_t MSP_SET_BATTERY_CONFIG          = 33;

static constexpr uint8_t MSP_MODE_RANGES                 = 34;    //out message         Returns all mode ranges
static constexpr uint8_t MSP_SET_MODE_RANGE              = 35;    //in message          Sets a single mode range

static constexpr uint8_t MSP_FEATURE_CONFIG              = 36;
static constexpr uint8_t MSP_SET_FEATURE_CONFIG          = 37;

static constexpr uint8_t MSP_BOARD_ALIGNMENT_CONFIG      = 38;
static constexpr uint8_t MSP_SET_BOARD_ALIGNMENT_CONFIG  = 39;

static constexpr uint8_t MSP_CURRENT_METER_CONFIG        = 40;
static constexpr uint8_t MSP_SET_CURRENT_METER_CONFIG    = 41;

static constexpr uint8_t MSP_MIXER_CONFIG                = 42;
static constexpr uint8_t MSP_SET_MIXER_CONFIG            = 43;

static constexpr uint8_t MSP_RX_CONFIG                   = 44;
static constexpr uint8_t MSP_SET_RX_CONFIG               = 45;

static constexpr uint8_t MSP_LED_COLORS                  = 46;
static constexpr uint8_t MSP_SET_LED_COLORS              = 47;

static constexpr uint8_t MSP_LED_STRIP_CONFIG            = 48;
static constexpr uint8_t MSP_SET_LED_STRIP_CONFIG        = 49;

static constexpr uint8_t MSP_RSSI_CONFIG                 = 50;
static constexpr uint8_t MSP_SET_RSSI_CONFIG             = 51;

static constexpr uint8_t MSP_ADJUSTMENT_RANGES           = 52;
static constexpr uint8_t MSP_SET_ADJUSTMENT_RANGE        = 53;

// private - only to be used by the configurator, the commands are likely to change
static constexpr uint8_t MSP_CF_SERIAL_CONFIG            = 54;
static constexpr uint8_t MSP_SET_CF_SERIAL_CONFIG        = 55;

static constexpr uint8_t MSP_VOLTAGE_METER_CONFIG        = 56;
static constexpr uint8_t MSP_SET_VOLTAGE_METER_CONFIG    = 57;

static constexpr uint8_t MSP_SONAR_ALTITUDE              = 58; //out message get sonar altitude [cm]

static constexpr uint8_t MSP_PID_CONTROLLER              = 59;
static constexpr uint8_t MSP_SET_PID_CONTROLLER          = 60;

static constexpr uint8_t MSP_ARMING_CONFIG               = 61;
static constexpr uint8_t MSP_SET_ARMING_CONFIG           = 62;

//
// Baseflight MSP commands (if enabled they exist in Cleanflight)
//
static constexpr uint8_t MSP_RX_MAP                      = 64; //out message get channel map (also returns number of channels total)
static constexpr uint8_t MSP_SET_RX_MAP                  = 65; //in message set rx map, numchannels to set comes from MSP_RX_MAP

// DEPRECATED - DO NOT USE "MSP_BF_CONFIG" and MSP_SET_BF_CONFIG.  In Cleanflight, isolated commands already exist and should be used instead.
// DEPRECATED - static constexpr uint8_t MSP_BF_CONFIG                   = 66 //out message baseflight-specific settings that aren't covered elsewhere
// DEPRECATED - static constexpr uint8_t MSP_SET_BF_CONFIG               = 67 //in message baseflight-specific settings save

static constexpr uint8_t MSP_REBOOT                      = 68; //in message reboot settings

// Use MSP_BUILD_INFO instead
// DEPRECATED - static constexpr uint8_t MSP_BF_BUILD_INFO               = 69 //out message build date as well as some space for future expansion

static constexpr uint8_t MSP_DATAFLASH_SUMMARY           = 70; //out message - get description of dataflash chip
static constexpr uint8_t MSP_DATAFLASH_READ              = 71; //out message - get content of dataflash chip
static constexpr uint8_t MSP_DATAFLASH_ERASE             = 72; //in message - erase dataflash chip

// No-longer needed
// DEPRECATED - static constexpr uint8_t MSP_LOOP_TIME                   = 73; //out message         Returns FC cycle time i.e looptime parameter // DEPRECATED
// DEPRECATED - static constexpr uint8_t MSP_SET_LOOP_TIME               = 74; //in message          Sets FC cycle time i.e looptime parameter    // DEPRECATED

static constexpr uint8_t MSP_FAILSAFE_CONFIG             = 75; //out message         Returns FC Fail-Safe settings
static constexpr uint8_t MSP_SET_FAILSAFE_CONFIG         = 76; //in message          Sets FC Fail-Safe settings

static constexpr uint8_t MSP_RXFAIL_CONFIG               = 77; //out message         Returns RXFAIL settings
static constexpr uint8_t MSP_SET_RXFAIL_CONFIG           = 78; //in message          Sets RXFAIL settings

static constexpr uint8_t MSP_SDCARD_SUMMARY              = 79; //out message         Get the state of the SD card

static constexpr uint8_t MSP_BLACKBOX_CONFIG             = 80; //out message         Get blackbox settings
static constexpr uint8_t MSP_SET_BLACKBOX_CONFIG         = 81; //in message          Set blackbox settings

static constexpr uint8_t MSP_TRANSPONDER_CONFIG          = 82; //out message         Get transponder settings
static constexpr uint8_t MSP_SET_TRANSPONDER_CONFIG      = 83; //in message          Set transponder settings

static constexpr uint8_t MSP_OSD_CONFIG                  = 84; //out message         Get osd settings - betaflight
static constexpr uint8_t MSP_SET_OSD_CONFIG              = 85; //in message          Set osd settings - betaflight

static constexpr uint8_t MSP_OSD_CHAR_READ               = 86; //out message         Get osd settings - betaflight
static constexpr uint8_t MSP_OSD_CHAR_WRITE              = 87; //in message          Set osd settings - betaflight

static constexpr uint8_t MSP_VTX_CONFIG                  = 88; //out message         Get vtx settings - betaflight
static constexpr uint8_t MSP_SET_VTX_CONFIG              = 89; //in message          Set vtx settings - betaflight

// Betaflight Additional Commands
static constexpr uint8_t MSP_ADVANCED_CONFIG             = 90;
static constexpr uint8_t MSP_SET_ADVANCED_CONFIG         = 91;

static constexpr uint8_t MSP_FILTER_CONFIG               = 92;
static constexpr uint8_t MSP_SET_FILTER_CONFIG           = 93;

static constexpr uint8_t MSP_PID_ADVANCED                = 94;
static constexpr uint8_t MSP_SET_PID_ADVANCED            = 95;

static constexpr uint8_t MSP_SENSOR_CONFIG               = 96;
static constexpr uint8_t MSP_SET_SENSOR_CONFIG           = 97;

static constexpr uint8_t MSP_CAMERA_CONTROL              = 98;

static constexpr uint8_t MSP_SET_ARMING_DISABLED         = 99;

//
// OSD specific
//
static constexpr uint8_t MSP_OSD_VIDEO_CONFIG            = 180;
static constexpr uint8_t MSP_SET_OSD_VIDEO_CONFIG        = 181;

// External OSD displayport mode messages
static constexpr uint8_t MSP_DISPLAYPORT                 = 182;

static constexpr uint8_t MSP_COPY_PROFILE                = 183;

static constexpr uint8_t MSP_BEEPER_CONFIG               = 184;
static constexpr uint8_t MSP_SET_BEEPER_CONFIG           = 185;

static constexpr uint8_t MSP_SET_TX_INFO                 = 186; // in message           Used to send runtime information from TX lua scripts to the firmware
static constexpr uint8_t MSP_TX_INFO                     = 187; // out message          Used by TX lua scripts to read information from the firmware

static constexpr uint8_t MSP_SET_OSD_CANVAS              = 188; // in message           Set osd canvas size COLSxROWS
static constexpr uint8_t MSP_OSD_CANVAS                  = 189; // out message          Get osd canvas size COLSxROWS

//
// Multiwii original MSP commands
//

// See MSP_API_VERSION and MSP_MIXER_CONFIG
//DEPRECATED - static constexpr uint8_t MSP_IDENT                = 100;    //out message         mixerMode + multiwii version + protocol version + capability variable


static constexpr uint8_t MSP_STATUS               = 101;    //out message         cycletime & errors_count & sensor present & box activation & current setting number
static constexpr uint8_t MSP_RAW_IMU              = 102;    //out message         9 DOF
static constexpr uint8_t MSP_SERVO                = 103;    //out message         servos
static constexpr uint8_t MSP_MOTOR                = 104;    //out message         motors
static constexpr uint8_t MSP_RC                   = 105;    //out message         rc channels and more
static constexpr uint8_t MSP_RAW_GPS              = 106;    //out message         fix, number of satellites, lat, lon, alt, speed, ground course
static constexpr uint8_t MSP_COMP_GPS             = 107;    //out message         distance home, direction home
static constexpr uint8_t MSP_ATTITUDE             = 108;    //out message         2 angles = 1 heading
static constexpr uint8_t MSP_ALTITUDE             = 109;    //out message         altitude, variometer
static constexpr uint8_t MSP_ANALOG               = 110;    // out message        vbat, powermetersum, rssi if available on RX
static constexpr uint8_t MSP_RC_TUNING            = 111;    //out message         rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID
static constexpr uint8_t MSP_PID                  = 112;    //out message         P I D coeff (9 are used currently)
// Legacy MultiWi command that was never used.
//DEPRECATED - static constexpr uint8_t MSP_BOX                  = 113;    //out message         BOX setup (number is dependant of your setup)
// Legacy command that was under constant change due to the naming vagueness, avoid at all costs - use more specific commands instead.
//DEPRECATED - static constexpr uint8_t MSP_MISC                 = 114;    //out message         powermeter trig
// Legacy MultiWi command that was never used and always wrong
//DEPRECATED - static constexpr uint8_t MSP_MOTOR_PINS           = 115;    //out message         which pins are in use for motors & servos, for GUI
static constexpr uint8_t MSP_BOXNAMES             = 116;    //out message         the aux switch names
static constexpr uint8_t MSP_PIDNAMES             = 117;    //out message         the PID names
static constexpr uint8_t MSP_WP                   = 118;    //out message         get a WP, WP# is in the payload, returns (WP#, lat, lon, alt, flags) WP#0-home, WP#16-poshold
static constexpr uint8_t MSP_BOXIDS               = 119;    //out message         get the permanent IDs associated to BOXes
static constexpr uint8_t MSP_SERVO_CONFIGURATIONS = 120;    //out message         All servo configurations.
static constexpr uint8_t MSP_NAV_STATUS           = 121;    //out message         Returns navigation status
static constexpr uint8_t MSP_NAV_CONFIG           = 122;    //out message         Returns navigation parameters
static constexpr uint8_t MSP_MOTOR_3D_CONFIG      = 124;    //out message         Settings needed for reversible ESCs
static constexpr uint8_t MSP_RC_DEADBAND          = 125;    //out message         deadbands for yaw alt pitch roll
static constexpr uint8_t MSP_SENSOR_ALIGNMENT     = 126;    //out message         orientation of acc,gyro,mag
static constexpr uint8_t MSP_LED_STRIP_MODECOLOR  = 127;    //out message         Get LED strip mode_color settings
static constexpr uint8_t MSP_VOLTAGE_METERS       = 128;    //out message         Voltage (per meter)
static constexpr uint8_t MSP_CURRENT_METERS       = 129;    //out message         Amperage (per meter)
static constexpr uint8_t MSP_BATTERY_STATE        = 130;    //out message         Connected/Disconnected, Voltage, Current Used
static constexpr uint8_t MSP_MOTOR_CONFIG         = 131;    //out message         Motor configuration (min/max throttle, etc)
static constexpr uint8_t MSP_GPS_CONFIG           = 132;    //out message         GPS configuration
static constexpr uint8_t MSP_COMPASS_CONFIG       = 133;    //out message         Compass configuration
static constexpr uint8_t MSP_ESC_SENSOR_DATA      = 134;    //out message         Extra ESC data from 32-Bit ESCs (Temperature, RPM)
static constexpr uint8_t MSP_GPS_RESCUE           = 135;    //out message         GPS Rescue angle, returnAltitude, descentDistance, groundSpeed, sanityChecks and minSats
static constexpr uint8_t MSP_GPS_RESCUE_PIDS      = 136;    //out message         GPS Rescue throttleP and velocity PIDS + yaw P
static constexpr uint8_t MSP_VTXTABLE_BAND        = 137;    //out message         vtxTable band/channel data
static constexpr uint8_t MSP_VTXTABLE_POWERLEVEL  = 138;    //out message         vtxTable powerLevel data
static constexpr uint8_t MSP_MOTOR_TELEMETRY      = 139;    //out message         Per-motor telemetry data (RPM, packet stats, ESC temp, etc.)

static constexpr uint8_t MSP_SIMPLIFIED_TUNING             = 140;    //out message    Simplified tuning values and enabled state
static constexpr uint8_t MSP_SET_SIMPLIFIED_TUNING         = 141;    //in message     Set simplified tuning positions and apply the calculated tuning
static constexpr uint8_t MSP_CALCULATE_SIMPLIFIED_PID      = 142;    //out message    Requests calculations of PID values based on sliders. Sends the calculated values back. But don't save anything to the firmware
static constexpr uint8_t MSP_CALCULATE_SIMPLIFIED_GYRO     = 143;    //out message    Requests calculations of gyro filter values based on sliders. Sends the calculated values back. But don't save anything to the firmware
static constexpr uint8_t MSP_CALCULATE_SIMPLIFIED_DTERM    = 144;    //out message    Requests calculations of gyro filter values based on sliders. Sends the calculated values back. But don't save anything to the firmware
static constexpr uint8_t MSP_VALIDATE_SIMPLIFIED_TUNING    = 145;    //out message    Returns an array of true/false showing which simplified tuning groups are matching with value and which are not

static constexpr uint8_t MSP_SET_RAW_RC           = 200;    //in message          8 rc chan
static constexpr uint8_t MSP_SET_RAW_GPS          = 201;    //in message          fix, number of satellites, lat, lon, alt, speed
static constexpr uint8_t MSP_SET_PID              = 202;    //in message          P I D coeff (9 are used currently)
// Legacy multiiwii command that was never used.
//DEPRECATED - static constexpr uint8_t MSP_SET_BOX              = 203;    //in message          BOX setup (number is dependant of your setup)
static constexpr uint8_t MSP_SET_RC_TUNING        = 204;    //in message          rc rate, rc expo, rollpitch rate, yaw rate, dyn throttle PID, yaw expo
static constexpr uint8_t MSP_ACC_CALIBRATION      = 205;    //in message          no param
static constexpr uint8_t MSP_MAG_CALIBRATION      = 206;    //in message          no param
// Legacy command that was under constant change due to the naming vagueness, avoid at all costs - use more specific commands instead.
//DEPRECATED - static constexpr uint8_t MSP_SET_MISC             = 207;    //in message          powermeter trig + 8 free for future use
static constexpr uint8_t MSP_RESET_CONF           = 208;    //in message          no param
static constexpr uint8_t MSP_SET_WP               = 209;    //in message          sets a given WP (WP#,lat, lon, alt, flags)
static constexpr uint8_t MSP_SELECT_SETTING       = 210;    //in message          Select Setting Number (0-2)
static constexpr uint8_t MSP_SET_HEADING          = 211;    //in message          define a new heading hold direction
static constexpr uint8_t MSP_SET_SERVO_CONFIGURATION = 212;    //in message          Servo settings
static constexpr uint8_t MSP_SET_MOTOR            = 214;    //in message          PropBalance function
static constexpr uint8_t MSP_SET_NAV_CONFIG       = 215;    //in message          Sets nav config parameters - write to the eeprom
static constexpr uint8_t MSP_SET_MOTOR_3D_CONFIG  = 217;    //in message          Settings needed for reversible ESCs
static constexpr uint8_t MSP_SET_RC_DEADBAND      = 218;    //in message          deadbands for yaw alt pitch roll
static constexpr uint8_t MSP_SET_RESET_CURR_PID   = 219;    //in message          resetting the current pid profile to defaults
static constexpr uint8_t MSP_SET_SENSOR_ALIGNMENT = 220;    //in message          set the orientation of the acc,gyro,mag
static constexpr uint8_t MSP_SET_LED_STRIP_MODECOLOR = 221; //in  message         Set LED strip mode_color settings
static constexpr uint8_t MSP_SET_MOTOR_CONFIG     = 222;    //out message         Motor configuration (min/max throttle, etc)
static constexpr uint8_t MSP_SET_GPS_CONFIG       = 223;    //out message         GPS configuration
static constexpr uint8_t MSP_SET_COMPASS_CONFIG   = 224;    //out message         Compass configuration
static constexpr uint8_t MSP_SET_GPS_RESCUE       = 225;    //in message          GPS Rescue angle, returnAltitude, descentDistance, groundSpeed and sanityChecks
static constexpr uint8_t MSP_SET_GPS_RESCUE_PIDS  = 226;    //in message          GPS Rescue throttleP and velocity PIDS + yaw P
static constexpr uint8_t MSP_SET_VTXTABLE_BAND    = 227;    //in message          set vtxTable band/channel data (one band at a time)
static constexpr uint8_t MSP_SET_VTXTABLE_POWERLEVEL = 228; //in message          set vtxTable powerLevel data (one powerLevel at a time)

// static constexpr uint8_t MSP_BIND                 = 240;    //in message          no param
// static constexpr uint8_t MSP_ALARMS               = 242;
static constexpr uint8_t MSP_SET_PASSTHROUGH      = 245;    // in message         Sets up passthrough to different peripherals (4way interface, uart, etc...)

static constexpr uint8_t MSP_EEPROM_WRITE         = 250;    //in message          no param
static constexpr uint8_t MSP_RESERVE_1            = 251;    //reserved for system usage
static constexpr uint8_t MSP_RESERVE_2            = 252;    //reserved for system usage
static constexpr uint8_t MSP_DEBUGMSG             = 253;    //out message         debug string buffer
static constexpr uint8_t MSP_DEBUG                = 254;    //out message         debug1,debug2,debug3,debug4
static constexpr uint8_t MSP_V2_FRAME             = 255;    //MSPv2 payload indicator

// Additional commands that are not compatible with MultiWii
static constexpr uint8_t MSP_STATUS_EX            = 150;    //out message         cycletime, errors_count, CPU load, CPU temperature, sensor present etc
static constexpr uint8_t MSP_UID                  = 160;    //out message         Unique device ID
static constexpr uint8_t MSP_GPSSVINFO            = 164;    //out message         get Signal Strength (only U-Blox)
static constexpr uint8_t MSP_GPSSTATISTICS        = 166;    //out message         get GPS debugging data
static constexpr uint8_t MSP_MULTIPLE_MSP         = 230;    //out message         request multiple MSPs in one request - limit is the TX buffer; returns each MSP in the order they were requested starting with length of MSP; MSPs with input arguments are not supported
static constexpr uint8_t MSP_MODE_RANGES_EXTRA    = 238;    //out message         Reads the extra mode range data
static constexpr uint8_t MSP_ACC_TRIM             = 240;    //out message         get acc angle trim values
static constexpr uint8_t MSP_SET_ACC_TRIM         = 239;    //in message          set acc angle trim values
static constexpr uint8_t MSP_SERVO_MIX_RULES      = 241;    //out message         Returns servo mixer configuration
static constexpr uint8_t MSP_SET_SERVO_MIX_RULE   = 242;    //in message          Sets servo mixer configuration
static constexpr uint8_t MSP_SET_RTC              = 246;    //in message          Sets the RTC clock
static constexpr uint8_t MSP_RTC                  = 247;    //out message         Gets the RTC clock
static constexpr uint8_t MSP_SET_BOARD_INFO       = 248;    //in message          Sets the board information for this board
static constexpr uint8_t MSP_SET_SIGNATURE        = 249;    //in message          Sets the signature of the board and serial number


static constexpr uint16_t MSP2_COMMON_SERIAL_CONFIG       = 0x1009;
static constexpr uint16_t MSP2_COMMON_SET_SERIAL_CONFIG   = 0x100A;

// Sensors
static constexpr uint16_t MSP2_SENSOR_GPS                 = 0x1F03;

static constexpr uint16_t MSP2_BETAFLIGHT_BIND                = 0x3000;
static constexpr uint16_t MSP2_MOTOR_OUTPUT_REORDERING        = 0x3001;
static constexpr uint16_t MSP2_SET_MOTOR_OUTPUT_REORDERING    = 0x3002;
static constexpr uint16_t MSP2_SEND_DSHOT_COMMAND             = 0x3003;
static constexpr uint16_t MSP2_GET_VTX_DEVICE_STATUS          = 0x3004;
static constexpr uint16_t MSP2_GET_OSD_WARNINGS               = 0x3005;  // returns active OSD warning message text
static constexpr uint16_t MSP2_GET_TEXT                       = 0x3006;
static constexpr uint16_t MSP2_SET_TEXT                       = 0x3007;
static constexpr uint16_t MSP2_GET_LED_STRIP_CONFIG_VALUES    = 0x3008;
static constexpr uint16_t MSP2_SET_LED_STRIP_CONFIG_VALUES    = 0x3009;
static constexpr uint16_t MSP2_SENSOR_CONFIG_ACTIVE           = 0x300A;

// MSP2_SET_TEXT and MSP2_GET_TEXT variable types
static constexpr uint8_t MSP2TEXT_PILOT_NAME                      = 1;
static constexpr uint8_t MSP2TEXT_CRAFT_NAME                      = 2;
static constexpr uint8_t MSP2TEXT_PID_PROFILE_NAME                = 3;
static constexpr uint8_t MSP2TEXT_RATE_PROFILE_NAME               = 4;
static constexpr uint8_t MSP2TEXT_BUILDKEY                        = 5;
static constexpr uint8_t MSP2TEXT_RELEASENAME                     = 6;

//NOLINTEND(modernize-macro-to-enum)
