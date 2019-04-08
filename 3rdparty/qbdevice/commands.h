// BSD 3-Clause License

// Copyright (c) 2015-2018, qbrobotics®
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:

// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.

// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.

// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

/**
 * \file        commands.h
 *
 *  \brief      Definitions for qbMove or qbHand commands, parameters and packages.
 *
 *  \details
 *  This file is included in the qbMove and qbHand firmware, in its libraries and
 *  applications. It contains all definitions that are necessary for the
 *  contruction of communication packages.
 *
 *  It includes definitions for all of the device commands, parameters and also
 *  the size of answer packages.
 *
**/

#ifndef COM_COMMANDS_DEFINITIONS_H_INCLUDED
#define COM_COMMANDS_DEFINITIONS_H_INCLUDED

#define API_VERSION "v6.2.0"

//==============================================================================
//                                                                      COMMANDS
//==============================================================================


/** \name qbMove and qbHand Commands
 * \{
**/

enum qbmove_command
{

//=========================================================     general commands

    CMD_PING                    = 0,    ///< Asks for a ping message
    CMD_SET_ZEROS               = 1,    ///< Command for setting the encoders zero position
    CMD_STORE_PARAMS            = 3,    ///< Stores all parameters in memory and
                                        ///  loads them
    CMD_STORE_DEFAULT_PARAMS    = 4,    ///< Store current parameters as factory parameters
    CMD_RESTORE_PARAMS          = 5,    ///< Restore default factory parameters
    CMD_GET_INFO                = 6,    ///< Asks for a string of information about

    CMD_SET_VALUE               = 7,    ///< Not Used
    CMD_GET_VALUE               = 8,    ///< Not Used

    CMD_BOOTLOADER              = 9,    ///< Sets the bootloader modality to update the
                                        ///  firmware
    CMD_INIT_MEM                = 10,   ///< Initialize the memory with the defalut values
    CMD_CALIBRATE               = 11,   ///< Starts the stiffness calibration of the qbMove
    CMD_GET_PARAM_LIST          = 12,   ///< Command to get the parameters list or to set
                                        ///  a defined value chosen by the use
    CMD_HAND_CALIBRATE          = 13,   ///< Starts a series of opening and closures of the hand

//=========================================================     qbcommands

    CMD_ACTIVATE                = 128,  ///< Command for activating/deactivating
                                        ///  the device
    CMD_GET_ACTIVATE            = 129,  ///< Command for getting device activation
                                        ///  state
    CMD_SET_INPUTS              = 130,  ///< Command for setting reference inputs
    CMD_GET_INPUTS              = 131,  ///< Command for getting reference inputs
    CMD_GET_MEASUREMENTS        = 132,  ///< Command for asking device's
                                        ///  position measurements
    CMD_GET_CURRENTS            = 133,  ///< Command for asking device's
                                        ///  current measurements
    CMD_GET_CURR_AND_MEAS       = 134,  ///< Command for asking device's
                                        ///  measurements and currents
    CMD_SET_POS_STIFF           = 135,  ///< Not used in the softhand firmware
    CMD_GET_EMG                 = 136,  ///< Command for asking device's emg sensors 
                                        ///  measurements
    CMD_GET_VELOCITIES          = 137,  ///< Command for asking device's
                                        ///  velocity measurements
    CMD_GET_COUNTERS            = 138,  ///< Command for asking device's counters
                                        ///  (mostly used for debugging sent commands)
    CMD_GET_ACCEL               = 139,  ///< Command for asking device's
                                        ///  acceleration measurements
    CMD_GET_CURR_DIFF           = 140,  ///< Command for asking device's 
                                        ///  current difference between a measured
                                        ///  one and an estimated one (Only for SoftHand)
    CMD_SET_CURR_DIFF           = 141,  ///< Command used to set current difference modality
                                        ///  (Only for Cuff device)
    CMD_SET_CUFF_INPUTS         = 142,  ///< Command used to set Cuff device inputs 
                                        ///  (Only for Cuff device)
    CMD_SET_WATCHDOG            = 143,  ///< Command for setting watchdog timer
                                        ///  or disable it
    CMD_SET_BAUDRATE            = 144,  ///< Command for setting baudrate
                                        ///  of communication
    CMD_EXT_DRIVE               = 145,  ///< Command to set the actual measurements as inputs
                                        ///  to another device (Only for Armslider device)
    CMD_GET_JOYSTICK            = 146,  ///< Command to get the joystick measurements (Only
                                        ///  for devices driven by a joystick)
    CMD_SET_INPUTS_ACK          = 147   ///< Command to set the device inputs and return an
                                        ///  acknowledgment signal (needed for less comm. errors)
};

/** \} */
//==============================================================================
//                                                                    PARAMETERS
//==============================================================================
/** \name qbMove and qbHand Parameters */
/** \{ */

enum qbmove_parameter
{

    PARAM_ID                     = 0,   ///< Device's ID number
    PARAM_PID_CONTROL            = 1,   ///< PID parameters
    PARAM_STARTUP_ACTIVATION     = 2,   ///< Start up activation byte
    PARAM_INPUT_MODE             = 3,   ///< Input mode

    PARAM_CONTROL_MODE           = 4,   ///< Choose the kind of control between
                                        ///  position control, current control,
                                        ///  direct PWM value or current+position control
    PARAM_MEASUREMENT_OFFSET     = 5,   ///< Adds a constant offset to the
                                        ///  measurements
    PARAM_MEASUREMENT_MULTIPLIER = 6,   ///< Adds a multiplier to the
                                        ///  measurements
    PARAM_POS_LIMIT_FLAG         = 7,   ///< Enable/disable position limiting
    PARAM_POS_LIMIT              = 8,   ///< Position limit values
                                        ///  | int32     | int32     | int32     | int32     |
                                        ///  | INF_LIM_1 | SUP_LIM_1 | INF_LIM_2 | SUP_LIM_2 |
    PARAM_MAX_STEP_POS           = 9,   ///< Used to slow down movements for positive values
    PARAM_MAX_STEP_NEG           = 10,  ///< Used to slow down movements for negative values
    PARAM_POS_RESOLUTION         = 11,  ///< Angle resolution for inputs and
                                        ///  measurements. Used during
                                        ///  communication.
    PARAM_CURRENT_LIMIT          = 12,  ///< Limit for absorbed current
    PARAM_EMG_CALIB_FLAG         = 13,  ///< Enable calibration on startup
    PARAM_EMG_THRESHOLD          = 14,  ///< Minimum value to have effect
    PARAM_EMG_MAX_VALUE          = 15,  ///< Maximum value of EMG
    PARAM_EMG_SPEED              = 16,  ///< Closure speed when using EMG
    PARAM_PID_CURR_CONTROL       = 18,  ///< PID current control
    PARAM_DOUBLE_ENC_ON_OFF      = 19,  ///< Double Encoder Y/N
    PARAM_MOT_HANDLE_RATIO       = 20,  ///< Multiplier between handle and motor
    PARAM_MOTOR_SUPPLY           = 21,  ///< Motor supply voltage of the hand
    PARAM_CURRENT_LOOKUP         = 23,  ///< Table of values used to calculate 
                                        ///  an estimated current of the SoftHand
    PARAM_DL_POS_PID             = 24,  ///< Double loop position PID
    PARAM_DL_CURR_PID            = 25   ///< Double loop current PID
};
//** \} */


//===================================================     resolution definitions

enum qbmove_resolution
{
    RESOLUTION_360      = 0,
    RESOLUTION_720      = 1,
    RESOLUTION_1440     = 2,
    RESOLUTION_2880     = 3,
    RESOLUTION_5760     = 4,
    RESOLUTION_11520    = 5,
    RESOLUTION_23040    = 6,
    RESOLUTION_46080    = 7,
    RESOLUTION_92160    = 8
};

//==============================================================     input modes

enum qbmove_input_mode
{
    INPUT_MODE_EXTERNAL = 0,            ///< References through external
                                        ///  commands (default)
    INPUT_MODE_ENCODER3 = 1,            ///< Encoder 3 drives all inputs
    INPUT_MODE_EMG_PROPORTIONAL = 2,    ///< Use EMG measure to proportionally
                                        ///  drive the position of the motor 1
    INPUT_MODE_EMG_INTEGRAL     = 3,    ///< Use 2 EMG signals to drive motor
                                        ///  position
    INPUT_MODE_EMG_FCFS         = 4,    ///< Use 2 EMG. First reaching threshold
                                        ///  wins and its value defines hand closure
    INPUT_MODE_EMG_FCFS_ADV     = 5     ///< Use 2 EMG. First reaching threshold
                                        ///  wins and its value defines hand closure
                                        ///  Wait for both EMG to lower under threshold
};

//============================================================     control modes

enum qbmove_control_mode {

    CONTROL_ANGLE           = 0,        ///< Classic position control
    CONTROL_PWM             = 1,        ///< Direct PWM value
    CONTROL_CURRENT         = 2,        ///< Current control
    CURR_AND_POS_CONTROL    = 3,        ///< Position and current control
    DEFLECTION_CONTROL      = 4,        ///< Deflection control
    DEFL_CURRENT_CONTROL    = 5         ///< Deflection and current control   

};

//======================================================== motor supply voltage values

enum motor_supply_tipe {
    MAXON_24V               = 0,
    MAXON_12V               = 1
};
//====================================================     acknowledgment values
enum acknowledgment_values
{
    ACK_ERROR           = 0,
    ACK_OK              = 1
};

//==============================================    data types enumeration

enum data_types {
    TYPE_FLAG    = 0,       //A uint8 but with a menu
    TYPE_INT8    = 1,
    TYPE_UINT8   = 2,
    TYPE_INT16   = 3,
    TYPE_UINT16  = 4,
    TYPE_INT32   = 5,
    TYPE_UINT32  = 6,
    TYPE_FLOAT   = 7,
    TYPE_DOUBLE  = 8
};

#define PARAM_BYTE_SLOT     50      //Number of bytes reserved to param's information
#define PARAM_MENU_SLOT     150     //Number of bytes reserved to a param menu

/** \} */

//==============================================================================
//                                                                   INFORMATION
//==============================================================================
/** \name QB Move Information Strings */
/** \{ */
#define INFO_ALL        0 ///< All system information.

/** \} */

// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */