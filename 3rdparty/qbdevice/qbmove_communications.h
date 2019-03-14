
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
 * \file        qbmove_communications.h
 *
 * \brief       Library of functions for SERIAL PORT communication with a qbMove.
 *              Function Prototypes.
 *
 * \details
 *
 *  This library contains all necessary functions for communicating with a qbMove when
 *  using a USB to RS485 connector that provides a Virtual COM interface.
**/

 /**
* \mainpage     qbAPI Libraries
*
* \brief        Those functions allows to use the qbMove or the qbHand through a serial port
*
* \version      6.0.0
*
* \author       qbrobotics
*
* \date         June 01, 2016
*
* \details      This is a set of functions that allows to use the qbMoves or the qbHands 
*               via a serial port.
*
*               Those APIs can be compiled for Unix systems like Linux and
*               Mac OS X and even for Windows. Refer to https://github.com/qbrobotics/qbAPI/blob/master/README.md
*               for detailed instructions.
*
* \copyright    (C)  qbrobotics. All rights reserved.
*/


#ifndef QBMOVE_SERIALPORT_H_INCLUDED
#define QBMOVE_SERIALPORT_H_INCLUDED

#if (defined(_WIN32) || defined(_WIN64))
    #include <windows.h>
#else
    #define INVALID_HANDLE_VALUE    -1
#endif

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux
    #include <termios.h>
#endif

#define BAUD_RATE_T_2000000     0           //< Define to identify 2M baudrate
#define BAUD_RATE_T_460800      1           //< Define to identify 460.8k baudrate
#define MAX_WATCHDOG_TIME       500         //< Maximum watchdog timer time
#define READ_TIMEOUT            4000        //< Timeout on readings

#include "commands.h"
#include <stdint.h>


//==============================================================================
//                                                              structures/enums
//==============================================================================

typedef struct comm_settings comm_settings;

struct comm_settings
{
#if (defined(_WIN32) || defined(_WIN64))
    HANDLE file_handle;
#else
    int file_handle;
#endif
};


//==============================================================================
//                                                          function definitions
//==============================================================================

/** \name Virtual COM (RS485) functions */
/** \{ */

//===========================================================     RS485listPorts

/** This function is used to return a list of available serial ports. A maximum of 10
 *  ports are found.
 *
 *  \param  list_of_ports   An array of strings with the serial ports paths.
 *
 *  \return Returns the number of serial ports found.
 *
 *  \par Example
 *  \code

    int     i, num_ports;
    char    list_of_ports[10][255];

    num_ports = RS485listPorts(ports);

    for(i = 0; i < num_ports; ++i)
    {
        puts(ports[i]);
    }

 *  \endcode
**/

int RS485listPorts( char list_of_ports[10][255] );

//================================================================     openRS485

/** This function is used to open a serial port for using with the qbMove or the qbHand.
 *
 *  \param comm_settings    A _comm_settings_ structure containing info about the
 *                          communication settings.
 *  \param port_s           The string to the serial port path.
 *  \param BAUD_RATE        The default baud rate value of the serial port
 *
 *  \return Returns the file descriptor associated to the serial port.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    if(comm_settings_t.file_handle == INVALID_HANDLE_VALUE)
    {
    // ERROR
    }

 *  \endcode
**/

#if !(defined(_WIN32) || defined(_WIN64)) && !(defined(__APPLE__)) //only for linux
    void openRS485( comm_settings *comm_settings_t, const char *port_s, int BAUD_RATE = B2000000);
#elif !(defined(_WIN32) || defined(_WIN64)) && (defined(__APPLE__)) //only for mac
    void openRS485( comm_settings *comm_settings_t, const char *port_s, int BAUD_RATE = 2000000);
#else
    void openRS485( comm_settings *comm_settings_t, const char *port_s, int BAUD_RATE = 2000000);
#endif


//===============================================================     closeRS485


/** This function is used to close a serial port being used with the qbMove or an qbHand.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void closeRS485( comm_settings *comm_settings_t );

//================================================================     RS485read

/** This function is used to read a package from the device.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  package             Package will be stored here.
 *
 *  \return Returns package length if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    char            data_read[1000];

    openRS485(&comm_settings_t, "/dev/tty.usbserial-128");
    commPing(&comm_settings_t, device_id);
    RS485read(&comm_settings_t, device_id, data_read);
    closeRS485(&comm_settings_t);

    printf(data_read);

 *  \endcode
**/

int RS485read( comm_settings *comm_settings_t, int id, char *package );

//=========================================================     RS485ListDevices

/** This function is used to list the number of devices connected to the serial port and
 *  get their relative IDs
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  list_of_ids[255]    Buffer that stores a list of IDs to ping, in order to 
 *                              see which of those IDs is connected. Is then filled with
 *                              the IDs connected to the serial port.
 *
 *  \return                     Returns the number of devices connected                  
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65; 
    int             device_num;
    char            list_of_ids[255];

    openRS485(&comm_settings_t, device_id);
    device_num = RS485ListDevices(&comm_settings_t, &list_of_ids);
    closeRS485(&comm_settings_t);
    printf("Number of devices connected: %d", i);

 *  \endcode
**/

int RS485ListDevices( comm_settings *comm_settings_t, char list_of_ids[255] );

//=============================================================     RS485GetInfo

/** This function is used to ping the serial port for a qbMove or a qbHand and to
 *  get information about the device. ONLY USE WHEN ONE DEVICE IS CONNECTED
 *  ONLY.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  buffer              Buffer that stores a string with information about
 *                              the device. BUFFER SIZE MUST BE AT LEAST 500.
 *
 *
 *  \par Example
 *  \code

    comm_settings    comm_settings_t;
    char             auxstring[500];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    RS485GetInfo(&comm_settings_t, auxstring);
    puts(auxstring);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void RS485GetInfo( comm_settings *comm_settings_t, char *buffer );

/** \} */


/** \name qbAPI Commands */
/** \{ */

//================================================================     commPing

/** This function is used to ping the qbMove or the qbHand.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  buffer              Buffer that stores a string with information about
 *                              the device. BUFFER SIZE MUST BE AT LEAST 500.
 *
 *  \return Returns 0 if ping was ok, -1 otherwise.
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    if ( commPing(&comm_settings_t, device_id) )
        puts("Device exists.");
    else
        puts("Device does not exist.");

   closeRS485(&comm_settings_t);

*  \endcode
**/

int commPing( comm_settings *comm_settings_t, int id );

//=============================================================     commActivate

/** This function activates or deactivates a qbMove or a qbHand connected to
 *  the serial port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  activate            TRUE to turn motors on.
 *                              FALSE to turn motors off.
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    commActivate(&comm_settings_t, device_id, TRUE);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void commActivate( comm_settings *comm_settings_t, int id, char activate );

//============================================================     commSetBaudRate

/** This function sets the baudrate of communication.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  baudrate            BaudRate requested 0 = 2M baudrate, 1 = 460.8k baudrate
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       baudrate = 0;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    commSetBaudRate(&comm_settings_t, global_args.device_id, baudrate);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void commSetBaudRate( comm_settings *comm_settings_t, int id, short int baudrate);

//============================================================     commSetWatchDog

/** This function sets watchdog timer of a qbMove or a qbHand.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  wdt                 Watchdog timer in [csec],
 *                              max value: 500 [cs] / min value: 0 (disable) [cs]
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       wdt = 60;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128);
    commSetWatchDog(&comm_settings_t, global_args.device_id, wdt);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void commSetWatchDog( comm_settings *comm_settings_t, int id, short int wdt);

//============================================================     commSetInputs

/** This function send reference inputs to a qbMove or a qbHand connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  inputs              Input references.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       inputs[2];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    inputs[0]   = 1000;
    inputs[1]   = -1000;
    commSetInputs(&comm_settings_t, device_id, inputs);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void commSetInputs( comm_settings *comm_settings_t, int id, short int inputs[]);

//============================================================     commSetInputsAck

/** This function send reference inputs to a qbMove or a qbHand connected to the serial
 *  port and expects an acknowledgment reply from the device.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  inputs              Input references.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       inputs[2];
    char            package_in[2];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    inputs[0]   = 1000;
    inputs[1]   = -1000;
    commSetInputsAck(&comm_settings_t, device_id, inputs);
    RS485Read(&comm_settings_t, device_id, package_in);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

int commSetInputsAck( comm_settings *comm_settings_t, int id, short int inputs[]);

//============================================================     commSetPosStiff

/** This function send reference inputs to a qbMove connected to the serial
 *  port. The reference is in shaft position and stiffness preset. IS VALID ONLY WHEN USED
 *  FOR THE qbMove, NOT FOR THE qbHand
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  inputs              Input references.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       inputs[2];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    inputs[0]   = 100;          //Degrees
    inputs[1]   = 30;           //stiffness preset
    commSetPosStiff(&comm_settings_t, device_id, inputs);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void commSetPosStiff(comm_settings *comm_settings_t, int id, short int inputs[]);

//============================================================     commGetInputs

/** This function gets input references from a qbMove or a qbHand connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  inputs          Input references.
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       inputs[2];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    if(!commGetInputs(&comm_settings_t, DEVICE_ID, inputs))
        printf("Inputs: %d\t%d\n",inputs[0], inputs[1]);
    else
        puts("Couldn't retrieve device inputs.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetInputs( comm_settings *comm_settings_t, int id, short int inputs[2] );

//======================================================     commGetMeasurements

/** This function gets position measurements from a qbMove or a qbHand connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  measurements        Measurements.
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       measurements[3];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    if(!commGetMeasurements(&comm_settings_t, DEVICE_ID, measurements))
        printf("Measurements: %d\t%d\t%d\n",measurements[0], measurements[1], measurements[2]);
    else
        puts("Couldn't retrieve measurements.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetMeasurements( comm_settings *comm_settings_t, int id, short int measurements[3] );

//======================================================     commGetCounters

/** This function gets counters values from a qbMove connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  counters            Counters
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings       comm_settings_t;
    int                 device_id = 65;
    short unsigned int  counters[20];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    if(!commGetCounters(&comm_settings_t, DEVICE_ID, counters))
        printf("Counters: %d\t%d\t {...} %d\n", counters[0], counters[1], {...}, counters[20]);
    else
        puts("Couldn't retrieve counters.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetCounters( comm_settings *comm_settings_t, int id, short unsigned int counters[20] );

//======================================================     commGetCurrents

/** This function gets currents from a qbMove or a qbHand connected to the serial
*  port.
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*  \param  currents            Currents.
*
*  \return Returns 0 if communication was ok, -1 otherwise.
*
*  \par Example
*  \code

   comm_settings    comm_settings_t;
   int              device_id = 65;
   short int        currents[2];

   openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

   if(!commGetCurrents(&comm_settings_t, device_id, currents))
       printf("Measurements: %d\t%d\t%d\n",currents[0], currents[1]);
   else
       puts("Couldn't retrieve currents.");

   closeRS485(&comm_settings_t);

*  \endcode

**/

int commGetCurrents( comm_settings *comm_settings_t, int id, short int currents[2] );

//=======================================================     commGetCurrAndMeas

/** This function gets currents and position measurements from a qbMove or a qbHand
*   connected to the serial port
*
*  \param  comm_settings_t      A _comm_settings_ structure containing info about the
*                               communication settings.
*
*  \param  id                   The device's id number.
*  \param  values               Current and position measurements. Currents are in first
*                               two positions
*
*  \return Returns 0 if communication was ok, -1 otherwise.
*
*  \par Example
*  \code

   comm_settings    comm_settings_t;
   int              device_id = 65;
   short int        values[5];

   openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    if(!commGetCurrAndMeas(&comm_settings_t, device_id, currents)){
        printf("Currents: %d\t%d\t%d\n",values[0], values[1]);
        printf("Measurements: %d\t%d\t%d\n", values[2], values[3], values[4]);
    }
    else
        puts("Couldn't retrieve currents.");

   closeRS485(&comm_settings_t);

*  \endcode

**/

int commGetCurrAndMeas( comm_settings *comm_settings_t, int id, short int *values);

//===============================================================     commGetEmg

/** This function gets measurements from electomyographics sensors connected 
*   to the qbHand. IS USED ONLY WHEN THE BOARD IS USED FOR A QBHAND
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*  \param  values              Emg sensors measurements.
*
*  \return Returns 0 if communication was ok, -1 otherwise.
*
*  \par Example
*  \code

   comm_settings    comm_settings_t;
   int              device_id = 65;
   short int        values[2];

   openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

   if(!commGetEmg(&comm_settings_t, device_id, values));
       printf("Measurements: %d\t%d\t%d\n", values[0], values[1]);
   else
       puts("Couldn't retrieve emg values.");

   closeRS485(&comm_settings_t);

*  \endcode

**/

int commGetEmg(comm_settings *comm_settings_t, int id, short int emg[2]);

//========================================================     commGetVelocities

/** This function gets velocities of the two motors and the shaft from a qbMove
*   connected to a serial port or from the only shaft of the qbHand
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*  \param  measurements        Velocity measurements.
*
*  \return Returns 0 if communication was ok, -1 otherwise.
*
*  \par Example
*  \code

   comm_settings    comm_settings_t;
   int              device_id = 65;
   short int        vel_measurements[3];

   openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

   if(!commGetVelocities(&comm_settings_t, device_id, vel_measurements))
       printf("Measurements: %d\t%d\t%d\n", vel_measurements[0], vel_measurements[1], vel_measurements[2]);
   else
       puts("Couldn't retrieve velocities.");

   closeRS485(&comm_settings_t);

*  \endcode

**/

int commGetVelocities(comm_settings *comm_settings_t, int id, short int measurements[]);


//========================================================     commGetAccelerations

/** This function gets the acceleration of the qbHand motor
*
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*  \param  measurements        Velocity measurements.
*
*  \return Returns 0 if communication was ok, -1 otherwise.
*
*  \par Example
*  \code

   comm_settings    comm_settings_t;
   int              device_id = 65;
   short int        acc_measurements[3];

   openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

   if(!commGetAccelerations(&comm_settings_t, device_id, acc_measurements))
       printf("Measurements: %d\t%d\t%d\n", acc_measurements[0], acc_measurements[1], acc_measurements[2]);
   else
       puts("Couldn't retrieve accelerations.");

   closeRS485(&comm_settings_t);

*  \endcode

**/

int commGetAccelerations(comm_settings *comm_settings_t, int id, short int measurements[] );

//==========================================================     commGetActivate

/** This function gets the activation status of a qbMove or a qbHand connected to the serial
 *  port.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  activation      Activation status.
 *
 *  \return Returns 0 if communication was ok, -1 otherwise.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;
    int     device_id = 65;
    char    activation_status;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    if(!commGetActivate(&comm_settings_t, DEVICE_ID, activation_status))
        printf("Activation status: %d\n", &activation_status);
    else
        puts("Couldn't retrieve activation status.");

    closeRS485(&comm_settings_t);

 *  \endcode

**/

int commGetActivate( comm_settings *comm_settings_t, int id, char *activate );


//==============================================================     commGetInfo

/** This function is used to ping the qbMove or the qbHand and get information about the
 *  device.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id              The device's id number.
 *  \param  buffer          Buffer that stores a string with information about
 *                          the device. BUFFER SIZE MUST BE AT LEAST 500.
 *  \param  info_type       Information to be retrieved.
 *
 *  \par Example
 *  \code

    comm_settings comm_settings_t;
    char    auxstring[500];
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    commGetInfo(&comm_settings_t, device_id, INFO_ALL, auxstring);
    puts(auxstring);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

int commGetInfo( comm_settings *comm_settings_t, int id, short int info_type, char *info );


//==============================================================     commBootloader

/** This function sends the board in bootloader modality in order to update
*   the firmware on the board
*
*   \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                               communication settings.
*
*   \param  id                  The device's id number.
*
*   \returns        Return 0 on success, -1 otherwise
*
*   \par Example
*   \code

    comm_settings comm_settings_t;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    commBootloader(&comm_settings_t, device_id);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

int commBootloader(comm_settings *comm_settings_t, int id);


//==============================================================     commCalibrate

/** This function is used to calibrate the maximum stiffness value of the qbMove
*
*   \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                               communication settings.
*
*   \param  id                  The device's id number.
*
*   \return         Returns 0 on success, -1 otherwise
*
*   \par Example
*   \code

    comm_settings comm_settings_t;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    commCalibrate(&comm_settings_t, device_id);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

int commCalibrate(comm_settings *comm_settings_t, int id);

//==============================================================     commHandCalibrate

/** This function is used to make a series of opening and closures of the qbHand
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*  \param  speed               The speed of hand closure and opening [0 - 200]
*  \param  repetitions         The nnumber of closures needed to be done [0 - 32767]
*
*  \par Example
*  \code

    comm_settings comm_settings_t;
    int     speed = 200
    int     repetitions = 400;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    commHandCalibrate(&comm_settings_t, device_id, speed, repetitions);
    closeRS485(&comm_settings_t);

*  \endcode
**/

int commHandCalibrate(comm_settings *comm_settings_t, int id, short int speed, short int repetitions);

/** \} */


/** \name qbAPI Parameters */
/** \{ */

//============================================================     commSetZeros

/** This function sets the encoders's zero positon value that remains stored in
 *  the qbMove or qbHand memory.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  value               An array with the encoder readings values.
 *  \param  num_of_values       The size of the values array, equal to the sensor number.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       measurements[3];


    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    commGetMeasurements(comm_settings_t, device_id, measurements)
    for(i = 0; i<3; i++)
        measurements[i] = -measurements[i];
    commSetZeros(&comm_settings_t, global_args.device_id, measurements, 3);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

int commSetZeros( comm_settings *comm_settings_t,
                    int id,
                    void *values,
                    unsigned short num_of_values );

//============================================================     commGetParamList

/** This function gets all the parameters that are stored in the qbMove or qbHand memory and sets
    one of them if requested
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*  \param  index               The index relative to the parameter to be get.
*  \param  values              An array with the parameter values.
*  \param  value_size          The byte size of the parameter to be get
*  \param  num_of_values       The size of the array of the parameter to be get
*  \param  buffer              The array where the parameters' values and descriptions are saved

*
*  \par Example
*  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    unsigned char   aux_string[2000];
    int             index = 0;
    int             value_size = 0;
    int             num_of_values = 0;

    // Get parameters
    commGetParamList(&comm_settings_t, device_id, index, NULL, value_size, num_of_values, aux_string);
    string_unpacking_and_printing(aux_string);

    // Set parameters

    float           pid[3];
    pid[0] = 0.1;
    pid[1] = 0.2;
    pid[2] = 0.3;
    index = 2;
    value_size = 4;
    num_of_values = 3;
    commGetParamList(&comm_settings_t, device_id, index, pid, value_size, num_of_values, NULL);

*  \endcode
**/

int commGetParamList(comm_settings *comm_settings_t, int id, unsigned short index,
                    void *values, unsigned short value_size, unsigned short num_of_values, 
                    uint8_t *buffer);

//============================================================     commStoreParams

/** This function stores all parameters that were set in the qbMove or the qbHand memory.
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*
*  \par Example
*  \code

    comm_settings comm_settings_t;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    commStoreParams(&comm_settings_t, device_id)

    closeRS485(&comm_settings_t);

*  \endcode
**/

int commStoreParams( comm_settings *comm_settings_t, int id);

//==========================================================     commStoreDefaultParams

/** This function stores the factory default parameters.
*
*  \param  comm_settings_t  A _comm_settings_ structure containing info about
*                           the communication settings.
*
*  \param  id               The device's id number.
*
*  \par Example
*  \code

    comm_settings comm_settings_t;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");
    commStoreDefaultParams(&comm_settings_t, device_id)
    closeRS485(&comm_settings_t);

*  \endcode
**/

int commStoreDefaultParams( comm_settings *comm_settings_t, int id);

//==========================================================     commRestoreParams


/** This function restores the factory default parameters.
*
*  \param  comm_settings_t  A _comm_settings_ structure containing info about
*                           the communication settings.
*
*  \param  id               The device's id number.
*
*  \par Example
*  \code

    comm_settings comm_settings_t;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    commRestoreParams(&comm_settings_t, device_id)

    closeRS485(&comm_settings_t);

*  \endcode
**/

int commRestoreParams( comm_settings *comm_settings_t, int id );

//==========================================================     commInitMem


/** This function initialize the EEPROM memory of the board by loading the default 
*   factory parameters. After the initialization a flag is set.
*
*  \param  comm_settings_t  A _comm_settings_ structure containing info about
*                           the communication settings.
*
*  \param  id               The device's id number.
*
*  \par Example
*  \code

    comm_settings comm_settings_t;
    int     device_id = 65;

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    commInitMem(&comm_settings_t, device_id)

    closeRS485(&comm_settings_t);

*  \endcode
**/

int commInitMem(comm_settings *comm_settings_t, int id);

/** \} */


/** \name General Functions */
/** \{ */


//==========================================================     timevaldiff

/** This functions returns a difference between two timeval structures in order 
*   to obtain time elapsed between the two timeval;
*
* \param   starttime        The timeval structure containing the start time
* \param   finishtime       The timeval structure containing the finish time
*
* \returns Returns the elapsed time between the two timeval structures. 
*
* \par Example
* \code

    struct timeval start, finish;
    gettimeofday(&start, NULL);
    // other instructions 
    gettimeofday(&now, NULL);
    long diff = timevaldiff(&start, &now);
    
    printf(Time elapsed: %ld, diff);

 *  \endcode

**/

long timevaldiff (struct timeval *starttime, struct timeval *finishtime);


//=================================================================     checksum


/** This functions returns an 8 bit LCR checksum over the lenght of a buffer.
 *
 * \param   data_buffer Buffer.
 * \param   data_length Buffer length.
 *
 *  \par Example
 *  \code

    char    aux;
    char    buffer[5];

    buffer  = "abcde";
    aux     = checksum(buffer,5);
    printf("Checksum: %d", (int) aux)

 *  \endcode

**/

char checksum ( char * data_buffer, int data_length );

/** \} */

/** \name Functions for other devices */
/** \{ */

/** This function is used with the armslider device. Is used to drive another
 *  board with the inputs of the first one
 *
 * \param   comm_settings_t     A _comm_settings_ structure containing info about
                                the comunication settings.
 * \param   id                  The id of the board drive.
 * \param   ext_input           A flag used to activate the external drive functionality
 *                              of the board.   
 *  
 * \returns A negative value if something went wrong, a zero if everything went fine.
**/

int commExtDrive(comm_settings *comm_settings_t, int id, char ext_input);

//============================================================     commSetCuffInputs

/** This function send reference inputs to a qbMove board connected to the serial
 *  port. Is used only when the device is a Cuff.
 *
 *  \param  comm_settings_t     A _comm_settings_ structure containing info about the
 *                              communication settings.
 *
 *  \param  id                  The device's id number.
 *  \param  flag                A flag that indicates used to activate the cuff driving 
 *                              functionality of the board.
 *
 *  \par Example
 *  \code

    comm_settings   comm_settings_t;
    int             device_id = 65;
    short int       cuff_inputs[2];

    openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

    int flag = 1;
    commSetCuffInputs(&comm_settings_t, device_id, flag);
    closeRS485(&comm_settings_t);

 *  \endcode
**/

void commSetCuffInputs(comm_settings *comm_settings_t, int id, int flag);

//======================================================     commGetJoystick

/** This function gets joystick measurementes from a softhand connected to the serial
*  port.
*
*  \param  comm_settings_t     A _comm_settings_ structure containing info about the
*                              communication settings.
*
*  \param  id                  The device's id number.
*  \param  joystick            Joystick analog measurements.
*
*  \return Returns 0 if communication was ok, -1 otherwise.
*
*  \par Example
*  \code

   comm_settings    comm_settings_t;
   int              device_id = 65;
   short int        joystick[2];

   openRS485(&comm_settings_t,"/dev/tty.usbserial-128");

   if(!commGetJoystick(&comm_settings_t, device_id, joystick))
       printf("Measurements: %d\t%d\t%d\n",joystick[0], joystick[1]);
   else
       puts("Couldn't retrieve joystick measurements.");

   closeRS485(&comm_settings_t);

*  \endcode

**/

int commGetJoystick( comm_settings *comm_settings_t, int id, short int joystick[2]);


/** \} */

// ----------------------------------------------------------------------------
#endif

/* [] END OF FILE */
