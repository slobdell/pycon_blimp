#ifndef __COM_H
#define __COM_H

#include "defines.h"


#ifdef __cplusplus
extern "C" {
#endif

RBAPI(bool) com_InUse(int com);

RBAPI(bool) com_Init(int com, int duplex);
//-- values for the "com" argument
     #define COM_PORT1              (0)
     #define COM_PORT2              (1)
     #define COM_PORT3              (2)
     #define COM_PORT4              (3)
//-- values for the "duplex" argument
     #define COM_ADUPLEX            (0xff)  // auto. select duplex mode according to RoBoard's version
     #define COM_FDUPLEX            (0)     // full-duplex mode
     #define COM_HDUPLEX            (1)     // half-duplex by TX/RX-short
     #define COM_HDUPLEX_RTS        (2)     // half-duplex by RTS-control
     #define COM_HDUPLEX_TXDEN      (3)     // half-duplex by TXDEN-control
//-- if the above function returns false, roboio_GetErrCode() may return:
//   #define ERROR_RBVER_UNKNOWN    (ERR_NOERROR + 801) //need include <common.h>
//   #define ERROR_RBVER_UNMATCH    (ERR_NOERROR + 800) //need include <common.h>
//   #define ERROR_IOINITFAIL       (ERR_NOERROR + 100) //need include <io.h>
//   #define ERROR_IOSECTIONFULL    (ERR_NOERROR + 101) //need include <io.h>
//   #define ERROR_CPUUNSUPPORTED	(ERR_NOERROR + 102) //need include <io.h>
     #define ERROR_COM_INUSE        (ERR_NOERROR + 700)
     #define ERROR_COM_INVALID      (ERR_NOERROR + 702)
     #define ERROR_COM_FAIL         (ERR_NOERROR + 704)

RBAPI(void) com_Close(int com);


RBAPI(bool) com_SetFormat(int com, int bytesize, int stopbit, int parity);
//-- values for the "bytesize" argument
     #define COM_BYTESIZE5          (5)
     #define COM_BYTESIZE6          (6)
     #define COM_BYTESIZE7          (7)
     #define COM_BYTESIZE8          (8)
//-- values for the "stopbit" argument
     #define COM_STOPBIT1           (1)
     #define COM_STOPBIT2           (2)
//-- values for the "parity" argument
     #define COM_NOPARITY           (0)
     #define COM_ODDPARITY          (1)
     #define COM_EVENPARITY         (2)
//-- if the above function returns false, roboio_GetErrCode() may return:
//   #define ERROR_COM_FAIL         (ERR_NOERROR + 704)

RBAPI(bool) com_SetBaud(int com, unsigned int baudrate);
//-- values for the "baudrate" argument
    #define COMBAUD_748800BPS       (0x8002)  // 57600 * 13 (invalid for RB-100)
    #define COMBAUD_499200BPS       (0x8003)  // 38400 * 13 (invalid for RB-100)
    #define COMBAUD_249600BPS       (0x8006)  // 19200 * 13 (invalid for RB-100)
    #define COMBAUD_115200BPS       (0x0001)
    #define COMBAUD_57600BPS        (0x0002)
    #define COMBAUD_38400BPS        (0x0003)
    #define COMBAUD_19200BPS        (0x0006)
    #define COMBAUD_9600BPS         (0x000c)
    #define COMBAUD_4800BPS         (0x0018)
    #define COMBAUD_2400BPS         (0x0030)
    #define COMBAUD_1200BPS         (0x0060)
    #define COMBAUD_300BPS          (0x0180)
    #define COMBAUD_50BPS           (0x0900)
//-- if the above function returns false, roboio_GetErrCode() may return:
//   #define ERROR_COM_FAIL         (ERR_NOERROR + 704)


RBAPI(bool) com_Receive(int com, unsigned char* buf, int bsize);
//-- if the above function returns false, roboio_GetErrCode() may return:
     #define ERROR_COM_READFAIL     (ERR_NOERROR + 720)

RBAPI(unsigned int) com_Read(int com);
//-- if the above function returns 0xffff, roboio_GetErrCode() may return:
//   #define ERROR_COM_READFAIL     (ERR_NOERROR + 720)

RBAPI(bool) com_ClearRFIFO(int com);
//-- if the above function returns false, roboio_GetErrCode() may return:
//   #define ERROR_COM_FAIL         (ERR_NOERROR + 704)

RBAPI(int) com_QueryRFIFO(int com);
//-- if the above function returns -1, roboio_GetErrCode() may return:
//   #define ERROR_COM_FAIL         (ERR_NOERROR + 704)


RBAPI(bool) com_Send(int com, unsigned char* buf, int bsize);
RBAPI(bool) com_Write(int com, unsigned char val);
//-- if the above function returns false, roboio_GetErrCode() may return:
     #define ERROR_COM_SENDFAIL     (ERR_NOERROR + 730)

RBAPI(bool) com_FlushWFIFO(int com);
RBAPI(bool) com_ClearWFIFO(int com);
//-- if the above function returns false, roboio_GetErrCode() may return:
//   #define ERROR_COM_FAIL         (ERR_NOERROR + 704)
//   #define ERROR_COM_READFAIL     (ERR_NOERROR + 720)
//   #define ERROR_COM_SENDFAIL     (ERR_NOERROR + 730)


#ifdef ROBOIO
    RBAPI(bool) com_ServoTRX(int com, unsigned char* cmd, int csize, unsigned char* buf, int bsize);
    //-- if the above function returns false, roboio_GetErrCode() may return:
    //   #define ERROR_COM_FAIL         (ERR_NOERROR + 704)
#endif


/*************************  Isolated COM lib Functions  ***********************/
RBAPI(void) com_EnableTurboMode(int com);
RBAPI(void) com_DisableTurboMode(int com);
RBAPI(bool) com_IsTurboMode(int com);

RBAPI(void) com_EnableFIFO32(int com);
RBAPI(void) com_DisableFIFO32(int com);
RBAPI(bool) com_IsFIFO32Mode(int com);
/*--------------------  end of Isolated COM lib Functions  -------------------*/


#if defined(USE_COMSTDIO) || defined(__COM_LIB)
    _RBAPI_C(bool) com_printf(int com, char* fmt, ...);
    _RBAPI_C(bool) com1_printf(char* fmt, ...);
    _RBAPI_C(bool) com2_printf(char* fmt, ...);
    _RBAPI_C(bool) com3_printf(char* fmt, ...);
    _RBAPI_C(bool) com4_printf(char* fmt, ...);

    RBAPI(bool) com_kbhit(int com);
    RBAPI(unsigned int) com_getch(int com);
#endif
     
#ifdef __cplusplus
}
#endif



/****************************  Inline Functions  **************************/
#ifdef ROBOIO_DLL //use no inline functions for DLL
#ifdef __cplusplus
extern "C" {
#endif
    RBAPI(bool) com1_InUse(void);
    RBAPI(bool) com2_InUse(void);
    RBAPI(bool) com3_InUse(void);
    RBAPI(bool) com4_InUse(void);

    RBAPI(bool) com1_Init(int duplex);
    RBAPI(bool) com2_Init(int duplex);
    RBAPI(bool) com3_Init(int duplex);
    RBAPI(bool) com4_Init(int duplex);
    RBAPI(void) com1_Close(void);
    RBAPI(void) com2_Close(void);
    RBAPI(void) com3_Close(void);
    RBAPI(void) com4_Close(void);

    RBAPI(bool) com1_Send(unsigned char* buf, int bsize);
    RBAPI(bool) com2_Send(unsigned char* buf, int bsize);
    RBAPI(bool) com3_Send(unsigned char* buf, int bsize);
    RBAPI(bool) com4_Send(unsigned char* buf, int bsize);
    RBAPI(bool) com1_Write(unsigned char val);
    RBAPI(bool) com2_Write(unsigned char val);
    RBAPI(bool) com3_Write(unsigned char val);
    RBAPI(bool) com4_Write(unsigned char val);

    RBAPI(bool) com1_Receive(unsigned char* buf, int bsize);
    RBAPI(bool) com2_Receive(unsigned char* buf, int bsize);
    RBAPI(bool) com3_Receive(unsigned char* buf, int bsize);
    RBAPI(bool) com4_Receive(unsigned char* buf, int bsize);
    RBAPI(unsigned int) com1_Read(void);
    RBAPI(unsigned int) com2_Read(void);
    RBAPI(unsigned int) com3_Read(void);
    RBAPI(unsigned int) com4_Read(void);

    RBAPI(bool) com1_ClearWFIFO(void);
    RBAPI(bool) com2_ClearWFIFO(void);
    RBAPI(bool) com3_ClearWFIFO(void);
    RBAPI(bool) com4_ClearWFIFO(void);
    RBAPI(bool) com1_FlushWFIFO(void);
    RBAPI(bool) com2_FlushWFIFO(void);
    RBAPI(bool) com3_FlushWFIFO(void);
    RBAPI(bool) com4_FlushWFIFO(void);
    RBAPI(bool) com1_ClearRFIFO(void);
    RBAPI(bool) com2_ClearRFIFO(void);
    RBAPI(bool) com3_ClearRFIFO(void);
    RBAPI(bool) com4_ClearRFIFO(void);
    RBAPI(int)  com1_QueryRFIFO(void);
    RBAPI(int)  com2_QueryRFIFO(void);
    RBAPI(int)  com3_QueryRFIFO(void);
    RBAPI(int)  com4_QueryRFIFO(void);

    RBAPI(bool) com1_SetFormat(int bytesize, int stopbit, int parity);
    RBAPI(bool) com2_SetFormat(int bytesize, int stopbit, int parity);
    RBAPI(bool) com3_SetFormat(int bytesize, int stopbit, int parity);
    RBAPI(bool) com4_SetFormat(int bytesize, int stopbit, int parity);
    RBAPI(bool) com1_SetBaud(int baudrate);
    RBAPI(bool) com2_SetBaud(int baudrate);
    RBAPI(bool) com3_SetBaud(int baudrate);
    RBAPI(bool) com4_SetBaud(int baudrate);

    #ifdef ROBOIO
        RBAPI(bool) com1_ServoTRX(unsigned char* cmd, int csize, unsigned char* buf, int bsize);
        RBAPI(bool) com2_ServoTRX(unsigned char* cmd, int csize, unsigned char* buf, int bsize);
        RBAPI(bool) com3_ServoTRX(unsigned char* cmd, int csize, unsigned char* buf, int bsize);
        RBAPI(bool) com4_ServoTRX(unsigned char* cmd, int csize, unsigned char* buf, int bsize);
    #endif

    RBAPI(void) com1_EnableTurboMode(void);
    RBAPI(void) com2_EnableTurboMode(void);
    RBAPI(void) com3_EnableTurboMode(void);
    RBAPI(void) com4_EnableTurboMode(void);
    RBAPI(void) com1_DisableTurboMode(void);
    RBAPI(void) com2_DisableTurboMode(void);
    RBAPI(void) com3_DisableTurboMode(void);
    RBAPI(void) com4_DisableTurboMode(void);
    RBAPI(bool) com1_IsTurboMode(void);
    RBAPI(bool) com2_IsTurboMode(void);
    RBAPI(bool) com3_IsTurboMode(void);
    RBAPI(bool) com4_IsTurboMode(void);

    #if defined(USE_COMSTDIO)
        RBAPI(bool) com1_kbhit(void);
        RBAPI(bool) com2_kbhit(void);
        RBAPI(bool) com3_kbhit(void);
        RBAPI(bool) com4_kbhit(void);
        RBAPI(unsigned int) com1_getch(void);
        RBAPI(unsigned int) com2_getch(void);
        RBAPI(unsigned int) com3_getch(void);
        RBAPI(unsigned int) com4_getch(void);
    #endif

#ifdef __cplusplus
}
#endif
#endif

#if !defined(ROBOIO_DLL) || defined(__COM_LIB)
    RB_INLINE RBAPI(bool) com1_InUse(void) {
        return com_InUse(COM_PORT1);
    }
    RB_INLINE RBAPI(bool) com2_InUse(void) {
        return com_InUse(COM_PORT2);
    }
    RB_INLINE RBAPI(bool) com3_InUse(void) {
        return com_InUse(COM_PORT3);
    }
    RB_INLINE RBAPI(bool) com4_InUse(void) {
        return com_InUse(COM_PORT4);
    }

    RB_INLINE RBAPI(bool) com1_Init(int duplex) {
        return com_Init(COM_PORT1, duplex);
    }
    RB_INLINE RBAPI(bool) com2_Init(int duplex) {
        return com_Init(COM_PORT2, duplex);
    }
    RB_INLINE RBAPI(bool) com3_Init(int duplex) {
        return com_Init(COM_PORT3, duplex);
    }
    RB_INLINE RBAPI(bool) com4_Init(int duplex) {
        return com_Init(COM_PORT4, duplex);
    }
    RB_INLINE RBAPI(void) com1_Close(void) {
        com_Close(COM_PORT1);
    }
    RB_INLINE RBAPI(void) com2_Close(void) {
        com_Close(COM_PORT2);
    }
    RB_INLINE RBAPI(void) com3_Close(void) {
        com_Close(COM_PORT3);
    }
    RB_INLINE RBAPI(void) com4_Close(void) {
        com_Close(COM_PORT4);
    }

    RB_INLINE RBAPI(bool) com1_Send(unsigned char* buf, int bsize) {
        return com_Send(COM_PORT1, buf, bsize);
    }
    RB_INLINE RBAPI(bool) com2_Send(unsigned char* buf, int bsize) {
        return com_Send(COM_PORT2, buf, bsize);
    }
    RB_INLINE RBAPI(bool) com3_Send(unsigned char* buf, int bsize) {
        return com_Send(COM_PORT3, buf, bsize);
    }
    RB_INLINE RBAPI(bool) com4_Send(unsigned char* buf, int bsize) {
        return com_Send(COM_PORT4, buf, bsize);
    }
    RB_INLINE RBAPI(bool) com1_Write(unsigned char val) {
        return com_Write(COM_PORT1, val);
    }
    RB_INLINE RBAPI(bool) com2_Write(unsigned char val) {
        return com_Write(COM_PORT2, val);
    }
    RB_INLINE RBAPI(bool) com3_Write(unsigned char val) {
        return com_Write(COM_PORT3, val);
    }
    RB_INLINE RBAPI(bool) com4_Write(unsigned char val) {
        return com_Write(COM_PORT4, val);
    }

    RB_INLINE RBAPI(bool) com1_Receive(unsigned char* buf, int bsize) {
        return com_Receive(COM_PORT1, buf, bsize);
    }
    RB_INLINE RBAPI(bool) com2_Receive(unsigned char* buf, int bsize) {
        return com_Receive(COM_PORT2, buf, bsize);
    }
    RB_INLINE RBAPI(bool) com3_Receive(unsigned char* buf, int bsize) {
        return com_Receive(COM_PORT3, buf, bsize);
    }
    RB_INLINE RBAPI(bool) com4_Receive(unsigned char* buf, int bsize) {
        return com_Receive(COM_PORT4, buf, bsize);
    }
    RB_INLINE RBAPI(unsigned int) com1_Read(void) {
        return com_Read(COM_PORT1);
    }
    RB_INLINE RBAPI(unsigned int) com2_Read(void) {
        return com_Read(COM_PORT2);
    }
    RB_INLINE RBAPI(unsigned int) com3_Read(void) {
        return com_Read(COM_PORT3);
    }
    RB_INLINE RBAPI(unsigned int) com4_Read(void) {
        return com_Read(COM_PORT4);
    }

    RB_INLINE RBAPI(bool) com1_ClearWFIFO(void) {
        return com_ClearWFIFO(COM_PORT1);
    }
    RB_INLINE RBAPI(bool) com2_ClearWFIFO(void) {
        return com_ClearWFIFO(COM_PORT2);
    }
    RB_INLINE RBAPI(bool) com3_ClearWFIFO(void) {
        return com_ClearWFIFO(COM_PORT3);
    }
    RB_INLINE RBAPI(bool) com4_ClearWFIFO(void) {
        return com_ClearWFIFO(COM_PORT4);
    }
    RB_INLINE RBAPI(bool) com1_FlushWFIFO(void) {
        return com_FlushWFIFO(COM_PORT1);
    }
    RB_INLINE RBAPI(bool) com2_FlushWFIFO(void) {
        return com_FlushWFIFO(COM_PORT2);
    }
    RB_INLINE RBAPI(bool) com3_FlushWFIFO(void) {
        return com_FlushWFIFO(COM_PORT3);
    }
    RB_INLINE RBAPI(bool) com4_FlushWFIFO(void) {
        return com_FlushWFIFO(COM_PORT4);
    }
    RB_INLINE RBAPI(bool) com1_ClearRFIFO(void) {
        return com_ClearRFIFO(COM_PORT1);
    }
    RB_INLINE RBAPI(bool) com2_ClearRFIFO(void) {
        return com_ClearRFIFO(COM_PORT2);
    }
    RB_INLINE RBAPI(bool) com3_ClearRFIFO(void) {
        return com_ClearRFIFO(COM_PORT3);
    }
    RB_INLINE RBAPI(bool) com4_ClearRFIFO(void) {
        return com_ClearRFIFO(COM_PORT4);
    }
    RB_INLINE RBAPI(int) com1_QueryRFIFO(void) {
        return com_QueryRFIFO(COM_PORT1);
    }
    RB_INLINE RBAPI(int) com2_QueryRFIFO(void) {
        return com_QueryRFIFO(COM_PORT2);
    }
    RB_INLINE RBAPI(int) com3_QueryRFIFO(void) {
        return com_QueryRFIFO(COM_PORT3);
    }
    RB_INLINE RBAPI(int) com4_QueryRFIFO(void) {
        return com_QueryRFIFO(COM_PORT4);
    }

    RB_INLINE RBAPI(bool) com1_SetFormat(int bytesize, int stopbit, int parity) {
        return com_SetFormat(COM_PORT1, bytesize, stopbit, parity);
    }
    RB_INLINE RBAPI(bool) com2_SetFormat(int bytesize, int stopbit, int parity) {
        return com_SetFormat(COM_PORT2, bytesize, stopbit, parity);
    }
    RB_INLINE RBAPI(bool) com3_SetFormat(int bytesize, int stopbit, int parity) {
        return com_SetFormat(COM_PORT3, bytesize, stopbit, parity);
    }
    RB_INLINE RBAPI(bool) com4_SetFormat(int bytesize, int stopbit, int parity) {
        return com_SetFormat(COM_PORT4, bytesize, stopbit, parity);
    }
    RB_INLINE RBAPI(bool) com1_SetBaud(int baudrate) {
        return com_SetBaud(COM_PORT1, baudrate);
    }
    RB_INLINE RBAPI(bool) com2_SetBaud(int baudrate) {
        return com_SetBaud(COM_PORT2, baudrate);
    }
    RB_INLINE RBAPI(bool) com3_SetBaud(int baudrate) {
        return com_SetBaud(COM_PORT3, baudrate);
    }
    RB_INLINE RBAPI(bool) com4_SetBaud(int baudrate) {
        return com_SetBaud(COM_PORT4, baudrate);
    }

    #ifdef ROBOIO
        RB_INLINE RBAPI(bool) com1_ServoTRX(unsigned char* cmd, int csize, unsigned char* buf, int bsize) {
            return com_ServoTRX(COM_PORT1, cmd, csize, buf, bsize);
        }
        RB_INLINE RBAPI(bool) com2_ServoTRX(unsigned char* cmd, int csize, unsigned char* buf, int bsize) {
            return com_ServoTRX(COM_PORT2, cmd, csize, buf, bsize);
        }
        RB_INLINE RBAPI(bool) com3_ServoTRX(unsigned char* cmd, int csize, unsigned char* buf, int bsize) {
            return com_ServoTRX(COM_PORT3, cmd, csize, buf, bsize);
        }
        RB_INLINE RBAPI(bool) com4_ServoTRX(unsigned char* cmd, int csize, unsigned char* buf, int bsize) {
            return com_ServoTRX(COM_PORT4, cmd, csize, buf, bsize);
        }
    #endif

    RB_INLINE RBAPI(void) com1_EnableTurboMode(void) {
        com_EnableTurboMode(COM_PORT1);
    }
    RB_INLINE RBAPI(void) com2_EnableTurboMode(void) {
        com_EnableTurboMode(COM_PORT2);
    }
    RB_INLINE RBAPI(void) com3_EnableTurboMode(void) {
        com_EnableTurboMode(COM_PORT3);
    }
    RB_INLINE RBAPI(void) com4_EnableTurboMode(void) {
        com_EnableTurboMode(COM_PORT4);
    }
    RB_INLINE RBAPI(void) com1_DisableTurboMode(void) {
        com_DisableTurboMode(COM_PORT1);
    }
    RB_INLINE RBAPI(void) com2_DisableTurboMode(void) {
        com_DisableTurboMode(COM_PORT2);
    }
    RB_INLINE RBAPI(void) com3_DisableTurboMode(void) {
        com_DisableTurboMode(COM_PORT3);
    }
    RB_INLINE RBAPI(void) com4_DisableTurboMode(void) {
        com_DisableTurboMode(COM_PORT4);
    }
    RB_INLINE RBAPI(bool) com1_IsTurboMode(void) {
        return com_IsTurboMode(COM_PORT1);
    }
    RB_INLINE RBAPI(bool) com2_IsTurboMode(void) {
        return com_IsTurboMode(COM_PORT2);
    }
    RB_INLINE RBAPI(bool) com3_IsTurboMode(void) {
        return com_IsTurboMode(COM_PORT3);
    }
    RB_INLINE RBAPI(bool) com4_IsTurboMode(void) {
        return com_IsTurboMode(COM_PORT4);
    }

    #if defined(USE_COMSTDIO) || defined(__COM_LIB)
        RB_INLINE RBAPI(bool) com1_kbhit(void) {
            return com_kbhit(COM_PORT1);
        }
        RB_INLINE RBAPI(bool) com2_kbhit(void) {
            return com_kbhit(COM_PORT2);
        }
        RB_INLINE RBAPI(bool) com3_kbhit(void) {
            return com_kbhit(COM_PORT3);
        }
        RB_INLINE RBAPI(bool) com4_kbhit(void) {
            return com_kbhit(COM_PORT4);
        }
        RB_INLINE RBAPI(unsigned int) com1_getch(void) {
            return com_getch(COM_PORT1);
        }
        RB_INLINE RBAPI(unsigned int) com2_getch(void) {
            return com_getch(COM_PORT2);
        }
        RB_INLINE RBAPI(unsigned int) com3_getch(void) {
            return com_getch(COM_PORT3);
        }
        RB_INLINE RBAPI(unsigned int) com4_getch(void) {
            return com_getch(COM_PORT4);
        }
    #endif

#endif
/*-----------------------  end of Inline Functions  ----------------------*/

#endif

