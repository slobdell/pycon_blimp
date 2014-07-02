#define __COM_LIB

#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#define  USE_COMMON
#include "common.h"
#include "io.h"
#include "com.h"

#if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
    #include <windows.h>
    #include <tchar.h>
#elif defined(RB_LINUX)
    #include <unistd.h>
    #include <termios.h>
    #include <fcntl.h>
    #include <errno.h>
    #include <sys/ioctl.h>
    #include <sys/resource.h>
#endif


#define COM_TIMEOUT         (5000L)  // timeout = 5s
static unsigned char  COM_ctrlREG[4]  = {0x53, 0xa0, 0xa4, 0xa8};  // SB UART-CTRL registers
static unsigned char  COM_addrREG[4]  = {0x54, 0xa0, 0xa4, 0xa8};  // SB UART-ADDR registers


#if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
    #ifdef RB_MSVC_WINCE
        static LPCTSTR COM_portname[4] = {_T("COM1:"), _T("COM2:"), _T("COM3:"), _T("COM4:")};
    #else
        static LPCTSTR COM_portname[4] = {_T("\\\\.\\COM1"), _T("\\\\.\\COM2"), _T("\\\\.\\COM3"), _T("\\\\.\\COM4")};
    #endif
    typedef struct com_port {
        HANDLE fp;
        DCB newstate;
        DCB oldstate;
        COMMTIMEOUTS newtimeouts;
        COMMTIMEOUTS oldtimeouts;
    } COM_t;
#elif defined(RB_LINUX)
    static char* COM_portname[4] = {"/dev/ttyS0", "/dev/ttyS1", "/dev/ttyS2", "/dev/ttyS3"};
    typedef struct com_port {
        int fp;
        termios newstate;
        termios oldstate;
    } COM_t;

#else
    // TODO ...
    typedef int* COM_t;
#endif
static COM_t COM_info[4];

static bool COM_oldTMode[4] = {false, false, false, false};  // old turbo-mode setting
static bool COM_oldFMode[4] = {false, false, false, false};  // old FIFO32-mode setting
static int  COM_duplex[4] = {COM_FDUPLEX, COM_FDUPLEX, COM_FDUPLEX, COM_FDUPLEX};  // duplex-mode stting
static unsigned short COM_baseaddr[4] = {0x00, 0x00, 0x00, 0x00};  // I/O base address of each UART


/****************************  Internal Functions  ****************************/
_RB_INLINE bool uart_isenabled(int com) {
    if (com == COM_PORT1)
        return ((read_sb_regb(COM_ctrlREG[com]) & 0x80) == 0)? false : true;
    else
        return ((read_sb_reg(COM_ctrlREG[com]) & (0x01L << 23)) == 0L)? false : true;
}

_RB_INLINE unsigned short uart_getbaseaddr(int com) {
    return (unsigned short)(read_sb_reg(COM_addrREG[com]) & 0xfff8L);
}

_RB_INLINE void clear_rts(int com) {
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        EscapeCommFunction(COM_info[com].fp, CLRRTS);
    #elif defined(RB_LINUX)
        int cmd;

        ioctl(COM_info[com].fp, TIOCMGET, &cmd);
        cmd &= ~TIOCM_RTS;
        ioctl(COM_info[com].fp, TIOCMSET, &cmd);
    #endif
}

_RB_INLINE void set_rts(int com) {
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        EscapeCommFunction (COM_info[com].fp, SETRTS);
    #elif defined(RB_LINUX)
        int cmd;

        ioctl(COM_info[com].fp, TIOCMGET, &cmd);
        cmd |= TIOCM_RTS;
        ioctl(COM_info[com].fp, TIOCMSET, &cmd);
    #endif
}

_RB_INLINE int check_rfifo(int com) {
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        COMSTAT stat;
        
        stat.cbInQue = 0;
        if (ClearCommError(COM_info[com].fp, NULL, &stat) == FALSE) return -1;
        return (int)(stat.cbInQue);
    #elif defined(RB_LINUX)
        int numbytes = 0;

        if (ioctl(COM_info[com].fp, FIONREAD, &numbytes) < 0) return -1;
        return numbytes;
    #else
        // TODO ...
        return -1;
    #endif
}
/*-----------------------  end of Internal Functions  ------------------------*/


static int COM_ioSection[4] = {-1, -1, -1, -1};
RBAPI(bool) com_InUse(int com) {
    if((com < 0) || (com > 3)) return false;
    if(COM_ioSection[com] != -1) return true; else return false;
}

RBAPI(bool) com_Init(int com, int duplex) {
    if (com_InUse(com) == true)
	{
        err_SetMsg(ERROR_COM_INUSE, "COM%d was already opened", com);
		return false;
	}

    #ifdef ROBOIO
        duplex = (duplex == COM_ADUPLEX)? COM_FDUPLEX : duplex;
        switch (roboio_GetRBVer())
        {
            case RB_100b1:
                switch (com)
                {
                    case COM_PORT1: COM_duplex[com] = (duplex != COM_HDUPLEX_TXDEN)? duplex : COM_HDUPLEX; break;
                    case COM_PORT2: COM_duplex[com] = COM_HDUPLEX_TXDEN; break;
                    case COM_PORT3: COM_duplex[com] = (duplex == COM_FDUPLEX)? duplex : COM_HDUPLEX; break;
                    case COM_PORT4: COM_duplex[com] = COM_HDUPLEX_RTS;   break;
                }
                break;
            case RB_100b2:
                switch (com)
                {
                    case COM_PORT1: COM_duplex[com] = COM_FDUPLEX;       break;
                    case COM_PORT2: COM_duplex[com] = COM_HDUPLEX_TXDEN; break;
                    case COM_PORT3: COM_duplex[com] = (duplex == COM_FDUPLEX)? duplex : COM_HDUPLEX; break;
                    case COM_PORT4: COM_duplex[com] = COM_HDUPLEX_RTS;   break;
                }
                break;
            case RB_100b3:
                switch (com)
                {
                    case COM_PORT1: COM_duplex[com] = (duplex != COM_HDUPLEX_TXDEN)? duplex : COM_HDUPLEX; break;
                    case COM_PORT2: COM_duplex[com] = COM_HDUPLEX_TXDEN; break;
                    case COM_PORT3: COM_duplex[com] = (duplex == COM_FDUPLEX)? duplex : COM_HDUPLEX; break;
                    case COM_PORT4: COM_duplex[com] = COM_HDUPLEX;       break;
                }
                break;
            case RB_100:
            case RB_100RD:
                switch (com)
                {
                    case COM_PORT1: COM_duplex[com] = COM_FDUPLEX;       break;
                    case COM_PORT2: COM_duplex[com] = COM_HDUPLEX_TXDEN; break;
                    case COM_PORT3: COM_duplex[com] = (duplex == COM_FDUPLEX)? duplex : COM_HDUPLEX; break;
                    case COM_PORT4: COM_duplex[com] = COM_HDUPLEX;       break;
                }
                break;
            case RB_110:
            case RB_050:
                switch (com)
                {
                    case COM_PORT1: COM_duplex[com] = COM_FDUPLEX;       break;
                    case COM_PORT2: COM_duplex[com] = COM_HDUPLEX_TXDEN; break;
                    case COM_PORT3: COM_duplex[com] = (duplex == COM_FDUPLEX)? duplex : COM_HDUPLEX; break;
                    case COM_PORT4: COM_duplex[com] = (duplex == COM_FDUPLEX)? duplex : COM_HDUPLEX; break;
                }
                break;
            default:
                err_SetMsg(ERROR_RBVER_UNKNOWN, "unrecognized RoBoard");
                return false;
        }
    #else
        COM_duplex[com] = duplex;
    #endif
    
    if((COM_ioSection[com] = io_Init()) == -1) return false;

    if(uart_isenabled(com) == false)
    {
        err_SetMsg(ERROR_COM_INVALID, "COM%d isn't enabled in BIOS", com);
        goto COMINIT_FAIL;
    }
    COM_baseaddr[com] = uart_getbaseaddr(com);
    COM_oldTMode[com] = com_IsTurboMode(com);
    COM_oldFMode[com] = com_IsFIFO32Mode(com);
    
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        {
        #ifdef RB_MSVC_WINCE
            int idx = com;
        #else
            int i, idx;

            // find the device name of the COM port
            for (idx=0, i=0; i<com; i++) if (uart_isenabled(i) == true) idx++;
        #endif
        
        COM_info[com].fp = CreateFile(
                               COM_portname[idx],             // device name of COM port
                               GENERIC_READ | GENERIC_WRITE,  // access mode
                               0,                             // share mode
                               0,                             // security attributes
                               OPEN_EXISTING,                 // opens a device only if it exists
                               0,                             // non-overlapped
                               NULL);                         // NULL when opening an existing file

    	if (COM_info[com].fp == INVALID_HANDLE_VALUE)
    	{
            err_SetMsg(ERROR_COM_FAIL, "cannot open COM%d device driver", com);
            goto COMINIT_FAIL;
    	}

        // backup the old DCB
	    if (GetCommState(COM_info[com].fp, &(COM_info[com].oldstate)) == FALSE)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to get DCB settings");
            goto COMINIT_FAIL2;
	    }
	    memcpy(&(COM_info[com].newstate), &(COM_info[com].oldstate), sizeof(DCB));

        // set new DCB
        COM_info[com].newstate.fBinary         = TRUE;                 // binary mode
        COM_info[com].newstate.fOutxCtsFlow    = FALSE;                // no CTS output control
        COM_info[com].newstate.fOutxDsrFlow    = FALSE;                // no DSR output control
        COM_info[com].newstate.fDtrControl     = DTR_CONTROL_DISABLE;  // no DRT control
        COM_info[com].newstate.fDsrSensitivity = FALSE;                // no sensitive to DSR
        COM_info[com].newstate.fOutX           = FALSE;                // no S/W output flow control
        COM_info[com].newstate.fInX            = FALSE;                // no S/W input flow control
        COM_info[com].newstate.fErrorChar      = FALSE;                // no replace parity-error byte
        COM_info[com].newstate.fNull           = FALSE;                // no discard NULL byte
        COM_info[com].newstate.fRtsControl     = RTS_CONTROL_DISABLE;  // no S/W input flow control
        COM_info[com].newstate.fAbortOnError   = FALSE;                // no terminate on errors
	    if (SetCommState(COM_info[com].fp, &(COM_info[com].newstate)) == FALSE)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to set DCB settings");
            goto COMINIT_FAIL2;
	    }

        // get old timeout parameters
        if (GetCommTimeouts(COM_info[com].fp, &(COM_info[com].oldtimeouts)) == FALSE)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to get TIMEOUTS settings");
            goto COMINIT_FAIL3;
	    }

        // set timeout parameters (no waiting on read/write)
        COM_info[com].newtimeouts.ReadIntervalTimeout         = MAXDWORD;
    	COM_info[com].newtimeouts.ReadTotalTimeoutConstant    = 0;
    	COM_info[com].newtimeouts.ReadTotalTimeoutMultiplier  = 0;
    	COM_info[com].newtimeouts.WriteTotalTimeoutConstant   = 0;
    	COM_info[com].newtimeouts.WriteTotalTimeoutMultiplier = 0;
        if (SetCommTimeouts(COM_info[com].fp, &(COM_info[com].newtimeouts)) == FALSE)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to set TIMEOUT parameters");
            goto COMINIT_FAIL3;
        }
        
        ClearCommBreak(COM_info[com].fp);
        ClearCommError(COM_info[com].fp, NULL, NULL);  // clear all communication errors
        SetupComm(COM_info[com].fp, 8192, 8192);          // set read/write FIFO to 8KB
        PurgeComm(COM_info[com].fp, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR);  // clear all communication buffers
        }
    #elif defined(RB_LINUX)
        if ((COM_info[com].fp = open(COM_portname[com], O_RDWR | O_NOCTTY | O_NONBLOCK)) < 0)
        {
            err_SetMsg(ERROR_COM_FAIL, "cannot open COM%d device driver", com);
            goto COMINIT_FAIL;
        }
    	
        // backup the old termios settings
    	if (tcgetattr(COM_info[com].fp, &(COM_info[com].oldstate)) < 0)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to get termios settings");
            goto COMINIT_FAIL2;
	    }
	    memcpy(&(COM_info[com].newstate), &(COM_info[com].oldstate), sizeof(termios));

        // set new termios settings
        COM_info[com].newstate.c_cflag     |= CLOCAL | CREAD;
        COM_info[com].newstate.c_cflag     &= ~CRTSCTS;                 // disable H/W flow control
    	COM_info[com].newstate.c_lflag     &= ~(ICANON |                // raw mode
                                                ISIG   |                // disable SIGxxxx signals
                                                IEXTEN |                // disable extended functions
                                                ECHO | ECHOE);          // disable all auto-echo functions
    	COM_info[com].newstate.c_iflag     &= ~(IXON | IXOFF | IXANY);  // disable S/W flow control
    	COM_info[com].newstate.c_oflag     &= ~OPOST;                   // raw output
    	COM_info[com].newstate.c_cc[VTIME]  = 0;                        // no waiting to read
        COM_info[com].newstate.c_cc[VMIN]   = 0;
    	if(tcsetattr(COM_info[com].fp, TCSANOW, &(COM_info[com].newstate)) < 0)
    	{
            err_SetMsg(ERROR_COM_FAIL, "fail to set termios settings");
            goto COMINIT_FAIL2;
        }
        
        // clear input/output buffers
    	tcflush(COM_info[com].fp, TCIOFLUSH);

    #else
        // TODO ...
        err_SetMsg(ERROR_COM_INVALID, "unsupported platform");
        goto COMINIT_FAIL;
    #endif

    if (COM_duplex[com] == COM_HDUPLEX_RTS) clear_rts(com);  // set COM direction as input

    com_SetFormat(com, COM_BYTESIZE8, COM_STOPBIT1, COM_NOPARITY);  // default data format: 8 bits, 1 stop bit, no parity
    com_SetBaud(com, COMBAUD_115200BPS);                            // default baudrate: 115200 bps
    com_EnableFIFO32(com);                                          // set Vortex86DX's UART FIFO as 32 bytes
    return true;


    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
    COMINIT_FAIL3:
        SetCommState(COM_info[com].fp, &(COM_info[com].oldstate));

    COMINIT_FAIL2:
        CloseHandle(COM_info[com].fp);
    #elif defined(RB_LINUX)
    COMINIT_FAIL2:
        close(COM_info[com].fp);
    #endif

COMINIT_FAIL:
    io_Close(COM_ioSection[com]);
    COM_ioSection[com] = -1;
    return false;
}

RBAPI(void) com_Close(int com) {
    if(com_InUse(com) == false) return;

    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        SetCommTimeouts(COM_info[com].fp, &COM_info[com].oldtimeouts);
        SetCommState(COM_info[com].fp, &(COM_info[com].oldstate));
    	CloseHandle(COM_info[com].fp);
    #elif defined(RB_LINUX)
        tcsetattr(COM_info[com].fp, TCSANOW, &(COM_info[com].oldstate));
        close(COM_info[com].fp);
    #endif

    if (COM_oldTMode[com] == true) com_EnableTurboMode(com); else com_DisableTurboMode(com);
    if (COM_oldFMode[com] == true) com_EnableFIFO32(com);    else com_DisableFIFO32(com);

    io_Close(COM_ioSection[com]);
    COM_ioSection[com] = -1;
}



RBAPI(bool) com_SetFormat(int com, int bytesize, int stopbit, int parity) {
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        COM_info[com].newstate.ByteSize = bytesize;
        
        switch (stopbit)
        {
            case COM_STOPBIT1: COM_info[com].newstate.StopBits = ONESTOPBIT;  break;
            case COM_STOPBIT2: COM_info[com].newstate.StopBits = TWOSTOPBITS; break;
        }
    
        switch (parity)
        {
            case COM_NOPARITY:
                COM_info[com].newstate.Parity = NOPARITY;   break;
            case COM_ODDPARITY:
                COM_info[com].newstate.Parity = ODDPARITY;  break;
            case COM_EVENPARITY:
                COM_info[com].newstate.Parity = EVENPARITY; break;
        }

	    if (SetCommState(COM_info[com].fp, &(COM_info[com].newstate)) == FALSE)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to set DCB settings");
            return false;
	    }
    #elif defined(RB_LINUX)
        switch (bytesize)
        {
            case COM_BYTESIZE5:
                COM_info[com].newstate.c_cflag = (COM_info[com].newstate.c_cflag & ~CSIZE) | CS5; break;
            case COM_BYTESIZE6:
                COM_info[com].newstate.c_cflag = (COM_info[com].newstate.c_cflag & ~CSIZE) | CS6; break;
            case COM_BYTESIZE7:
                COM_info[com].newstate.c_cflag = (COM_info[com].newstate.c_cflag & ~CSIZE) | CS7; break;
            case COM_BYTESIZE8:
                COM_info[com].newstate.c_cflag = (COM_info[com].newstate.c_cflag & ~CSIZE) | CS8; break;
        }

        switch (stopbit)
        {
            case COM_STOPBIT1: COM_info[com].newstate.c_cflag &= ~CSTOPB; break;
            case COM_STOPBIT2: COM_info[com].newstate.c_cflag |= CSTOPB;  break;
        }

        switch (parity)
        {
            case COM_NOPARITY:
                COM_info[com].newstate.c_cflag &= ~PARENB;
                COM_info[com].newstate.c_iflag |= IGNPAR;
                COM_info[com].newstate.c_iflag &= ~(INPCK | ISTRIP | PARMRK);
                break;
            case COM_ODDPARITY:
                COM_info[com].newstate.c_cflag |= (PARENB | PARODD);
                //COM_info[com].newstate.c_iflag |= (INPCK | ISTRIP | PARMRK);
                COM_info[com].newstate.c_iflag |= INPCK;                        // enable input parity check
                COM_info[com].newstate.c_iflag &= ~(ISTRIP | PARMRK | IGNPAR);  // neither strip the 8-th bit nor mark parity errors
                break;
            case COM_EVENPARITY:
                COM_info[com].newstate.c_cflag |= PARENB;
                COM_info[com].newstate.c_cflag &= ~PARODD;
                //COM_info[com].newstate.c_iflag |= (INPCK | ISTRIP | PARMRK);
                COM_info[com].newstate.c_iflag |= INPCK;                        // enable input parity check
                COM_info[com].newstate.c_iflag &= ~(ISTRIP | PARMRK | IGNPAR);  // neither strip the 8-th bit nor mark parity errors
                break;
        }

    	if(tcsetattr(COM_info[com].fp, TCSANOW, &(COM_info[com].newstate)) < 0)
    	{
            err_SetMsg(ERROR_COM_FAIL, "fail to set termios settings");
            return false;
        }
    #else
        // TODO ...
        err_SetMsg(ERROR_COM_INVALID, "unsupported platform");
        return false;
    #endif

	return true;
}

RBAPI(bool) com_SetBaud(int com, unsigned int baudrate) {
    unsigned int baud = baudrate;

    switch (baudrate)
    {
        case COMBAUD_748800BPS: baud = COMBAUD_57600BPS; break;
        case COMBAUD_499200BPS: baud = COMBAUD_38400BPS; break;
        case COMBAUD_249600BPS: baud = COMBAUD_19200BPS; break;
    }

    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        {
        DWORD oldbaud = COM_info[com].newstate.BaudRate;

        switch (baud)
        {
            case  COMBAUD_50BPS:     COM_info[com].newstate.BaudRate = 50;     break;
            case  COMBAUD_300BPS:    COM_info[com].newstate.BaudRate = 300;    break;
            case  COMBAUD_1200BPS:   COM_info[com].newstate.BaudRate = 1200;   break;
            case  COMBAUD_2400BPS:   COM_info[com].newstate.BaudRate = 2400;   break;
            case  COMBAUD_4800BPS:   COM_info[com].newstate.BaudRate = 4800;   break;
            case  COMBAUD_9600BPS:   COM_info[com].newstate.BaudRate = 9600;   break;
            case  COMBAUD_19200BPS:  COM_info[com].newstate.BaudRate = 19200;  break;
            case  COMBAUD_38400BPS:  COM_info[com].newstate.BaudRate = 38400;  break;
            case  COMBAUD_57600BPS:  COM_info[com].newstate.BaudRate = 57600;  break;
            case  COMBAUD_115200BPS: COM_info[com].newstate.BaudRate = 115200; break;
        }

	    if (SetCommState(COM_info[com].fp, &(COM_info[com].newstate)) == FALSE)
        {
            COM_info[com].newstate.BaudRate = oldbaud;
            err_SetMsg(ERROR_COM_FAIL, "fail to set DCB settings");
            return false;
	    }
	    
	    }
    #elif defined(RB_LINUX)
        {
        speed_t oldospeed = cfgetospeed(&(COM_info[com].newstate));
        speed_t oldispeed = cfgetispeed(&(COM_info[com].newstate));
        speed_t newspeed  = B50;

        switch (baud)
        {
            case  COMBAUD_50BPS:     newspeed = B50;     break;
            case  COMBAUD_300BPS:    newspeed = B300;    break;
            case  COMBAUD_1200BPS:   newspeed = B1200;   break;
            case  COMBAUD_2400BPS:   newspeed = B2400;   break;
            case  COMBAUD_4800BPS:   newspeed = B4800;   break;
            case  COMBAUD_9600BPS:   newspeed = B9600;   break;
            case  COMBAUD_19200BPS:  newspeed = B19200;  break;
            case  COMBAUD_38400BPS:  newspeed = B38400;  break;
            case  COMBAUD_57600BPS:  newspeed = B57600;  break;
            case  COMBAUD_115200BPS: newspeed = B115200; break;
        }
        cfsetospeed(&(COM_info[com].newstate), newspeed);
        cfsetispeed(&(COM_info[com].newstate), newspeed);

    	if(tcsetattr(COM_info[com].fp, TCSANOW, &(COM_info[com].newstate)) < 0)
    	{
            cfsetospeed(&(COM_info[com].newstate), oldospeed);
            cfsetispeed(&(COM_info[com].newstate), oldispeed);
            err_SetMsg(ERROR_COM_FAIL, "fail to set termios settings");
            return false;
        }
        
        }
    #else
        // TODO ...
        err_SetMsg(ERROR_COM_INVALID, "unsupported platform");
        return false;
    #endif
    
    if ((baudrate & 0x8000) != 0) com_EnableTurboMode(com); else com_DisableTurboMode(com);

    return true;
}



#define MAXRWSIZE   (256)
RBAPI(bool) com_Receive(int com, unsigned char* buf, int bsize) {
    int numbytes, bsize2;
    unsigned long nowtime;

    if (bsize <= 0) return true;

    for (; bsize > 0; bsize -= bsize2, buf += bsize2)
    {
        bsize2 = (bsize <= MAXRWSIZE)? bsize : MAXRWSIZE;

        // wait enough bytes to read
        for (nowtime = timer_nowtime(); (numbytes = check_rfifo(com)) < bsize2; )
        {
            if (numbytes < 0)
            {
                err_SetMsg(ERROR_COM_READFAIL, "fail to check read FIFO");
                return false;
            }

            if ((timer_nowtime() - nowtime) > COM_TIMEOUT)
            {
                err_SetMsg(ERROR_COM_READFAIL, "time-out to read bytes");
                return false;
            }
        } // end for (nowtime...
    
        #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
            if (ReadFile(COM_info[com].fp, buf, bsize2, (LPDWORD)&numbytes, NULL) == FALSE)
            {
                err_SetMsg(ERROR_COM_READFAIL, "ReadFile() fails");
                return false;
            }
        #elif defined(RB_LINUX)
            /*
            if ((numbytes = read(COM_info[com].fp, buf, bsize2)) < 0)
            {
                err_SetMsg(ERROR_COM_READFAIL, "read() fails");
                return false;
            }
            */
            while ((numbytes = read(COM_info[com].fp, buf, bsize2)) < 0)
            {
                if (errno != EINTR)
                {
                    err_SetMsg(ERROR_COM_READFAIL, "read() fails");
                    return false;
                }

                if ((timer_nowtime() - nowtime) > (2L*COM_TIMEOUT))  // note nowtime's value is from the above code of waiting rfifo
                {
                    err_SetMsg(ERROR_COM_READFAIL, "time-out to read bytes");
                    return false;
                }
            }
        #else
            // TODO ...
            err_SetMsg(ERROR_COM_INVALID, "unsupported platform");
            return false;
        #endif
    
        if (numbytes != bsize2)
        {
            err_SetMsg(ERROR_COM_READFAIL, "cannot read enough bytes");
            return false;
        }
    } // end for (; bsize...
    
    return true;
}

RBAPI(unsigned int) com_Read(int com) {
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE) || defined(RB_LINUX)
        unsigned char val;

        if (com_Receive(com, &val, 1) == false) return 0xffff;
        return (unsigned int)val;
    #else
        // TODO ...
        err_SetMsg(ERROR_COM_INVALID, "unsupported platform");
        return 0xffff;
    #endif
}

RBAPI(bool) com_ClearRFIFO(int com) {
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        if (PurgeComm(COM_info[com].fp, PURGE_RXCLEAR) == FALSE)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to clear read FIFO");
            return false;
        }
    #elif defined(RB_LINUX)
        if (tcflush(COM_info[com].fp, TCIFLUSH) < 0)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to clear read FIFO");
            return false;
        }
	#endif

    return true;
}

RBAPI(int) com_QueryRFIFO(int com) {
    int numbytes = check_rfifo(com);

    if (numbytes < 0) err_SetMsg(ERROR_COM_FAIL, "fail to query read FIFO");
    return numbytes;
}



#if defined(RB_MSVC_WIN32)
    static HANDLE hApp, hThread;
    static int iPriorityClass, iPriority;
#elif defined(RB_MSVC_WINCE)
    static HANDLE hThread;
    static int iPriority;
#elif defined(RB_LINUX)
	static int iPriority;
#endif

static void MPOS_Start(void) {
    #if defined(RB_MSVC_WIN32)
	    hApp = GetCurrentProcess();
        hThread = GetCurrentThread();

        iPriorityClass = GetPriorityClass(hApp);
        iPriority = GetThreadPriority(hThread);

        if (iPriorityClass != 0) SetPriorityClass(hApp, REALTIME_PRIORITY_CLASS);
        if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
	#elif defined(RB_MSVC_WINCE)
        hThread = GetCurrentThread();
        iPriority = GetThreadPriority(hThread);
        if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, THREAD_PRIORITY_TIME_CRITICAL);
    #elif defined(RB_LINUX)
	    iPriority = getpriority(PRIO_PROCESS, 0); //lazy to check error:p
        setpriority(PRIO_PROCESS, 0, -20);
    #else
        //backup & disable all IRQ except COM port's IRQ
    #endif
}

static void MPOS_End(void) {
    #if defined(RB_MSVC_WIN32)
        if (iPriorityClass != 0) SetPriorityClass(hApp, iPriorityClass);
		if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, iPriority);
	#elif defined(RB_MSVC_WINCE)
		if (iPriority != THREAD_PRIORITY_ERROR_RETURN) SetThreadPriority(hThread, iPriority);
    #elif defined(RB_LINUX)
		setpriority(PRIO_PROCESS, 0, iPriority);
    #else
        //restore all IRQ settings
    #endif
}

RBAPI(bool) com_Send(int com, unsigned char* buf, int bsize) {
    unsigned long nowtime;
    int numbytes = 0, bsize2;

    if (bsize <= 0) return true;

    if (COM_duplex[com] == COM_HDUPLEX_RTS)
    {
        MPOS_Start();
        set_rts(com);
    }

    while (bsize > 0)
    {
        bsize2 = (bsize <= MAXRWSIZE)? bsize : MAXRWSIZE;
        
        for (nowtime = timer_nowtime(); bsize2 > 0; buf += numbytes, bsize2 -= numbytes, bsize -= numbytes)
        {
            #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
                if (WriteFile(COM_info[com].fp, buf, bsize2, (LPDWORD)&numbytes, NULL) == FALSE)
                {
                    err_SetMsg(ERROR_COM_SENDFAIL, "WriteFile() fails");
                    goto SEND_FAIL;
                }
            #elif defined(RB_LINUX)
                if ((numbytes = write(COM_info[com].fp, buf, bsize2)) < 0)
                {
                    if ((errno != EINTR) && (errno != EAGAIN))
                    {
                        err_SetMsg(ERROR_COM_SENDFAIL, "write() fails");
                        goto SEND_FAIL;
                    }
                    numbytes = 0;
                }
            #else
                // TODO ...
                err_SetMsg(ERROR_COM_INVALID, "unsupported platform");
                goto SEND_FAIL;
            #endif
            
            if ((timer_nowtime() - nowtime) > COM_TIMEOUT)
            {
                err_SetMsg(ERROR_COM_SENDFAIL, "time-out to write bytes");
                goto SEND_FAIL;
            }
        } // for (nowtime...
    } // end while (bsize...

    if (COM_duplex[com] == COM_HDUPLEX_RTS)
    {
        com_FlushWFIFO(com);
        clear_rts(com);
        MPOS_End();
    }

    return true;
    
SEND_FAIL:
    if (COM_duplex[com] == COM_HDUPLEX_RTS)
    {
        clear_rts(com);
        MPOS_End();
    }
    return false;
}

RBAPI(bool) com_Write(int com, unsigned char val) {
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE) || defined(RB_LINUX)
        return com_Send(com, &val, 1);
    #else
        // TODO ...
        err_SetMsg(ERROR_COM_INVALID, "unsupported platform");
        return false;
    #endif
}

RBAPI(bool) com_FlushWFIFO(int com) {
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        if (FlushFileBuffers(COM_info[com].fp) == FALSE)
        {
            err_SetMsg(ERROR_COM_FAIL, "FlushFileBuffers() fails");
            return false;
        }
    #elif defined(RB_LINUX)
        if (COM_duplex[com] != COM_HDUPLEX_RTS)
        {
            if (tcdrain(COM_info[com].fp) < 0)
            {
                err_SetMsg(ERROR_COM_FAIL, "tcdrain() fails");
                return false;
            }
        }
        else  // tcdrain() is too slow in the purpose of switching RTS to read servos in readtime
        {
            //use "ioctl(fd, FIONWRITE, &nBytes)" to wait write-FIFO empty...
        }
    #endif
    
    if (COM_duplex[com] == COM_HDUPLEX_RTS)  // ensure that all bytes in 16550 FIFO have been sent
        while((io_inpb(COM_baseaddr[com] + 5) & 0x60) != 0x60);
    
    return true;
}

RBAPI(bool) com_ClearWFIFO(int com) {
    #if defined(RB_MSVC_WIN32) || defined(RB_MSVC_WINCE)
        if (PurgeComm(COM_info[com].fp, PURGE_TXCLEAR) == FALSE)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to clear write FIFO");
            return false;
        }
    #elif defined(RB_LINUX)
        if (tcflush(COM_info[com].fp, TCOFLUSH) < 0)
        {
            err_SetMsg(ERROR_COM_FAIL, "fail to clear write FIFO");
            return false;
        }
    #endif
    
    return true;
}



#ifdef ROBOIO
    RBAPI(bool) com_ServoTRX(int com, unsigned char* cmd, int csize, unsigned char* buf, int bsize) {
        com_ClearRFIFO(com);
    
        if (com_Send(com, cmd, csize) == false) return false;

        if (COM_duplex[com] == COM_HDUPLEX)  // discard the first-received csize bytes in case of TX/RX-short
        {
            int i; unsigned int tmpbyte; bool errflag = false;

            for (i=0; i<csize; i++)
            {
                if ((tmpbyte = com_Read(com)) == 0xffff) return false;
        
                if ((unsigned char)tmpbyte != cmd[i]) errflag = true;
            }

            if (errflag == true)
            {
                err_SetMsg(ERROR_COM_FAIL, "receive a wrong self-feedback byte");
                return false;
            }
        } // end if (COM_duplex[com] ...
    
        if ((buf != NULL) && (bsize > 0)) //&&
        if (com_Receive(com, buf, bsize) == false) return false;

        return true;
    }
#endif



/*************************  Isolated COM lib Functions  ***********************/
RBAPI(void) com_EnableTurboMode(int com) {
    if ((com < 0) || (com > 3) || (io_CpuID() != CPU_VORTEX86DX_3)) return;

    if (com == COM_PORT1)
        write_sb_regb(COM_ctrlREG[com], read_sb_regb(COM_ctrlREG[com]) | ((unsigned char)1<<6));
    else  // COM2 ~ COM4
        write_sb_reg(COM_ctrlREG[com], read_sb_reg(COM_ctrlREG[com]) | (1L<<22));
}

RBAPI(void) com_DisableTurboMode(int com) {
    if ((com < 0) || (com > 3) || (io_CpuID() != CPU_VORTEX86DX_3)) return;

    if (com == COM_PORT1)
        write_sb_regb(COM_ctrlREG[com], read_sb_regb(COM_ctrlREG[com]) & ~((unsigned char)1<<6));
    else  // COM2 ~ COM4
        write_sb_reg(COM_ctrlREG[com], read_sb_reg(COM_ctrlREG[com]) & ~(1L<<22));
}

RBAPI(bool) com_IsTurboMode(int com) {
    if ((com < 0) || (com > 3) || (io_CpuID() != CPU_VORTEX86DX_3)) return false;
    
    if (com == COM_PORT1)
    {
        if ((read_sb_regb(COM_ctrlREG[com]) & ((unsigned char)1<<6)) == 0) return false;
    }
    else
    {
        if ((read_sb_reg(COM_ctrlREG[com]) & (1L<<22)) == 0L) return false;
    }

    return true;
}

RBAPI(void) com_EnableFIFO32(int com) {
    if ((com < 0) || (com > 3)) return;

    if (com == COM_PORT1)
        write_sb_regb(COM_ctrlREG[com], read_sb_regb(COM_ctrlREG[com]) | ((unsigned char)1<<4));
    else  // COM2 ~ COM4
        write_sb_reg(COM_ctrlREG[com], read_sb_reg(COM_ctrlREG[com]) | (1L<<21));
}

RBAPI(void) com_DisableFIFO32(int com) {
    if ((com < 0) || (com > 3)) return;

    if (com == COM_PORT1)
        write_sb_regb(COM_ctrlREG[com], read_sb_regb(COM_ctrlREG[com]) & ~((unsigned char)1<<4));
    else  // COM2 ~ COM4
        write_sb_reg(COM_ctrlREG[com], read_sb_reg(COM_ctrlREG[com]) & ~(1L<<21));
}

RBAPI(bool) com_IsFIFO32Mode(int com) {
    if ((com < 0) || (com > 3)) return false;
    
    if (com == COM_PORT1)
    {
        if ((read_sb_regb(COM_ctrlREG[com]) & ((unsigned char)1<<4)) == 0) return false;
    }
    else
    {
        if ((read_sb_reg(COM_ctrlREG[com]) & (1L<<21)) == 0L) return false;
    }

    return true;
}
/*--------------------  end of Isolated COM lib Functions  -------------------*/



/****************************  COM STDIO Functions  ***************************/
#ifdef _MANAGED
	#pragma managed(push, off)
#endif
_RBAPI_C(bool) com_printf(int com, char* fmt, ...) {
    char buf[512];
    va_list args;

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);
	
	return com_Send(com, (unsigned char*)buf, (int)strlen(buf));
}

_RBAPI_C(bool) com1_printf(char* fmt, ...) {
    char buf[512];
    va_list args;

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);
	
	return com1_Send((unsigned char*)buf, (int)strlen(buf));
}

_RBAPI_C(bool) com2_printf(char* fmt, ...) {
    char buf[512];
    va_list args;

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);
	
	return com2_Send((unsigned char*)buf, (int)strlen(buf));
}

_RBAPI_C(bool) com3_printf(char* fmt, ...) {
    char buf[512];
    va_list args;

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);
	
	return com3_Send((unsigned char*)buf, (int)strlen(buf));
}

_RBAPI_C(bool) com4_printf(char* fmt, ...) {
    char buf[512];
    va_list args;

	va_start(args, fmt);
	vsprintf(buf, fmt, args);
	va_end(args);
	
	return com4_Send((unsigned char*)buf, (int)strlen(buf));
}
#ifdef _MANAGED
	#pragma managed(pop)
#endif

RBAPI(bool) com_kbhit(int com) {
    if (check_rfifo(com) > 0) return true;
	return false;
}

RBAPI(unsigned int) com_getch(int com) {
    int numbytes;
    
    while ((numbytes = check_rfifo(com)) <= 0)
        if (numbytes < 0) return 0xffff;

    return (int)com_Read(com);
}
/*---------------------- end of COM STDIO Functions  -------------------------*/


