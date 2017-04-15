/**********************************************************************************
 **********************************************************************************
 ***
 ***    nvtcomm.c
 ***    - routines to access the bootloader in the ESP
 ***
 ***    Copyright (C) 2014 Christian Klippel <ck@atelier-klippel.de>
 ***
 ***    This program is free software; you can redistribute it and/or modify
 ***    it under the terms of the GNU General Public License as published by
 ***    the Free Software Foundation; either version 2 of the License, or
 ***    (at your option) any later version.
 ***
 ***    This program is distributed in the hope that it will be useful,
 ***    but WITHOUT ANY WARRANTY; without even the implied warranty of
 ***    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 ***    GNU General Public License for more details.
 ***
 ***    You should have received a copy of the GNU General Public License along
 ***    with this program; if not, write to the Free Software Foundation, Inc.,
 ***    51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 ***
 **/
#include "nvtcomm.h"

static bool nvtcomm_is_open = false;

static const char *nvtcomm_fileName = "file1";

static const char *nvtcomm_port =
#if defined(LINUX)
"/dev/ttyUSB0";
#elif defined(WINDOWS)
"COM1";
#elif defined(OSX)
"/dev/tty.usbserial";
#else
"";
#endif

static unsigned int nvtcomm_baudrate = 115200;
static uint32_t nvtcomm_address = 0x00000;                            

static int file_uploaded = 0;
static int fileReady = 0;

unsigned int g_packno= 1;

unsigned short nvtcomm_calc_checksum(unsigned char *data, uint16_t data_size)
{
    uint16_t cnt;
    unsigned short c;
    
    for(c = 0, cnt = 0; cnt < data_size; cnt++)
    {
        c += data[cnt];
    }
    
    return c;
}

static bool nvtcomm_write()
{
    unsigned int _result;
    unsigned int len;
    unsigned char *pBuf;
    pBuf = WriteReportBuffer + 1;
	len = MAX_PACKET;

	gcksum = nvtcomm_calc_checksum(pBuf, MAX_PACKET);
	
	_result = serialport_write(pBuf, len);
	
	if(_result != len)
	{
		LOGDEBUG("Data length sent dosn't match!");
		return false;
	}
	
	return true;
}

//read data 
static bool nvtcomm_read()
{
    unsigned long nRead;
    unsigned int len;
    unsigned short lcksum;
    unsigned char *pBuf;
    unsigned int uWaitProgress;
    unsigned long tick1,tick2,timeout;
    unsigned int curPacketNo=0;

    len = MAX_PACKET;
    memset(ReadReportBuffer,0,sizeof(ReadReportBuffer));
	
    if ((m_curCmd == CMD_UPDATE_DATAFLASH) ||
    (m_curCmd == CMD_UPDATE_APROM) ||
    (m_curCmd == CMD_ERASE_ALL)) 			// need longer time to wait flash erase
        timeout = 20; //ms
    else
        timeout = 10;

    if (bDetectingSaved) {
        timeout = 1;
    }

//Progress
    if (m_curCmd == CMD_UPDATE_CONFIG)
        uWaitProgress = 160;
    else if ((m_curCmd == CMD_UPDATE_APROM) || (m_curCmd == CMD_UPDATE_DATAFLASH) ||
    (m_curCmd == CMD_ERASE_ALL))
        uWaitProgress = 500;
		
	tick1 = 0;
	tick2 = 0;
	
    while(1) {
		
        pBuf = ReadReportBuffer;
		nRead = 0;
		
		do {
		
		nRead += serialport_read(pBuf, len);
		//LOGDEBUG("nRead = %i", nRead);	
		if (nRead == len)
			break;
		
		//Progress
		if ((m_curCmd == CMD_UPDATE_CONFIG) && (g_packno == 4)) {
			//20% - 100%
			uWaitProgress ++;
			if (uWaitProgress >= 800)
				uWaitProgress = 800;
			nvtcomm_delay_ms(10);
        } 
					
		else if (((m_curCmd == CMD_UPDATE_APROM) ||
        (m_curCmd == CMD_UPDATE_DATAFLASH) ||
        (m_curCmd == CMD_ERASE_ALL)) &&
        (g_packno==4)) {
                        
            uWaitProgress ++;
            if (uWaitProgress >= 2000)
                 uWaitProgress = 2000;

            nvtcomm_delay_ms(10);
        }

        //Time out
        tick2 += 1;
        if ((tick2 - tick1) > timeout)
			return false;

        } while (nRead != len);
                    
        
        //len = MAX_PACKET; //test
        memcpy(&lcksum, pBuf, 2);
        pBuf += 4;

        memcpy(&curPacketNo, pBuf, 4);
		//LOGDEBUG("curPacketNo = %i", curPacketNo);
		//LOGDEBUG("g_packno = %i", g_packno);
        if (curPacketNo != g_packno) {
		//Progress
            if ( (( m_curCmd == CMD_UPDATE_CONFIG)||( m_curCmd == CMD_ERASE_ALL)) && (g_packno == 4) ) {
                //20% - 100%
                
                uWaitProgress ++;
                if(uWaitProgress >= 800)
                    uWaitProgress = 800;

            } else if ( (( m_curCmd == CMD_UPDATE_APROM) || (m_curCmd == CMD_UPDATE_DATAFLASH)) && (g_packno==4) ) {
                uWaitProgress ++;
                if (uWaitProgress >= 2000)
                    uWaitProgress = 2000;

            }

            tick2 += 1;
			if ((tick2 - tick1) > timeout * 10000000)
				return false;
			else
                continue;
        } 
		
		else {
  	//temporary fix for Mac Book, ignore checksum
	//todo: find out the reason why checksum always results in error in MacBook
 	#ifndef __APPLE__
            if(lcksum != gcksum) {
            	
            	LOGDEBUG("Checksum error");
                g_packno++;
                return false;
            }
	#endif

            if( ( m_curCmd == CMD_UPDATE_APROM) && (g_packno==4) )
                LOGDEBUG("Erase success,sending next packet...");

            g_packno++;
            break;
        }
    }

    return true;
}

static bool nvtcomm_send_command(unsigned int cmd)
{
	if(cmd != CMD_RESEND_PACKET)
	    m_curCmd = cmd;
	
    memset(WriteReportBuffer, 0, MAX_PACKET+1);
    memcpy(WriteReportBuffer+1, &cmd, 4);
    memcpy(WriteReportBuffer+5, &g_packno, 4);
    g_packno++;
	
	if(nvtcomm_write() == false)
    {
		//LOGDEBUG("\nWrite unsuccessful\n");
		LOGDEBUG("Sending command unsuccessful");
		return false;
	}
	
    return nvtcomm_read();
}

static bool nvtcomm_sync_packno(void)
{
    bool _result;
    unsigned long cmdData;

    //sync send & recv packno
    memset(WriteReportBuffer, 0, MAX_PACKET+1);
    cmdData = CMD_SYNC_PACKNO;
    memcpy(WriteReportBuffer+1, &cmdData, 4);
    memcpy(WriteReportBuffer+5, &g_packno, 4);
    memcpy(WriteReportBuffer+9, &g_packno, 4);
    g_packno++;

    _result = nvtcomm_write();
    if(_result == false)
        goto failSync;

    _result = nvtcomm_read();
	if(_result == false)
		goto failSync;
	
	return true;
	
failSync:
    LOGDEBUG("nvtcomm_sync failed");
    return false;
}

int nvtcomm_open(void)
{
	bool bResult;
	unsigned int errcnt;
	
	if (nvtcomm_is_open)
		return 1;
		
	LOGINFO("Opening bootloader...");
	
	if(serialport_open(nvtcomm_port, nvtcomm_baudrate))
    {
		errcnt = 0;
		bDetecting = true;
    
		while (bDetecting) {
			bDetectingSaved = true;
			
			bResult = nvtcomm_send_command(CMD_CONNECT);
			g_packno = 1;
        
			if (bResult)
				break;
            
			errcnt += 1;
			if(errcnt > 10) {
				LOGERR("Failed to communicate with the chip.");
				nvtcomm_close();
				return 0;
			}
			
		}			
    }
	else
	{
		LOGDEBUG("Failed to open %s.", nvtcomm_port);
        return 0;
	}

    bDetecting = false;
    bDetectingSaved = false;
	serialport_set_timeout(1000);

	if (bResult)
	{
		LOGINFO("Device detected and enter ISP mode.");
		nvtcomm_is_open = true;
		nvtcomm_delay_ms(100); //Waiting for ISP Firmware Working
		return 1;
    }
    
    return 0;
	
}

void nvtcomm_close(void)
{
    LOGINFO("Closing bootloader");
    serialport_close();
}

bool nvtcomm_load_file()
{
	LOGINFO("Loading binary file information...");
	LOGDEBUG("Target file: %s", nvtcomm_fileName);
    FILE *f;
    struct stat st;
	uint32_t fsize;
	bool Result;
	
    if(stat(nvtcomm_fileName, &st) == 0) {
		
		fsize = (uint32_t) st.st_size;
		m_sMyFileInfo.uCodeFileSize = fsize;
        
		LOGDEBUG("Sketch file size: %i Bytes", m_sMyFileInfo.uCodeFileSize);
        
		if(m_sMyFileInfo.uCodeFileSize > m_sMyChipType.uFlashSize)
		{
			LOGERR("File size is too large!\n");
			Result = false;
			return Result;
		}
		
		 f = fopen(nvtcomm_fileName, "rb");
		 fread(&CodeFileBuffer, fsize , 1, f);
		 fclose(f);
		 m_sMyFileInfo.uCodeFileCheckSum = nvtcomm_calc_checksum(CodeFileBuffer, fsize);
		 LOGDEBUG("Filechecksum: %i", m_sMyFileInfo.uCodeFileCheckSum);
		 m_sMyFileInfo.uCodeFileStartAddr = 0; //address 0
		 Result = true;
    }
	else
	{
		LOGERR("The file doesn't exist");
		Result = false;
	}
    
    return Result;
}

int nvtcomm_upload_file()
{
	if(!nvtcomm_load_file())
		return 0;
	
	INFO("Uploading %i bytes from %s to flash at 0x%08X\n", m_sMyFileInfo.uCodeFileSize, nvtcomm_fileName, nvtcomm_address);
    
	bool Result;
    //unsigned int nCurProgress = 0;
    unsigned long readcn, cmdData;
	unsigned long sendcn = 0;
	//unsigned long sendPacketSize;
    unsigned short get_cksum;
    unsigned char *tranBuf;
    unsigned long   tranBufStartAddr;
    unsigned long   tranBufSize;
    unsigned long  tranBufCheckSum;
	int percentage = 0;
	int idx = 0;
    
    tranBuf = CodeFileBuffer; 
    tranBufStartAddr = m_sMyFileInfo.uCodeFileStartAddr;
    tranBufSize = m_sMyFileInfo.uCodeFileSize;
    tranBufCheckSum = m_sMyFileInfo.uCodeFileCheckSum;
            
	Result = nvtcomm_sync_packno();
    if(Result == false) {
        LOGDEBUG("Send sync packno cmd fail");
        goto out;
    }
    
	LOGDEBUG("Send sync packno cmd OK");

    /** send updata aprom command**/
    memset(WriteReportBuffer, 0, MAX_PACKET+1);
    m_curCmd = CMD_UPDATE_APROM;
    cmdData = m_curCmd;
    memcpy(WriteReportBuffer+1, &cmdData, 4);
    memcpy(WriteReportBuffer+5, &g_packno, 4);
    g_packno++;

    memcpy(WriteReportBuffer+9, &tranBufStartAddr, 4);
    memcpy(WriteReportBuffer+13, &tranBufSize, 4);

    readcn = tranBufSize;
    sendcn = MAX_PACKET - 16;
    if(sendcn > readcn)
        sendcn = readcn;
    memcpy(WriteReportBuffer+17, tranBuf, sendcn);

    LOGDEBUG("Sending first packet...it will take a long time");
    //send CMD
    Result = nvtcomm_write();
    if(Result == false)
        goto out;

    Result = nvtcomm_read();
	if(Result == false)
        goto out;

	percentage = (tranBufSize*2) /100;
	idx = sendcn / percentage;
	if(idx > 50) idx = 50;
	
	LOGDEBUG("Sending next packets...");
	INFO("In progress: ");
	
	for(int i = 0; i < idx;i++)
		INFO("#");
	
	idx += 1;
	
	while(sendcn < readcn) { 
        WriteReportBuffer[0] = 0x00;
        cmdData = 0x00000000;//continue

        memcpy(WriteReportBuffer+1, &cmdData, 4);
        memcpy(WriteReportBuffer+5, &g_packno, 4);
        g_packno++;

        if((readcn - sendcn) < (MAX_PACKET-8)) { 
            memcpy(WriteReportBuffer+9, tranBuf+sendcn, readcn - sendcn);
            //sendPacketSize = readcn - sendcn;
            sendcn = readcn;
        } else {
            memcpy(WriteReportBuffer+9, tranBuf+sendcn, MAX_PACKET-8);
            //sendPacketSize = MAX_PACKET-8;
            sendcn += MAX_PACKET-8;
        }
        Result = nvtcomm_write();
        if(Result == false)
            goto out;
        Result = nvtcomm_read();
        if(Result == false) {
                goto out;
        }

		if(sendcn >= idx*percentage)
		{
			INFO("#");
			fflush(stdout);
			idx += 1;
		}
    }
	INFO("\nDone uploading\n");

//Check sum again
    memcpy(&get_cksum, ReadReportBuffer+8, 2);
    
	if(Result == true) {
        if(tranBufCheckSum == get_cksum) 
            Result = true;
		else 
            Result = false;
    } 
    
	if(!Result)
		LOGDEBUG("Get checksum error");

out:

    if (Result == true) {
		file_uploaded = 1;
        LOGDEBUG("Send success");
    } 
	else {
        LOGDEBUG("Send fail:progress=%d%%", (sendcn * 100)/tranBufSize);
    }
    
    return Result;
}

int nvtcomm_start_app(int reboot)
{
    if(nvtcomm_is_open)
		serialport_close();
	
	if(reboot)
	{
		LOGINFO("Resetting board...");
		if(serialport_open(nvtcomm_port, nvtcomm_baudrate))
		{
			nvtcomm_close();
		}
	}
	
	fileReady = 0;
	file_uploaded = 0;	
	//todo
	return 1;

}

int nvtcomm_file_uploaded()
{
    return file_uploaded;
}

int nvtcomm_file_ready()
{
	return fileReady;
}

int nvtcomm_set_port(char *port)
{
    LOGDEBUG("setting port from %s to %s", nvtcomm_port, port);
    nvtcomm_port = port;
    return 1;
}

int nvtcomm_set_baudrate(const char *baudrate)
{
    uint32_t new_baudrate = (uint32_t) strtol(baudrate, NULL, 10);
    LOGDEBUG("setting baudrate from %i to %i", nvtcomm_baudrate, new_baudrate);
    nvtcomm_baudrate = new_baudrate;
    return 1;
}

int nvtcomm_set_address(const char *address)
{
    uint32_t new_address = (uint32_t) strtol(address, NULL, 16);
    LOGDEBUG("setting address from 0x%08X to 0x%08X", nvtcomm_address, new_address);
    nvtcomm_address = new_address;
    return 1;
}

int nvtcomm_set_filename(char *name)
{	
	LOGDEBUG("setting target file %s", (const char*)name);
	nvtcomm_fileName = (const char*)name;
	fileReady = 1;
	return 1;
}

static bool GetChipInfo()
{
    if(!m_sMyChipType.uChipID)
        return false;
    
	// can provide info for several boards
	 
    switch(m_sMyChipType.uChipID)
    {
    	case NUEDU_UNO: 
    		m_sMyChipType.uFlashSize = 64 * 1024;
    		m_sMyChipType.uRamSize = 8 * 1024;
    		strcpy(m_sMyChipType.cChipName,"NUC131SD2AE");
    		break;
			
		case CT_ARM: 
    		m_sMyChipType.uFlashSize = 64 * 1024;
    		m_sMyChipType.uRamSize = 8 * 1024;
    		strcpy(m_sMyChipType.cChipName,"CT-ARM");
    		break;
    	
    	default:
    		m_sMyChipType.uFlashSize = 64 * 1024;
    		m_sMyChipType.uRamSize = 8 * 1024;
    		strcpy(m_sMyChipType.cChipName, "Unknown");
    		break;
	}
	
	LOGDEBUG("Chip Name: %s", m_sMyChipType.cChipName);
    LOGDEBUG("Chip RAM Size: %i Bytes", m_sMyChipType.uRamSize);
    LOGDEBUG("Chip Flash Size: %i Bytes", m_sMyChipType.uFlashSize);
    
    return true;

}

bool nvtcomm_get_chip_info()
{
    bool Result;
	LOGINFO("Obtaining onboard chip info...");
//CMD_SYNC_PACKNO
    LOGDEBUG("Synchronizing with chip...");
    Result = nvtcomm_sync_packno();
    if(Result == false) {
        Result = nvtcomm_sync_packno();
        if(Result == false)
            goto fail;
    }

//CMD_GET_VERSION
    LOGDEBUG("Getting ISP version...");
    Result = nvtcomm_sync_packno();
    if(Result == false)
        goto fail;

    Result = nvtcomm_send_command(CMD_GET_FWVER);
    if (Result == false)
        goto fail;
	
	memcpy(&m_IspVersion,ReadReportBuffer+8,1);
	LOGDEBUG("Current FW version V%x.%x",m_IspVersion>>4,m_IspVersion&0xF);

    if (m_IspVersion == 0) //ISP FW version
        goto fail;

    if (m_IspVersion > FW_VER_NUM) {
        LOGDEBUG("Firmware version problem!\nCurrent FW version V%x.%x is newer than V%x.%x\n\n",m_IspVersion>>4,m_IspVersion&0xF, FW_VER_NUM>>4,FW_VER_NUM&0xF);
		goto fail;
    }

//CMD_GET_DEVICEID
    LOGDEBUG("Getting device ID...");
    Result = nvtcomm_send_command(CMD_GET_DEVICEID);
    if(Result == false)
        goto fail;
	
	memcpy(&m_sMyChipType.uChipID,ReadReportBuffer+8,4);

    LOGDEBUG("Device ID:%08X",m_sMyChipType.uChipID);

    if (m_sMyChipType.uChipID == 0)
        goto fail;
    
    GetChipInfo();
    
//CMD_READ_CONFIG
    LOGDEBUG("Getting Config...");
    Result = nvtcomm_send_command(CMD_READ_CONFIG);
    if(Result == false)
        goto fail;
	
	memcpy(&m_hexConfig0,ReadReportBuffer+8,4); //Config0
	memcpy(&m_hexConfig1,ReadReportBuffer+12,4); //Config0
	LOGDEBUG("Config0: %08X",m_hexConfig0);
	LOGDEBUG("Config1: %08X",m_hexConfig1);
 
//Check if locked
    if(m_hexConfig0 & (1<<1))
        bIsChipLocked = false;
    else
        bIsChipLocked = true;

    if(bIsChipLocked) {
        LOGDEBUG("Chip is locked");
    }

    MyDevFound=true;
    return true;


fail:
    
    LOGDEBUG("Connection failed");
    
    MyDevFound=false;
    return false;
}


/*
int nvtcomm_set_board(const char* name)
{
    LOGDEBUG("setting board to %s", name);
    nvtcomm_board = nvtcomm_board_by_name(name);
    if (!nvtcomm_board)
    {
        LOGERR("unknown board: %s", name);
        INFO("known boards are: ");
        for (nvtcomm_board_t* b = nvtcomm_board_first(); b; b = nvtcomm_board_next(b))
        {
            INFO("%s ", nvtcomm_board_name(b));
        }
        INFO("\n");
    }
    return 1;
}

int nvtcomm_set_chip(const char* name)
{
    LOGDEBUG("setting chip to %s", name);
    if (strcmp(name, "esp8266") == 0 || strcmp(name, "8266") == 0) {
        s_chip = nvtcomm_8266;
    }
    else if (strcmp(name, "esp32") == 0 || strcmp(name, "32") == 0) {
        s_chip = nvtcomm_32;
    }
    else {
        LOGERR("unknown chip: %s", name);
        return 0;
    }
    return 1;
}
*/
