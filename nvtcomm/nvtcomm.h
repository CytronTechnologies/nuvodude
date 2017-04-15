/**********************************************************************************
 **********************************************************************************
 ***
 ***    nvtcomm.h
 ***    - include file for nvtcomm.c
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

#ifndef NVTCOMM_H
#define NVTCOMM_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <sys/stat.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>

#include "infohelper.h"
#include "serialport.h"
#include "delay.h"

#define FW_VER_NUM         		0x27
#define MAX_PACKET 64
#define FILE_BUFFER				2048
#define MAX_BIN_FILE_SIZE 		(0x200000 - 0x4000)

/*-----------------------------------------------------------------------------
 *  ISP Commands
 *----------------------------------------------------------------------------*/
enum
{
    CMD_SET_CAN_ID = 0x000000BB,
	CMD_GET_FWVER = 0x000000A6,
	CMD_UPDATE_APROM = 0x000000A0,
	CMD_SYNC_PACKNO	= 0x000000A4,
	CMD_UPDATE_CONFIG = 0x000000A1,
	CMD_UPDATE_DATAFLASH = 0x000000C3,
//	CMD_UPDATE_DATA1 = 0x000000C2,
//	CMD_UPDATE_DATA2 = 0x000000C3,
    CMD_READ_CHECKSUM = 0x000000C8,
	CMD_ERASE_ALL = 0x000000A3,
	CMD_GET_APPINFO = 0x000000A8,
	CMD_READ_CONFIG = 0x000000A2,
	CMD_APROM_SIZE = 0x000000AA,
	CMD_GET_DEVICEID = 0x000000B1,
	CMD_WRITE_CHECKSUM = 0x000000C9,
	CMD_GET_FLASHMODE = 0x000000CA,
	CMD_RUN_APROM = 0x000000AB,
	CMD_RUN_LDROM = 0x000000AC,
	CMD_RESEND_PACKET = 0x000000FF,
	CMD_CONNECT = 0x000000AE,
//	CMD_DISCONNECT      	0x000000AF,
};

/*-----------------------------------------------------------------------------
 *  Error Code
 *----------------------------------------------------------------------------*/
#define ERR_CODE_LOST_PACKET    -1
#define ERR_CODE_CHECKSUM_ERROR -2
#define ERR_CODE_TIME_OUT       -3
#define ERR_CODE_COM_ERROR_OPEN	-4

/*-----------------------------------------------------------------------------
 *  Chip ID
 *----------------------------------------------------------------------------*/
#define NUEDU_UNO 0x10013110
#define CT_ARM 0x10013100

unsigned char CodeFileBuffer[MAX_BIN_FILE_SIZE];
unsigned short gcksum;
unsigned char WriteReportBuffer[256];
unsigned char ReadReportBuffer[256];
unsigned int m_curCmd;
bool bDetecting;
bool bDetectingSaved;
bool MyDevFound;
extern unsigned int g_packno;
unsigned char m_IspVersion;
unsigned long m_hexConfig0;
unsigned long m_hexConfig1;
bool bIsChipLocked;

typedef struct{
	unsigned char uCodeFileType; // 0:bin 1:hex
	unsigned long uCodeFileStartAddr; 
    unsigned long uCodeFileSize;
	unsigned long uCodeFileCheckSum;
	
}MY_FILE_INFO_TYPE;

typedef struct{
	unsigned long uChipID;
	unsigned long uRamSize;
	char cChipName[128];
	unsigned long uFlashSize;
	unsigned long uCodeFlashSize;
	//Any more...

}MY_CHIP_TYPE;

MY_FILE_INFO_TYPE m_sMyFileInfo;
MY_CHIP_TYPE m_sMyChipType;

int nvtcomm_set_port(char *port);
int nvtcomm_set_baudrate(const char *baudrate);
int nvtcomm_set_address(const char *address);
//int nvtcomm_set_board(const char* name);
//int nvtcomm_set_chip(const char* name);
int nvtcomm_set_filename(char *name);

int nvtcomm_open(void);
void nvtcomm_close(void);
bool nvtcomm_get_chip_info();

int nvtcomm_upload_file();
bool nvtcomm_load_file();
int nvtcomm_file_uploaded();
int nvtcomm_file_ready();
int nvtcomm_start_app(int reboot);

#endif
