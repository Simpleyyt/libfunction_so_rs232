#include     <stdio.h>
#include     <stdlib.h> 
#include     <unistd.h>  
#include     <sys/types.h>
#include     <sys/stat.h>
#include     <fcntl.h> 
#include     <termios.h>
#include     <errno.h>
#include	 "function.h"
#include	 "define.h"
#define FALSE -1
#define TRUE 0

unsigned char Buffer[263] = {STX};
int p = 1;

void copyData(unsigned char *s, int spos, unsigned char *d, int dpos, int len)
{
	int i;
	for (i=0;i<len;i++)
	{
		d[i+dpos] = s[i+spos];
	}
}
/**
*@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return  void
*/
int speed_arr[] = { B38400, B19200, B9600, B4800, B2400, B1200, B300,
                                        B38400, B19200, B9600, B4800, B2400, B1200, B300, };
int name_arr[] = {38400,  19200,  9600,  4800,  2400,  1200,  300, 38400, 
                                        19200,  9600, 4800, 2400, 1200,  300, };
int set_BaudRate(int fd, int speed)
{
	int i;
	int status;
    struct termios Opt;
    tcgetattr(fd, &Opt);
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) 
    {
		if  (speed == name_arr[i]) 
        {    
			tcflush(fd, TCIOFLUSH);    
            cfsetispeed(&Opt, speed_arr[i]); 
            cfsetospeed(&Opt, speed_arr[i]);  
            status = tcsetattr(fd, TCSANOW, &Opt); 
            if  (status != 0) 
            {       
				perror("tcsetattr"); 
                return FALSE;    
            }   
            tcflush(fd,TCIOFLUSH);
            return TRUE;  
        } 
    }
    return FALSE;
}
int checkData(unsigned char *data, int h, int t)
{
	int check;
	int i;
	check  = data[h];
	for (i=h+1;i<=t;i++)
	{
		check = check^data[i];
	}
	return check;
}

void writeBuffer(unsigned char data)
{
	Buffer[p] = data;
	p++;
}

void writeBuffers(unsigned char *data, int length)
{
	int i;
	for (i=0;i<length;i++)
		writeBuffer(data[i]);
}

void clearBuffer(int fd)
{
	p = 1;
	Buffer[0] = STX;
	tcflush(fd,TCIOFLUSH);
}

int sendData(int fd)
{
	int length;
	int BCC;
	int i;
	BCC = checkData(Buffer,1, p-1);
	writeBuffer(BCC);
	writeBuffer(ETX);
	
	printf ("Send data:");
	for (i=0;i<p;i++)
		printf("%02x ",Buffer[i]);
	printf("\n");
	
	length = write(fd, Buffer, p);
	usleep(500000);
	printf("length send: %02x\n", length);
	
	if(length != p)
		return 0x05;
		
	length=read(fd, Buffer,263);
	printf("length read: %02x\n", length);
	/*
	if(length != recieve_len)
		return 0x05;*/
		
	p = length;
	
	printf("Recieve data:");
	for (i=0;i<length;i++)
		printf("%02x ",Buffer[i]);
	printf("\n");

    if(p<6 || Buffer[0]!=STX || Buffer[p-1]!=ETX || Buffer[2]+5!=p)
		return 0x05;
	if (checkData(Buffer,1,p-3) != Buffer[p-2])
		return 0x02;
	return 0;
}


/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*/
int set_Parity(int fd,int databits,int stopbits,int parity)
{
	struct termios options;
    if ( tcgetattr( fd,&options)  !=  0) 
    {
            perror("SetupSerial 1");    
            return(FALSE); 
    }
    options.c_cflag &= ~CSIZE;
    switch (databits) /*设置数据位数*/
    {  
	case 7:        
		options.c_cflag |= CS7;
        break;
    case 8:    
        options.c_cflag |= CS8;
        break;  
    default:   
        fprintf(stderr,"Unsupported data size\n"); return (FALSE); 
    }
	switch (parity)
	{  
    case 'n':
    case 'N':   
		options.c_cflag &= ~PARENB;   /* Clear parity enable */
        options.c_iflag &= ~INPCK;     /* Enable parity checking */
        break; 
    case 'o':  
    case 'O':    
        options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/ 
        options.c_iflag |= INPCK;             /* Disnable parity checking */
        break;
    case 'e': 
    case 'E':  
        options.c_cflag |= PARENB;     /* Enable parity */   
        options.c_cflag &= ~PARODD;   /* 转换为偶效验*/    
        options.c_iflag |= INPCK;       /* Disnable parity checking */
        break;
    case 'S':
    case 's':  /*as no parity*/  
		options.c_cflag &= ~PARENB;
		options.c_cflag &= ~CSTOPB;break; 
    default:  
		fprintf(stderr,"Unsupported parity\n");   
		return (FALSE); 
    } 
	/* 设置停止位*/ 
	switch (stopbits)
	{  
    case 1:   
		options.c_cflag &= ~CSTOPB; 
		break; 
    case 2:   
		options.c_cflag |= CSTOPB; 
		break;
    default:   
		fprintf(stderr,"Unsupported stop bits\n"); 
		return (FALSE);
	}

	/* Set input parity option */
	if (parity != 'n')  
        options.c_iflag |= INPCK;
	tcflush(fd,TCIFLUSH);
	
	options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
	options.c_oflag  &= ~OPOST;   /*Output*/

    options.c_iflag &= ~(ICRNL | INLCR | IXON); /* not to set software control */
	
	options.c_cc[VTIME] = 150; /* 设置超时 seconds*/  
	options.c_cc[VMIN] = 0; /* Update the options and do it NOW */
	if (tcsetattr(fd,TCSANOW,&options) != 0)  
	{
        perror("SetupSerial 3");  
        return (FALSE); 
	}
	return (TRUE); 
}

int sendCommand(int fd, int id, int command,  unsigned char *sDATA, int sDLen,unsigned char *rDATA, int*Statue)
{
	int result;
	clearBuffer(fd);
	if (fd==0)
		return 0x03;
	writeBuffer(id);
	writeBuffer(sDLen+1);
	writeBuffer(command);
	writeBuffers(sDATA, sDLen);

	result = sendData(fd);
	
	if(result != 0)
		return result;
	copyData(Buffer,4,rDATA,0,Buffer[2]-1);
	*Statue = Buffer[3];
	return 0;
}

HANDLE API_OpenComm(char *com,int Baudrate)
{
	int fd = open( com, O_RDWR | O_NOCTTY| O_NDELAY);
	if (-1 == fd)
    {                      
		perror("Can't Open Serial Port");
		return 0;             
    }
	if (set_BaudRate(fd,Baudrate) == FALSE)
	{
		perror("Set Baud Error\n");
	}
    if (set_Parity(fd,8,1,'n') == FALSE)  
    {
        perror("Set Parity Error\n");
        return 0;
    }
	return fd;
}

int API_CloseComm(const  HANDLE commHandle)
{
	if (commHandle==0)
		return -1;
	return close(commHandle);
}

int  API_SetDeviceAddress(HANDLE commHandle, int DeviceAddress, unsigned char newAddr, unsigned char *buffer)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,SetAddress,&newAddr,1,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue;
}

int  API_SetBandrate(HANDLE commHandle, int DeviceAddress, unsigned char newBaud, unsigned char *buffer)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,SetBaudrate,&newBaud,1,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue;
}

int  API_SetSerNum( HANDLE commHandle, int DeviceAddress, unsigned char *newValue, unsigned char *buffer)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,SetSerlNum,newValue,8,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue;
}

int  API_GetSerNum( HANDLE commHandle, int  DeviceAddress, unsigned char *buffer)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,GetSerlNum,NULL,0,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue;
}

int  WriteUserInfo(HANDLE commHandle, int DeviceAddress, int num_blk, int num_length, char *user_info)
{
	int Statue;
	unsigned char DATA[121];
	DATA[0] = num_blk;
	DATA[1] = num_length;
	copyData((unsigned char*)user_info, 0, DATA, 2, num_length);
	int result = sendCommand(commHandle,DeviceAddress,Write_UserInfo,DATA,num_length+2,(unsigned char*)user_info,&Statue);
	if (result != 0)
		return result;
	return Statue;
}

int ReadUserInfo(HANDLE commHandle, int DeviceAddress, int num_blk, int num_length, char *user_info)
{
	int Statue;
	unsigned char DATA[2];
	DATA[0] = num_blk;
	DATA[1] = num_length;
	int result = sendCommand(commHandle,DeviceAddress,Read_UserInfo,DATA,2,(unsigned char*)user_info,&Statue);
	if (result != 0)
		return result;
	return Statue;	
}

int GetVersionNum(HANDLE commHandle,int DeviceAddress, char *VersionNum)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,Get_VersionNum,NULL,0,(unsigned char*)VersionNum,&Statue);
	if (result != 0)
		return result;
	return Statue;
}

int  API_ControlLED(HANDLE commHandle, int DeviceAddress, unsigned char freq, unsigned char duration, unsigned char *buffer)
{
	int Statue;
	unsigned char DATA[2];
	DATA[0] = freq;
	DATA[1] = duration;
	int result = sendCommand(commHandle,DeviceAddress,Control_Led2,DATA,2,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue;
}

int API_ControlBuzzer(HANDLE commHandle, int DeviceAddress, unsigned char freq, unsigned char duration, unsigned char *buffer)
{
	int Statue;
	unsigned char DATA[2];
	DATA[0] = freq;
	DATA[1] = duration;
	int result = sendCommand(commHandle,DeviceAddress,Control_Buzzer,DATA,2,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue;
}

int  MF_Request(HANDLE commHandle,int DeviceAddress, unsigned char inf_mode, unsigned char *buffer)
{
	int Statue;
	/*
	unsigned char DATA[1];
	if (inf_mode == 0x00)
		DATA[0] = 0x52;
	else 
		DATA[0] = 0x26;
		*/
	int result = sendCommand(commHandle,DeviceAddress,REQA,&inf_mode,1,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue;
}

int MF_Anticoll(HANDLE commHandle,int DeviceAddress, unsigned char *snr,unsigned char *status)
{
	int Statue;
	unsigned char DATA[5];
	int result = sendCommand(commHandle,DeviceAddress,Anticoll_A,NULL,0,DATA,&Statue);
	if (result != 0)
		return result;
	*status = DATA[0];
	copyData(DATA, 1, snr, 0, 4);
	return Statue;
}


int MF_Select(HANDLE commHandle, int DeviceAddress, unsigned char *snr)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,Select_A,snr,4,snr,&Statue);
	if (result != 0)
		return result;
	return Statue;
}


int MF_Halt(HANDLE commHandle, int DeviceAddress)
{
	int Statue;
	unsigned char error;
	int result = sendCommand(commHandle,DeviceAddress,Halt_A,NULL,0,&error,&Statue);
	if (result != 0)
		return result;
	return Statue;
}


int API_PCDRead(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char blk_add, unsigned char num_blk, unsigned char *snr, unsigned char *buffer)
{
	int Statue;
	const int num = (num_blk * 16 + 4) > 9? (num_blk*16+4):9;
	unsigned char DATA[num];
	DATA[0] = mode;
	DATA[1] = num_blk;
	DATA[2] = blk_add;
	copyData(snr,0,DATA,3,6);
	int result = sendCommand(commHandle,DeviceAddress,MF_Read,DATA,9,DATA,&Statue);
	if (result != 0)
		return result;
	copyData(DATA,0,snr,0,4);
	copyData(DATA,4,buffer,0,num_blk*16);
	return Statue;
}

int API_PCDWrite(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char blk_add, unsigned char num_blk, unsigned char *snr, unsigned char *buffer)
{
	int Statue;
	const int num = num_blk * 16 + 9;
	unsigned char DATA[num];
	DATA[0] = mode;
	DATA[1] = num_blk;
	DATA[2] = blk_add;
	copyData(snr,0,DATA,3,6);
	copyData(buffer,0,DATA,9,num);
	int result = sendCommand(commHandle,DeviceAddress,MF_Write,DATA,num,DATA,&Statue);
	if (result != 0)
		return result;
	copyData(DATA,0,snr,0,4);
	return Statue;
}

int API_PCDInitVal(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char SectNum, unsigned char *snr, unsigned char *value)
{
	int Statue;
	unsigned char DATA[12];
	DATA[0] = mode;
	DATA[1] = SectNum;
	copyData(snr,0,DATA,2,6);
	copyData(value,0,DATA,8,4);
	int result = sendCommand(commHandle,DeviceAddress,MF_InitVal,DATA,12,DATA,&Statue);
	if (result != 0)
		return result;
	copyData(DATA,0,snr,0,4);
	return Statue;          
}



int  API_PCDDec(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char SectNum, unsigned char *snr,  unsigned char *value)
{
	int Statue;
	unsigned char DATA[12];
	DATA[0] = mode;
	DATA[1] = SectNum;
	copyData(snr,0,DATA,2,6);
	copyData(value,0,DATA,8,4);
	int result = sendCommand(commHandle,DeviceAddress,MF_Decrement,DATA,12,DATA,&Statue);
	if (result != 0)
		return result;
	copyData(DATA,0,snr,0,4);
	copyData(DATA,4,value,0,4);
	return Statue;   
}

int  API_PCDInc(HANDLE commHandle, int DeviceAddress, unsigned char   mode, unsigned char SectNum, unsigned char *snr, unsigned char *value)
{
	int Statue;
	unsigned char DATA[12];
	DATA[0] = mode;
	DATA[1] = SectNum;
	copyData(snr,0,DATA,2,6);
	copyData(value,0,DATA,8,4);
	int result = sendCommand(commHandle,DeviceAddress,MF_Increment,DATA,12,DATA,&Statue);
	if (result != 0)
		return result;
	copyData(DATA,0,snr,0,4);
	copyData(DATA,4,value,0,4);
	return Statue;   
}

int GET_SNR(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char API_halt, unsigned char *snr, unsigned char*value)
{
	int Statue;
	unsigned char DATA[5];
	DATA[0] = mode;
	DATA[1] = API_halt;
	int result = sendCommand(commHandle,DeviceAddress,MF_GET_SNR,DATA,2,DATA,&Statue);
	if (result != 0)
		return result;
	snr[0] = DATA[0];
	copyData(DATA,0,value,1,4);
	return Statue;   
}


int MF_Restore(HANDLE commHandle,int DeviceAddress, unsigned char mode, int cardlength, unsigned char*carddata )
{
	int Statue;
	const int num = cardlength+2;
	unsigned char DATA[num];
	DATA[0] = mode;
	DATA[1] = cardlength;
	copyData(carddata, 0, DATA, 2, cardlength);
	int result = sendCommand(commHandle,DeviceAddress,ISO14443_TypeA_Transfer_Command,DATA,num,carddata,&Statue);
	if (result != 0)
		return result;
	return Statue;   
}


int RequestType_B(HANDLE commHandle, int DeviceAddress, unsigned char *buffer)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,ReqB,NULL,0,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue;   
}


int AntiType_B(HANDLE commHandle, int DeviceAddress, unsigned char *buffer)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,AnticollB,NULL,0,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int SelectType_B(HANDLE commHandle, int DeviceAddress, unsigned char*SerialNum)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,Attrib_TypeB,SerialNum,4,SerialNum,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int Request_AB(HANDLE commHandle, int DeviceAddress, unsigned char* buffer)
{
	int Statue;
	int result = sendCommand(commHandle,DeviceAddress,Rst_TypeB,buffer,4,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int  API_ISO14443TypeBTransCOSCmd(HANDLE commHandle, int DeviceAddress, unsigned char *cmd,  int cmdSize, unsigned char *buffer)
{
	int Statue;
	unsigned char DATA[256];
	DATA[0] = cmdSize;
	copyData(buffer, 0, DATA, 1 ,cmdSize);
	int result = sendCommand(commHandle,DeviceAddress,ISO14443_TypeB_Transfer_Command,buffer,cmdSize+1,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int  API_ISO15693_Inventory(HANDLE commHandle, int DeviceAddress, unsigned char flag, unsigned char afi, unsigned char *pData, unsigned char *nrOfCard, unsigned char *pBuffer)
{
	int Statue;
	unsigned char DATA[256];
	DATA[0] = flag;
	DATA[1] = afi;
	DATA[2] = 0;
	copyData(pData, 0, DATA, 3 ,8);
	int result = sendCommand(commHandle,DeviceAddress,ISO14443_TypeB_Transfer_Command,DATA,11,DATA,&Statue);
	if (result != 0)
		return result;
	*nrOfCard = DATA[0];
	copyData(DATA,1,pBuffer,0,8*DATA[0]);
	return Statue; 
}


int  API_ISO15693Read(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char blk_add, unsigned char num_blk, unsigned char *uid, unsigned char *buffer)
{
	int Statue;
	int num;
	unsigned char DATA[11];
	DATA[0] = flags;
	DATA[1] = blk_add;
	DATA[2] = num_blk;
	num = 3;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 3 ,8);
		num = 11;
	}
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Read,DATA,num,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int  API_ISO15693Write(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char blk_add, unsigned char num_blk, unsigned char *uid, unsigned char *data)
{
	int Statue;
	int num, k;
	unsigned char DATA[256];
	k = 4;
	DATA[0] = flags;
	DATA[1] = blk_add;
	DATA[2] = num_blk;
	num = num_blk*k + 3;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 3 ,8);
		num = num_blk*k + 11;
	}
	copyData(data,0,DATA,3,num_blk*4);
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Write,DATA,num,data,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_ISO15693Lock(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char num_blk, unsigned char *uid, unsigned char *buffer)
{
	int Statue;
	int num;
	unsigned char DATA[10];
	DATA[0] = flags;
	DATA[1] = num_blk;
	num = 2;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 2 ,8);
		num = 10;
	}
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Lockblock,DATA,num,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_ISO15693StayQuiet(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer)
{
	int Statue;
	unsigned char DATA[9];
	DATA[0] = flags;
	copyData(uid, 0, DATA, 1 ,8);
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_StayQuiet,DATA,9,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_ISO15693Select(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer)
{
	int Statue;
	unsigned char DATA[9];
	DATA[0] = flags;
	copyData(uid, 0, DATA, 1 ,8);
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Select,DATA,9,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_ResetToReady(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer)
{
	int Statue;
	int num;
	unsigned char DATA[9];
	DATA[0] = flags;
	num = 1;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 1 ,8);
		num = 9;
	}
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Resetready,DATA,num,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int  API_WriteAFI(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char afi, unsigned char *uid, unsigned char *buffer)
{
	int Statue;
	int num;
	unsigned char DATA[10];
	DATA[0] = flags;
	DATA[1] = afi;
	num = 2;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 2 ,8);
		num = 10;
	}
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Write_Afi,DATA,num,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_LockAFI(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer)
{
	int Statue;
	int num;
	unsigned char DATA[9];
	DATA[0] = flags;
	num = 1;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 1 ,8);
		num = 9;
	}
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Lock_Afi,DATA,num,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_WriteDSFID(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char DSFID, unsigned char *uid, unsigned char *buffer)
{
	int Statue;
	int num;
	unsigned char DATA[10];
	DATA[0] = flags;
	DATA[1] = DSFID;
	num = 2;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 2 ,8);
		num = 10;
	}
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Write_Dsfid,DATA,num,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_LockDSFID(HANDLE commHandle, int  DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer )
{
	int Statue;
	int num;
	unsigned char DATA[9];
	DATA[0] = flags;
	num = 1;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 1 ,8);
		num = 9;
	}
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Lock_Dsfid,DATA,num,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_ISO15693_GetSysInfo(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer)
{
	int Statue;
	int num;
	unsigned char DATA[9];
	DATA[0] = flags;
	num = 1;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 1 ,8);
		num = 9;
	}
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Get_Information,DATA,num,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_ISO15693_GetMulSecurity(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char blkAddr, unsigned char blkNum, unsigned char  *uid, unsigned char *pBuffer)
{
	int Statue;
	int num;
	unsigned char DATA[11];
	DATA[0] = flags;
	DATA[1] = blkAddr;
	DATA[2] = blkNum;
	num = 3;
	if (flags == 0x22)
	{
		copyData(uid, 0, DATA, 3 ,8);
		num = 11;
	}
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Get_Multiple_Block_Security,DATA,num,pBuffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int API_ISO15693TransCOSCmd(HANDLE commHandle, int DeviceAddress, unsigned char *cmd, int cmdSize, unsigned char *buffer)
{
	int Statue;
	unsigned char DATA[256];
	DATA[0] = cmdSize;
	copyData(buffer,0,DATA,1,cmdSize);
	int result = sendCommand(commHandle,DeviceAddress,ISO15693_Transfer_Command,DATA,cmdSize,buffer,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}

int	UL_HLRead(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char blk_add, unsigned char *snr, unsigned char *buffer)
{
	int Statue;
	unsigned char DATA[23];
	DATA[0] = mode;
	DATA[1] = blk_add;
	int result = sendCommand(commHandle,DeviceAddress,CMD_UL_HLRead,DATA,2,DATA,&Statue);
	if (result != 0)
		return result;
	copyData(DATA,0,buffer,0,16);
	copyData(DATA,16,snr,0,7);
	return Statue; 
}


int UL_HLWrite(HANDLE commHandle, int DeviceAddress,unsigned char mode, unsigned char blk_add, unsigned char *snr, unsigned char *buffer)
{
	int Statue;
	unsigned char DATA[7];
	DATA[0] = mode;
	DATA[1] = 1;
	DATA[2] = blk_add;
	copyData(buffer, 0, DATA, 3, 4);
	int result = sendCommand(commHandle,DeviceAddress,CMD_UL_HLWrite,DATA,7,snr,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}


int UL_Request(HANDLE commHandle, int DeviceAddress, unsigned char mode,unsigned char *snr)
{
	int Statue;
	unsigned char DATA[1];
	DATA[0] = mode;
	int result = sendCommand(commHandle,DeviceAddress,CMD_UL_Request,DATA,1,snr,&Statue);
	if (result != 0)
		return result;
	return Statue; 
}
