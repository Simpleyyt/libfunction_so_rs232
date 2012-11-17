#define HANDLE int

//System Setting
HANDLE API_OpenComm(char *com,int Baudrate);

int API_CloseComm(const  HANDLE commHandle);

int API_SetDeviceAddress(HANDLE commHandle, int DeviceAddress, unsigned char newAddr, unsigned char *buffer);

int API_SetBandrate(HANDLE commHandle, int DeviceAddress, unsigned char newBaud, unsigned char *buffer);

int API_SetSerNum( HANDLE commHandle, int DeviceAddress, unsigned char *newValue, unsigned char *buffer);

int API_GetSerNum( HANDLE commHandle, int  DeviceAddress, unsigned char *buffer);

int WriteUserInfo(HANDLE commHandle, int DeviceAddress, int num_blk, int num_length, char *user_info);

int ReadUserInfo(HANDLE commHandle, int DeviceAddress, int num_blk, int num_length, char *user_info);

int GetVersionNum(HANDLE commHandle,int DeviceAddress, char *VersionNum);

int API_ControlLED(HANDLE commHandle, int DeviceAddress, unsigned char freq, unsigned char duration, unsigned char *buffer);

int API_ControlBuzzer(HANDLE commHandle, int DeviceAddress, unsigned char freq, unsigned char duration, unsigned char *buffer);

//ISO14443A
int MF_Request(HANDLE commHandle, int DeviceAddress, unsigned char inf_mode, unsigned char *buffer);

int MF_Anticoll(HANDLE commHandle, int DeviceAddress, unsigned char *snr, unsigned char *tatus);

int MF_Select(HANDLE commHandle, int DeviceAddress, unsigned char *snr);

int MF_Halt(HANDLE commHandle, int DeviceAddress);

int API_PCDRead(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char blk_add, unsigned char num_blk, unsigned char *snr, unsigned char *buffer);

int API_PCDWrite(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char blk_add, unsigned char num_blk, unsigned char *snr, unsigned char *buffer);

int API_PCDInitVal(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char SectNum, unsigned char *snr, unsigned char *value);

int API_PCDDec(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char SectNum, unsigned char *snr,  unsigned char *value);

int API_PCDInc(HANDLE commHandle, int DeviceAddress, unsigned char   mode, unsigned char SectNum, unsigned char *snr, unsigned char *value);

int GET_SNR(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char API_halt, unsigned char *snr, unsigned char*value);

int MF_Restore(HANDLE commHandle,int DeviceAddress, unsigned char mode, int cardlength, unsigned char*carddata );

//ISO14443B
int RequestType_B(HANDLE commHandle, int DeviceAddress, unsigned char *buffer);

int AntiType_B(HANDLE commHandle, int DeviceAddress, unsigned char *buffer);

int SelectType_B(HANDLE commHandle, int DeviceAddress, unsigned char*SerialNum);

int Request_AB(HANDLE commHandle, int DeviceAddress, unsigned char* buffer);

int API_ISO14443TypeBTransCOSCmd(HANDLE commHandle, int DeviceAddress, unsigned char *cmd,  int cmdSize, unsigned char *buffer);

//ISO15693
int API_ISO15693_Inventory(HANDLE commHandle, int DeviceAddress, unsigned char flag, unsigned char afi, unsigned char *pData, unsigned char *nrOfCard, unsigned char *pBuffer);

int API_ISO15693Read(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char blk_add, unsigned char num_blk, unsigned char *uid, unsigned char *buffer);

int API_ISO15693Write(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char blk_add, unsigned char num_blk, unsigned char *uid, unsigned char *data);

int API_ISO15693Lock(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char num_blk, unsigned char *uid, unsigned char *buffer);

int API_ISO15693StayQuiet(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer);

int API_ISO15693Select(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer);

int API_ResetToReady(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer);

int API_WriteAFI(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char afi, unsigned char *uid, unsigned char *buffer);

int API_LockAFI(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer);

int API_WriteDSFID(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char DSFID, unsigned char *uid, unsigned char *buffer);

int API_LockDSFID(HANDLE commHandle, int  DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer );

int API_ISO15693_GetSysInfo(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char *uid, unsigned char *buffer); 

int API_ISO15693_GetMulSecurity(HANDLE commHandle, int DeviceAddress, unsigned char flags, unsigned char blkAddr, unsigned char blkNum, unsigned char  *uid, unsigned char *pBuffer);

int API_ISO15693TransCOSCmd(HANDLE commHandle, int DeviceAddress, unsigned char *cmd, int cmdSize, unsigned char *buffer);

//ultralight
int UL_HLRead(HANDLE commHandle, int DeviceAddress,unsigned char mode, unsigned char blk_add, unsigned char *snr, unsigned char *buffer);

int	UL_HLWrite(HANDLE commHandle, int DeviceAddress, unsigned char mode, unsigned char blk_add, unsigned char *snr, unsigned char *buffer);

int UL_Request(HANDLE commHandle, int DeviceAddress, unsigned char mode,unsigned char *snr);
