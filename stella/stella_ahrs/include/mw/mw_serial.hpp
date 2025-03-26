bool MW_AHRS_Connect(char *portname_, uint32_t baudrate_);
bool MW_AHRS_DisConnect();
bool MW_AHRS_Read(unsigned char value[8]);
bool MW_AHRS_SetValI (long &value, unsigned short index, char sub_index=0);
bool MW_AHRS_GetValI (long &value, unsigned short index, char sub_index=0);
bool MW_AHRS_NvicReset ();

