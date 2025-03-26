enum ePortIndex {
    CI_CAN             = 8,
    CI_FDCAN          = 4,
    CI_RS232           = 2,
    CI_USB              = 1,
};

enum eDataMode {
    CI_Binary           = 0,
    CI_Text             = 1,
};

enum eVariableIndex {
	CI_VENDOR_ID		= 1,	
	CI_PRODUCT_ID		= 2,	
	CI_SW_VERSION		= 3,	
	CI_HW_VERSION		= 4,	
	CI_FN_VERSION		= 5,   
	CI_SYS_COMMAND		= 7,	

	CI_DEVICE_ID		= 11,	

	CI_SYNC_DATA		= 21,	//  Set/Get synchronous data transmission to RS-232 (0 ~ 15)
	CI_SYNC_PORT		= 22,	//  Set/Get synchronous data transmission to CAN (0 ~ 15)
	CI_SYNC_PERIOD 		= 24,	//  Set/Get synchronous data period to CAN/RS-232 (1 ~ 60000) [ms]
	CI_SYNC_TRMODE		= 25,	//  Set/Get synchronous data transfer mode for RS-232 (0 ~ 2)
};
