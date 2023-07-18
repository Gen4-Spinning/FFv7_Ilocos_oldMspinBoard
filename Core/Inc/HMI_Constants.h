
#define HMI_START_END_CHAR  126 // 0x7E

#define MAX_PACKET_SIZE_FROM_HMI 95

//msg type
#define HMI_SCREEN_DATA 0x01
#define HMI_BG_DATA 0x02
//Screens
#define HMI_IDLE_SCREEN 0x01
#define HMI_RUN_SCREEN 0x02
#define HMI_STOP_SCREEN 0x03
// Information
#define HMI_SETTINGS_INFO 0x04
#define HMI_PID_INFO 0x06
#define HMI_SETTINGS_PACKET_LEN_FF 28 //without the /r in the end
#define HMI_DIAG_RESULTS 0x05 
//DIAG_TEST_Results
#define HMI_DIAG_TEST_SUCCESSFUL 0x98
#define HMI_DIAG_TEST_FAIL 0x97
#define HMI_DIAG_TEST_MOTORERROR 0x96


//SubScreens
#define HMI_RUN_NORMAL 0x01
#define HMI_RUN_RAMPUP 0x02
#define HMI_RUN_PIECING 0x03
#define HMI_RUN_HOMING 0x04
#define HMI_RUN_PACKET_LENGTH_FF 23
#define HMI_STOP_PAUSE 0x11 
#define HMI_STOP_HALT 0x12
#define HMI_STOP_PACKET_LENGTH 19
#define HMI_END_DIAG_TEST 0x98

//Senders
#define MACHINE_TO_HMI 0x01
#define HMI_TO_MACHINE 0x02


//ATTRIBUTE CODES
//1. FOR RUN SCREENS
#define HMI_FF_LAYERS_RUN  0x01 //INT
#define HMI_FF_RTF 0x02
#define HMI_FF_PRODUCTION_RUN 0x03  

//2. for STOP Screens
#define HMI_STOP_REASON_CODE  0x01
#define HMI_MOTOR_FAULT_CODE 0x02
#define HMI_ERROR_VAL_CODE 0x03
#define HMI_USER_PAUSE 0x99

//3.BLOW CARD MOTORS and error reasons.
//Same as RPI.

//4. DIAG CODES // NOT USED
#define HMI_DIAG_TEST_CODE 0x01
#define HMI_DIAG_MOTOR_ID_CODE 0x02
#define HMI_DIAG_SIGNAL_VOLTAGE_CODE 0x03
#define HMI_DIAG_TARGET_RPM_CODE 0x04
#define HMI_DIAG_TEST_TIME_CODE 0x05
//attibutes
#define HMI_DIAG_CLOSED_LOOP  0x01
#define HMI_DIAG_OPEN_LOOP  0x02
#define HMI_DIAG_END_OF_TEST_CODE 0x0A //RETURN

// for sending PID vals on request from Machine
#define HMI_PID_ATTRIBUTES 0x01 // for the TLV type
#define PID_ATTRIBUTE_LEN 20

//SETTINGS
#define HMI_FF_ALL_SETTING_CODE 0x60
#define HMI_FF_ALL_SETTINGS_PKT_LEN 24

// RETURN MSGS FROM HMI TO MACHINE
#define FROM_HMI_IM_PAIRED 0x99
#define IM_PAIRED_PACKET_LENGTH 26  // including the /r
#define FROM_HMI_DISABLE_MACHINE_START 0x03
#define FROM_HMI_ENABLE_MACHINE_START 0x02
#define FROM_HMI_RTF_UPDATE 0x07
#define FROM_HMI_IN_SETTINGS_CHANGE 0x01
#define FROM_HMI_IN_DIAGNOSTICS_SET 0x02
#define FROM_HMI_BACK_TO_IDLE 0x00
#define FROM_DIAG_STOP_TEST 0x03
#define RTF_ADD 0x02
#define RTF_SUBTRACT 0x03

#define FROM_HMI_CHANGE_PROCESS_PARAMS 0x04
#define FROM_HMI_CHANGE_HW_PARAMS 0x03 //unused
#define FROM_HMI_PID_PROCESS 0x06 // same as PID INFO
#define FROM_HMI_DIAGNOSTIC_TEST 0x05
#define FROM_HMI_ENABLE_LOGGING 0x0A
#define FROM_HMI_DISABLE_LOGGING 0x0B
//msgs defined in the first TLV type
#define FROM_HMI_PID_REQUEST 0x01
#define FROM_HMI_PID_NEWSETTINGS 0x02

#define BAD_HMI_MSG 0x77
//INTERNALLY DEFINED MSGS
#define FROM_HMI_UPDATED_SETTINGS 0x06
#define FROM_MC_ACK_UPDATED_SETTINGS 0x99
#define FROM_MC_ACK_SAVED_SETTINGS 0x88
#define FROM_MC_ACK_PKT_LEN 11
#define FROM_DIAG_UPDATED_TEST_DETAILS 0x10
#define RUN_DIAG_TEST 0x12
#define SEND_PID_VALS 0x13
#define SAVE_PID_VALS 0x14
#define ADD_TO_RTF 0x09
#define SUBTRACT_FROM_RTF 0x0A
#define HOMING_RTF_CHANGE 0x0B

//MOTOR CODES FOR HMI ONLY
#define HMI_FF_FLYER 0x32
#define HMI_FF_BOBBIN 0x33
#define HMI_FF_FRONTROLLER 0x34
#define HMI_FF_BACKROLLER 0x35
#define HMI_FF_LIFTRIGHT 0x36
#define HMI_FF_LIFTLEFT 0x37
#define HMI_LIFT 0x38
#define HMI_DRAFTING 0x39
#define HMI_WINDING 0x3A

// Lift codes from HMI
#define HMI_LIFT_BOTH 0x01
#define HMI_LIFT_LEFT_ONLY 0x02
#define HMI_LIFT_RIGHT_ONLY 0x03

#define HMI_LIFT_UP 0x01
#define HMI_LIFT_DOWN 0x02

#define HMI_PID_START_VARS 0x21
#define HMI_PID_STOP_VARS 0x22


