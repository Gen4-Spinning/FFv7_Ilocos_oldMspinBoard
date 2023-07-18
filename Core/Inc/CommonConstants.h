
//Constants common to Logic, and the two Communication channels

//GENERAL
#define OFF 0
#define ON 1
#define UP 1
#define DOWN 2

//current machine SETTINGs
#define CURRENT_MACHINE FLYER
#define CURRENT_MACHINE_ID 1

//MACHINES

#define CARDING 1
#define DRAWFRAME 2
#define FLYER 3


//STATES
#define OFF_STATE 0
#define IDLE_STATE 1
#define DIAGNOSTIC_STATE 2
#define RUN_STATE 3
#define PAUSE_STATE 4
#define HALT_STATE 5
#define HOMING_STATE 6
#define UPDATESETTINGS 7 // NOT SURE IF WE NEED THIS
#define END_STATE 8
//change states
#define TO_RUN 0
#define TO_DIAG 1
#define TO_SETTINGS 2
#define TO_PAUSE 3
#define TO_HALT 4
#define TO_UPDATE_SETTINGS 5
#define TO_IDLE 6
#define TO_HOMING 7
#define TO_ENDSTATE 8
#define TO_UPDATE_PIDSETTINGS 9
#define INTERNAL_TO_IDLE 0x0A // this is a code to use inside the embedded coded wherever you want a falg taht says go to idle, like in 
#define TO_DIAG_SWITCH_OFF 0x0B 

//MOTORS
#define FF_FLYER 0x50
#define FF_BOBBIN 0x51
#define FF_FRONTROLLER 0x52
#define FF_BACKROLLER 0x53
#define FF_LIFTRIGHT 0x54
#define FF_LIFTLEFT 0x55
#define FF_LIFT 0x56

//EXTRA VARS FOR PID OPTIONS APART FROM MOTORS
#define PID_START_VARS 0x33
#define PID_STOP_VARS 0x34

//DIAG CODES. HMI AND RPI have different test type codes. TO FIX in next round
#define DIAG_ATTR_MOTORID 0x01
#define DIAG_ATTR_SIGNAL_IP_PERCENT 0x02
#define DIAG_ATTR_TARGET_RPM 0x03
#define DIAG_ATTR_TEST_TIME 0x04
#define DIAG_ATTR_TEST_RESULT 0x05

// ERROR CODES
#define ERR_RPM_ERROR 0x02
#define ERR_MOTOR_VOLTAGE_ERROR 0x03
#define ERR_DRIVER_VOLTAGE_ERROR 0x04
#define ERR_USER_PAUSE 0x08
#define ERR_SLIVER_CUT_ERROR 0x09
#define ERR_LAYERS_COMPLETE 0x0A
#define ERR_BOBBIN_BED_LIFT 0x0B
#define BOBBIN_BEDERROR_THRESH 6.5


//DATATYPES
#define FLOAT 4
#define INT 2
#define CHAR 1

//MISC
#define NOTHING 0x00
#define NO_VAR 0x00
#define NO_FLOAT_VAR (float)0.01
#define	FRONTROLLER_OD_MM 30

//RTF MULTIPLIER
#define INIT_RTF_MULTIPLIER 1.0

//DEFAULTS
#define DEFAULT_SPINDLESPEED 650
#define DEFAULT_DRAFT 8
#define DEFAULT_TPI 1.0
#define DEFAULT_RTF 1.0
#define DEFAULT_ROVINGLAYERS 1200
#define DEFAULT_MAXCONTENT_HEIGHT 250
#define DEFAULT_ROVINGWIDTH 2
#define DEFAULT_DELTA_BOBBIN_DIA 1.6
#define DEFAULT_BARE_BOBBIN_DIA 48

#define MOTOR_LIFT_MM_TO_PULSE 288 // 24 gear ratio 24*16  --> 4mm in 1 rotation, 1 rotation means 24 motor rotations, -> so 1 mm means  6 motor rotations -each rotatation 48 pulses
//4mm = 48 * 24 pulses, so for 1 mm-> 288 pulses

#define FR_CIRCUMFERENCE_M 0.0942 // 30mm dia
#define ILOCOS_FR_GEAR_RATIO 15.5
