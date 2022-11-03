#include <stdio.h>
#include <iostream>

// General ID definition
#define ID_ALL 0xfe
#define ID_WRITE_CHK 0xaa
#define ID_DEFALUT_CHK 0x55 // Default setting(write)
#define ID_DEVELOPER_CHK 0x77
///////////////////////////////////////////////////////
// Command : RMID, TMID, ID, PID, Data number, Data.., CHK
/////////////////////////////////////////////
// PID one-byte data : PID 0~127
#define PID_DEFAULT_SET 3
#define PID_REQ_PID_DATA 4 // request data(ex 196 PID value read: 04 C4 00 00 00 00 00 00 00) C4:196, C9:201
#define PID_TQ_OFF 5 // all motors torque off
#define PID_BRAKE 6
#define PID_ACK 7
/////////////////////////////////////////
#define PID_COMMAND 10
#define CMD_TQ_OFF 2
#define CMD_BRAKE 4
#define CMD_MAIN_DATA_BC_ON 5
#define CMD_MAIN_DATA_BC_OFF 6
#define CMD_ALARM_RESET 8
#define CMD_POSI_RESET 10
#define CMD_MONITOR_BC_ON 11
#define CMD_MONITOR_BC_OFF 12
#define CMD_MONITOR2_BC_ON 13
#define CMD_MONITOR2_BC_OFF 14
#define CMD_FAN_ON 15
#define CMD_FAN_OFF 16
#define CMD_CLUTCH_ON 17
#define CMD_CLUTCH_OFF 18
#define CMD_TAR_VEL_OFF 20
#define CMD_SLOW_START_OFF 21
#define CMD_SLOW_DOWN_OFF 22
#define CMD_CAN_RESEND_ON 23
#define CMD_CAN_RESEND_OFF 24
#define CMD_MAX_LOAD_OFF 25
#define CMD_ENC_PPR_OFF 26
#define CMD_LOW_SPEED_LIMIT_OFF 27
#define CMD_HIGH_SPEED_LIMIT_OFF 28
#define CMD_SPEED_LIMIT_OFF 29
#define CMD_CURVE_FITTING_OFF 31
#define CMD_STEP_INPUT_OFF 32
#define CMD_UICOM_OFF 44
#define CMD_UICOM_ON 45
#define CMD_MAX_RPM_OFF 46
#define CMD_HALL_TYPE_OFF 47
#define CMD_LOW_POT_LIMIT_OFF 48
#define CMD_HIGH_POT_LIMIT_OFF 49
#define CMD_MAIN_DATA_BC_ON2 50
#define CMD_MAIN_DATA_BC_OFF2 51
#define CMD_MONITOR_BC_ON2 52
#define CMD_MONITOR_BC_OFF2 53
#define CMD_IO_MONITOR_BC_ON2 54
#define CMD_IO_MONITOR_BC_OFF2 55
#define CMD_SYS_START 56
#define CMD_SYS_STOP 57
////////////////////////////////////////////////////////////////
#define PID_ALARM_RESET 12
#define PID_POSI_RESET 13
#define PID_MAIN_BC_STATUS 14
#define PID_MONITOR_BC_STATUS 15
#define PID_INV_SIGN_CMD 16
#define PID_USE_LIMIT_SW 17
#define PID_INV_SIGN_CMD2 18
#define PID_INV_ALARM 19
#define PID_HALL_TYPE 21
#define PID_STOP_STATUS 24
#define PID_INPUT_TYPE 25
#define PID_PRESET_SAVE 30
#define PID_PRESET_RECALL 31
#define PID_PHASE_FILTER 32
#define PID_CTRL_STATUS 34
#define PID_START_INV_SIGN 36
#define PID_RUN_INV_SIGN 37
#define PID_REGENERATION 38
#define PID_CTRL_STATUS2 39
#define PID_LIMIT_STOP_COND 40
#define PID_TQ_LIMIT_SW 41
#define PID_POSI_INPUT_MODE 42
#define PID_SINE_CTRL 43
#define PID_TQ_CTRL 44
#define PID_BLUETOOTH 45
#define PID_START_STOP 100
// PID two-byte data : PID 128 ~ 190
#define PID_VEL_CMD 130
#define PID_VEL_CMD2 131
#define PID_ID 133
#define PID_OPEN_VEL_CMD 134
#define PID_BAUD_RATE 135 // 9600, 19200, 38400, 57600 , 115200
#define PID_ECAN_BITRATE 137 // 50K,100K,250K,500K,1M
#define PID_INT_RPM_DATA 138
#define PID_TQ_DATA 139
#define PID_TQ_CMD 140
#define PID_VOLT_IN 143
#define PID_DIFF_PO 145
#define PID_CCW_PHASE_OFFSET 146
#define PID_CW_PHASE_OFFSET 147
#define PID_RETURN_TYPE 149
#define RETURN_TYPE_MONITOR 1
#define RETURN_TYPE_ACK 2
#define RETURN_TYPE_MONITOR2 3
#define PID_OVER_MODULATION 152
#define PID_SLOW_START 153
#define PID_SLOW_DOWN 154
#define PID_TAR_VEL 155
#define PID_ENC_PPR 156
#define PID_LOW_SPEED_LIMIT 157
#define PID_HIGH_SPEED_LIMIT 158
#define PID_PWM_OUT 160 
#define PID_SPEED_RESOLUTION 161
#define PID_DEAD_ZONE 162
#define PID_READ_ADDR 163
#define PID_REQ_PID_DATA2 164 
#define PID_AUTO_PAN 165
#define PID_REF_RPM 166
#define PID_PV_GAIN 167
#define PID_P_GAIN 168
#define PID_I_GAIN 169
#define PID_TQ_P_GAIN 170
#define PID_IN_POSITION 171
#define PID_LOW_POT_LIMIT 172
#define PID_HIGH_POT_LIMIT 173
#define PID_PNT_TQ_OFF 174
#define PID_PNT_BRAKE 175
#define PID_TAR_POSI_VEL 176
#define PID_TQ_I_GAIN 177
#define PID_POSI_SS 178
#define PID_POSI_SD 179
#define PID_COM_TAR_SPEED 180
#define PID_FUNC_CMD_TYPE 183
#define PID_FUNC_CMD 184
#define PID_COM_WATCH_DELAY 185
// PID N-byte data : PID 190 ~ 253
#define PID_MAIN_DATA 193
#define PID_IO_MONITOR 194
#define PID_MONITOR 196         //Motor1(Left) Velocity, count value monitor
#define PID_POSI_DATA 197
#define PID_RPM_DATA 198
#define PID_INC_TAR_POSI 199
#define PID_MAIN_DATA2 200 
#define PID_MONITOR2 201        //Motor2(Right) Velocity, count value monitor
#define PID_IO_MONITOR2 202
#define PID_GAIN 203
#define PID_POSI_VEL_DATA 204
#define PID_TYPE 205
#define PID_PNT_POSI_VEL_CMD 206
#define PID_PNT_VEL_CMD 207     // Reference velocity input (Left & Right) 1~6 byte 
#define PID_PNT_TQ_CMD 209
#define PID_PNT_MAIN_DATA 210
#define PID_MAX_LOAD 211
#define PID_LIMIT_TQ 212
#define PID_PNT_INC_POSI_CMD 215
#define PID_POSI_SET 217
#define PID_POSI_SET2 218
#define PID_POSI_VEL_CMD 219
#define PID_INC_POSI_VEL_CMD 220 
#define PID_MAX_RPM 221
#define PID_SPEED_LIMIT 222
#define PID_MIN_RPM 223
#define PID_SPEED_LIMIT2 224
#define PID_STEP_INPUT 225 
#define PID_CURVE_PT 226 
#define PID_PRESET_DATA 227 
#define PID_POSI_MIN_LIMIT 231
#define PID_POSI_CEN 232
#define PID_POSI_MAX_LIMIT 233
#define PID_TIME 234
#define PID_CAN_RESEND 238
#define PID_FUNC_SPEED 239
#define PID_PHASE_OFFSET 241
#define PID_POSI_CMD 243
#define PID_INC_POSI_CMD 244
#define PID_WRITE_ADDR 245 
#define PID_PNT_POSI_CMD 246
#define PID_FUNC_POSI 250


// Make interger from two bytes
int BYTE2Int(uint8_t byteLow, uint8_t byteHigh);

// Make long type data from four uint8_ts
int BYTE2LongInt(uint8_t byteData1, uint8_t byteData2, uint8_t byteData3, uint8_t byteData4);
