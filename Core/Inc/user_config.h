/// Values stored in flash, which are modifieable by user actions ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H


#define E_OFFSET                __float_reg[0]                                  // Encoder electrical offset
#define M_OFFSET                __float_reg[1]                                  // Encoder mechanical offset
#define I_BW                    __float_reg[2]                                  // Current loop bandwidth
#define I_MAX                   __float_reg[3]                                  // Torque limit (current limit = torque_limit/(kt*gear ratio))
#define THETA_MIN               __float_reg[4]                                  // Minimum position setpoint
#define THETA_MAX               __float_reg[5]                                  // Maximum position setpoint
#define I_FW_MAX                __float_reg[6]                                  // Maximum field weakening current
#define R_NOMINAL               __float_reg[7]                                  // Nominal motor resistance, set during calibration
#define TEMP_MAX                __float_reg[8]                                  // Temperature safety lmit
#define I_MAX_CONT              __float_reg[9]                                  // Continuous max current

#define PHASE_ORDER             __int_reg[0]                                    // Phase swapping during calibration
#define CAN_ID                  __int_reg[1]                                    // CAN bus ID
#define CAN_MASTER              __int_reg[2]                                    // CAN bus "master" ID
#define CAN_TIMEOUT             __int_reg[3]                                    // CAN bus timeout period
#define ENCODER_LUT             __int_reg[5]                                    // Encoder offset LUT - 128 elements long




extern float __float_reg[];
extern int __int_reg[];

#endif
