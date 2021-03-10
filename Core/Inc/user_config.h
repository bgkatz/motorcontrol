/// Values stored in flash, which are modified by user actions ///

#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif


#define I_BW                    __float_reg[2]                                  // Current loop bandwidth
#define I_MAX                   __float_reg[3]                                  // Current limit
#define THETA_MIN               __float_reg[4]                                  // Minimum position setpoint
#define THETA_MAX               __float_reg[5]                                  // Maximum position setpoint
#define I_FW_MAX                __float_reg[6]                                  // Maximum field weakening current
#define R_NOMINAL               __float_reg[7]                                  // Nominal motor resistance, set during calibration
#define TEMP_MAX                __float_reg[8]                                  // Temperature safety lmit
#define I_MAX_CONT              __float_reg[9]                                  // Continuous max current
#define PPAIRS					__float_reg[10]									// Number of motor pole-pairs
//#define L_D						__float_reg[11]									// D-axis inductance
//#define L_Q						__float_reg[12]									// Q-axis inductance
#define R_PHASE					__float_reg[13]									// Single phase resistance
#define KT						__float_reg[14]									// Torque Constant (N-m/A)
#define R_TH					__float_reg[15]									// Thermal resistance (C/W)
#define C_TH					__float_reg[16]									// Thermal mass (C/J)
#define GR						__float_reg[17]									// Gear ratio
#define I_CAL					__float_reg[18]									// Calibration Current
#define P_MIN					__float_reg[19]									// Position setpoint lower limit (rad)
#define P_MAX					__float_reg[20]									// Position setupoint upper bound (rad)
#define V_MIN					__float_reg[21]									// Velocity setpoint lower bound (rad/s)
#define V_MAX					__float_reg[22]									// Velocity setpoint upper bound (rad/s)
#define KP_MAX					__float_reg[23]									// Max position gain (N-m/rad)
#define KD_MAX					__float_reg[24]									// Max velocity gain (N-m/rad/s)


#define PHASE_ORDER             __int_reg[0]                                    // Phase swapping during calibration
#define CAN_ID                  __int_reg[1]                                    // CAN bus ID
#define CAN_MASTER              __int_reg[2]                                    // CAN bus "master" ID
#define CAN_TIMEOUT             __int_reg[3]                                    // CAN bus timeout period
#define M_ZERO					__int_reg[4]
#define E_ZERO					__int_reg[5]
#define ENCODER_LUT             __int_reg[6]                                    // Encoder offset LUT - 128 elements long




extern float __float_reg[];
extern int __int_reg[];

#ifdef __cplusplus
}
#endif

#endif
