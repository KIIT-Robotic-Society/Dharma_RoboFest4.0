
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED_RPM           1000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM           0 /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS     10 /*!< Number of speed measurement errors before main sensor goes in fault */

/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD                  0 /*!< Maximum accepted variance on speed estimates (percentage) */

/* State observer scaling factors F1 */
#define F1                                  16384
#define F2                                  4096
#define F1_LOG                              LOG2((16384))
#define F2_LOG                              LOG2((4096))

/* State observer constants */
#define GAIN1                               -23040
#define GAIN2                               28897

/* Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                         372
#define PLL_KI_GAIN                         13
#define PLL_KPDIV                           16384
#define PLL_KPDIV_LOG                       LOG2((PLL_KPDIV))
#define PLL_KIDIV                           65535
#define PLL_KIDIV_LOG                       LOG2((PLL_KIDIV))

#define STO_FIFO_DEPTH_DPP                  5 /*!< Depth of the FIFO used  to average mechanical speed in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG              LOG2((5))
#define STO_FIFO_DEPTH_UNIT                 10 /*!< Depth of the FIFO used to average mechanical speed in the unit defined by #SPEED_UNIT */
#define M1_BEMF_CONSISTENCY_TOL             1 /* Parameter for B-emf amplitude-speed consistency */
#define M1_BEMF_CONSISTENCY_GAIN            13 /* Parameter for B-emf amplitude-speed consistency */

/* USER CODE BEGIN angle reconstruction M1 */
#define PARK_ANGLE_COMPENSATION_FACTOR      0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR  0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                       40000
#define PWM_FREQ_SCALING                    1
#define LOW_SIDE_SIGNALS_ENABLING           LS_PWM_TIMER
#define SW_DEADTIME_NS                      750 /*!< Dead-time to be inserted by FW, only if low side signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE           2 /*!< FOC execution rate in number of PWM cycles */
#define ISR_FREQUENCY_HZ                    (PWM_FREQUENCY/REGULATION_EXECUTION_RATE) /*!< @brief FOC execution rate in Hz */

/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT               2466
#define PID_TORQUE_KI_DEFAULT               2466
#define PID_TORQUE_KD_DEFAULT               100
#define PID_FLUX_KP_DEFAULT                 2466
#define PID_FLUX_KI_DEFAULT                 2466
#define PID_FLUX_KD_DEFAULT                 100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                            2048
#define TF_KIDIV                            16384
#define TF_KDDIV                            8192
#define TF_KPDIV_LOG                        LOG2((2048))
#define TF_KIDIV_LOG                        LOG2((16384))
#define TF_KDDIV_LOG                        LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING        DISABLE

#define PID_SPEED_KP_DEFAULT                3728/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT                15/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT                0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ             (uint16_t)1000 /*!<Execution rate of speed regulation loop (Hz) */

/* Speed PID parameter dividers */
#define SP_KPDIV                            1024
#define SP_KIDIV                            16384
#define SP_KDDIV                            16
#define SP_KPDIV_LOG                        LOG2((1024))
#define SP_KIDIV_LOG                        LOG2((16384))
#define SP_KDDIV_LOG                        LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV         1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING      DISABLE
#define IQMAX_A                             20

/* Default settings */
#define DEFAULT_CONTROL_MODE                MCM_TORQUE_MODE
#define DEFAULT_TARGET_SPEED_RPM            1000
#define DEFAULT_TARGET_SPEED_UNIT           (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A          25
#define DEFAULT_FLUX_COMPONENT_A            0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V              28 /*!< Over-voltage threshold */
#define UD_VOLTAGE_THRESHOLD_V              8 /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                     TURN_OFF_PWM /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C          70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C         10 /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS           DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this
                                                         feature may be used to bypass HW over-current protection
                                                         (if supported by power stage) */
#define OVP_INVERTINGINPUT_MODE             INT_MODE
#define OVP_INVERTINGINPUT_MODE2            INT_MODE
#define OVP_SELECTION                       COMP_Selection_COMP1
#define OVP_SELECTION2                      COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/

/* Phase 1 */
#define PHASE1_DURATION                     1000 /*milliseconds */
#define PHASE1_FINAL_SPEED_UNIT             (0*SPEED_UNIT/U_RPM)
#define PHASE1_FINAL_CURRENT_A              0

/* Phase 2 */
#define PHASE2_DURATION                     2000 /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT             (0*SPEED_UNIT/U_RPM)
#define PHASE2_FINAL_CURRENT_A              0

/* Phase 3 */
#define PHASE3_DURATION                     2000 /*milliseconds */
#define PHASE3_FINAL_SPEED_UNIT             (150*SPEED_UNIT/U_RPM)
#define PHASE3_FINAL_CURRENT_A              17

/* Phase 4 */
#define PHASE4_DURATION                     2000 /*milliseconds */
#define PHASE4_FINAL_SPEED_UNIT             (150*SPEED_UNIT/U_RPM)
#define PHASE4_FINAL_CURRENT_A              17

/* Phase 5 */
#define PHASE5_DURATION                     2000 /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT             (151*SPEED_UNIT/U_RPM)
#define PHASE5_FINAL_CURRENT_A              17

#define ENABLE_SL_ALGO_FROM_PHASE           1

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG                  0  /*!< degrees [0...359] */

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM               50
#define NB_CONSECUTIVE_TESTS                5 /* corresponding to former
                                                 NB_CONSECUTIVE_TESTS / (TF_REGULATION_RATE / MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT              19 /*!< It expresses how much estimated speed can exceed forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT              11 /*!< It expresses how much estimated speed can be below forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */

#define TRANSITION_DURATION                 0 /* Switch over duration, ms */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(47)

/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(47)

/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES                 (2 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

/*  Feed-forward parameters */
#define FEED_FORWARD_CURRENT_REG_ENABLING ENABLE
#define M1_CONSTANT1_Q                      144613
#define M1_CONSTANT1_D                      144613
#define M1_CONSTANT2_QD                     18896

/* **** Potentiometer parameters **** */
/** @brief Sampling time set to the ADC channel used by the potentiometer component */
#define POTENTIOMETER_ADC_SAMPLING_TIME_M1  LL_ADC_SAMPLING_CYCLE(47)

/**
 * @brief Speed reference set to Motor 1 when the potentiometer is at its maximum
 *
 * This value is expressed in #SPEED_UNIT.
 *
 * Default value is #MAX_APPLICATION_SPEED_UNIT.
 *
 * @sa POTENTIOMETER_MIN_SPEED_M1
 */
#define POTENTIOMETER_MAX_SPEED_M1          MAX_APPLICATION_SPEED_UNIT

/**
 * @brief Speed reference set to Motor 1 when the potentiometer is at its minimum
 *
 * This value is expressed in #SPEED_UNIT.
 *
 * Default value is 10 % of #MAX_APPLICATION_SPEED_UNIT.
 *
 * @sa POTENTIOMETER_MAX_SPEED_M1
 */
#define POTENTIOMETER_MIN_SPEED_M1          ((MAX_APPLICATION_SPEED_UNIT)/10)

/**
 * @brief Potentiometer change threshold to trigger speed reference update for Motor 1
 *
 * When the potentiometer value differs from the current speed reference by more than this
 * threshold, the speed reference set to the motor is adjusted to match the potentiometer value.
 *
 * The threshold is expressed in u16digits. Its default value is set to 13% of the potentiometer
 * aquisition range
 *
 */
 #define POTENTIOMETER_SPEED_ADJUSTMENT_RANGE_M1 (655)

/**
 * @brief Acceleration used to compute ramp duration when setting speed reference to Motor 1
 *
 * This acceleration is expressed in #SPEED_UNIT/s. Its default value is 100 Hz/s (provided
 * that #SPEED_UNIT is #U_01HZ).
 *
 */
 #define POTENTIOMETER_RAMP_SLOPE_M1        1000

/**
 * @brief Bandwith of the low pass filter applied on the potentiometer values
 *
 * @see SpeedPotentiometer_Handle_t::LPFilterBandwidthPOW2
 */
#define POTENTIOMETER_LPF_BANDWIDTH_POW2_M1 4

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2024 STMicroelectronics *****END OF FILE****/
