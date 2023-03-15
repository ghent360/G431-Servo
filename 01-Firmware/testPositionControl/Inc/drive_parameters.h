
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
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#define MAX_APPLICATION_SPEED_RPM       2000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM       0 /*!< rpm, mechanical,
                                                           absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS 3 /*!< Number of speed
                                                             measurement errors before
                                                             main sensor goes in fault */
/*** Encoder **********************/

#define ENC_AVERAGING_FIFO_DEPTH        16 /*!< depth of the FIFO used to
                                                              average mechanical speed in
                                                              0.1Hz resolution */
#ifdef OBSERVER_PLL
/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD             0.25 /*!<Maximum accepted
                                                            variance on speed
                                                            estimates (percentage) */
/* State observer scaling factors F1 */
#define F1                               16384
#define F2                               4096
#define F1_LOG                           LOG2((16384))
#define F2_LOG                           LOG2((4096))

/* State observer constants */
#define GAIN1                            -23218
#define GAIN2                            18593
/*Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                      709
#define PLL_KI_GAIN                      17
#define PLL_KPDIV     16384
#define PLL_KPDIV_LOG LOG2((PLL_KPDIV))
#define PLL_KIDIV     65535
#define PLL_KIDIV_LOG LOG2((PLL_KIDIV))

#define STO_FIFO_DEPTH_DPP               64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG           LOG2((64))

#define STO_FIFO_DEPTH_UNIT              64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in the unit defined by #SPEED_UNIT */
#define BEMF_CONSISTENCY_TOL             64   /* Parameter for B-emf
                                                            amplitude-speed consistency */
#define BEMF_CONSISTENCY_GAIN            64   /* Parameter for B-emf
                                                           amplitude-speed consistency */

#elif defined(OBSERVER_CORDIC)
/****** State Observer + CORDIC ***/
#define CORD_VARIANCE_THRESHOLD          25 /*!<Maxiumum accepted
                                                            variance on speed
                                                            estimates (percentage) */
#define CORD_F1                          16384
#define CORD_F2                          4096
#define CORD_F1_LOG                      LOG2((16384))
#define CORD_F2_LOG                      LOG2((4096))

/* State observer constants */
#define CORD_GAIN1                       -23218
#define CORD_GAIN2                       18593

#define CORD_FIFO_DEPTH_DPP              64  /*!< Depth of the FIFO used
                                                            to average mechanical speed
                                                            in dpp format */
#define CORD_FIFO_DEPTH_DPP_LOG          LOG2((64))

#define CORD_FIFO_DEPTH_UNIT            64  /*!< Depth of the FIFO used
                                                           to average mechanical speed
                                                           in dpp format */
#define CORD_MAX_ACCEL_DPPP              81  /*!< Maximum instantaneous
                                                              electrical acceleration (dpp
                                                              per control period) */
#define CORD_BEMF_CONSISTENCY_TOL        64  /* Parameter for B-emf
                                                           amplitude-speed consistency */
#define CORD_BEMF_CONSISTENCY_GAIN       64  /* Parameter for B-emf
                                                          amplitude-speed consistency */
#endif

/* USER CODE BEGIN angle reconstruction M1 */
#define REV_PARK_ANGLE_COMPENSATION_FACTOR 0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */

#define PWM_FREQUENCY   30000
#define PWM_FREQ_SCALING 1

#define LOW_SIDE_SIGNALS_ENABLING        LS_PWM_TIMER
#define SW_DEADTIME_NS                   750 /*!< Dead-time to be inserted
                                                           by FW, only if low side
                                                           signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    /*!< FOC execution rate in
                                                           number of PWM cycles */
/* Gains values for torque and flux control loops */
#define PID_TORQUE_KP_DEFAULT         3719
#define PID_TORQUE_KI_DEFAULT         1644
#define PID_TORQUE_KD_DEFAULT         100
#define PID_FLUX_KP_DEFAULT           3719
#define PID_FLUX_KI_DEFAULT           1644
#define PID_FLUX_KD_DEFAULT           100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                      4096
#define TF_KIDIV                      16384
#define TF_KDDIV                      8192
#define TF_KPDIV_LOG                  LOG2((4096))
#define TF_KIDIV_LOG                  LOG2((16384))
#define TF_KDDIV_LOG                  LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING  DISABLE

#define POSITION_LOOP_FREQUENCY_HZ    ( uint16_t )2000 /*!<Execution rate of position control regulation loop (Hz) */

#define PID_SPEED_KP_DEFAULT          3859/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KI_DEFAULT          363/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT          0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* Speed PID parameter dividers */
#define SP_KPDIV                      2048
#define SP_KIDIV                      16384
#define SP_KDDIV                      16
#define SP_KPDIV_LOG                  LOG2((2048))
#define SP_KIDIV_LOG                  LOG2((16384))
#define SP_KDDIV_LOG                  LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING DISABLE
#define IQMAX                          13607

/* Default settings */
#define DEFAULT_CONTROL_MODE           MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM       886
#define DEFAULT_TARGET_SPEED_UNIT      (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT       0
#define DEFAULT_FLUX_COMPONENT         0

#define PID_POSITION_KP_GAIN			10000
#define PID_POSITION_KI_GAIN			1000
#define PID_POSITION_KD_GAIN			1000
#define PID_POSITION_KPDIV				1024
#define PID_POSITION_KIDIV				32768
#define PID_POSITION_KDDIV				16
#define PID_POSITION_KPDIV_LOG			LOG2((1024))
#define PID_POSITION_KIDIV_LOG			LOG2((32768))
#define PID_POSITION_KDDIV_LOG			LOG2((16))
#define PID_POSITION_ANGLE_STEP			10.0
#define PID_POSITION_MOV_DURATION		10.0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_THRESHOLD_V          27 /*!< Over-voltage
                                                         threshold */
#define UD_VOLTAGE_THRESHOLD_V          12 /*!< Under-voltage
                                                          threshold */
#ifdef NOT_IMPLEMENTED

#define ON_OVER_VOLTAGE                 TURN_OFF_PWM /*!< TURN_OFF_PWM,
                                                         TURN_ON_R_BRAKE or
                                                         TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */

#define OV_TEMPERATURE_THRESHOLD_C      70 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     10 /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS       DISABLE /*!< In case ON_OVER_VOLTAGE
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by
                                                          power stage) */

#define OVP_INVERTINGINPUT_MODE         INT_MODE
#define OVP_INVERTINGINPUT_MODE2        INT_MODE
#define OVP_SELECTION                   COMP_Selection_COMP1
#define OVP_SELECTION2                  COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/
/* Encoder alignment */
#define ALIGNMENT_DURATION              1000 /*!< milliseconds */
#define ALIGNMENT_ANGLE_DEG             90 /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT               2721 /*!< s16A */
// With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment
// phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt)
// being Av the voltage gain between Rshunt and A/D input

/* USER CODE BEGIN OPENLOOP M1 */

#define OPEN_LOOP_VOLTAGE_d           0      /*!< Three Phase voltage amplitude
                                                      in int16_t format */
#define OPEN_LOOP_SPEED_RPM           100       /*!< Final forced speed in rpm */
#define OPEN_LOOP_SPEED_RAMP_DURATION_MS  1000  /*!< 0-to-Final speed ramp duration  */      
#define OPEN_LOOP_VF                  false     /*!< true to enable V/F mode */
#define OPEN_LOOP_K                   44        /*! Slope of V/F curve expressed in int16_t Voltage for 
                                                     each 0.1Hz of mecchanical frequency increment. */
#define OPEN_LOOP_OFF                 4400      /*! Offset of V/F curve expressed in int16_t Voltage 
                                                     applied when frequency is zero. */
/* USER CODE END OPENLOOP M1 */

#if defined(OBSERVER_PLL) || defined(OBSERVER_CORDIC)
/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM          1000

#define NB_CONSECUTIVE_TESTS           2 /* corresponding to
                                                         former NB_CONSECUTIVE_TESTS/
                                                         (TF_REGULATION_RATE/
                                                         MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT         17 /*!< It expresses how much
                                                            estimated speed can exceed
                                                            forced stator electrical
                                                            without being considered wrong.
                                                            In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT         15  /*!< It expresses how much
                                                             estimated speed can be below
                                                             forced stator electrical
                                                             without being considered wrong.
                                                             In 1/16 of forced speed */
#endif

#define TRANSITION_DURATION            25  /* Switch over duration, ms */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(47)
/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME  LL_ADC_SAMPLING_CYCLE(47)
/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES (6 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

#ifdef FEED_FORWARD
/*  Feed-forward parameters */
#define FEED_FORWARD_CURRENT_REG_ENABLING ENABLE
#define CONSTANT1_Q                    163593
#define CONSTANT1_D                    163593
#define CONSTANT2_QD                   16449
#endif

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2022 STMicroelectronics *****END OF FILE****/
