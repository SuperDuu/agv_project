#ifndef __AGV_BODY_STEP_H
#define __AGV_BODY_STEP_H

#include "main.h"
extern TIM_HandleTypeDef htim3;

/* Motor 200 full-step/rev, vi buoc 1/8 = 1600 microstep/rev */
#define MICROSTEPS_PER_REV   1600
#define DEG_PER_MICROSTEP    (360.0f / MICROSTEPS_PER_REV)
#define TIMER_CLK            1000000  /* 1 MHz (PSC=169) */
#define RPM_TO_DPS(rpm)      ((rpm) * 6.0f)
#define MAX_RPM              60.0f

/* TB6600 pin: DIR=PG3, EN=PG2 (active LOW) */
#define DIR_GPIO_PORT        GPIOE
#define DIR_GPIO_PIN         GPIO_PIN_4
#define EN_GPIO_PORT         GPIOE
#define EN_GPIO_PIN          GPIO_PIN_5

/* Khung lenh dieu khien */
typedef struct {
  float angle;   /* Goc quay (do): duong=CW, am=CCW */
  float rpm;     /* Toc do (RPM, max 60) */
} step_command_t;

/* Trang thai phan hoi */
typedef struct {
  uint8_t  running;       /* 1 = dang chay */
  uint8_t  done;          /* 1 = lenh cuoi da hoan thanh */
  int32_t  position_steps;/* Vi tri tuyet doi (microstep) */
  float    position_deg;  /* Vi tri tuyet doi (do) */
  uint32_t steps_done;    /* So buoc da di trong lenh hien tai */
  uint32_t steps_total;   /* Tong so buoc cua lenh hien tai */
} step_status_t;

/* API */
void             step_Run(const volatile step_command_t *cmd);
void             step_Stop(void);
step_status_t step_GetStatus(void);
void             step_ResetPosition(void);
void             step_TIM_Callback(TIM_HandleTypeDef *htim);

#endif
