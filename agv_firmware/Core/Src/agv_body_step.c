#include "agv_body_step.h"

/* Internal state */
volatile uint8_t  step_running = 0;
volatile uint32_t step_count  = 0;
volatile uint32_t step_target = 0;
volatile int8_t   current_dir = 1;       /* +1=CW, -1=CCW */
volatile int32_t  position_steps = 0;     /* Vi tri tuyet doi (microstep) */
volatile uint8_t  move_done = 0;

void step_Run(const volatile step_command_t *cmd)
{
  step_Stop();

  float current_angle = position_steps * DEG_PER_MICROSTEP;
  float delta_angle = cmd->angle - current_angle;

  current_dir = (delta_angle > 0.0f) ? 1 : -1;
  float abs_angle = (delta_angle > 0.0f) ? delta_angle : -delta_angle;

  step_target = (uint32_t)(abs_angle / DEG_PER_MICROSTEP + 0.5f);
  step_count  = 0;
  if (step_target == 0) { move_done = 1; return; }

  move_done = 0;

  HAL_GPIO_WritePin(DIR_GPIO_PORT, DIR_GPIO_PIN, (current_dir > 0) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  float rpm = cmd->rpm;
  if (rpm > MAX_RPM) rpm = MAX_RPM;
  if (rpm < 1.0f)    rpm = 1.0f;

  float steps_per_sec = RPM_TO_DPS(rpm) / DEG_PER_MICROSTEP;
  uint32_t arr = (uint16_t)(TIMER_CLK / steps_per_sec) - 1;
  if (arr < 1) arr = 1;

  __HAL_TIM_SET_PRESCALER(&htim3, 249);
  __HAL_TIM_SET_AUTORELOAD(&htim3, arr);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, arr / 2);
  htim3.Instance->EGR = TIM_EGR_UG;
  __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
  __HAL_TIM_SET_COUNTER(&htim3, 0);

  HAL_GPIO_WritePin(EN_GPIO_PORT, EN_GPIO_PIN, EN_PIN_ACTIVE);
  step_running = 1;

  /* Enable TIM3 global interrupt in NVIC */
  HAL_NVIC_SetPriority(TIM3_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);

  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_Base_Start_IT(&htim3);
}

void step_Stop(void)
{
  HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
  HAL_TIM_Base_Stop_IT(&htim3);
  step_running = 0;
  HAL_GPIO_WritePin(EN_GPIO_PORT, EN_GPIO_PIN, EN_PIN_INACTIVE);
}

step_status_t step_GetStatus(void)
{
	step_status_t s;
  s.running        = step_running;
  s.done           = move_done;
  s.position_steps = position_steps;
  s.position_deg   = position_steps * DEG_PER_MICROSTEP;
  s.steps_done     = step_count;
  s.steps_total    = step_target;
  return s;
}

void step_ResetPosition(void)
{
  position_steps = 0;
}

void step_TIM_Callback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance != TIM3) return;

  position_steps += current_dir;  /* Cap nhat vi tri moi buoc */
  if (++step_count >= step_target) {
    step_Stop();
    move_done = 1;
  }
}
