/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "zdt42.h"
#include "usart.h"
#include <limits.h>
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static osMutexId_t motorUartMutex;
/* USER CODE END Variables */
/* Definitions for motorcontrol */
osThreadId_t motorcontrolHandle;
const osThreadAttr_t motorcontrol_attributes = {
  .name = "motorcontrol",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Motorcontrol(void *argument);

static void ExecuteMotorCommand(char *command);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  motorUartMutex = osMutexNew(NULL);
  if (motorUartMutex == NULL)
  {
    Error_Handler();
  }
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of motorcontrol */
  motorcontrolHandle = osThreadNew(Motorcontrol, NULL, &motorcontrol_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Motorcontrol */
/**
  * @brief  Function implementing the motorcontrol thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Motorcontrol */
void Motorcontrol(void *argument)
{
  char cmd_buf[48];
  uint8_t idx = 0U;
  uint8_t dropping_line = 0U;
  uint8_t received_byte;

  ZDT42_Init();
  if (HAL_UART_Receive_IT(&huart2, (uint8_t *)&g_uart2_data, 1U) != HAL_OK)
  {
    Error_Handler();
  }
  if (osMutexAcquire(motorUartMutex, osWaitForever) != osOK)
  {
    Error_Handler();
  }
  ZDT42_Enable(1, 1);
  if (osMutexRelease(motorUartMutex) != osOK)
  {
    Error_Handler();
  }

  for(;;)
  {
    if (USART2_TakeRxOverflow() != 0U)
    {
      idx = 0U;
      dropping_line = 1U;
      memset(cmd_buf, 0, sizeof(cmd_buf));
    }

    while (USART2_ReadByte(&received_byte) != 0U)
    {
      if (received_byte == '\r' || received_byte == '\n')
      {
        cmd_buf[idx] = '\0';
        ExecuteMotorCommand(cmd_buf);
        idx = 0U;
        memset(cmd_buf, 0, sizeof(cmd_buf));
        dropping_line = 0U;
      }
      else if (dropping_line == 0U)
      {
        if (idx < (sizeof(cmd_buf) - 1U))
        {
          cmd_buf[idx++] = (char)received_byte;
          if (idx >= (sizeof(cmd_buf) - 1U))
          {
            idx = 0U;
            dropping_line = 1U;
            memset(cmd_buf, 0, sizeof(cmd_buf));
          }
        }
        else
        {
          idx = 0U;
          dropping_line = 1U;
          memset(cmd_buf, 0, sizeof(cmd_buf));
        }
      }
    }
    osDelay(1U);
  }
}

static void ExecuteMotorCommand(char *command)
{
  char *start = command;
  size_t length;
  int value;
  char extra;
  int parsed;
  enum
  {
    MOTOR_COMMAND_NONE,
    MOTOR_COMMAND_SPEED,
    MOTOR_COMMAND_RELATIVE,
    MOTOR_COMMAND_ABSOLUTE,
    MOTOR_COMMAND_ON,
    MOTOR_COMMAND_OFF,
    MOTOR_COMMAND_STOP
  } motor_command = MOTOR_COMMAND_NONE;

  while (*start == ' ' || *start == '\t')
  {
    start++;
  }
  length = strlen(start);
  while (length > 0U &&
         (start[length - 1U] == ' ' || start[length - 1U] == '\t'))
  {
    start[--length] = '\0';
  }

  parsed = sscanf(start, "speed:%d %c", &value, &extra);
  if (parsed != 1)
  {
    parsed = sscanf(start, "speed %d %c", &value, &extra);
  }
  if (parsed == 1 && value >= INT16_MIN && value <= INT16_MAX)
  {
    motor_command = MOTOR_COMMAND_SPEED;
  }
  else
  {
    parsed = sscanf(start, "rel:%d %c", &value, &extra);
    if (parsed != 1)
    {
      parsed = sscanf(start, "rel %d %c", &value, &extra);
    }
    if (parsed == 1)
    {
      motor_command = MOTOR_COMMAND_RELATIVE;
    }
    else
    {
      parsed = sscanf(start, "abs:%d %c", &value, &extra);
      if (parsed != 1)
      {
        parsed = sscanf(start, "abs %d %c", &value, &extra);
      }
      if (parsed == 1)
      {
        motor_command = MOTOR_COMMAND_ABSOLUTE;
      }
    }
  }

  if (motor_command == MOTOR_COMMAND_NONE)
  {
    if (strcmp(start, "on") == 0)
    {
      motor_command = MOTOR_COMMAND_ON;
    }
    else if (strcmp(start, "off") == 0)
    {
      motor_command = MOTOR_COMMAND_OFF;
    }
    else if (strcmp(start, "stop") == 0)
    {
      motor_command = MOTOR_COMMAND_STOP;
    }
  }

  if (motor_command == MOTOR_COMMAND_NONE)
  {
    return;
  }

  if (osMutexAcquire(motorUartMutex, osWaitForever) != osOK)
  {
    Error_Handler();
  }

  switch (motor_command)
  {
    case MOTOR_COMMAND_SPEED:
      ZDT42_SetSpeed(1, (int16_t)value, 1000);
      break;
    case MOTOR_COMMAND_RELATIVE:
      ZDT42_MoveRelative(1, (float)value, 500, 10);
      break;
    case MOTOR_COMMAND_ABSOLUTE:
      ZDT42_MoveAbsolute(1, (float)value, 500, 10);
      break;
    case MOTOR_COMMAND_ON:
      ZDT42_Enable(1, 1);
      break;
    case MOTOR_COMMAND_OFF:
      ZDT42_Enable(1, 0);
      break;
    case MOTOR_COMMAND_STOP:
      ZDT42_Stop(1);
      break;
    default:
      break;
  }

  if (osMutexRelease(motorUartMutex) != osOK)
  {
    Error_Handler();
  }

  if (HAL_UART_Transmit(&huart2, (uint8_t *)"CMD_OK\r\n",
                        sizeof("CMD_OK\r\n") - 1U, 100U) != HAL_OK)
  {
    Error_Handler();
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

