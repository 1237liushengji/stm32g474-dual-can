/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 双机CAN通信应用（两块DRG ST-10 STM32G474VET6板 + SN65HVD230）
  *
  * 应用层协议（详见 can.h 与工程README）：
  *   板A（主节点，CAN_NODE_ROLE = CAN_NODE_A）：
  *     - 每1000ms发送一帧命令帧（ID 0x321，载荷含序号seq与开机秒数）；
  *     - 收到板B应答帧（ID 0x322）后刷新链路存活时间，并按应答中的
  *       LED状态同步点亮/熄灭本板LED1（两板LED闪烁同步）；
  *     - 超过CAN_LINK_TIMEOUT_MS未收到应答 -> LED1以100ms快闪指示链路故障。
  *   板B（从节点，CAN_NODE_ROLE = CAN_NODE_B）：
  *     - 收到命令帧后翻转本板LED1，并回发应答帧（回显seq、LED状态、累计帧数）；
  *     - 超过CAN_LINK_TIMEOUT_MS未收到命令 -> LED1快闪指示链路故障。
  *
  * 时钟：HSE 8MHz -> PLL(M=2,N=75,R=2) -> SYSCLK/HCLK/PCLK1 = 150MHz
  * FDCAN内核时钟 = PCLK1 = 150MHz，500kbit/s经典CAN（位时序见can.h）
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "main.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "can.h"
#include "bsp_uart.h"
#include "console.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define APP_CMD_PERIOD_MS        1000U    /* 板A命令帧发送周期 */
#define APP_ERR_BLINK_PERIOD_MS  100U     /* 链路故障指示快闪周期 */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static CAN_RxMsg s_rxMsg;                          /* 最近一帧接收报文 */

static uint32_t  s_bootTick;                       /* 开机时刻（ms） */
static uint32_t  s_lastCmdTick;                    /* 板A：最近发送/板B：最近收到命令帧时刻 */
static uint32_t  s_lastRespTick;                   /* 板A：最近收到应答帧时刻 */
static uint32_t  s_lastBlinkTick;                  /* 故障指示快闪节拍 */
static uint8_t   s_seq;                            /* 板A命令帧序号 */
static uint8_t   s_bLedState;                      /* 板B当前LED状态（两板同步显示） */
static uint16_t  s_rxCmdCount;                     /* 板B累计收到命令帧数 */
static uint8_t   s_linkEverOk;                     /* 链路是否曾经建立 */
static uint32_t  s_lastErrBlinkTick;               /* LED2错误指示闪烁节拍 */
static uint32_t  s_errTick;                        /* 最近一次总线错误事件时刻 */
static uint32_t  s_errTotalLast;                   /* 上次统计的错误总数快照 */
static uint8_t   s_errActive;                      /* 错误指示窗口激活标志 */

static IWDG_HandleTypeDef hiwdg;                   /* 独立看门狗：LSI/32=1kHz，重装载3s */

/* 调试计数器（定位sniff下LED冻结：观察协议分发路径是否执行） */
uint32_t g_dbgHx;                                  /* App_HandleRx调用次数 */
uint32_t g_dbgCmd;                                 /* App_HandleCmd调用次数 */
uint32_t g_dbgResp;                                /* App_HandleResp调用次数 */
uint32_t g_dbgPoll;                                /* CAN_PollRx取到帧次数 */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void App_HandleCmd(void);
static void App_HandleResp(void);
static void App_HandleRx(void);
static void App_Process(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
  * @brief  主节点侧：处理应答帧（ID 0x322）——刷新链路状态并同步LED
  */
static void App_HandleResp(void)
{
  g_dbgResp++;
  s_lastRespTick = HAL_GetTick();
  s_linkEverOk   = 1U;
  s_bLedState    = (s_rxMsg.data[1] != 0U) ? 1U : 0U;

  if (s_bLedState != 0U)                        /* LED与板B保持同步 */
  {
    LED1_ON;
  }
  else
  {
    LED1_OFF;
  }
}

/**
  * @brief  从节点侧：处理命令帧（ID 0x321）——翻转LED并立即应答
  */
static void App_HandleCmd(void)
{
  g_dbgCmd++;
  s_lastCmdTick = HAL_GetTick();
  s_linkEverOk  = 1U;
  s_rxCmdCount++;

  LED1_Toggle;                                   /* 每收到一帧命令，LED翻转一次 */
  s_bLedState = (HAL_GPIO_ReadPin(LED1_PORT, LED1_PIN) == GPIO_PIN_RESET) ? 1U : 0U;

  CAN_EncodeResp(s_rxMsg.data, s_rxMsg.data[0], s_bLedState, s_rxCmdCount);
  (void)CAN_Send(CAN_ID_RESP_B2A, s_rxMsg.data, CAN_PAYLOAD_LEN);
}

/**
  * @brief  处理一帧接收报文（按节点角色分发；自测试模式单板扮演双角色）
  */
static void App_HandleRx(void)
{
  g_dbgHx++;
#if (CAN_DEBUG_SELFTEST != 0)
  /* 单板自测试：回环收到自己发的命令帧，同时扮演两个节点 */
  if (s_rxMsg.id == CAN_ID_CMD_A2B)
  {
    App_HandleCmd();
  }
  else if (s_rxMsg.id == CAN_ID_RESP_B2A)
  {
    App_HandleResp();
  }
  else
  {
    /* 其他ID不处理 */
  }
#elif (CAN_NODE_ROLE == CAN_NODE_A)
  /* 板A：处理板B的应答帧（ID 0x322） */
  if (s_rxMsg.id == CAN_ID_RESP_B2A)
  {
    App_HandleResp();
  }
#else
  /* 板B：处理板A的命令帧（ID 0x321），翻转LED并应答 */
  if (s_rxMsg.id == CAN_ID_CMD_A2B)
  {
    App_HandleCmd();
  }
#endif
}

/**
  * @brief  主循环周期任务：收帧处理 + 发送调度 + 链路监视
  */
static void App_Process(void)
{
  uint32_t now = HAL_GetTick();

  /* 1. 取走并处理接收报文：监视模式下"打印+协议处理"并行——观察不打扰业务 */
  if (CAN_PollRx(&s_rxMsg))
  {
    g_dbgPoll++;
    if (Console_SniffEnabled())
    {
      Console_PrintFrame(&s_rxMsg);
    }
    App_HandleRx();
  }

  /* 2. 总线错误指示（LED2绿色）：出现bus-off/错误被动事件后5秒内以250ms闪烁。
   *    发送端发帧无人应答（ACK缺失）时必然出现错误被动→bus-off，据此可判断
   *    "本板在发、但对端收发器不在总线上"；无任何错误则说明总线电气层沉默。 */
  {
    const CAN_Stats *st = CAN_GetStats();
    uint32_t errTotal = st->busOffCount + st->errPassiveCount;

    if (errTotal != s_errTotalLast)
    {
      s_errTotalLast = errTotal;
      s_errTick      = now;
      s_errActive    = 1U;
    }

    if (s_errActive != 0U)
    {
      if ((now - s_errTick) < 5000U)
      {
        if ((now - s_lastErrBlinkTick) >= 250U)
        {
          s_lastErrBlinkTick = now;
          LED2_Toggle;
        }
      }
      else
      {
        s_errActive = 0U;
        LED2_OFF;
      }
    }
  }

#if (CAN_NODE_ROLE == CAN_NODE_A)
  /* 2. 板A：周期发送命令帧 */
  if ((now - s_lastCmdTick) >= APP_CMD_PERIOD_MS)
  {
    s_lastCmdTick = now;
    s_seq++;

    uint8_t payload[CAN_PAYLOAD_LEN];
    CAN_EncodeCmd(payload, s_seq, (uint16_t)((now - s_bootTick) / 1000U));
    (void)CAN_Send(CAN_ID_CMD_A2B, payload, CAN_PAYLOAD_LEN);
  }

  /* 3. 板A链路监视：曾建立链路后超过门限无应答 -> 快闪报警 */
  if (s_linkEverOk != 0U)
  {
    if ((now - s_lastRespTick) > CAN_LINK_TIMEOUT_MS)
    {
      if ((now - s_lastBlinkTick) >= APP_ERR_BLINK_PERIOD_MS)
      {
        s_lastBlinkTick = now;
        LED1_Toggle;
      }
    }
  }
  else
  {
    /* 上电后从未建立链路：给首个周期收发建立时间后同样快闪提示 */
    if (((now - s_bootTick) > (CAN_LINK_TIMEOUT_MS + APP_CMD_PERIOD_MS)) &&
        ((now - s_lastBlinkTick) >= APP_ERR_BLINK_PERIOD_MS))
    {
      s_lastBlinkTick = now;
      LED1_Toggle;
    }
  }
#else
  /* 板B链路监视：曾收到命令后超过门限无新命令 -> 快闪报警 */
  if (s_linkEverOk != 0U)
  {
    if ((now - s_lastCmdTick) > CAN_LINK_TIMEOUT_MS)
    {
      if ((now - s_lastBlinkTick) >= APP_ERR_BLINK_PERIOD_MS)
      {
        s_lastBlinkTick = now;
        LED1_Toggle;
      }
    }
  }
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  /* USER CODE BEGIN 2 */
  LED_Init();                                      /* LED1（PE0，蓝色）作通信状态指示 */
  LED1_OFF;                                        /* 应用接管LED：初始熄灭 */

  BSP_UART_Init();                                 /* 控制台串口 USART1 PA9/PA10 115200 8N1 */
  Console_Init();                                  /* 打印横幅，等待命令 */
  BSP_UART_Send("main: sniff-parallel M2\r\n", 27U); /* main.c构建指纹：无此行=烧的旧main.o */

  if (CAN_Init() != HAL_OK)                        /* FDCAN2 + 过滤器 + 中断 + 启动 */
  {
    Error_Handler();
  }

  /* 上电自检：内部回环自发自收，验证FDCAN内核+消息RAM+中断+驱动软件链路 */
  if (CAN_SelfTest())
  {
    BSP_UART_Send("CAN self-test: PASS\r\n", 21U);
  }
  else
  {
    BSP_UART_Send("CAN self-test: FAIL\r\n", 21U);
  }

  /* 独立看门狗：主循环喂狗；调试器halt时冻结计数，断点调试不误复位 */
  DBGMCU->APB1FZR1 |= DBGMCU_APB1FZR1_DBG_IWDG_STOP;
  hiwdg.Instance     = IWDG;                       /* HAL经Instance访问寄存器，必须赋值（漏赋=空指针HardFault） */
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;        /* LSI 32kHz/32 = 1kHz */
  hiwdg.Init.Reload    = 3000U;                    /* 3s超时（含裕量） */
  hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }

  s_bootTick    = HAL_GetTick();
  s_lastCmdTick = s_bootTick;
  s_lastRespTick = s_bootTick;
  s_lastBlinkTick = s_bootTick;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    App_Process();
    Console_Task();                                /* 串口命令行控制台 */
    CAN_Task();                                    /* bus-off指数退避恢复调度 */
    HAL_IWDG_Refresh(&hiwdg);                      /* 喂狗（主循环健康证明） */
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 75;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
