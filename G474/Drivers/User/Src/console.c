/**
  ******************************************************************************
  * @file    console.c
  * @brief   串口命令行控制台实现
  *
  * 命令表（help可查看）：
  *   help               命令列表
  *   version            固件版本/节点角色/编译时间
  *   stats              通信统计与运行时间
  *   bitrate <kbps>     切换总线波特率(125/250/500/1000)，两板需同步切换
  *   sniff <on|off>     总线监视模式（接收所有报文并打印）
  *   send <id> [b0..b7] 发送一帧（id与数据均为十六进制）
  *
  * 设计说明：
  *   1. 行编辑：回显、退格(0x08/0x7F)处理、超长截断、\r\n双字符兼容；
  *   2. 解析用strtok+strtol(16进制)，无动态内存；
  *   3. 输出统一ASCII（串口终端编码无关），经Put()按strlen发送；
  *   4. 命令处理在主循环上下文执行，与CAN业务同优先级，天然无竞争。
  ******************************************************************************
  */
#include "console.h"
#include "bsp_uart.h"
#include "led.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define CONSOLE_LINE_MAX     40U                     /* 单行命令最大长度 */

/*--------------------------------------- 模块内部变量 --------------------------------------*/
static char    s_line[CONSOLE_LINE_MAX + 1U];        /* 行编辑缓冲 */
static uint8_t s_lineLen;
static char    s_prevChar;                           /* 上一字符（\r\n组合识别） */
static bool    s_sniffOn;

/*--------------------------------------- 内部函数 --------------------------------------*/
static void Put(const char *s);
static void Console_Execute(char *line);
static void Console_PrintHelp(void);
static void Cmd_Version(void);
static void Cmd_Stats(void);
static void Cmd_Bitrate(char *args);
static void Cmd_Sniff(char *args);
static void Cmd_Send(char *args);
static void Cmd_Led(void);
static void Cmd_Tick(void);

/**
  * @brief  输出字符串（按实际长度）
  */
static void Put(const char *s)
{
  BSP_UART_Send(s, (uint16_t)strlen(s));
}

/**
  * @brief  控制台初始化：打印横幅
  */
void Console_Init(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_A)
  const char *role = "A(master)";
#else
  const char *role = "B(slave)";
#endif

  BSP_UART_Printf("\r\n=== STM32G474 CAN Node v%s | role:%s | %s %s ===\r\n",
                  CAN_APP_VERSION, role, __DATE__, __TIME__);
  Put("type 'help' for commands\r\n> ");
}

/**
  * @brief  控制台任务：行编辑 + 命令分发（主循环轮询调用）
  */
void Console_Task(void)
{
  int16_t c;

  while ((c = BSP_UART_GetChar()) >= 0)
  {
    char ch = (char)c;

    if ((ch == '\r') || (ch == '\n'))
    {
      bool isLoneLf = (ch == '\n') && (s_prevChar == '\r');

      s_prevChar = ch;
      if (isLoneLf)
      {
        continue;                                    /* \r\n组合的第二字符，忽略 */
      }

      Put("\r\n");
      s_line[s_lineLen] = '\0';
      if (s_lineLen != 0U)
      {
        Console_Execute(s_line);
      }
      s_lineLen = 0U;
      Put("> ");
    }
    else
    {
      s_prevChar = ch;

      if ((ch == 0x08U) || (ch == 0x7FU))            /* 退格/删除 */
      {
        if (s_lineLen != 0U)
        {
          s_lineLen--;
          Put("\b \b");
        }
      }
      else if ((ch >= 0x20U) && (ch < 0x7FU))        /* 可见字符 */
      {
        if (s_lineLen < CONSOLE_LINE_MAX)
        {
          s_line[s_lineLen++] = ch;
          BSP_UART_Send(&ch, 1U);                    /* 回显 */
        }
      }
      else
      {
        /* 其他控制字符忽略 */
      }
    }
  }
}

/**
  * @brief  解析并执行一条命令行
  * @param  line 已去除结尾的命令行
  */
static void Console_Execute(char *line)
{
  char *cmd  = strtok(line, " ");
  char *args = strtok(NULL, "");

  if (cmd == NULL)
  {
    return;
  }

  if (strcmp(cmd, "help") == 0)
  {
    Console_PrintHelp();
  }
  else if (strcmp(cmd, "version") == 0)
  {
    Cmd_Version();
  }
  else if (strcmp(cmd, "stats") == 0)
  {
    Cmd_Stats();
  }
  else if (strcmp(cmd, "bitrate") == 0)
  {
    Cmd_Bitrate(args);
  }
  else if (strcmp(cmd, "sniff") == 0)
  {
    Cmd_Sniff(args);
  }
  else if (strcmp(cmd, "send") == 0)
  {
    Cmd_Send(args);
  }
  else if (strcmp(cmd, "led") == 0)
  {
    Cmd_Led();
  }
  else if (strcmp(cmd, "tick") == 0)
  {
    Cmd_Tick();
  }
  else
  {
    BSP_UART_Printf("unknown cmd: %s (try 'help')\r\n", cmd);
  }
}

/**
  * @brief  命令列表
  */
static void Console_PrintHelp(void)
{
  Put("commands:\r\n");
  Put("  help               this help\r\n");
  Put("  version            fw version / node role\r\n");
  Put("  stats              comm statistics\r\n");
  Put("  bitrate <kbps>     125/250/500/1000, switch on BOTH nodes\r\n");
  Put("  sniff <on|off>     bus monitor mode\r\n");
  Put("  send <id> [b0..b7] send frame, hex, e.g. send 321 11 22\r\n");
  Put("  led                LED/GPIOE diagnose: regs + toggle test\r\n");
}

/**
  * @brief  version命令
  */
static void Cmd_Version(void)
{
#if (CAN_NODE_ROLE == CAN_NODE_A)
  const char *role = "A";
#else
  const char *role = "B";
#endif

  BSP_UART_Printf("v%s role=%s build=%s %s bitrate=%lubps\r\n",
                  CAN_APP_VERSION, role, __DATE__, __TIME__,
                  (unsigned long)CAN_GetBitrate());
}

/**
  * @brief  stats命令
  */
static void Cmd_Stats(void)
{
  const CAN_Stats *st = CAN_GetStats();

  BSP_UART_Printf("uptime=%lus bitrate=%lubps\r\n",
                  (unsigned long)(HAL_GetTick() / 1000U),
                  (unsigned long)CAN_GetBitrate());
  BSP_UART_Printf("tx=%lu rx=%lu txErr=%lu ovr=%lu busOff=%lu errPas=%lu txEvtLost=%lu protoErr=%lu\r\n",
                  (unsigned long)st->txCount, (unsigned long)st->rxCount,
                  (unsigned long)st->txErrors, (unsigned long)st->rxOverruns,
                  (unsigned long)st->busOffCount, (unsigned long)st->errPassiveCount,
                  (unsigned long)st->txEvtLost, (unsigned long)st->protocolErrors);
  BSP_UART_Printf("txAck=%lu txMaxLatency=%lums\r\n",
                  (unsigned long)st->txAckCount, (unsigned long)st->txMaxLatencyMs);
  BSP_UART_Printf("dbg poll=%lu hx=%lu cmd=%lu resp=%lu (poll应=rx, cmd/resp应随帧增长)\r\n",
                  (unsigned long)g_dbgPoll, (unsigned long)g_dbgHx,
                  (unsigned long)g_dbgCmd, (unsigned long)g_dbgResp);
}

/**
  * @brief  bitrate命令：bitrate <kbps>
  */
static void Cmd_Bitrate(char *args)
{
  long kbps;

  if ((args == NULL) || (*args == '\0'))
  {
    BSP_UART_Printf("usage: bitrate <125|250|500|1000>, now %lubps\r\n",
                    (unsigned long)CAN_GetBitrate());
    return;
  }

  kbps = strtol(args, NULL, 10);
  if ((kbps != 125L) && (kbps != 250L) && (kbps != 500L) && (kbps != 1000L))
  {
    Put("only 125/250/500/1000 kbps supported\r\n");
    return;
  }

  if (CAN_SetBitrate((uint32_t)kbps * 1000U) == HAL_OK)
  {
    BSP_UART_Printf("switched to %ldkbps (set the SAME on the other node!)\r\n", kbps);
  }
  else
  {
    Put("switch failed\r\n");
  }
}

/**
  * @brief  sniff命令：sniff <on|off>
  */
static void Cmd_Sniff(char *args)
{
  if ((args != NULL) && (strncmp(args, "on", 2U) == 0))
  {
    if (CAN_SetPromiscuous(true) == HAL_OK)
    {
      s_sniffOn = true;
      Put("sniff ON, format: RX <ms> <ID> <DLC> <data...>\r\n");
    }
  }
  else if ((args != NULL) && (strncmp(args, "off", 3U) == 0))
  {
    if (CAN_SetPromiscuous(false) == HAL_OK)
    {
      s_sniffOn = false;
      Put("sniff OFF\r\n");
    }
  }
  else
  {
    Put("usage: sniff <on|off>\r\n");
  }
}

/**
  * @brief  send命令：send <hexId> [hexByte]...
  */
static void Cmd_Send(char *args)
{
  uint8_t  data[CAN_PAYLOAD_LEN];
  uint8_t  len = 0U;
  long     id;
  char    *tok;

  if ((args == NULL) || (*args == '\0'))
  {
    Put("usage: send <id> [b0..b7], hex, e.g. send 321 11 22 33\r\n");
    return;
  }

  tok = strtok(args, " ");
  id  = strtol(tok, NULL, 16);
  if ((id < 0L) || (id > 0x7FFL))
  {
    Put("id range: 000..7FF\r\n");
    return;
  }

  while (((tok = strtok(NULL, " ")) != NULL) && (len < CAN_PAYLOAD_LEN))
  {
    long v = strtol(tok, NULL, 16);
    if ((v < 0L) || (v > 0xFFL))
    {
      Put("data byte range: 00..FF\r\n");
      return;
    }
    data[len++] = (uint8_t)v;
  }

  if (CAN_Send((uint32_t)id, data, len) == HAL_OK)
  {
    BSP_UART_Printf("sent %03lX len=%u\r\n", (unsigned long)id, len);
  }
  else
  {
    Put("send failed (tx fifo full?)\r\n");
  }
}

/**
  * @brief  sniff模式是否开启
  */
bool Console_SniffEnabled(void)
{
  return s_sniffOn;
}

/**
  * @brief  tick命令：以100ms间隔连续采样PE0电平30次（3秒=3个翻转周期，
  *         不可能漏采边沿），直接观察LED翻转是否存在
  */
static void Cmd_Tick(void)
{
  Put("sampling PE0 (bit0 of ODR) every 100ms x30:\r\n");
  for (uint8_t i = 0U; i < 30U; i++)
  {
    BSP_UART_Printf("%lu", (unsigned long)(GPIOE->ODR & 1UL));
    HAL_Delay(100U);
  }
  Put("\r\n");
}

/**
  * @brief  led命令：LED/GPIOE寄存器诊断（定位LED冻结问题）
  * @note   每次执行翻转LED1/LED2并打印关键寄存器：
  *         MODER（模式，输出=01）、ODR（输出锁存）、IDR（引脚实际电平）、
  *         RCC_AHB2ENR（bit4=GPIOE时钟）。MODER/ODR正常变化而LED不亮
  *         => 引脚/LED电气问题；MODER被改 => 有代码在动GPIOE。
  */
static void Cmd_Led(void)
{
  BSP_UART_Printf("before MODER=%08lX OTYPER=%08lX ODR=%08lX IDR=%08lX AHB2ENR=%08lX\r\n",
                  (unsigned long)GPIOE->MODER, (unsigned long)GPIOE->OTYPER,
                  (unsigned long)GPIOE->ODR,   (unsigned long)GPIOE->IDR,
                  (unsigned long)RCC->AHB2ENR);

  LED1_Toggle;
  LED2_Toggle;

  BSP_UART_Printf("after  MODER=%08lX OTYPER=%08lX ODR=%08lX IDR=%08lX (PE0/PE1应翻转)\r\n",
                  (unsigned long)GPIOE->MODER, (unsigned long)GPIOE->OTYPER,
                  (unsigned long)GPIOE->ODR,   (unsigned long)GPIOE->IDR);
}

/**
  * @brief  打印一帧接收报文（监视模式，格式供上位机解析）
  */
void Console_PrintFrame(const CAN_RxMsg *msg)
{
  BSP_UART_Printf("RX %lu %03lX %u",
                  (unsigned long)HAL_GetTick(),
                  (unsigned long)msg->id,
                  msg->len);

  for (uint8_t i = 0U; i < msg->len; i++)
  {
    BSP_UART_Printf(" %02X", msg->data[i]);
  }

  Put("\r\n");
}
