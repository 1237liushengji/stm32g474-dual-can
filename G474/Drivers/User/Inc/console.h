/**
  ******************************************************************************
  * @file    console.h
  * @brief   串口命令行控制台（运行期调测入口：统计/发帧/波特率切换/总线监视）
  *
  * 报文输出格式（总线监视模式，供PC上位机解析）：
  *   RX <毫秒时间戳> <ID十六进制3位> <DLC> <数据十六进制2位xN>
  *   例：RX 12345 321 8 01 A2 03 ...
  ******************************************************************************
  */
#ifndef __CONSOLE_H
#define __CONSOLE_H

#include <stdbool.h>
#include <stdint.h>
#include "can.h"

void Console_Init(void);
void Console_Task(void);
bool Console_SniffEnabled(void);
void Console_PrintFrame(const CAN_RxMsg *msg);

#endif /* __CONSOLE_H */
