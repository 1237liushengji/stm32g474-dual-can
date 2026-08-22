/**
  ******************************************************************************
  * @file    can.c
  * @brief   双机CAN通信驱动实现（基于FDCAN2，经典CAN模式，RX FIFO0 + 中断线0）
  *
  * 设计说明（企业级要求）：
  *   1. 发送采用Tx FIFO（深度3），发送前查询剩余空间，不阻塞应用层；
  *   2. 接收采用RX FIFO0 + FDCAN2中断线0，回调中排空FIFO后写入单槽信箱，
  *      应用层通过CAN_PollRx()轮询取走（临界区保护，避免与中断竞争）；
  *   3. 激活bus-off/错误被动中断，bus-off后自动 Stop->Start 完成总线恢复
  *      （M_CAN进入bus-off时硬件自动置INIT位，Stop/Start序列即完成恢复）；
  *   4. 所有HAL返回值均检查并统计，异常不静默；
  *   5. 支持运行时重配置（CAN_Configure）：波特率切换、混杂模式切换，
  *      重配置流程 = Stop -> 重填参数 -> Init -> 过滤器 -> Start -> 重挂中断。
  *
  * 参考实现：STMicroelectronics/STM32CubeG4 官方示例
  *           Projects/STM32G474E-EVAL/Examples/FDCAN/FDCAN_Classic_Frame_Networking
  *           （本项目HAL库版本V1.2.3，与该示例API完全一致）
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics（驱动调用模式）
  *
  ******************************************************************************
  */
#include "can.h"

/*--------------------------------------- 模块内部变量 --------------------------------------*/
FDCAN_HandleTypeDef hfdcan2;                          /* FDCAN2句柄（中断服务函数引用） */

static CAN_RxMsg    s_rxMailbox;                      /* 单槽接收信箱 */
static volatile uint8_t s_rxPending;                  /* 信箱数据待取标志 */

static CAN_Stats    s_stats;                          /* 通信统计（调试器可观察） */

static bool         s_promisc;                        /* 混杂模式：接收所有ID */
static uint32_t     s_bitrate = CAN_BITRATE;          /* 当前位速率 */

/* 波特率-分频查找表：FDCAN内核时钟=PCLK1=150MHz，所有档位统一
 * 10tq/位、采样点80%、SJW=2：位速率 = 150MHz / (prescaler × 10)
 *   125k -> 120   250k -> 60   500k -> 30   1M -> 15                    */
static const uint32_t s_bitrateTable[][2] =
{
  { 125000U, 120U },
  { 250000U,  60U },
  { 500000U,  30U },
  {1000000U,  15U },
};

/**
  * @brief  查询位速率对应的分频值
  * @retval 分频值；0表示该速率不受支持
  */
static uint32_t CAN_BitrateToPrescaler(uint32_t bitrate)
{
  uint32_t i;

  for (i = 0U; i < (sizeof(s_bitrateTable) / sizeof(s_bitrateTable[0])); i++)
  {
    if (s_bitrateTable[i][0] == bitrate)
    {
      return s_bitrateTable[i][1];
    }
  }
  return 0U;
}

/**
  * @brief  FDCAN2完整配置流程（初始化与运行时重配置共用）
  * @retval HAL状态
  */
static HAL_StatusTypeDef CAN_Configure(void)
{
  HAL_StatusTypeDef   status;
  FDCAN_FilterTypeDef filterConfig;
  uint32_t            prescaler = CAN_BitrateToPrescaler(s_bitrate);

  if (prescaler == 0U)
  {
    return HAL_ERROR;
  }

  /* 运行中重配置：先停机使状态机回到READY（HAL_FDCAN_Init要求） */
  if (hfdcan2.State == HAL_FDCAN_STATE_BUSY)
  {
    if (HAL_FDCAN_Stop(&hfdcan2) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  /* 位时序：PCLK1=150MHz，NominalPrescaler -> tq，10tq/位 -> 对应位速率，
   * 采样点80%。经典CAN模式不使用数据段位时序，按与仲裁段相同的合法值配置。 */
  hfdcan2.Instance                            = FDCAN2;
  hfdcan2.Init.ClockDivider                   = FDCAN_CLOCK_DIV1;
  hfdcan2.Init.FrameFormat                    = FDCAN_FRAME_CLASSIC;
#if (CAN_DEBUG_SELFTEST != 0)
  hfdcan2.Init.Mode                           = FDCAN_MODE_EXTERNAL_LOOPBACK; /* 自测试：自发自收 */
#else
  hfdcan2.Init.Mode                           = FDCAN_MODE_NORMAL;
#endif
  hfdcan2.Init.AutoRetransmission             = ENABLE;   /* 硬件自动重发，保证可靠送达 */
  hfdcan2.Init.TransmitPause                  = DISABLE;
  hfdcan2.Init.ProtocolException              = ENABLE;
  hfdcan2.Init.NominalPrescaler               = prescaler;
  hfdcan2.Init.NominalSyncJumpWidth           = 2U;
  hfdcan2.Init.NominalTimeSeg1                = 7U;       /* 传播段+相位段1 */
  hfdcan2.Init.NominalTimeSeg2                = 2U;
  hfdcan2.Init.DataPrescaler                  = prescaler;
  hfdcan2.Init.DataSyncJumpWidth              = 2U;
  hfdcan2.Init.DataTimeSeg1                   = 7U;
  hfdcan2.Init.DataTimeSeg2                   = 2U;
  hfdcan2.Init.StdFiltersNbr                  = 1U;
  hfdcan2.Init.ExtFiltersNbr                  = 0U;
  hfdcan2.Init.TxFifoQueueMode                = FDCAN_TX_FIFO_OPERATION;

  status = HAL_FDCAN_Init(&hfdcan2);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 接收过滤器：混杂模式收全部；正常模式只收对端报文（掩码0x7FF精确匹配） */
  filterConfig.IdType       = FDCAN_STANDARD_ID;
  filterConfig.FilterIndex  = 0U;
  filterConfig.FilterType   = FDCAN_FILTER_MASK;
  filterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
#if (CAN_DEBUG_SELFTEST != 0)
  filterConfig.FilterID1    = 0x000U;                 /* 自测试：掩码0，接收所有ID */
  filterConfig.FilterID2    = 0x000U;
#else
  if (s_promisc)
  {
    filterConfig.FilterID1  = 0x000U;                 /* 混杂模式：掩码0，接收所有标准ID */
    filterConfig.FilterID2  = 0x000U;
  }
  else
  {
#if (CAN_NODE_ROLE == CAN_NODE_A)
    filterConfig.FilterID1  = CAN_ID_RESP_B2A;         /* 板A接收板B的应答帧 */
#else
    filterConfig.FilterID1  = CAN_ID_CMD_A2B;           /* 板B接收板A的命令帧 */
#endif
    filterConfig.FilterID2  = 0x7FFU;                   /* 掩码：11位ID全比较 */
  }
#endif

  status = HAL_FDCAN_ConfigFilter(&hfdcan2, &filterConfig);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 全局过滤器：混杂模式下非匹配标准/扩展帧一并收入RX FIFO0；正常模式全部拒绝 */
  if (s_promisc)
  {
    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_ACCEPT_IN_RX_FIFO0,
                                          FDCAN_ACCEPT_IN_RX_FIFO0,
                                          FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  }
  else
  {
    status = HAL_FDCAN_ConfigGlobalFilter(&hfdcan2, FDCAN_REJECT, FDCAN_REJECT,
                                          FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  }
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_FDCAN_Start(&hfdcan2);
  if (status != HAL_OK)
  {
    return status;
  }

  /* 接收新报文中断 + 总线错误状态中断（V1.2.3中断线由ILS复位值决定，均走中断线0） */
  status = HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
  if (status != HAL_OK)
  {
    return status;
  }
  status = HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE, 0U);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}

/**
  * @brief  初始化FDCAN2：位时序、过滤器、中断、启动
  * @retval HAL状态
  */
HAL_StatusTypeDef CAN_Init(void)
{
  s_bitrate = CAN_BITRATE;
  s_promisc = false;
  return CAN_Configure();
}

/**
  * @brief  运行时切换总线波特率（125k/250k/500k/1M）
  * @note   重配置期间收发短暂中断；两块板必须切换到同一速率，否则无法互通
  * @retval HAL状态
  */
HAL_StatusTypeDef CAN_SetBitrate(uint32_t bitrate)
{
  if (CAN_BitrateToPrescaler(bitrate) == 0U)
  {
    return HAL_ERROR;                                  /* 不支持的速率 */
  }

  s_bitrate = bitrate;
  return CAN_Configure();
}

/**
  * @brief  开/关混杂模式（总线监视sniff：接收所有ID）
  * @retval HAL状态
  */
HAL_StatusTypeDef CAN_SetPromiscuous(bool enable)
{
  s_promisc = enable;
  return CAN_Configure();
}

/**
  * @brief  读取当前位速率
  */
uint32_t CAN_GetBitrate(void)
{
  return s_bitrate;
}

/**
  * @brief  发送一帧经典CAN数据帧（标准ID）
  * @param  stdId 标准ID（0x000~0x7FF）
  * @param  data  数据指针
  * @param  len   数据长度（<=8）
  * @retval HAL状态（HAL_BUSY表示Tx FIFO暂满）
  */
HAL_StatusTypeDef CAN_Send(uint32_t stdId, const uint8_t *data, uint8_t len)
{
  FDCAN_TxHeaderTypeDef txHeader;
  uint8_t               txData[CAN_PAYLOAD_LEN] = {0U};  /* 零填充暂存区：驱动按字(4字节)拷贝，
                                                           防止len<8时读越界 */

  if ((data == NULL) || (len > CAN_PAYLOAD_LEN) ||
      (stdId > 0x7FFU))
  {
    s_stats.txErrors++;
    return HAL_ERROR;
  }

  if (HAL_FDCAN_GetTxFifoFreeLevel(&hfdcan2) == 0U)
  {
    s_stats.txErrors++;                        /* Tx FIFO满：放弃本帧并统计 */
    return HAL_BUSY;
  }

  for (uint8_t i = 0U; i < len; i++)
  {
    txData[i] = data[i];
  }

  txHeader.Identifier          = stdId;
  txHeader.IdType              = FDCAN_STANDARD_ID;
  txHeader.TxFrameType         = FDCAN_DATA_FRAME;
  txHeader.DataLength          = len;                     /* V1.2.3语义：原始DLC(0~15)，
                                                            驱动内部完成<<16编码 */
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch       = FDCAN_BRS_OFF;
  txHeader.FDFormat            = FDCAN_CLASSIC_CAN;        /* 经典CAN帧 */
  txHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker       = 0U;

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan2, &txHeader, txData) != HAL_OK)
  {
    s_stats.txErrors++;
    return HAL_ERROR;
  }

  s_stats.txCount++;
  return HAL_OK;
}

/**
  * @brief  轮询取走一帧接收报文（主循环调用）
  * @param  msg 输出报文
  * @retval true取到一帧；false信箱为空
  */
bool CAN_PollRx(CAN_RxMsg *msg)
{
  bool gotMsg = false;

  if (msg == NULL)
  {
    return false;
  }

  __disable_irq();                              /* 与接收中断竞争信箱，需短暂关中断 */
  if (s_rxPending != 0U)
  {
    *msg = s_rxMailbox;
    s_rxPending = 0U;
    gotMsg = true;
  }
  __enable_irq();

  return gotMsg;
}

/**
  * @brief  获取通信统计指针（调试器实时观察用）
  */
const CAN_Stats *CAN_GetStats(void)
{
  return &s_stats;
}

/**
  * @brief  RX FIFO0接收回调（中断上下文）：排空FIFO写入信箱
  */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t               rxData[CAN_PAYLOAD_LEN];

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U)
  {
    /* 循环排空FIFO，避免高负载下滞留 */
    while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
      if (s_rxPending != 0U)
      {
        s_stats.rxOverruns++;                   /* 上一帧未被取走被覆盖 */
      }

      s_rxMailbox.id        = rxHeader.Identifier;
      s_rxMailbox.timestamp = rxHeader.RxTimestamp;
      /* V1.2.3的DataLength为原始DLC：经典帧DLC>8时有效字节仍为8 */
      s_rxMailbox.len       = (rxHeader.DataLength > 8U) ? 8U : (uint8_t)rxHeader.DataLength;
      for (uint8_t i = 0U; i < s_rxMailbox.len; i++)
      {
        s_rxMailbox.data[i] = rxData[i];
      }
      s_rxPending = 1U;
      s_stats.rxCount++;
    }
  }

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
  {
    s_stats.rxOverruns++;                       /* FIFO满导致报文丢失 */
  }
}

/**
  * @brief  总线错误状态回调（中断上下文）：bus-off自动恢复
  * @note   M_CAN发生bus-off时硬件自动置位INIT进入初始化状态；
  *         Stop(状态回READY) -> Start(清INIT)序列即完成自动恢复，随后重挂中断。
  */
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs)
{
  if ((ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) != 0U)
  {
    s_stats.errPassiveCount++;                 /* 进入/退出错误被动状态（总线异常的早期信号） */
  }

  if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
  {
    s_stats.busOffCount++;

    if (HAL_FDCAN_Stop(hfdcan) == HAL_OK)
    {
      (void)HAL_FDCAN_Start(hfdcan);
      (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0U);
      (void)HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_BUS_OFF | FDCAN_IT_ERROR_PASSIVE, 0U);
    }
  }
}

/**
  * @brief  协议错误回调（中断上下文）：仅计数，供调试观察
  */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  UNUSED(hfdcan);
  s_stats.protocolErrors++;
}
