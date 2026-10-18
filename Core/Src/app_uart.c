/**
 * @file app_uart.c
 * @brief 串口 DMA 收发和回传
 */
#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "usart.h"
#include "app_uart.h"

#include <string.h>

#define UART_RX_DMA_BUF_SIZE   1024U

#define UART_FRAME_SLOTS       4U

typedef struct
{
    uint16_t len;
    uint8_t data[UART_RX_DMA_BUF_SIZE];
} UartFrame;

static uint8_t s_uart_rx_dma_buf[UART_RX_DMA_BUF_SIZE];

static UartFrame s_uart_rx_frame;

static UartFrame s_uart_tx_frame;

static QueueHandle_t s_uart_frame_q = NULL;

static SemaphoreHandle_t s_uart_tx_done = NULL;

/* 接收回调：IDLE 或 DMA 完成时把数据放进队列 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    BaseType_t woken = pdFALSE;

    if (huart->Instance != USART1)
    {
        return;
    }

    /* 半传输事件不是一帧结束，忽略 */
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT)
    {
        return;
    }

    if (Size > UART_RX_DMA_BUF_SIZE)
    {
        Size = UART_RX_DMA_BUF_SIZE;
    }

    if ((Size > 0U) && (s_uart_frame_q != NULL))
    {
        s_uart_rx_frame.len = Size;
        memcpy(s_uart_rx_frame.data, s_uart_rx_dma_buf, Size);
        (void)xQueueSendFromISR(s_uart_frame_q, &s_uart_rx_frame, &woken);
    }

    /* Normal 模式下一帧结束，HAL 已把 RxState 置为 READY，立即重新挂接收 */
    if (huart->RxState == HAL_UART_STATE_READY)
    {
        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart_rx_dma_buf,
                                           UART_RX_DMA_BUF_SIZE);
    }

    portYIELD_FROM_ISR(woken);
}

/* 发送完成，释放信号量 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    BaseType_t woken = pdFALSE;

    if ((huart->Instance == USART1) && (s_uart_tx_done != NULL))
    {
        (void)xSemaphoreGiveFromISR(s_uart_tx_done, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/* 出错后重新挂接收 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);

        if (huart->RxState == HAL_UART_STATE_READY)
        {
            (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart_rx_dma_buf,
                                               UART_RX_DMA_BUF_SIZE);
        }
    }
}

/* 串口回传任务 */
void StartUARTDMA(void *argument)
{
    HAL_StatusTypeDef st;
    (void)argument;

    s_uart_frame_q = xQueueCreate(UART_FRAME_SLOTS, sizeof(UartFrame));
    s_uart_tx_done = xSemaphoreCreateBinary();

    if ((s_uart_frame_q == NULL) || (s_uart_tx_done == NULL))
    {
        for (;;)
        {
            osDelay(1000U);
        }
    }

    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart_rx_dma_buf,
                                       UART_RX_DMA_BUF_SIZE);

    for (;;)
    {
        /* 20ms 超时用于检查接收是否意外停止，并自动重新挂起 */
        if (xQueueReceive(s_uart_frame_q, &s_uart_tx_frame,
                          pdMS_TO_TICKS(20U)) == pdTRUE)
        {
            (void)xSemaphoreTake(s_uart_tx_done, 0U);

            do
            {
                st = HAL_UART_Transmit_DMA(&huart1, s_uart_tx_frame.data,
                                           s_uart_tx_frame.len);
                if (st == HAL_BUSY)
                {
                    osDelay(1U);
                }
            } while (st == HAL_BUSY);

            if (st == HAL_OK)
            {
                if (xSemaphoreTake(s_uart_tx_done,
                                   pdMS_TO_TICKS(2000U)) != pdTRUE)
                {
                    (void)HAL_UART_AbortTransmit(&huart1);
                }
            }
        }

        if (huart1.RxState == HAL_UART_STATE_READY)
        {
            (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_uart_rx_dma_buf,
                                               UART_RX_DMA_BUF_SIZE);
        }
    }
}


