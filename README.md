# STM32F407 + FreeRTOS ICM20602 姿态显示系统

> Git 仓库链接：https://github.com/wsxjr/freertos001

## 1. 项目简介

本项目基于 STM32F407VET6、FreeRTOS 和 HAL 库，实现以下功能：

- USART1 + DMA 不定长收发，收到一帧数据后按原字节回传；
- ICM20602 六轴 IMU 原始数据读取；
- 互补滤波解算 Roll、Pitch、Yaw；
- 0.96 寸 SSD1306 OLED 实时显示姿态角；
- 使用 FreeRTOS 划分串口、IMU、OLED 三个任务。

当前 Keil 编译结果：

```text
0 Error(s), 0 Warning(s)
```

## 2. 硬件清单与芯片型号

| 器件 | 型号/规格 | 数量 | 说明 |
|---|---|---|---|
| 主控 | STM32F407VET6 | 1 | LQFP100，168MHz |
| IMU | ICM20602 | 1 | 六轴加速度计 + 陀螺仪，SPI 接口 |
| OLED | 0.96 寸 SSD1306 | 1 | 128x64，I2C 接口 |
| USB-TTL | CH340/CP2102 等 | 1 | 3.3V 电平 |
| 下载器 | ST-Link V2 | 1 | SWD 接口 |
| 电源 | 3.3V | - | 所有模块共地 |

主控芯片：**STM32F407VET6**  
IMU：**ICM20602**  
OLED：**SSD1306 128x64**

## 3. 引脚接线表

### 3.1 ICM20602（SPI1）

| ICM20602 模块 | STM32F407 | 说明 |
|---|---|---|
| VCC | 3.3V | 不要接 5V |
| GND | GND | 必须共地 |
| CS | PA4 | 软件片选，空闲高 |
| SCL / SPC | PA5 | SPI1_SCK |
| SDO / SA0 | PA6 | SPI1_MISO |
| SDA / SDI | PA7 | SPI1_MOSI |

模块如有 SPI/I2C 选择跳线，必须选择 **SPI 模式**。

### 3.2 OLED（I2C2）

| OLED | STM32F407 | 说明 |
|---|---|---|
| VCC | 3.3V | 不要接 5V |
| GND | GND | 必须共地 |
| SCL | PB10 | I2C2_SCL |
| SDA | PB11 | I2C2_SDA |

### 3.3 USB-TTL（USART1）

| USB-TTL | STM32F407 | 说明 |
|---|---|---|
| TXD | PA10 | STM32 RX |
| RXD | PA9 | STM32 TX |
| GND | GND | 共地 |
| VCC | 不接 | 板子单独供电 |

### 3.4 ST-Link（SWD）

| ST-Link | STM32F407 |
|---|---|
| SWDIO | PA13 |
| SWCLK | PA14 |
| GND | GND |
| 3.3V | 可不接 |

## 4. CubeMX 配置说明

CubeMX 配置文件：`freertos001.ioc`

### 4.1 时钟

- 系统时钟：HSI + PLL，168MHz
- APB1：42MHz
- APB2：84MHz
- HAL 时基：TIM14

### 4.2 SPI1

- 模式：Master，Full-Duplex
- 数据位：8bit
- CPOL：Low
- CPHA：1 Edge（Mode 0）
- NSS：Software
- 默认波特率预分频：16（约 5.25MHz）
- PA4：GPIO Output，标签 `ICM_CS`，空闲高
- PA5：SPI1_SCK
- PA6：SPI1_MISO
- PA7：SPI1_MOSI

ICM20602 驱动初始化时会自动尝试 Mode 0/Mode 3 和不同速率，以提高接线兼容性。

### 4.3 USART1 + DMA

- 波特率：115200
- 数据位：8
- 停止位：1
- 校验：None
- PA9：USART1_TX
- PA10：USART1_RX
- DMA2 Stream2：USART1_RX，Normal 模式
- DMA2 Stream7：USART1_TX，Normal 模式
- USART1 中断优先级：5

### 4.4 I2C2

- PB10：I2C2_SCL
- PB11：I2C2_SDA
- 速率：400kHz
- 用于 SSD1306 OLED

> 说明：工程里仍保留了 I2C1（PB6/PB7）的初始化配置，但当前没有接器件，可以忽略；如不需要可在 CubeMX 中删除。

### 4.5 FreeRTOS

- CMSIS-RTOS2
- Tick：1000Hz
- FreeRTOS 堆：20480 字节
- 任务：UARTDMA、IMUTask、OLEDTask
- 队列：AngleQ，长度 1，元素类型 `Angles`

## 5. FreeRTOS 任务结构

| 任务 | 优先级 | 栈大小 | 作用 |
|---|---|---|---|
| UARTDMA | High（40） | 1536 字节 | 串口 DMA 回传 |
| IMUTask | AboveNormal（32） | 1536 字节 | ICM20602 采样和姿态解算 |
| OLEDTask | Low（8） | 1536 字节 | OLED 显示 |

数据流：

```text
ICM20602 --SPI1--> IMUTask --AngleQ--> OLEDTask --I2C2--> OLED

电脑 --USB-TTL--> USART1 RX DMA --> UARTDMA任务 --> USART1 TX DMA --> 电脑
```

`AngleQ` 只保留最新一帧姿态角，避免 OLED 刷新慢时积压旧数据。

## 6. 代码目录说明

```text
freertos001/
├─ Core/
│  ├─ Inc/
│  │  ├─ app.h              // Angles 结构体、IMU 状态
│  │  ├─ app_uart.h         // 串口任务接口
│  │  ├─ app_imu.h          // IMU 任务接口
│  │  ├─ app_oled.h         // OLED 任务接口
│  │  ├─ icm20602.h         // ICM20602 驱动接口
│  │  ├─ attitude.h         // 姿态解算接口
│  │  ├─ oled.h             // OLED 接口
│  │  ├─ oled_font.h        // 8x16 字模
│  │  ├─ spi.h              // SPI1 接口
│  │  ├─ usart.h            // USART1 接口
│  │  ├─ i2c.h              // I2C 接口
│  │  └─ FreeRTOSConfig.h   // FreeRTOS 配置
│  └─ Src/
│     ├─ main.c             // 系统时钟、外设初始化、启动 RTOS
│     ├─ freertos.c         // 任务、优先级、栈、队列
│     ├─ app_uart.c         // 串口 DMA 收发和回传
│     ├─ app_imu.c          // IMU 采样和姿态解算任务
│     ├─ app_oled.c         // OLED 显示任务
│     ├─ icm20602.c         // ICM20602 SPI 驱动
│     ├─ attitude.c         // 互补滤波姿态解算
│     ├─ oled.c             // SSD1306 OLED 驱动
│     ├─ spi.c              // SPI1 初始化
│     ├─ usart.c            // USART1 + DMA 配置
│     ├─ i2c.c              // I2C1/I2C2 配置
│     ├─ dma.c              // DMA 时钟和中断
│     ├─ gpio.c             // GPIO 时钟
│     └─ stm32f4xx_it.c     // 中断服务函数
├─ Drivers/                 // STM32F4 HAL 和 CMSIS
├─ Middlewares/             // FreeRTOS
├─ MDK-ARM/
│  ├─ freertos001.uvprojx   // Keil5 工程文件
│  ├─ startup_stm32f407xx.s // 启动文件
│  └─ RTE/                  // Keil 组件配置
├─ freertos001.ioc          // STM32CubeMX 配置
├─ Doxyfile                 // Doxygen 配置
├─ README.md                // 本文件
└─ .gitignore               // Git 忽略规则
```

## 7. 串口通信收发处理流程

### 7.1 接收

1. `StartUARTDMA()` 启动 `HAL_UARTEx_ReceiveToIdle_DMA()`；
2. USART1 通过 DMA2 Stream2 接收数据；
3. 一帧结束后触发 IDLE 中断，或者 DMA 缓冲区满触发传输完成；
4. `HAL_UARTEx_RxEventCallback()` 忽略半传输事件；
5. 将收到的数据复制到 `s_uart_rx_frame`；
6. 通过 `xQueueSendFromISR()` 放入帧队列；
7. 立即重新启动 IDLE + DMA 接收。

### 7.2 回传

1. `StartUARTDMA()` 从帧队列取出数据；
2. 调用 `HAL_UART_Transmit_DMA()`，通过 DMA2 Stream7 发送；
3. `HAL_UART_TxCpltCallback()` 释放发送完成信号量；
4. 发送的数据长度和内容与接收完全一致，不添加换行或协议头。

### 7.3 参数

- 接受 1~1024 字节的一帧数据；
- 帧队列深度：4；
- 支持连续多帧接收；
- 串口助手设置：115200、8N1。

## 8. IMU 型号、接口、校准与姿态解算

### 8.1 型号和接口

- 型号：ICM20602
- 接口：SPI1
- 引脚：PA4/PA5/PA6/PA7
- 量程：加速度 ±4g，陀螺 ±500°/s
- 采样率：约 100Hz

### 8.2 初始化

`ICM20602_Init()` 会：

1. 拉低 CS，等待上电稳定；
2. 依次尝试 SPI Mode 0/Mode 3 和不同速率；
3. 写 `I2C_IF` 锁定 SPI 模式；
4. 读取 `WHO_AM_I`，标准值为 `0x12`；
5. 配置电源管理、低通滤波、量程和采样率。

### 8.3 陀螺零偏校准

`ICM20602_CalibrateGyro(200)` 会在板子静止时采样约 1 秒，计算三轴陀螺零偏。

校准期间 OLED 显示：

```text
Calibrating...
Keep still
```

### 8.4 姿态解算

使用互补滤波：

- 加速度计计算 Roll/Pitch；
- 陀螺积分计算 Yaw；
- 滤波系数 `0.98`（陀螺权重）；
- 静止检测：三轴角速度小于 1.5°/s，且加速度模长接近 1g；
- 连续静止约 0.2 秒后，冻结 Yaw 积分并缓慢跟踪零偏；
- Yaw 积分死区：0.35°/s。

> 六轴 IMU 没有磁力计，Yaw 没有绝对航向基准，长时间仍会缓慢漂移，这是正常现象。静止检测和零偏跟踪只能减小漂移。

如果安装方向与实际方向相反，可以修改 `Core/Src/attitude.c` 顶部的：

```c
#define INVERT_PITCH_AXIS  0
#define INVERT_ROLL_AXIS   0
#define INVERT_YAW_AXIS    0
```

## 9. OLED 显示说明

- 型号：SSD1306，128x64
- 接口：I2C2，PB10/PB11
- 速率：400kHz
- 地址：0x3C/0x78，兼容 0x3D/0x7A
- 字模：8x16 ASCII
- 刷新率：约 10Hz

显示内容：

```text
IMU Attitude
Roll:  12.34°
Pitch: -5.67°
Yaw:   89.01°
```

OLED 驱动使用 1KB RAM 显存，先写显存，再按页刷新到屏幕。

## 10. 编译烧录步骤

### 10.1 编译

1. 用 Keil MDK5 打开：
   `MDK-ARM/freertos001.uvprojx`
2. 选择目标 `freertos001`；
3. 点击 Build（F7）；
4. 确认输出：
   `0 Error(s), 0 Warning(s)`

### 10.2 烧录

1. 连接 ST-Link：PA13/PA14/GND；
2. Keil 选择 ST-Link Debugger；
3. 点击 Download（F8）；
4. 也可以直接烧录：
   `MDK-ARM/freertos001/freertos001.hex`

### 10.3 运行测试

1. USB-TTL 连接 PA9/PA10/GND；
2. 串口助手设置 115200、8N1；
3. 发送任意内容，检查是否逐字节原样返回；
4. 上电后保持板子静止约 1 秒，等待 IMU 校准；
5. 校准完成后观察 OLED 上的 Roll/Pitch/Yaw。

## 11. 调试中的问题与处理

### 11.1 AngleQ 元素类型

`freertos.c` 创建 `AngleQ` 时需要知道队列元素类型，因此在 `app.h` 中定义 `Angles` 结构体，`freertos.c` 包含该头文件后使用 `sizeof(Angles)` 创建队列。

### 11.2 UART DMA 模式

USART1 的 RX/TX DMA 使用 Normal 模式。每帧结束后在 IDLE/TC 回调中重新调用 `HAL_UARTEx_ReceiveToIdle_DMA()`，避免 Circular 模式下回调长度累计，导致旧数据被重复回传。

### 11.3 ICM20602 SPI 配置

ICM20602 使用 SPI1，接线为 PA4=CS、PA5=SCK、PA6=MISO、PA7=MOSI。工程中启用了 `HAL_SPI_MODULE_ENABLED`，并把 `stm32f4xx_hal_spi.c` 加入 Keil 工程。CS 由软件控制，空闲保持高电平。

### 11.4 ICM20602 读不到 WHO_AM_I

ICM20602 对 SPI 模式和速率比较敏感。驱动会依次尝试 Mode 0/Mode 3 和不同预分频，并读取 `WHO_AM_I` 进行判断。如果读不到，重点检查：

- 模块是否切到 SPI 模式；
- CS、SCK、MISO、MOSI 是否接反；
- 模块是否共地、是否 3.3V 供电。

### 11.5 OLED 不显示

常见原因包括：

- I2C 地址不是 0x3C；
- SCL/SDA 接反；
- 模块没有共地；
- I2C2 没有配置成 400kHz。

OLED 驱动会自动尝试 0x78 和 0x7A 两个 8 位地址。

### 11.6 Yaw 漂移

ICM20602 是六轴 IMU，没有磁力计，Yaw 只能通过陀螺积分得到，长时间会缓慢漂移。工程中增加了静止检测、Yaw 积分死区和陀螺零偏跟踪，用于减小静止漂移，但不能完全消除。

## 12. 仓库地址

GitHub：https://github.com/wsxjr/freertos001

仓库包含完整源码、Keil5 工程、STM32CubeMX 配置文件、README 和 Doxyfile。
