#ifndef __BSP_DRIVER_H
#define __BSP_DRIVER_H

#include "main.h"
#include "xl32f003xx_ll_Start_Kit.h"
#include "stdint.h"
/* 串口 */
#define UART_TX_PORT GPIOA
#define UART_TX_PIN LL_GPIO_PIN_2
#define UART_RX_PORT GPIOA
#define UART_RX_PIN LL_GPIO_PIN_1

#define UART_IDLE_Timeout 50 // 50 * 100us = 5ms
typedef struct
{
	uint8_t busy;	  // 串口忙
	uint8_t IDLE;	  // 串口空闲-0:空闲
	uint8_t has_data; // 串口一帧数据接收完成

	uint8_t rx_buff[256];
	uint8_t rx_len;
} uart_state_t;
extern uart_state_t uart_state;

/* 呼叫铃 */
#define CallBell_Data_PORT GPIOA
#define CallBell_Data_PIN LL_GPIO_PIN_0
#define CallBell_Enable_PORT GPIOB
#define CallBell_Enable_PIN LL_GPIO_PIN_3

/* M1卡 */
#define Soft_SPI_MISO_PORT GPIOA
#define Soft_SPI_MISO_PIN LL_GPIO_PIN_5
#define Soft_SPI_MOSI_PORT GPIOA
#define Soft_SPI_MOSI_PIN LL_GPIO_PIN_6
#define Soft_SPI_SCK_PORT GPIOA
#define Soft_SPI_SCK_PIN LL_GPIO_PIN_7
#define Soft_SPI_CS_PORT GPIOB
#define Soft_SPI_CS_PIN LL_GPIO_PIN_1
#define Soft_SPI_RST_PORT GPIOB
#define Soft_SPI_RST_PIN LL_GPIO_PIN_2
/* ID卡 */
#define ID_PWM_PORT GPIOA
#define ID_PWM_PIN LL_GPIO_PIN_3
#define ID_DATA_PORT GPIOA
#define ID_DATA_PIN LL_GPIO_PIN_4
#define ID_RST_PORT GPIOB
#define ID_RST_PIN LL_GPIO_PIN_2

typedef enum
{
	ID_IDLE = 0,
	ID_START,
	ID_DATA,
} ID_Card_State_t;

typedef struct
{
	uint8_t ID[16]; // 64位数据
	uint8_t bit;	// 当前bit num
	uint8_t value;
	uint8_t count;
	ID_Card_State_t state;
	uint8_t time[256];
	uint8_t time_index;
} ID_Card_t;
extern volatile ID_Card_t ID_Card;

typedef struct
{
	/* data */
	uint8_t ID[5];
	uint8_t has_card;
} ID_Card_Info_t;
extern volatile ID_Card_Info_t ID_Card_Info;
extern volatile ID_Card_Info_t M1_Card_Info;

// 定义不同呼叫铃的类型
typedef enum
{
	RF433_OLD,
	RF433_NEW,
	// 可以在此添加更多的类型
} BellType;

// 定义呼叫铃参数结构体
typedef struct
{
	uint16_t startMin;
	uint16_t startMax;
	uint8_t lowMin;
	uint8_t lowMax;
	uint8_t highMin;
	uint8_t highMax;
} BellParameters;
typedef struct
{
	uint8_t id[3];
	uint8_t bit;
	uint8_t isBusy;
	BellType type;
	uint16_t lowLevelCount; // 低电平持续时间
	uint8_t syncFlag;		// 同步标志
	uint8_t receiveOk;		// 接收成功标志
	uint16_t NeedSample;	// 准备采样
	uint16_t time[256];
	uint8_t time_index;
} RF433_t;
extern volatile RF433_t RF433;

typedef struct
{
	/* data */
	uint8_t ID[3];
	uint8_t has_data;
	BellType type;
} RF433_Info_t;
extern volatile RF433_Info_t RF433_Info;

void gpio_init(void);
void uart2_init(void);
void uart_send_data(uint8_t *data, uint8_t len);

// timer1 1ms定时
extern volatile uint32_t sys_ms;
void timer1_init(void);

// 看门狗
void IWDG_Config(void);
#endif //
