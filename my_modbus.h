/*
 * @Description: 
 * @Version: 
 * @Autor: wigen
 * @Date: 2021-05-21 16:37:24
 * @LastEditors: wigen
 * @LastEditTime: 2021-06-08 11:19:46
 */
#ifndef __MY_MODBUS_H
#define __MY_MODBUS_H

#include "main.h"

#define RS485_RX  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define RS485_TX  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define BUF_MAXSIZE  100
#define REG_MAXSIZE  32

#define undefined_func_code   0x01  //未定义功能码，询问中接收到的功能码是不可允许的操作
#define illegal_reg_address   0x02  //非法的寄存器数据地址，接收到的数据地址是不可允许的地址
#define illegal_reg_data   0x03     //非法数据值，寄存器中被提交存储的数据项有一个应用程序期望之外的值
#define slave_device_busy  0x04     //从设备忙，正在处理上一个请求

typedef enum {
    mb_init_flag = 0,
    mb_idle_flag,
    mb_recv_busy_flag,
    mb_data_prasing_flag
}   mb_work_flag_t;




typedef struct {
    uint8_t address;                //本机作为从机的地址
    mb_work_flag_t work_flag;       //工作状态
    uint8_t recvbuf[BUF_MAXSIZE];    //接收缓存区
    uint8_t sendbuf[BUF_MAXSIZE];    //发送缓存区
    uint32_t regbuf[REG_MAXSIZE];     //寄存器缓存区

    uint8_t  frame_timeout_flag;    //帧间隔超时状态 大于 3.5个字符 = frame_interval(us)
    uint32_t frame_interval;        //定时器溢出中断时间
	uint8_t  tim_cnt;               //tim_cnt*定时器溢出中断时间 = 3.5个字符时间

} modbus_t;

int modbus_init(uint8_t slave_addr, UART_HandleTypeDef * huart, TIM_HandleTypeDef * tim, modbus_t * p_mb);
void mb_recv_handler(UART_HandleTypeDef * huart, TIM_HandleTypeDef * tim, modbus_t * p_mb);
void mb_tim_handler(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart, TIM_HandleTypeDef * htim);
void mb_data_prase(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart);
void read_holdregister(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart);
void write_single_register(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart);
void write_multi_register(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart);
void error_handle(modbus_t * p_mb, uint8_t err_code, UART_HandleTypeDef * huart);
void mb_send_frame(modbus_t * p_mb, UART_HandleTypeDef * huart, uint8_t len);
unsigned short CRC16 (unsigned char * puchMsg, unsigned short usDataLen );
#endif
