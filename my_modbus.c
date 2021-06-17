/*
 * @Description: 只支持modbus rtu模式从机通讯，从机地址为0x3a,波特率不建议大于38400
 * @Version: 
 * @Autor: wigen
 * @Date: 2021-05-21 16:36:49
 * @LastEditors: wigen
 * @LastEditTime: 2021-06-08 14:13:55
 */

#include "my_modbus.h"
#include "flash_in.h"

extern variable_t g_variable;
modbus_t modbus = { 0, mb_init_flag, {0}, {0}, {0}, 0, 0, 0 };
uint8_t mb_running_lock = 0;


int modbus_init(uint8_t slave_addr, UART_HandleTypeDef * huart, TIM_HandleTypeDef * tim, modbus_t * p_mb)
{   
    if(p_mb->work_flag == mb_init_flag)
    {
        if(slave_addr < 1 || slave_addr > 247)
            return -1; 

        p_mb->address = slave_addr;
        p_mb->sendbuf[1] = slave_addr;

        p_mb->frame_interval = 38500000/(huart->Init.BaudRate)/(tim->Init.Period);  
        if(p_mb->frame_interval < 1)
            p_mb->frame_interval = 1;
        

        RS485_RX;
        //__HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC | UART_IT_RXNE);
        __HAL_UART_DISABLE_IT(huart, UART_IT_TXE|UART_IT_TC);
        __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
				
				__HAL_TIM_DISABLE(tim);
				__HAL_TIM_SET_COUNTER(tim, 0);
				__HAL_TIM_ENABLE_IT(tim, TIM_IT_UPDATE);
				
        p_mb->frame_timeout_flag = 1;
        p_mb->work_flag = mb_idle_flag;
        return 0;
    }

    return -1;
}

void mb_recv_handler(UART_HandleTypeDef * huart, TIM_HandleTypeDef * tim, modbus_t * p_mb)
{
    uint8_t u8temp;
    
    if(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE))
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_RXNE);
        if(p_mb->work_flag == mb_idle_flag && p_mb->frame_timeout_flag == 1)
        {
            if(huart->Instance->DR == modbus.address) {
                p_mb->recvbuf[0] = 1;
                p_mb->recvbuf[p_mb->recvbuf[0]] = huart->Instance->DR;
                p_mb->work_flag = mb_recv_busy_flag;
                p_mb->frame_timeout_flag = 0;
							
                __HAL_TIM_DISABLE(tim);
                __HAL_TIM_SET_COUNTER(tim, 0);
                __HAL_TIM_ENABLE(tim);
							  
            }    
        }
        else if(p_mb->work_flag == mb_recv_busy_flag && p_mb->frame_timeout_flag == 0)
        {
            modbus.recvbuf[0]++;
            p_mb->recvbuf[p_mb->recvbuf[0]] = huart->Instance->DR;
			p_mb->tim_cnt = 0;
            __HAL_TIM_DISABLE(tim);
            __HAL_TIM_SET_COUNTER(tim, 0);
            __HAL_TIM_ENABLE(tim);
        }
        else if(p_mb->work_flag == mb_data_prasing_flag)
        {
            u8temp = huart->Instance->DR;
			UNUSED(u8temp);
        }
    }
    else if(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE))
    {
        huart->Instance->DR = (uint8_t)(*huart->pTxBuffPtr++ & 0x00FF);
        if(--huart->TxXferCount == 0) {
            __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);
            while (__HAL_UART_GET_FLAG(huart, UART_IT_TC) == SET);
            __HAL_UART_CLEAR_FLAG(huart, UART_FLAG_TC);

            p_mb->work_flag = mb_idle_flag;
            RS485_RX;
			memset(p_mb->sendbuf, 0, BUF_MAXSIZE);
			__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
        }   
    }
}




void mb_tim_handler(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart, TIM_HandleTypeDef * htim)
{
    //static uint8_t cnt = 0;
		__HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_UPDATE);
    if(p_mb->work_flag == mb_idle_flag)
    {   
        if(p_mb->frame_timeout_flag == 0)
            p_mb->tim_cnt++;
        if(p_mb->tim_cnt >= p_mb->frame_interval) {
            p_mb->frame_timeout_flag = 1;
            p_mb->tim_cnt = 0;
        }
    }
    else if(p_mb->work_flag == mb_recv_busy_flag)
    {
        if(p_mb->frame_timeout_flag == 0)
            p_mb->tim_cnt++;
        if(p_mb->tim_cnt >= p_mb->frame_interval) {
            p_mb->tim_cnt = 0;
            p_mb->frame_timeout_flag = 1;
            if(p_mb->recvbuf[0] >= 8)      
                mb_data_prase(p_mb, p_var, huart);
            else 
                error_handle(p_mb, illegal_reg_data, huart);
        }
    }
}


void mb_data_prase(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart)
{
    
    p_mb->work_flag = mb_data_prasing_flag;

    
    switch (p_mb->recvbuf[2])
    {
    case 0x03:
        read_holdregister(p_mb, p_var, huart);
        break;
    case 0x06:
        write_single_register(p_mb, p_var, huart);
        break;
    case 0x10:
        write_multi_register(p_mb, p_var, huart);
        break;
    default:
        error_handle(p_mb, undefined_func_code, huart);
        break;
    }
}


/** 0x03:读取保持寄存器连续块的内容
 * @param {modbus_t} modbus指针
 * @param {variable_t} 全局运行变量指针
 * @param {UART_HandleTypeDef} 使用的串口指针
 * @return {null}
 */
void read_holdregister(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart)
{
    uint16_t crc16_temp;
    uint16_t reg_addr;
    uint16_t reg_num;
    uint8_t index = 4;

    crc16_temp = ((p_mb->recvbuf[p_mb->recvbuf[0]]<<8)&0xFF00) | (p_mb->recvbuf[p_mb->recvbuf[0] - 1]);
    if(crc16_temp == CRC16(&(p_mb->recvbuf[1]), p_mb->recvbuf[0]-2)) 
    {
        
        reg_addr = ((p_mb->recvbuf[3]<<8) & 0xff00) | p_mb->recvbuf[4];
        reg_num = ((p_mb->recvbuf[5]<<8) & 0xff00) | p_mb->recvbuf[6];

        if(reg_addr + reg_num < REG_MAXSIZE && reg_num != 0) 
        {
			p_mb->sendbuf[1] = p_mb->address;
            p_mb->sendbuf[2] = 0x03;
            p_mb->sendbuf[3] = (uint8_t)reg_num*2;
            p_mb->sendbuf[0] = 3;

            p_mb->regbuf[0] = p_var->g_crtl_mode;
            p_mb->regbuf[1] = p_var->g_u32power;
            p_mb->regbuf[2] = p_var->g_u32start_freq;
            p_mb->regbuf[3] = p_var->g_u32end_freq;
            p_mb->regbuf[4] = mb_running_lock;
            p_mb->regbuf[5] = p_var->g_u32mannul_freq;
            p_mb->regbuf[6] = p_var->g_u32pid_i;
            p_mb->regbuf[7] = p_var->g_u32pid_p;
            while (reg_num--)
            {
                p_mb->sendbuf[index++] = p_mb->regbuf[reg_addr]>>8 & 0x00FF;
                p_mb->sendbuf[index++] = p_mb->regbuf[reg_addr] & 0x00FF;

                reg_addr++;
                p_mb->sendbuf[0] += 2; 
            }
            crc16_temp = CRC16(&(p_mb->sendbuf[1]), p_mb->sendbuf[0]);
            p_mb->sendbuf[p_mb->sendbuf[0]+1] = crc16_temp>>8 & 0x00ff;
            p_mb->sendbuf[p_mb->sendbuf[0]+2] = crc16_temp & 0x00ff;
            p_mb->sendbuf[0] += 2;
            mb_send_frame(p_mb, huart, p_mb->sendbuf[0]+2);
        }
        else 
            error_handle(p_mb, illegal_reg_address, huart);
    }
    else 
        error_handle(p_mb, illegal_reg_data, huart);
}


/** 0x06:写单个保持寄存器的内容
 * @param {modbus_t} modbus指针
 * @param {variable_t} 全局运行变量指针
 * @param {UART_HandleTypeDef} 使用的串口指针
 * @return {null}
 */
void write_single_register(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart)
{
    uint16_t crc16_temp;
    uint16_t reg_addr;
    uint16_t reg_data;
    uint8_t err_temp = 0;
	uint8_t i;
	
    crc16_temp = ((p_mb->recvbuf[p_mb->recvbuf[0]]<<8)&0xFF00) | (p_mb->recvbuf[p_mb->recvbuf[0] - 1]);
    if(crc16_temp == CRC16(&(p_mb->recvbuf[1]), p_mb->recvbuf[0]-2)) 
    {

        reg_addr = ((p_mb->recvbuf[3]<<8) & 0xff00) | p_mb->recvbuf[4];
        if(reg_addr < REG_MAXSIZE) 
        {
            reg_data = ((p_mb->recvbuf[5]<<8)&0xff00) | p_mb->recvbuf[6];
            switch (reg_addr)
            {
            case 0x00:
                if((reg_data == manual_set) || (reg_data == auto_search))
                    p_var->g_crtl_mode = (controlmode_t)reg_data;
                else    
                    err_temp = illegal_reg_data;
                break;
            case 0x01:
                if(g_variable.g_u32power <= Max_power)
                    g_variable.g_u32power = reg_data;
                else
                    err_temp = illegal_reg_data;
                break;
            case 0x02:
                if((g_variable.g_u32start_freq <= UINT16_MAX) && (g_variable.g_u32start_freq >= Min_freq))
                    g_variable.g_u32start_freq = reg_data;
                else
                    err_temp = illegal_reg_data;    
                break;
            case 0x03:
                if((g_variable.g_u32end_freq <= (g_variable.g_u32start_freq-1000)) && (g_variable.g_u32end_freq >= Min_freq))
                    g_variable.g_u32end_freq = reg_data;
                else
                    err_temp = illegal_reg_data;    
                break;
            case 0x04:
                if((mb_running_lock == 0) || (mb_running_lock == 1))
                    mb_running_lock = reg_data;
                else
                    err_temp = illegal_reg_data;    
                break;
            case 0x05:
                if((g_variable.g_u32mannul_freq <= UINT16_MAX) && (g_variable.g_u32mannul_freq >= Min_freq))
                    g_variable.g_u32mannul_freq = reg_data; 
                else
                    err_temp = illegal_reg_data;    
                break;
            case 0x06:
                    g_variable.g_u32pid_i = reg_data;   
                break;
            case 0x07:
                    g_variable.g_u32pid_p = reg_data;
                break;
            default:
                break;
            }
            if(err_temp != illegal_reg_data)
            {
				for(i=0; i<=p_mb->recvbuf[0]; i++)
                {
					p_mb->sendbuf[i] = p_mb->recvbuf[i];
                }
//                strncpy((char*)(p_mb->sendbuf), (char*)(p_mb->recvbuf), p_mb->recvbuf[0]+1);
                mb_send_frame(p_mb, huart, p_mb->sendbuf[0]+2);
            } else 
                error_handle(p_mb, illegal_reg_data, huart);   
        }
        else
            error_handle(p_mb, illegal_reg_address, huart);
    }
    else 
        error_handle(p_mb, illegal_reg_data, huart);
}

/** 0x10 一次写多个读写寄存器
 * @param {modbus_t} *
 * @param {variable_t} *
 * @param {UART_HandleTypeDef} *
 * @return {null}
 */
void write_multi_register(modbus_t * p_mb, variable_t * p_var, UART_HandleTypeDef * huart)
{
    uint16_t crc16_temp;
    uint16_t reg_addr;  //起始地址
    uint16_t reg_num;   //寄存器数量
    uint16_t data_byte; //数据字节数
    uint16_t reg_data;  //寄存器数据
    uint8_t err_temp=0, i, j;

    crc16_temp = ((p_mb->recvbuf[p_mb->recvbuf[0]]<<8)&0xFF00) | (p_mb->recvbuf[p_mb->recvbuf[0] - 1]);
    if(crc16_temp == CRC16(&(p_mb->recvbuf[1]), p_mb->recvbuf[0]-2)) 
    {
        reg_addr = ((p_mb->recvbuf[3]<<8)&0xff00) | (p_mb->recvbuf[4]);
        reg_num  = ((p_mb->recvbuf[5]<<8)&0xff00) | (p_mb->recvbuf[6]);
        data_byte  = p_mb->recvbuf[7];

        if((reg_addr + reg_num < REG_MAXSIZE) && (reg_num*2 == (p_mb->recvbuf[0]-9)) && (data_byte == (p_mb->recvbuf[0]-9)))
        {
            j = 8;
            for (i = reg_addr; i < reg_num ; i++)
            {
                reg_data = ((p_mb->recvbuf[j]<<8)&0xff00) | (p_mb->recvbuf[j+1]);
                j+=2;
                switch(i) 
                {
                case 0x00:
                if((reg_data == manual_set) || (reg_data == auto_search))
                    p_var->g_crtl_mode = (controlmode_t)reg_data;
                else    
                    err_temp = illegal_reg_data;
                    break;
                case 0x01:
                if(g_variable.g_u32power <= Max_power)
                    g_variable.g_u32power = reg_data;
                else
                    err_temp = illegal_reg_data;
                    break;
                case 0x02:
                    if((g_variable.g_u32start_freq <= UINT16_MAX) && (g_variable.g_u32start_freq >= Min_freq))
                        g_variable.g_u32start_freq = reg_data;
                    else
                        err_temp = illegal_reg_data;    
                    break;
                case 0x03:
                    if((g_variable.g_u32end_freq <= (g_variable.g_u32start_freq-1000)) && (g_variable.g_u32end_freq >= Min_freq))
                        g_variable.g_u32end_freq = reg_data;
                    else
                        err_temp = illegal_reg_data;    
                    break;
                case 0x04:
                    if((mb_running_lock == 0) || (mb_running_lock == 1))
                        mb_running_lock = reg_data;
                    else
                        err_temp = illegal_reg_data;    
                    break;
                case 0x05:
                    if((g_variable.g_u32mannul_freq <= UINT16_MAX) && (g_variable.g_u32mannul_freq >= Min_freq))
                        g_variable.g_u32mannul_freq = reg_data; 
                    else
                        err_temp = illegal_reg_data;    
                    break;
                case 0x06:
                        g_variable.g_u32pid_i = reg_data;   
                    break;
                case 0x07:
                        g_variable.g_u32pid_p = reg_data;
                    break;
                default:
                    break;
                }
            }
            if(err_temp != illegal_reg_data){
                p_mb->sendbuf[0] = 6;
				for(i=1; i<=6; i++)
                {
					p_mb->sendbuf[i] = p_mb->recvbuf[i];
                }
                //strncpy((char *)&(p_mb->sendbuf[1]), (char *)&(p_mb->recvbuf[1]), 6);
                crc16_temp = CRC16(&(p_mb->sendbuf[1]), p_mb->sendbuf[0]);
                p_mb->sendbuf[p_mb->sendbuf[0]+1] = crc16_temp>>8 & 0x00ff;
                p_mb->sendbuf[p_mb->sendbuf[0]+2] = crc16_temp & 0x00ff;
                p_mb->sendbuf[0] += 2;
                mb_send_frame(p_mb, huart, p_mb->sendbuf[0]+2);
            } else 
                error_handle(p_mb, illegal_reg_data, huart);
        }
        else
            error_handle(p_mb, illegal_reg_address, huart);
    }
    else
        error_handle(p_mb, illegal_reg_data, huart);
}

/** modbus错误处理函数
 * @param {modbus_t} mobus结构体指针
 * @param {uint8_t} err_code modbus错误码
 * @return {null}
 */
void error_handle(modbus_t * p_mb, uint8_t err_code, UART_HandleTypeDef * huart)
{
    uint16_t crc16_temp;
        p_mb->sendbuf[0] = 3;
        p_mb->sendbuf[1] = p_mb->address;
        p_mb->sendbuf[2] = p_mb->recvbuf[2] | 0x80;
        switch (err_code)
        {
        case undefined_func_code:
            p_mb->sendbuf[3] = undefined_func_code;
            break;
        case illegal_reg_address:
            p_mb->sendbuf[3] = illegal_reg_address;
            break;
        case illegal_reg_data:
            p_mb->sendbuf[3] = illegal_reg_data;
            break;
        case slave_device_busy:
            p_mb->sendbuf[3] = slave_device_busy;
            break;
        default:
            break;
        }
        crc16_temp = CRC16(&(p_mb->sendbuf[1]), p_mb->sendbuf[0]);
        p_mb->sendbuf[p_mb->sendbuf[0]+1] = crc16_temp>>8 & 0x00ff;
        p_mb->sendbuf[p_mb->sendbuf[0]+2] = crc16_temp & 0x00ff;
        p_mb->sendbuf[0] += 2;
    // memset(p_mb->recvbuf, 0, BUF_MAXSIZE);
	// memset(p_mb->sendbuf, 0, BUF_MAXSIZE);
	p_mb->work_flag = mb_idle_flag;
    mb_send_frame(p_mb, huart, p_mb->sendbuf[0]+2);
}

/** 回应帧或错误帧的发送
 * @param {modbus_t} modbus结构体指针
 * @param {UART_HandleTypeDef} 串口结构指针
 * @param {uint8_t} len 发送帧的长度
 * @return {null}
 */
void mb_send_frame(modbus_t * p_mb, UART_HandleTypeDef * huart, uint8_t len)
{
    if(len>BUF_MAXSIZE)
        return;

    __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    RS485_TX;
	huart->pTxBuffPtr = &(p_mb->sendbuf[1]);
    huart->TxXferSize = len;
    huart->TxXferCount = len;
    __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);

}
    

/*CRC高位字节值表*/
static uint8_t auchCRCHi[]={
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
	0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,
	0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
	0x81,0x40,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x01,0xC0,0x80,0x41,0x00,0xC1,0x81,0x40,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40,0x00,0xC1,0x81,0x40,
	0x01,0xC0,0x80,0x41,0x01,0xC0,0x80,0x41,0x00,0xC1,
	0x81,0x40,0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,
	0x00,0xC1,0x81,0x40,0x01,0xC0,0x80,0x41,0x01,0xC0,
	0x80,0x41,0x00,0xC1,0x81,0x40
	};
/*CRC低位字节值表*/
static uint8_t auchCRCLo[]={
	0x00,0xC0,0xC1,0x01,0xC3,0x03,0x02,0xC2,0xC6,0x06,
	0x07,0xC7,0x05,0xC5,0xC4,0x04,0xCC,0x0C,0x0D,0xCD,
	0x0F,0xCF,0xCE,0x0E,0x0A,0xCA,0xCB,0x0B,0xC9,0x09,
	0x08,0xC8,0xD8,0x18,0x19,0xD9,0x1B,0xDB,0xDA,0x1A,
	0x1E,0xDE,0xDF,0x1F,0xDD,0x1D,0x1C,0xDC,0x14,0xD4,
	0xD5,0x15,0xD7,0x17,0x16,0xD6,0xD2,0x12,0x13,0xD3,
	0x11,0xD1,0xD0,0x10,0xF0,0x30,0x31,0xF1,0x33,0xF3,
	0xF2,0x32,0x36,0xF6,0xF7,0x37,0xF5,0x35,0x34,0xF4,
	0x3C,0xFC,0xFD,0x3D,0xFF,0x3F,0x3E,0xFE,0xFA,0x3A,
	0x3B,0xFB,0x39,0xF9,0xF8,0x38,0x28,0xE8,0xE9,0x29,
	0xEB,0x2B,0x2A,0xEA,0xEE,0x2E,0x2F,0xEF,0x2D,0xED,
	0xEC,0x2C,0xE4,0x24,0x25,0xE5,0x27,0xE7,0xE6,0x26,
	0x22,0xE2,0xE3,0x23,0xE1,0x21,0x20,0xE0,0xA0,0x60,
	0x61,0xA1,0x63,0xA3,0xA2,0x62,0x66,0xA6,0xA7,0x67,
	0xA5,0x65,0x64,0xA4,0x6C,0xAC,0xAD,0x6D,0xAF,0x6F,
	0x6E,0xAE,0xAA,0x6A,0x6B,0xAB,0x69,0xA9,0xA8,0x68,
	0x78,0xB8,0xB9,0x79,0xBB,0x7B,0x7A,0xBA,0xBE,0x7E,
	0x7F,0xBF,0x7D,0xBD,0xBC,0x7C,0xB4,0x74,0x75,0xB5,
	0x77,0xB7,0xB6,0x76,0x72,0xB2,0xB3,0x73,0xB1,0x71,
	0x70,0xB0,0x50,0x90,0x91,0x51,0x93,0x53,0x52,0x92,
	0x96,0x56,0x57,0x97,0x55,0x95,0x94,0x54,0x9C,0x5C,
	0x5D,0x9D,0x5F,0x9F,0x9E,0x5E,0x5A,0x9A,0x9B,0x5B,
	0x99,0x59,0x58,0x98,0x88,0x48,0x49,0x89,0x4B,0x8B,
	0x8A,0x4A,0x4E,0x8E,0x8F,0x4F,0x8D,0x4D,0x4C,0x8C,
	0x44,0x84,0x85,0x45,0x87,0x47,0x46,0x86,0x82,0x42,
	0x43,0x83,0x41,0x81,0x80,0x40
	};

/**
 * @param {puchMsg} 用于计算 CRC 的报文
 * @param {usDataLen} 报文中的字节数 
 * @return CRC校验值
 */
unsigned short CRC16 (unsigned char * puchMsg, unsigned short usDataLen )
{
    unsigned char uchCRCHi = 0xFF ;  /* CRC 的高字节初始化 */
    unsigned char uchCRCLo = 0xFF ;  /* CRC 的低字节初始化 */
    unsigned uIndex ;  /* CRC 查询表索引 */
    while (usDataLen--)  /* 完成整个报文缓冲区 */
    {
        uIndex = uchCRCLo ^ *puchMsg++ ; /* 计算 CRC */
        uchCRCLo = uchCRCHi ^ auchCRCHi[uIndex] ;
        uchCRCHi = auchCRCLo[uIndex] ;
    }
    return (uchCRCHi << 8 | uchCRCLo);
}
