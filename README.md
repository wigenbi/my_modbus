# my_modbus
modbus从机的实现(STM32F1平台_RTU模式)
最近工作中需要用到modbus通信，在查阅了相关资料后在stm32f1中实现了符合要求的modbus协议。因为我的主机只需对保持寄存器(RW)进行单个或多个寄存器的读写，所以只需要实现对0x03(读寄存器)、0x06(写单个寄存器)、0x10(写多个寄存器)这三个功能码的响应。
我们首先要知道modbus的命令帧结构如下：
| 从机地址 |功能码  | Data |CRC高字节|CRC低字节| 
|--|--|--|--|--|
|address|8Bit|N*8Bit|8Bit|8Bit|
 然后分析下modbus的工作流程：
 1、从机初始化
 2、主机发送命令帧
 3、从机开启定时器并解析收到的命令帧作出相应

将流程转化成如下代码：
```c
rs485收发这两个宏根据需要自行更改
#define RS485_RX  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_RESET)
#define RS485_TX  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_12, GPIO_PIN_SET)
#define BUF_MAXSIZE  100 串口收发缓冲区大小
#define REG_MAXSIZE  32	 只读寄存器缓冲区大小

#define undefined_func_code   0x01  //未定义功能码，询问中接收到的功能码是不可允许的操作
#define illegal_reg_address   0x02  //非法的寄存器数据地址，接收到的数据地址是不可允许的地址
#define illegal_reg_data   0x03     //非法数据值，寄存器中被提交存储的数据项有一个应用程序期望之外的值
#define slave_device_busy  0x04     //从设备忙，正在处理上一个请求

modbus所处状态声明
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
定义一个全局结构体modbus：
modbus_t modbus = { 0, mb_init_flag, {0}, {0}, {0}, 0, 0, 0 }; 
```

```c
 从机初始化函数：slave_addr（从机地址）、huart（收发命令帧的串口）、tim（处理定时器）、p_mb（指向modbus_t的全局变量）
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
        
        从机开启侦听，关闭发送
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
```

```c
命令帧处理函数：(放在对应的串口中断中)
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

定时器处理函数（放在定时器中断中）
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

命令帧解析以及相应发送
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
```
总结： 完整实现了要求，工作稳定。不足的是程序耦合度比较高，如果将流程转换成有限状态机，然后用表驱动的写法程序可移植性会好很多。
