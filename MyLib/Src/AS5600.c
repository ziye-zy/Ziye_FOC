#include "AS5600.h"

AS5600_Status AS5600_S = {0};

void AS5600_W_SCL(uint8_t x)
{
	HAL_GPIO_WritePin(AS5600_SCL_GPIO_Port, AS5600_SCL_Pin, (GPIO_PinState)(x));
    AS5600_Delay();
}
 
void AS5600_W_SDA(uint8_t x)
{
	HAL_GPIO_WritePin(AS5600_SDA_GPIO_Port, AS5600_SDA_Pin, (GPIO_PinState)(x));
    AS5600_Delay();
}
 
uint8_t AS5600_R_SDA(void)
{
    uint8_t temp;
	temp = HAL_GPIO_ReadPin(AS5600_SDA_GPIO_Port, AS5600_SDA_Pin);
    AS5600_Delay();
    return temp;
}

/***********************************************************
*@fuction	: AS5600_IIC_Start
*@brief		: 软件iic起始信号
*@param		: None
*@return	: None
*@author	: HongScholar
*@date		: 2024.02.01
***********************************************************/
void AS5600_IIC_Start(void)
{    
	AS5600_W_SDA(1);
	AS5600_W_SCL(1);
	AS5600_W_SDA(0);
	AS5600_W_SCL(0);
}

/***********************************************************
*@fuction	: AS5600_IIC_Stop
*@brief		: 软件iic结束信号
*@param		: None
*@return	: None
*@author	: HongScholar
*@date		: 2024.02.01
***********************************************************/
void AS5600_IIC_Stop(void)
{    
	AS5600_W_SDA(0);
	AS5600_W_SCL(1);
	AS5600_W_SDA(1);
}

/***********************************************************
*@fuction	: AS5600_IIC_SendAck
*@brief		: 软件IIC发送应答位
*@param		: 应答位 0:应答;  1:无应答
*@return	: None
*@author	: HongScholar
*@date		: 2024.02.01
***********************************************************/
void AS5600_IIC_SendAck(uint8_t AckBit)
{
    AS5600_W_SDA(AckBit);
    AS5600_W_SCL(1);
    AS5600_W_SCL(0);
}
 
/***********************************************************
*@fuction	: AS5600_IIC_WaitAck
*@brief		: 软件IIC等待应答位
*@param		: None
*@return	: 应答位, 0:应答;  1:无应答
*@author	: HongScholar
*@date		: 2024.02.01
***********************************************************/
uint8_t AS5600_IIC_WaitAck(void)
{
    uint8_t AckBit = 1;
    
	AS5600_W_SDA(1);
	AS5600_W_SCL(1);
	AckBit = AS5600_R_SDA();
	AS5600_W_SCL(0);
    return AckBit;
}
 
/***********************************************************
*@fuction	: AS5600_IIC_SendByte
*@brief		: 软件IIC发送一个字节
*@param		: 数据
*@return	: None
*@author	: HongScholar
*@date		: 2024.02.01
***********************************************************/
void AS5600_IIC_SendByte(uint8_t Byte)
{
	uint8_t i;
 
	for (i = 0; i < 8; i++)
	{
		AS5600_W_SDA(Byte & (0x80 >> i));
		AS5600_W_SCL(1);
		AS5600_W_SCL(0);
	}
}
 
/***********************************************************
*@fuction	: AS5600_IIC_ReceiveByte
*@brief		: 软件IIC接收一个字节并发送ack
*@param		: 应答位
*@return	: 数据
*@author	: HongScholar
*@date		: 2024.02.01
***********************************************************/
uint8_t AS5600_IIC_ReceiveByte(uint8_t Ack)
{
    uint8_t dat = 0;
    
    for(uint8_t i=0; i<8; i++)
    {
        AS5600_W_SCL(0);
        AS5600_W_SCL(1);
        
        dat <<= 1;
        if(AS5600_R_SDA())
            dat |= 0x01;
    }
    
    AS5600_IIC_SendAck(Ack);
    
    return  dat;
}
 
/***********************************************************
*@fuction	: AS5600_WriteReg
*@brief		: 写编码器AS5600的寄存器
*@param		: 地址, 数据
*@return	: None
*@author	: HongScholar
*@date		: 2025.03.02
***********************************************************/
void AS5600_WriteReg(uint16_t addr, uint8_t dat)
{
	AS5600_IIC_Start();
    // 发送写命令  最后一位是读写位，1: 读，0: 写
	AS5600_IIC_SendByte((AS5600_ID_ADDR<<1) & 0xfe);
	AS5600_IIC_WaitAck();	
	AS5600_IIC_SendByte(addr);		
	AS5600_IIC_WaitAck();
    
	AS5600_IIC_Start();
	AS5600_IIC_SendByte(dat);		
	AS5600_IIC_WaitAck();	
	AS5600_IIC_Stop();
    
	AS5600_Delay();
}
 
 
/***********************************************************
*@fuction	: AS5600_ReadReg
*@brief		: 读编码器AS5600的寄存器
*@param		: 地址
*@return	: 数据
*@author	: HongScholar
*@date		: 2025.03.02
***********************************************************/
uint8_t AS5600_ReadReg(uint16_t addr)
{
    uint8_t	dat = 0;
    
	AS5600_IIC_Start();
    // 发送写命令  最后一位是读写位，1是读，0是写
	AS5600_IIC_SendByte((AS5600_ID_ADDR<<1) & 0xfe);
	AS5600_IIC_WaitAck();
	AS5600_IIC_SendByte(addr);
	AS5600_IIC_WaitAck();
    
    AS5600_IIC_Start();
	AS5600_IIC_SendByte((AS5600_ID_ADDR<<1) | 0x01);
	AS5600_IIC_WaitAck();
	dat = AS5600_IIC_ReceiveByte(0);
	AS5600_IIC_Stop();
    
	return dat;
}
 
 
/***********************************************************
*@fuction	: AS5600_Check
*@brief		: 检测AS5600是否存在
*@param		: None
*@return	: 1: 存在
              0: 从机设备连接失败
*@author	: HongScholar
*@date		: 2024.02.01
***********************************************************/
uint8_t AS5600_CheckDevice(void)
{
    uint8_t ack;
    AS5600_IIC_Start();
    AS5600_IIC_SendByte(0x36<<1);
    ack = AS5600_IIC_WaitAck();
    AS5600_IIC_Stop();
    return !ack;
}
 
 
/***********************************************************
*@fuction	: AS5600_GetBurnTime
*@brief		: 获取 AS5600 的编程次数, 最多三次
*@param		: None
*@return	: 次数
*@author	: HongScholar
*@date		: 2025.03.02
***********************************************************/
uint8_t AS5600_GetBurnTime(void)
{
    return AS5600_ReadReg (AS5600_ZMCO);
}
 
 
/***********************************************************
*@fuction	: AS5600_GetMagnetStatus
*@brief		: 获取 AS5600 的磁环状态寄存器的数据
*@param		: None
*@return	: 磁铁状态
*@author	: HongScholar
*@date		: 2025.03.02
***********************************************************/
uint8_t AS5600_GetMagnetStatus(void)
{
    return AS5600_ReadReg (AS5600_STATUS);
}
 
 
/***********************************************************
*@fuction	: AS5600_CheckMagnet
*@brief		: 检查是否存在磁铁和磁场强弱
*@param		: None
*@return	: 磁铁状态
*@author	: HongScholar
*@date		: 2025.03.02
***********************************************************/
MagnetStatus AS5600_CheckMagnet(void)
{
    uint8_t status;
    status = AS5600_ReadReg(AS5600_STATUS);
    if(status & 0x20)
        return MD;  // 存在 且磁场刚好
    if(status&0x10)
        return ML;  // 磁场太弱
    if(status&0x08)
        return MH;  // 太强
    
    return 0;
}
 
 
/***********************************************************
*@fuction	: AS5600_GetRAWAngleData
*@brief		: 读取编码器角度值
*@param		: None
*@return	: 角度原始数据, 0-4096
*@author	: HongScholar
*@date		: 2025.03.02
***********************************************************/
uint16_t AS5600_GetRAWAngleData(void)
{
    uint16_t RAW_angle= -1;
    RAW_angle = AS5600_ReadReg(AS5600_RAWAngleADDR_H);
    RAW_angle <<= 8;
    RAW_angle |= AS5600_ReadReg(AS5600_RAWAngleADDR_L);
    return RAW_angle;
}
 
 
/***********************************************************
*@fuction	: AS5600_GetAngleData
*@brief		: 读取编码器角度原始数据
*@param		: None
*@return	: 角度, 0-4096
*@author	: HongScholar
*@date		: 2025.03.02
***********************************************************/
uint16_t AS5600_GetAngleData(void)
{
    uint16_t angle;
    angle = AS5600_ReadReg(AS5600_AngleADDR_H);
    angle <<= 8;
    angle |= AS5600_ReadReg(AS5600_AngleADDR_L);
//    angle = angle*360 /4096;
    return angle;
}
 
 
/***********************************************************
*@fuction	: AS5600_GetAngle
*@brief		: 读取编码器角度
*@param		: None
*@return	: 角度, 0-360, 精度是 360/4096 = 
*@author	: HongScholar
*@date		: 2025.03.02
***********************************************************/
float AS5600_GetAngle(void)
{
    return AS5600_GetRAWAngleData()/4096.0000000 *360;
}
 
 
/***********************************************************
*@fuction	: AS5600_Init
*@brief		: 编码器AS5600的初始化
*@param		: None
*@return	: None
*@author	: HongScholar
*@date		: 2025.03.02
***********************************************************/
void AS5600_Init(void)
{
    AS5600_S.as5600Status = AS5600_CheckDevice();
    AS5600_S.BurnTime = AS5600_GetBurnTime();
    AS5600_S.MagStatus = AS5600_CheckMagnet();
}