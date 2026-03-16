#ifndef __AS5600_H__
#define __AS5600_H__

#include "main.h"
#include "gpio.h"
#include "Delay.h"
#include <stdint.h>

#define AS5600_Delay()         Delay_us(1)

/**** 宏定义 ****/
 
/* AS5600寄存器地址*/
#define AS5600_ID_ADDR           (0x36)     /* IIC 从机地址*/
/* Only Read Reg */
#define AS5600_RAWAngleADDR_H   (0x0c)     /* Only low 4 bit */
#define AS5600_RAWAngleADDR_L   (0x0d)     /* original angle val */
#define AS5600_AngleADDR_H      (0x0E)     /* Only low 4 bit */
#define AS5600_AngleADDR_L      (0x0F)     /* Calculated angle val */
#define AS5600_STATUS            (0x0b)     /* Magnet status Reg */
#define AS5600_AGC               (0x1a)     /* Magnet status Reg */
#define AS5600_MAGNTUDE_H       (0x1b)     /* Only low 4 bit */
#define AS5600_MAGNTUDE_L       (0x1c)     /* Config Reg */
/* Only Write Reg*/
#define AS5600_BURN              (0xff)     /* Write EEPROM Reg */
/* R/W/P */
#define AS5600_ZMCO              (0x00)     /* Zero Magnetic Center offset */
#define AS5600_ZPOS_H            (0x01)     /* Only low 4 bit */
#define AS5600_ZPOS_L            (0x02)     /* Zero Postion */
#define AS5600_MPOS_H            (0x03)     /* Only low 4 bit */
#define AS5600_MPOS_L            (0x04)     /* Max Postion */
#define AS5600_MANG_H            (0x05)     /* Only low 4 bit */
#define AS5600_MANG_L            (0x06)     /* Max Angle */
#define AS5600_CONF_H            (0x07)     /* Only low 5 bit */
#define AS5600_CONF_L            (0x08)     /* Config Reg */
 
typedef enum /* 磁铁的状态*/
{
    MD =1,     // 检测到有磁铁且强度刚好
    ML =2,     // too weak
    MH =3,      // too strong

    MN =0       //未检测到磁铁

}MagnetStatus;


//AS5600状态结构体
typedef struct
{
    uint8_t as5600Status;   //设备是否存在 1为存在，0为不存在
    MagnetStatus MagStatus; //磁铁状态
    uint8_t BurnTime;       //编程次数
}AS5600_Status;


extern AS5600_Status AS5600_S;

/**** 函数 ****/
void AS5600_Init(void);
uint8_t AS5600_CheckDevice(void);
 
uint16_t AS5600_GetAngleData(void);
uint16_t AS5600_GetRAWAngleData(void);
float AS5600_GetAngle(void);
 
uint8_t   AS5600_GetMagnetStatus(void);
MagnetStatus AS5600_CheckMagnet(void);
uint8_t   AS5600_GetBurnTime(void);
 
/* 读写AS5600寄存器*/
void AS5600_WriteReg(uint16_t addr, uint8_t dat);
uint8_t   AS5600_ReadReg(uint16_t addr);

#endif