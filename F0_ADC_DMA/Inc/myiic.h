#ifndef _MYIIC_H
#define _MYIIC_H
#include "sys.h"

//IO��������
#define SDA_IN()  {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;} //PB7����ģʽ
#define SDA_OUT() {GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;} //PB7���ģʽ

//IO����
#define GPIOB_SET(PIN)              (GPIOB->BSRR = (PIN))
#define GPIOB_RST(PIN)              (GPIOB->BSRR = (((uint32_t)(PIN)) << 16U))
#define GPIOB_INPUT(PIN)            ((GPIOB->IDR & (PIN)) == (PIN))

#define IIC_SCL_H   GPIOB_SET(6) //SCL
#define IIC_SCL_L   GPIOB_RST(6) //SCL
#define IIC_SDA_H   GPIOB_SET(7) //SDA
#define IIC_SDA_L   GPIOB_SET(7) //SDA

#define READ_SDA    GPIOB_INPUT(7)  //����SDA

//IIC���в�������
void IIC_Init(void);                //��ʼ��IIC��IO��
void IIC_Start(void);               //����IIC��ʼ�ź�
void IIC_Stop(void);                //����IICֹͣ�ź�
void IIC_Send_Byte(u8 txd);         //IIC����һ���ֽ�
u8 IIC_Read_Byte(unsigned char ack);//IIC��ȡһ���ֽ�
u8 IIC_Wait_Ack(void);              //IIC�ȴ�ACK�ź�
void IIC_Ack(void);                 //IIC����ACK�ź�
void IIC_NAck(void);                //IIC������ACK�ź�

void IIC_Write_One_Byte(u8 daddr, u8 addr, u8 data);
u8 IIC_Read_One_Byte(u8 daddr, u8 addr);
#endif

