#include "myiic.h"
#include "delay.h"

//IIC��ʼ��
void IIC_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;

    __HAL_RCC_GPIOB_CLK_ENABLE();   //ʹ��GPIOBʱ��

    //PH4,5��ʼ������
    GPIO_Initure.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_Initure.Mode = GPIO_MODE_OUTPUT_PP; //�������
    GPIO_Initure.Pull = GPIO_PULLUP;        //����
    GPIO_Initure.Speed = GPIO_SPEED_FREQ_HIGH;   //����
    HAL_GPIO_Init(GPIOB, &GPIO_Initure);

    IIC_SDA_H;
    IIC_SCL_H;
}

//����IIC��ʼ�ź�
void IIC_Start(void)
{
    SDA_OUT();     //sda�����
    IIC_SDA_H;
    IIC_SCL_H;
    delay_us(4);
    IIC_SDA_L; //START:when CLK is high,DATA change form high to low
    delay_us(4);
    IIC_SCL_L; //ǯסI2C���ߣ�׼�����ͻ��������
}
//����IICֹͣ�ź�
void IIC_Stop(void)
{
    SDA_OUT();//sda�����
    IIC_SCL_L;
    IIC_SDA_L; //STOP:when CLK is high DATA change form low to high
    delay_us(4);
    IIC_SCL_H;
    IIC_SDA_H; //����I2C���߽����ź�
    delay_us(4);
}
//�ȴ�Ӧ���źŵ���
//����ֵ��1������Ӧ��ʧ��
//        0������Ӧ��ɹ�
u8 IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    SDA_IN();      //SDA����Ϊ����
    IIC_SDA_H; delay_us(1);
    IIC_SCL_H; delay_us(1);
    while (READ_SDA)
    {
        ucErrTime++;
        if (ucErrTime > 250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL_L; //ʱ�����0
    return 0;
}
//����ACKӦ��
void IIC_Ack(void)
{
    IIC_SCL_L;
    SDA_OUT();
    IIC_SDA_L;
    delay_us(2);
    IIC_SCL_H;
    delay_us(2);
    IIC_SCL_L;
}
//������ACKӦ��
void IIC_NAck(void)
{
    IIC_SCL_L;
    SDA_OUT();
    IIC_SDA_H;
    delay_us(2);
    IIC_SCL_H;
    delay_us(2);
    IIC_SCL_L;
}
//IIC����һ���ֽ�
//���شӻ�����Ӧ��
//1����Ӧ��
//0����Ӧ��
void IIC_Send_Byte(u8 txd)
{
    u8 t;
    SDA_OUT();
    IIC_SCL_L; //����ʱ�ӿ�ʼ���ݴ���
    for (t = 0; t < 8; t++)
    {
        if (1 == (txd & 0x80) >> 7)
        {
            IIC_SDA_H;
        }
        else
        {
            IIC_SDA_L;
        }
        // IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        delay_us(2);   //��TEA5767��������ʱ���Ǳ����
        IIC_SCL_H;
        delay_us(2);
        IIC_SCL_L;
        delay_us(2);
    }
}
//��1���ֽڣ�ack=1ʱ������ACK��ack=0������nACK
u8 IIC_Read_Byte(unsigned char ack)
{
    unsigned char i, receive = 0;
    SDA_IN();//SDA����Ϊ����
    for (i = 0; i < 8; i++ )
    {
        IIC_SCL_L;
        delay_us(2);
        IIC_SCL_H;
        receive <<= 1;
        if (READ_SDA)receive++;
        delay_us(1);
    }
    if (!ack)
        IIC_NAck();//����nACK
    else
        IIC_Ack(); //����ACK
    return receive;
}


