#ifndef __USRATX_H
#define __USRATX_H 

#include "stdio.h"
#include "sys.h"
#include "system.h"

#define HEADER_0 0xA5
#define HEADER_1 0x5A
#define Length_ 0x6C

#define POINT_PER_PACK 32

typedef struct PointData
{
	uint8_t distance_h;
	uint8_t distance_l;
	uint8_t Strong;

}LidarPointStructDef;

typedef struct PackData
{
	uint8_t header_0;
	uint8_t header_1;
	uint8_t ver_len;
	
	uint8_t speed_h;
	uint8_t speed_l;
	uint8_t start_angle_h;
	uint8_t start_angle_l;	
	LidarPointStructDef point[POINT_PER_PACK];
	uint8_t end_angle_h;
	uint8_t end_angle_l;
	uint8_t crc;
}LiDARFrameTypeDef;

typedef struct PointDataProcess_
{
	uint16_t distance;
	float angle;
}PointDataProcessDef;

extern PointDataProcessDef PointDataProcess[1200];//����225������
extern LiDARFrameTypeDef Pack_Data;
extern PointDataProcessDef Dataprocess[1200];//����С�����ϡ����桢��ֱ�ߡ�ELE�״���ϵ��״�����

extern float Distance_KP,Distance_KD,Distance_KI;		//�������PID����
extern float Follow_KP,Follow_KD,Follow_KI;  //ת��PID
extern float Follow_KP_Akm,Follow_KD_Akm,Follow_KI_Akm;

extern float Diff_Along_Distance_KP,Diff_Along_Distance_KD,Diff_Along_Distance_KI;	//�������PID����
extern float Akm_Along_Distance_KP,Akm_Along_Distance_KD,Akm_Along_Distance_KI;	//�������PID����
extern float FourWheel_Along_Distance_KP,FourWheel_Along_Distance_KD,FourWheel_Along_Distance_KI;	//�������PID����
extern float Along_Distance_KP,Along_Distance_KD,Along_Distance_KI;		//�������PID����

extern volatile u8 OpenMV_Line_Available;
extern volatile u16 OpenMV_Median;
extern volatile u8 OpenMV_Lost;
extern volatile u8 OpenMV_Stop;
extern volatile u8 OpenMV_Light;
extern volatile u8 OpenMV_Light_State;

//void data_task(void *pvParameters);

void CAN_SEND(void);
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void OpenMV_USART3_Init(u32 bound);
void uart5_init(u32 bound);

int USART1_IRQHandler(void);
int USART2_IRQHandler(void);
int USART3_IRQHandler(void);
int UART5_IRQHandler(void);
void data_process(void);

float Vz_to_Akm_Angle(float Vx, float Vz);
float XYZ_Target_Speed_transition(u8 High,u8 Low);
void usart1_send(u8 data);
void usart2_send(u8 data);
void usart3_send(u8 data);
void usart5_send(u8 data);

//u8 Check_Sum(unsigned char Count_Number,unsigned char Mode);
u8 AT_Command_Capture(u8 uart_recv);
void _System_Reset_(u8 uart_recv);

#endif

