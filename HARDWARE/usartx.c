#include "usartx.h"

PointDataProcessDef PointDataProcess[1200];//����225������
LiDARFrameTypeDef Pack_Data;
PointDataProcessDef Dataprocess[1200];      //����С�����ϡ����桢��ֱ�ߡ�ELE�״���ϵ��״�����

float Diff_Along_Distance_KP = -0.030f,Diff_Along_Distance_KD = -0.245f,Diff_Along_Distance_KI = -0.001f;	//����С����ֱ�߾������PID����
float Akm_Along_Distance_KP = -0.115f*1000,Akm_Along_Distance_KD = -1000.245f*1000,Akm_Along_Distance_KI = -0.001f*1000;	//��������ֱ�߾������PID����
float FourWheel_Along_Distance_KP = -0.115f*1000,FourWheel_Along_Distance_KD = -100.200f*1000,FourWheel_Along_Distance_KI = -0.001f*1000;	//������ֱ�߾������PID����
float Along_Distance_KP = 0.163f*1000,Along_Distance_KD = 0.123*1000,Along_Distance_KI = 0.001f*1000;	//���ֺ�ȫ����ֱ�߾������PID����

float Distance_KP = -0.150f,Distance_KD = -0.052f,Distance_KI = -0.001;	//����������PID����

float Follow_KP = -2.566f,Follow_KD = -1.368f,Follow_KI = -0.001f;  //ת��PID
float Follow_KP_Akm = -0.550f,Follow_KD_Akm = -0.121f,Follow_KI_Akm = -0.001f;
volatile u8 OpenMV_Line_Available=0;
volatile u16 OpenMV_Median=64;
volatile u8 OpenMV_Lost=1;
volatile u8 OpenMV_Stop=0;
volatile u8 OpenMV_Light=0; // 0=none, 1=red, 2=green (from $OMV frame)
volatile u8 OpenMV_Light_State='N';

static void OpenMV_Parse_Byte(u8 data);
static void OpenMV_Parse_Line(char *line);
static int OpenMV_Parse_Number(char **cursor,int *value);

/**************************************************************************
Function: Serial port 1 initialization
Input   : none
Output  : none
�������ܣ�����1��ʼ��
��ڲ�������
�� �� ֵ����
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	 //Enable the gpio clock //ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1);	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10 ,GPIO_AF_USART1);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9|GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOA, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���	
	NVIC_Init(&NVIC_InitStructure);	
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure); //Initialize serial port 1 //��ʼ������1
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                     //Enable serial port 1 //ʹ�ܴ���1
}
/**************************************************************************
Function: Serial port 2 initialization
Input   : none
Output  : none
�������ܣ�����2��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��
	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource5,GPIO_AF_USART2);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource6 ,GPIO_AF_USART2);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
	//UsartNVIC configuration //UsartNVIC����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;
	//Subpriority //�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
  //Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);	
	
	//USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);      //Initialize serial port 2 //��ʼ������2
	
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                     //Enable serial port 2 //ʹ�ܴ���2 
}
/**************************************************************************
Function: Serial port 3 initialization for OpenMV
Input   : none
Output  : none
�������ܣ�OpenMV����3��ʼ����PC10=TX��PC11=RX
��ڲ�������
����  ֵ����
**************************************************************************/
void OpenMV_USART3_Init(u32 bound)
{  	 
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_USART3);	
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_USART3);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);	
	
	USART_InitStructure.USART_BaudRate = bound;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART3, &USART_InitStructure);
	
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	USART_Cmd(USART3, ENABLE);
}
/**************************************************************************
Function: Serial port 5 initialization
Input   : none
Output  : none
�������ܣ�����5��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart5_init(u32 bound)
{  	 
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	//PC12 TX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
	//PD2 RX
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	 //Enable the gpio clock  //ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE); //Enable the Usart clock //ʹ��USARTʱ��

	GPIO_PinAFConfig(GPIOC,GPIO_PinSource12,GPIO_AF_UART5);	
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource2 ,GPIO_AF_UART5);	 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOC, &GPIO_InitStructure);  		          //��ʼ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;            //���ģʽ
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;          //�������
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;       //����50MHZ
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;            //����
	GPIO_Init(GPIOD, &GPIO_InitStructure);  		          //��ʼ��
	
  //UsartNVIC configuration //UsartNVIC����
  NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;
	//Preempt priority //��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		
	//Enable the IRQ channel //IRQͨ��ʹ��	
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;	
  //Initialize the VIC register with the specified parameters 
	//����ָ���Ĳ�����ʼ��VIC�Ĵ���		
	NVIC_Init(&NVIC_InitStructure);
	
  //USART Initialization Settings ��ʼ������
	USART_InitStructure.USART_BaudRate = bound; //Port rate //���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b; //The word length is 8 bit data format //�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1; //A stop bit //һ��ֹͣ
	USART_InitStructure.USART_Parity = USART_Parity_No; //Prosaic parity bits //����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //No hardware data flow control //��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//Sending and receiving mode //�շ�ģʽ
  USART_Init(UART5, &USART_InitStructure);      //Initialize serial port 5 //��ʼ������5
  USART_ITConfig(UART5, USART_IT_RXNE, ENABLE); //Open the serial port to accept interrupts //�������ڽ����ж�
  USART_Cmd(UART5, ENABLE);                     //Enable serial port 5 //ʹ�ܴ���5
}
/**************************************************************************
Function: Serial port 1 receives interrupted
Input   : none
Output  : none
�������ܣ�����1�����ж�
��ڲ�������
�� �� ֵ����
**************************************************************************/
int USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{
//		u8 Usart_Receive;		
//		Usart_Receive = 
		USART_ReceiveData(USART1); //Read the data //��ȡ����	
	}
	return 0;	
}
/**************************************************************************
Function: Serial port 3 receives interrupted
Input   : none
Output  : none
�������ܣ�OpenMV����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART3_IRQHandler(void)
{	
	u8 Usart_Receive;
	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)
	{
		Usart_Receive=USART_ReceiveData(USART3);
		OpenMV_Parse_Byte(Usart_Receive);
	}
	return 0;	
}
/**************************************************************************
Function: Refresh the OLED screen
Input   : none
Output  : none
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART2_IRQHandler(void)
{	
	static int temp_count = 0;				//���ڼ�¼ǰ����ָ��Ĵ�������һ������������ʱ����Ҫ�õ�
	int Usart_Receive;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //Check if data is received //�ж��Ƿ���յ�����
	{	      
		static u8 Flag_PID,i,j,Receive[50];
		static float Data;
				
		Usart_Receive=USART2->DR; //Read the data //��ȡ����
		//ץ��������ʹ��
		if(AT_Command_Capture(Usart_Receive)) return 1;
	  
		if(Usart_Receive==0x4B) 
			//Enter the APP steering control interface
		  //����APPת����ƽ���
			Turn_Flag=1;   
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 
      // Enter the APP direction control interface		
			//����APP������ƽ���	
			Turn_Flag=0;	
		
		if(Turn_Flag==0) 
		{
			//App rocker control interface command
			//APPҡ�˿��ƽ�������
			if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
			{	
				Mode=APP_Control_Mode;
				Flag_Direction=Usart_Receive-0x40;
			}
			else	if(Usart_Receive<=8)   
			{			
				Flag_Direction=Usart_Receive;
			}	
			else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)
		{
			//APP steering control interface command
			//APPת����ƽ�������
			if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //Right rotation //����ת
			else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //Left rotation  //����ת
			else                         Flag_Left=0,Flag_Right=0;
			if     (Usart_Receive==0x41||Usart_Receive==0x45) 
			{
				if((++temp_count) == 5)					//��Ҫ��������5��ǰ����ָ�����ת��һ��ʱ��ɿ�ʼapp����
				{
					temp_count = 0;
					APP_ON_Flag = RC_ON;		
					PS2_ON_Flag = RC_OFF;
					Mode=APP_Control_Mode;
				}
				Flag_Direction=Usart_Receive-0x40;
			}
			else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //Accelerate the keys, +100mm/s //���ٰ�����+100mm/s
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //Slow down buttons,   -100mm/s //���ٰ�����-100mm/s
	  
	if(Usart_Receive=='a')	RC_Lidar_Avoid_FLAG=!RC_Lidar_Avoid_FLAG;
	 // The following is the communication with the APP debugging interface
	 //��������APP���Խ���ͨѶ
	 if(Usart_Receive==0x7B) Flag_PID=1;   //The start bit of the APP parameter instruction //APP����ָ����ʼλ
	 if(Usart_Receive==0x7D) Flag_PID=2;   //The APP parameter instruction stops the bit    //APP����ָ��ֹͣλ

	 if(Flag_PID==1) //Collect data //�ɼ�����
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2) //Analyze the data //��������
	 {
			if(Receive[3]==0x50) 	 PID_Send=1;
			else  if(Receive[1]!=0x23) 
      {								
				for(j=i;j>=4;j--)
				{
					Data+=(Receive[j-1]-48)*pow(10,i-j);
				}
				
				if(Mode == APP_Control_Mode)
					{
					switch(Receive[1])			//APP�������
					 {
						 case 0x30:  RC_Velocity=Data;break;
						 case 0x31:  Velocity_KP=Data;break;
						 case 0x32:  Velocity_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 				
					 } 
					}
					else if(Mode == ELE_Line_Patrol_Mode)		//���Ѳ��z���ٶȵ���
					{
					switch(Receive[1])
					 {
						 case 0x30:  RC_Velocity_ELE=Data;break;
						 case 0x31:  ELE_KP=Data;break;
						 case 0x32:  ELE_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 				
					 } 
					}
					else 	if(Mode == CCD_Line_Patrol_Mode)									//CCDѲ��z���ٶȵ���
					{
					switch(Receive[1])
					 {
						 case 0x30:  RC_Velocity_CCD=Data;break;
						 case 0x31:  CCD_KP=Data;break;
						 case 0x32:  CCD_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 				
					 } 
					}
					else if(Mode == Lidar_Along_Mode)
					{
						switch(Receive[1])
						{
						 case 0x30:  Along_Distance_KP=Data;break;
						 case 0x31:  Along_Distance_KD=Data;break;
						 case 0x32:  Along_Distance_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 	
						}
					}
					else if(Mode == Lidar_Follow_Mode)
					{
						switch(Receive[1])
						{
						 case 0x30:  break;
						 case 0x31:  break;
						 case 0x32:  break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 	
						}
					}
				}		
      //Relevant flag position is cleared			
      //��ر�־λ����			
			Flag_PID=0;
			i=0;
			j=0;
			Data=0;
			memset(Receive, 0, sizeof(u8)*50); //Clear the array to zero//��������
	 }
   if(RC_Velocity<0)   RC_Velocity=0; 
	 if(RC_Velocity_CCD<0)   RC_Velocity_CCD=0;
	 if(RC_Velocity_ELE<0)   RC_Velocity_ELE=0;		 
  }
  return 0;	
}

/**************************************************************************
Function: Serial port 5 receives interrupted
Input   : none
Output  : none
�������ܣ�����5�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int UART5_IRQHandler(void)
{	
	static u8 state = 0;//״̬λ	
	static u8 crc_sum = 0;//У���
	static u8 cnt = 0;//����һ֡16����ļ���
	u8 temp_data;

	if(USART_GetITStatus(UART5, USART_IT_RXNE) != RESET) //���յ�����
	{	
		//USART_ClearITPendingBit(UART5, USART_IT_RXNE);
		temp_data=USART_ReceiveData(UART5);	
		switch(state)
		{
			case 0:
				if(temp_data == HEADER_0)//ͷ�̶�
				{
					Pack_Data.header_0= temp_data;
					state++;
					//У��
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 1:
				if(temp_data == HEADER_1)//ͷ�̶�
				{
					Pack_Data.header_1 = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 2:
				if(temp_data == Length_)//�ֳ��̶�
				{
					Pack_Data.ver_len = temp_data;
					state++;
					crc_sum += temp_data;
				} else state = 0,crc_sum = 0;
				break;
			case 3:
				Pack_Data.speed_h = temp_data;//�ٶȸ߰�λ
				state++;
				crc_sum += temp_data;			
				break;
			case 4:
				Pack_Data.speed_l = temp_data;//�ٶȵͰ�λ
				state++;
				crc_sum += temp_data;
				break;
			case 5:
				Pack_Data.start_angle_h = temp_data;//��ʼ�Ƕȸ߰�λ
				state++;
				crc_sum += temp_data;
				break;
			case 6:
				Pack_Data.start_angle_l = temp_data;//��ʼ�ǶȵͰ�λ
				state++;
				crc_sum += temp_data;
				break;
			
			case 7:case 10:case 13:case 16:
			case 19:case 22:case 25:case 28:
			case 31:case 34:case 37:case 40:
			case 43:case 46:case 49:case 52:
				
			case 55:case 58:case 61:case 64:
			case 67:case 70:case 73:case 76:
			case 79:case 82:case 85:case 88:
			case 91:case 94:case 97:case 100:
				Pack_Data.point[cnt].distance_h = temp_data & 0x7f ;//16����ľ������ݣ����ֽ�
				state++;
				crc_sum += temp_data;
				break;
			
			case 8:case 11:case 14:case 17:
			case 20:case 23:case 26:case 29:
			case 32:case 35:case 38:case 41:
			case 44:case 47:case 50:case 53:
				
			case 56:case 59:case 62:case 65:
			case 68:case 71:case 74:case 77:
			case 80:case 83:case 86:case 89:
			case 92:case 95:case 98:case 101:
				Pack_Data.point[cnt].distance_l = temp_data;//16����ľ������ݣ����ֽ�
				state++;
				crc_sum += temp_data;
				break;
			
			case 9:case 12:case 15:case 18:
			case 21:case 24:case 27:case 30:
			case 33:case 36:case 39:case 42:
			case 45:case 48:case 51:case 54:
				
			case 57:case 60:case 63:case 66:
			case 69:case 72:case 75:case 78:
			case 81:case 84:case 87:case 90:
			case 93:case 96:case 99:case 102:
				Pack_Data.point[cnt].Strong = temp_data;//16�����ǿ������
				state++;
				crc_sum += temp_data;
				cnt++;
				break;
			case 103:case 104:
				state++;
				crc_sum += temp_data;
				cnt++;
				break;
			case 105:
				Pack_Data.end_angle_h = temp_data;//�����Ƕȵĸ߰�λ
				state++;
				crc_sum += temp_data;			
				break;
			case 106:
				Pack_Data.end_angle_l = temp_data;//�����ǶȵĵͰ�λ
				state++;
				crc_sum += temp_data;
				break;
			case 107:
				Pack_Data.crc = temp_data;//У��
				state = 0;
				cnt = 0;
				if(crc_sum == Pack_Data.crc)
				{
					data_process();//���ݴ�����У����ȷ����ˢ�´洢������	
				}
				else 
				{
					memset(&Pack_Data,0,sizeof(Pack_Data));//����
				}
				crc_sum = 0;//У�������
				break;
			default: break;
		}
	}		
  return 0;	
}

/**************************************************************************
Function: data_process
Input   : none
Output  : none
�������ܣ����ݴ�������
��ڲ�������
����  ֵ����
**************************************************************************/
//���һ֡���պ���д���
static int data_cnt = 0;
void data_process(void) //���ݴ���
{
	int m;
	u32 distance_sum[32]={0};//2����ľ���͵�����
	float start_angle = (((u16)Pack_Data.start_angle_h<<8)+Pack_Data.start_angle_l)/100.0;//����32����Ŀ�ʼ�Ƕ�
	float end_angle = (((u16)Pack_Data.end_angle_h<<8)+Pack_Data.end_angle_l)/100.0;//����32����Ľ����Ƕ�
	float area_angle[32]={0};
	Lidar_Success_Receive_flag=1;Lidar_flag_count=0;//�״����߱�־λ����
	if(start_angle>end_angle)//�����ǶȺͿ�ʼ�Ƕȱ�0�ȷָ�����
		end_angle +=360;
	
	for(m=0;m<32;m++)
	{
		area_angle[m]=start_angle+(end_angle-start_angle)/32*m;
		if(area_angle[m]>360)  area_angle[m] -=360;
		
		distance_sum[m] +=((u16)Pack_Data.point[m].distance_h<<8)+Pack_Data.point[m].distance_l;//���ݸߵ�8λ�ϲ�

		Dataprocess[data_cnt].angle=area_angle[m];
		Dataprocess[data_cnt++].distance=distance_sum[m];  //һ֡����Ϊ32����
		if(data_cnt == 1152) data_cnt = 0;
	}
	
		
}

/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ��������Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz��ת��Ϊ������С������ǰ��ת��
��ڲ�����Ŀ��ǰ���ٶ�Vx��Ŀ����ٶ�Vz����λ��m/s��rad/s
����  ֵ��������С������ǰ��ת�ǣ���λ��rad
**************************************************************************/
float Vz_to_Akm_Angle(float Vx, float Vz)
{
	float R, AngleR, Min_Turn_Radius;
	//float AngleL;
	
	//Ackermann car needs to set minimum turning radius
	//If the target speed requires a turn radius less than the minimum turn radius,
	//This will greatly improve the friction force of the car, which will seriously affect the control effect
	//������С����Ҫ������Сת��뾶
	//���Ŀ���ٶ�Ҫ���ת��뾶С����Сת��뾶��
	//�ᵼ��С���˶�Ħ���������ߣ�����Ӱ�����Ч��
	Min_Turn_Radius=MINI_AKM_MIN_TURN_RADIUS;
	
	if(Vz!=0 && Vx!=0)
	{
		//If the target speed requires a turn radius less than the minimum turn radius
		//���Ŀ���ٶ�Ҫ���ת��뾶С����Сת��뾶
		if(float_abs(Vx/Vz)<=Min_Turn_Radius)
		{
			//Reduce the target angular velocity and increase the turning radius to the minimum turning radius in conjunction with the forward speed
			//����Ŀ����ٶȣ����ǰ���ٶȣ����ת��뾶����Сת��뾶
			if(Vz>0)
				Vz= float_abs(Vx)/(Min_Turn_Radius);
			else	
				Vz=-float_abs(Vx)/(Min_Turn_Radius);	
		}		
		R=Vx/Vz;
		//AngleL=atan(Axle_spacing/(R-0.5*Wheel_spacing));
		AngleR=atan(Axle_spacing/(R+0.5f*Wheel_spacing));
	}
	else
	{
		AngleR=0;
	}
	
	return AngleR;
}
/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
	//Data conversion intermediate variable
	//����ת�����м����
	short transition; 
	
	//����8λ�͵�8λ���ϳ�һ��16λ��short������
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return 
		transition/1000+(transition%1000)*0.001; //Unit conversion, mm/s->m/s //��λת��, mm/s->m/s						
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
�������ܣ�����1��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 2 sends data
Input   : The data to send
Output  : none
�������ܣ�����2��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}

/**************************************************************************
Function: Serial port 3 sends data
Input   : The data to send
Output  : none
�������ܣ�����3��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}
/**************************************************************************
Function: Serial port 5 sends data
Input   : The data to send
Output  : none
�������ܣ�����5��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart5_send(u8 data)
{
	UART5->DR = data;
	while((UART5->SR&0x40)==0);	
}
static void OpenMV_Parse_Byte(u8 data)
{
	static char line[32];
	static u8 index=0;

	if(data=='R' || data=='G' || data=='N')
	{
		OpenMV_Light_State=data;
		return;
	}

	if(index==0 && data!='$') return;

	if(data=='\r' || data=='\n')
	{
		if(index>0)
		{
			line[index]='\0';
			OpenMV_Parse_Line(line);
			index=0;
		}
		return;
	}

	if(index<sizeof(line)-1)
	{
		line[index++]=(char)data;
	}
	else
	{
		index=0;
	}
}

static void OpenMV_Parse_Line(char *line)
{
	char *cursor=line;
	int median,lost,stop,light;

	if(cursor[0]!='$' || cursor[1]!='O' || cursor[2]!='M' || cursor[3]!='V' || cursor[4]!=',') return;
	cursor+=5;

	if(!OpenMV_Parse_Number(&cursor,&median)) return;
	if(*cursor++!=',') return;
	if(!OpenMV_Parse_Number(&cursor,&lost)) return;
	if(*cursor++!=',') return;
	if(!OpenMV_Parse_Number(&cursor,&stop)) return;

	// light field is optional for backward compatibility
	light=0;
	if(*cursor==',')
	{
		cursor++;
		if(!OpenMV_Parse_Number(&cursor,&light)) light=0;
	}

	if(median<0) median=0;
	else if(median>128) median=128;
	if(light<0 || light>2) light=0;

	OpenMV_Median=(u16)median;
	OpenMV_Lost=lost ? 1 : 0;
	OpenMV_Stop=stop ? 1 : 0;
	OpenMV_Light=(u8)light;
	OpenMV_Line_Available=1;
}

static int OpenMV_Parse_Number(char **cursor,int *value)
{
	int result=0;
	u8 count=0;

	while(**cursor>='0' && **cursor<='9')
	{
		result=result*10+(**cursor-'0');
		(*cursor)++;
		count++;
	}

	if(count==0) return 0;
	*value=result;
	return 1;
}
/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
//u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
//{
//	unsigned char check_sum=0,k;
//	
//	//Validate the data to be sent
//	//��Ҫ���͵����ݽ���У��
//	if(Mode==1)
//	for(k=0;k<Count_Number;k++)
//	{
//	check_sum=check_sum^Send_Data.buffer[k];
//	}
//	
//	//Verify the data received
//	//�Խ��յ������ݽ���У��
//	if(Mode==0)
//	for(k=0;k<Count_Number;k++)
//	{
//	check_sum=check_sum^Receive_Data.buffer[k];
//	}
//	return check_sum;
//}


//����ATָ��ץ������ָֹ����ŵ�����������������ͨ��
u8 AT_Command_Capture(u8 uart_recv)
{
	/*
	��������ʱ���͵��ַ���00:11:22:33:44:55Ϊ������MAC��ַ
	+CONNECTING<<00:11:22:33:44:55\r\n
	+CONNECTED\r\n
	��44���ַ�
	
	�����Ͽ�ʱ���͵��ַ�
	+DISC:SUCCESS\r\n
	+READY\r\n
	+PAIRABLE\r\n
	��34���ַ�
	\r -> 0x0D
	\n -> 0x0A
	*/
	
	static u8 pointer = 0; //��������ʱָ���¼��
	static u8 bt_line = 0; //��ʾ�����ڵڼ���
	static u8 disconnect = 0;
	static u8 connect = 0;
	
	//�Ͽ�����
	static char* BlueTooth_Disconnect[3]={"+DISC:SUCCESS\r\n","+READY\r\n","+PAIRABLE\r\n"};
	
	//��ʼ����
	static char* BlueTooth_Connect[2]={"+CONNECTING<<00:00:00:00:00:00\r\n","+CONNECTED\r\n"};


	//�����ʶ������ʼ����(ʹ��ʱҪ-1)
	if(uart_recv=='+') 
	{
		bt_line++,pointer=0; //�յ���+������ʾ�л�������	
		disconnect++,connect++;
		return 1;//ץ������ֹ����
	}

	if(bt_line!=0) 
	{	
		pointer++;

		//��ʼ׷�������Ƿ���϶Ͽ�������������ʱȫ�����Σ�������ʱȡ������
		if(uart_recv == BlueTooth_Disconnect[bt_line-1][pointer])
		{
			disconnect++;
			if(disconnect==34) disconnect=0,connect=0,bt_line=0,pointer=0;
			return 1;//ץ������ֹ����
		}			

		//׷���������� (bt_line==1&&connect>=13)����������MAC��ַ��ÿһ������MAC��ַ������ͬ������ֱ�����ι�ȥ
		else if(uart_recv == BlueTooth_Connect[bt_line-1][pointer] || (bt_line==1&&connect>=13) )
		{		
			connect++;
			if(connect==44) connect=0,disconnect=0,bt_line=0,pointer=0;		
			return 1;//ץ������ֹ����
		}	

		//��ץ���ڼ��յ��������ֹͣץ��
		else
		{
			disconnect = 0;
			connect = 0;
			bt_line = 0;		
			pointer = 0;
			return 0;//�ǽ�ֹ���ݣ����Կ���
		}			
	}
	
	return 0;//�ǽ�ֹ���ݣ����Կ���
}

//����λ��BootLoader����
void _System_Reset_(u8 uart_recv)
{
	static u8 res_buf[5];
	static u8 res_count=0;
	
	res_buf[res_count]=uart_recv;
	
	if( uart_recv=='r'||res_count>0 )
		res_count++;
	else
		res_count = 0;
	
	if(res_count==5)
	{
		res_count = 0;
		//���ܵ���λ������ĸ�λ�ַ���reset����ִ��������λ
		if( res_buf[0]=='r'&&res_buf[1]=='e'&&res_buf[2]=='s'&&res_buf[3]=='e'&&res_buf[4]=='t' )
		{
			NVIC_SystemReset();//����������λ����λ��ִ�� BootLoader ����
		}
	}
}




