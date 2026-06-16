/***********************************************
��˾����Ȥ�Ƽ�����ݸ�����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V5.0
�޸�ʱ�䣺2022-05-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V5.0
Update��2022-05-05

All rights reserved
***********************************************/

#include "system.h"

//Robot software fails to flag bits
//����������ʧ�ܱ�־λ
u8 Flag_Stop=0;   

//The ADC value is variable in segments, depending on the number of car models. Currently there are 6 car models
//ADCֵ�ֶα�����ȡ����С���ͺ�������Ŀǰ��6��С���ͺ�
int Divisor_Mode;

// Robot type variable
//�������ͺű���
//0=Mec_Car��1=Omni_Car��2=Akm_Car��3=Diff_Car��4=FourWheel_Car��5=Tank_Car
u8 Car_Mode=0; 

//Servo control PWM value, Ackerman car special
//�������PWMֵ��������С��ר��
int Servo;  

//Default speed of remote control car, unit: mm/s
//ң��С����Ĭ���ٶȣ���λ��mm/s
float RC_Velocity=350; 

//The Tank_Car CCD related variables
//ȫ��С��CCD��ر���
float Tank_Car_CCD_KP=42,Tank_Car_CCD_KI=140;  // KP +20% for sharper turn

//The Tank_Car ELE related variables
//ȫ��С��ELE��ر���
float Tank_Car_ELE_KP=25,Tank_Car_ELE_KI=200;

//The Omni_Car CCD related variables
//ȫ��С��CCD��ر���
float Omni_Car_CCD_KP=12,Omni_Car_CCD_KI=110;  // KP +20% for sharper turn

//The Omni_Car ELE related variables
//ȫ��С��ELE��ر���
float Omni_Car_ELE_KP=9,Omni_Car_ELE_KI=100;

//The other CCD related variables
//����С��CCD��ر���
float CCD_KP=66,CCD_KI=154;  // KP +20% for sharper turn

//The other ELE related variables
//����С��ELE��ر���
float ELE_KP=30,ELE_KI=200;

//The PS2 gamepad controls related variables
//PS2�ֱ�������ر���
float PS2_LX=128,PS2_LY=128,PS2_RX=128,PS2_RY=128,PS2_KEY; 

//Vehicle three-axis target moving speed, unit: m/s
//С������Ŀ���˶��ٶȣ���λ��m/s
float Move_X, Move_Y, Move_Z;   

//PID parameters of Speed control
//�ٶȿ���PID����
float Velocity_KP=700,Velocity_KI=700; 

//Smooth control of intermediate variables, dedicated to omni-directional moving cars
//ƽ�������м������ȫ���ƶ�С��ר��
Smooth_Control smooth_control;  

//The parameter structure of the motor
//����Ĳ����ṹ��
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  

/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/
//Encoder accuracy
//����������
float Encoder_precision; 
//Wheel circumference, unit: m
//�����ܳ�����λ��m
float Wheel_perimeter; 
//Drive wheel base, unit: m
//�������־࣬��λ��m
float Wheel_spacing; 
//The wheelbase of the front and rear axles of the trolley, unit: m
//С��ǰ�������࣬��λ��m
float Axle_spacing; 
//All-directional wheel turning radius, unit: m
//ȫ����ת��뾶����λ��m
float Omni_turn_radiaus; 
/************ С���ͺ���ر��� **************************/
/************ Variables related to car model ************/

//PS2 controller, Bluetooth APP, aircraft model controller, CAN communication, serial port 1 communication control flag bit.
//These 5 flag bits are all 0 by default, representing the serial port 3 control mode
//PS2�ֱ�������APP����ģ�ֱ���CANͨ�š�����1ͨ�ſ��Ʊ�־λ����5����־λĬ�϶�Ϊ0����������3����ģʽ
u8 CCD_ON_Flag=0, APP_ON_Flag=0, ELE_ON_Flag=0, Avoid_ON_Flag=0, Follow_ON_Flag=0, Along_wall=0, PS2_ON_Flag=0; 

//Bluetooth remote control associated flag bits
//����ң����صı�־λ
u8 Flag_Left, Flag_Right, Flag_Direction=0, Turn_Flag; 

//Sends the parameter's flag bit to the Bluetooth APP
//������APP���Ͳ����ı�־λ
u8 PID_Send;

//ϵͳ��ر���
SYS_VAL_t SysVal;

int POT_val;

int Full_rotation = 16799;



void systemInit(void)
{
	//Interrupt priority group setti  ng
	//�ж����ȼ���������
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	//Delay function initialization
	//��ʱ������ʼ��
    delay_init(168);
	//IIC initialization for IMU
    //IIC��ʼ��������IMU
    I2C_GPIOInit();
	//ϵͳ�������������ʼ��
	SYS_VAL_t_Init(&SysVal);
	
	//Serial port 1 initialization, communication baud rate 115200,
    //can be used to communicate with ROS terminal
    //����1��ʼ����ͨ�Ų�����115200����������ROS��ͨ��
    uart1_init(115200);

    //Serial port 2 initialization, communication baud rate 9600,
    //used to communicate with Bluetooth APP terminal
    //����2��ʼ����ͨ�Ų�����9600������������APP��ͨ��
    uart2_init(9600);
	//OpenMV serial port initialization, PC10=TX, PC11=RX, baud rate 115200
	//OpenMV���ڳ�ʼ����PC10=TX��PC11=RX��ͨ�Ų�����115200
	OpenMV_USART3_Init(115200);

	
    //Serial port 5 initialization, communication baud rate 115200,
    //can be used to communicate with ROS terminal
    //����5��ʼ����ͨ�Ų�����115200����������ROS��ͨ��
    uart5_init(460800);
	
	//���IMUΪMPU6050,���Ǿɰ�C30D
	if( MPU6050_DEFAULT_ADDRESS == MPU6050_getDeviceID() )
	{
		SysVal.HardWare_Ver = V1_0;
		//Initialize the hardware interface to the PS2 controller
		//��ʼ����PS2�ֱ����ӵ�Ӳ���ӿ�
		PS2_Init();
		//PS2 gamepad configuration is initialized and configured in analog mode
		//PS2�ֱ����ó�ʼ��,����Ϊģ����ģʽ	
		PS2_SetInit();		 
		//Initialize the hardware interface connected to the LED lamp
		//��ʼ����LED�����ӵ�Ӳ���ӿ�
		V1_0_LED_Init(); 
	}
	//���IMU�ͺ�ΪICM20948,�����°�C30D
	else if( REG_VAL_WIA == ICM20948_getDeviceID() )//��ȡICM20948 id
	{
		SysVal.HardWare_Ver = V1_1;
		//USBHOS��ʼ��
		MX_USB_HOST_Init();
		//Initialize the hardware interface connected to the LED lamp
		//��ʼ����LED�����ӵ�Ӳ���ӿ�
		V1_1_LED_Init();
	}
	else //�޷�ʶ���������,��λϵͳ
	{
		NVIC_SystemReset();
	}         

    //Initialize the hardware interface connected to the buzzer
    //��ʼ������������ӵ�Ӳ���ӿ�
    Buzzer_Init();

    //Initialize the hardware interface connected to the enable switch
    //��ʼ����ʹ�ܿ������ӵ�Ӳ���ӿ�
    Enable_Pin();

    //Initialize the hardware interface connected to the OLED display
    //��ʼ����OLED��ʾ�����ӵ�Ӳ���ӿ�
    OLED_Init();

    //Initialize the hardware interface connected to the user's key
    //��ʼ�����û��������ӵ�Ӳ���ӿ�
    KEY_Init();

    //ADC pin initialization, used to read the battery voltage and potentiometer gear,
    //potentiometer gear determines the car after the boot of the car model
    //ADC���ų�ʼ�������ڶ�ȡ��ص�ѹ���λ����λ����λ����λ����С���������С�������ͺ�
    Adc_Init();
    Adc_POWER_Init();


    //According to the tap position of the potentiometer, determine which type of car needs to be matched,
    //and then initialize the corresponding parameters
    //���ݵ�λ���ĵ�λ�ж���Ҫ���������һ���ͺŵ�С����Ȼ����ж�Ӧ�Ĳ�����ʼ��
    Robot_Select();

    //��ͨС��Ĭ�϶�ʱ��8������ģ�ӿ�
    TIM8_SERVO_Init(9999,168-1);//APB2��ʱ��Ƶ��Ϊ168M , Ƶ��=168M/((9999+1)*(167+1))=100Hz

    //Initialize motor speed control and, for controlling motor speed, PWM frequency 10kHz
    //��ʼ������ٶȿ����Լ������ڿ��Ƶ���ٶȣ�PWMƵ��10KHZ
    //APB2ʱ��Ƶ��Ϊ168M����PWMΪ16799��Ƶ��=168M/((16799+1)*(0+1))=10k
	TIM1_PWM_Init(16799,0);
	TIM9_PWM_Init(16799,0);
    TIM10_PWM_Init(16799,0);
    TIM11_PWM_Init(16799,0);
	
    //Encoder A is initialized to read the real time speed of motor C
    //������A��ʼ�������ڶ�ȡ���C��ʵʱ�ٶ�
    Encoder_Init_TIM2();
    //Encoder B is initialized to read the real time speed of motor D
    //������B��ʼ�������ڶ�ȡ���D��ʵʱ�ٶ�
    Encoder_Init_TIM3();
    //Encoder C is initialized to read the real time speed of motor B
    //������C��ʼ�������ڶ�ȡ���B��ʵʱ�ٶ�
    Encoder_Init_TIM4();
    //Encoder D is initialized to read the real time speed of motor A
    //������D��ʼ�������ڶ�ȡ���A��ʵʱ�ٶ�
    Encoder_Init_TIM5();
	
}

