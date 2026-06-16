#include "balance.h"

//int Time_count=0; //Time variable //��ʱ���� 

u8 Lidar_Detect = Lidar_Detect_ON;			//���Ѳ��ģʽ�״����ϰ��Ĭ�Ͽ���
int RC_Lidar_Avoid_FLAG = RC_Avoid_OFF;			//ң��ģʽ�״���ϣ�Ĭ�Ϲر�
u8 Mode;
float RC_Velocity_CCD=350.0f/3.0f,RC_Velocity_ELE=350; 
float PS2_Velocity=350,PS2_Turn_Velocity;			//ң�ؿ��Ƶ��ٶ�
Encoder OriginalEncoder; //Encoder raw data //������ԭʼ����    
u8 Lidar_Success_Receive_flag=0,Lidar_flag_count=0;//�״����߱�־λ���״���߼�����

// Robot mode is wrong to detect flag bits
//������ģʽ�Ƿ��������־λ
int robot_mode_check_flag=0; 

short test_num;
u8 command_lost_count=0;//

Encoder OriginalEncoder; //Encoder raw data //������ԭʼ����     

//========== PWM���ʹ�ñ��� ==========//
u8 start_check_flag = 0;//����Ƿ���Ҫ���PWM
u8 wait_clear_times = 0;
u8 start_clear = 0;     //��ǿ�ʼ���PWM
u8 clear_done_once = 0; //�����ɱ�־λ
u16 clear_again_times = 0;
float debug_show_diff = 0;
//void auto_pwm_clear(void);
volatile u8 clear_state = 0x00;
/*------------------------------------*/


/**************************************************************************
Function: The inverse kinematics solution is used to calculate the target speed of each wheel according to the target speed of three axes
Input   : X and Y, Z axis direction of the target movement speed
Output  : none
�������ܣ��˶�ѧ��⣬��������Ŀ���ٶȼ��������Ŀ��ת��
��ڲ�����X��Y��Z�᷽���Ŀ���˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5; //Wheel target speed limit //����Ŀ���ٶ��޷�
	
	  //Speed smoothing is enabled when moving the omnidirectional trolley
	  //ȫ���ƶ�С���ſ����ٶ�ƽ������
	if((Mode == PS2_Control_Mode||Mode == APP_Control_Mode)&&RC_Lidar_Avoid_FLAG&&Lidar_Success_Receive_flag)//app�Լ�ps2�����£��ж��Ƿ���ң�ر���
	{
		RC_Lidar_Aviod(&Vx,&Vy,&Vz);
	}
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //Smoothing the input speed //�������ٶȽ���ƽ������
  
      //Get the smoothed data 
			//��ȡƽ�������������			
			Vx=smooth_control.VX;     
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
		//Mecanum wheel car
	  //�����ķ��С��
	  if (Car_Mode==Mec_Car) 
    {
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); 
		} 
		
		//Omni car
		//ȫ����С��
		else if (Car_Mode==Omni_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=0;	//Out of use //û��ʹ�õ�
		}
		
		//Ackermann structure car
		//������С��
		else if (Car_Mode==Akm_Car) 
		{
			//Ackerman car specific related variables //������С��ר����ر���
			float R, Ratio=636.56, AngleR, Angle_Servo;
			
			// For Ackerman small car, Vz represents the front wheel steering Angle
			//���ڰ�����С��Vz������ǰ��ת��Ƕ�
			AngleR=Vz;
			R=Axle_spacing/tan(AngleR)-0.5f*Wheel_spacing;
			
			// Front wheel steering Angle limit (front wheel steering Angle controlled by steering engine), unit: rad
			//ǰ��ת��Ƕ��޷�(�������ǰ��ת��Ƕ�)����λ��rad
			AngleR=target_limit_float(AngleR,-0.49f,0.32f);
			
			//Inverse kinematics //�˶�ѧ���
			if(AngleR!=0)
			{
				MOTOR_A.Target = Vx*(R-0.5f*Wheel_spacing)/R;
				MOTOR_B.Target = Vx*(R+0.5f*Wheel_spacing)/R;			
			}
			else 
			{
				MOTOR_A.Target = Vx;
				MOTOR_B.Target = Vx;
			}
			// The PWM value of the servo controls the steering Angle of the front wheel
			//���PWMֵ���������ǰ��ת��Ƕ�
			//Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.573f;
			Angle_Servo    =  -0.628f*pow(AngleR, 3) + 1.269f*pow(AngleR, 2) - 1.772f*AngleR + 1.755f;
			Servo=SERVO_INIT + (Angle_Servo - 1.755f)*Ratio;

			
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
			Servo=target_limit_int(Servo,800,2200);	//Servo PWM value limit //���PWMֵ�޷�
			}
		
		//Differential car
		//����С��
		else if (Car_Mode==Diff_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
			
		}
		
		//FourWheel car
		//������
		else if(Car_Mode==FourWheel_Car) 
		{	
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
					
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); 
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); 		
		}
		
		//Tank Car
		//�Ĵ���
		else if (Car_Mode==Tank_Car) 
		{
			//Inverse kinematics //�˶�ѧ���
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
			
			//Wheel (motor) target speed limit //����(���)Ŀ���ٶ��޷�
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); 
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); 
			MOTOR_C.Target=0; //Out of use //û��ʹ�õ�
			MOTOR_D.Target=0; //Out of use //û��ʹ�õ�
		}
}
/**************************************************************************
Function: FreerTOS task, core motion control task
Input   : none
Output  : none
�������ܣ�FreeRTOS���񣬺����˶���������
��ڲ�������
����  ֵ����
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
//	static u8 Count_CCD = 0;								//����CCD����Ƶ��
	static u8 last_mode = 0;
	u32 lastWakeTime = getSysTickCnt();

	while(1)
	{	
		// This task runs at a frequency of 100Hz (10ms control once)
		//��������100Hz��Ƶ�����У�10ms����һ�Σ�
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); 
		//Time count is no longer needed after 30 seconds
		//ʱ�������30�������Ҫ
		if(SysVal.Time_count<3000)SysVal.Time_count++;
		//Get the encoder data, that is, the real time wheel speed, 
		//and convert to transposition international units
		//��ȡ���������ݣ�������ʵʱ�ٶȣ���ת��λ���ʵ�λ
		Get_Velocity_Form_Encoder();   

		if(++Lidar_flag_count==10) Lidar_Success_Receive_flag=0,Lidar_flag_count=0,RC_Lidar_Avoid_FLAG=RC_Avoid_OFF; //100ms�����״���յ����ݱ�־λ
		switch(click_N_Double(50))
		{
			case 1:    //���������л�ģʽ
				Mode+=1;
				if(Mode == ELE_Line_Patrol_Mode)			//ѡ����Ѳ�߿���ģʽ
				{
						ele_Init();							//��ʼ��ELE
				}
				else if(Mode == CCD_Line_Patrol_Mode)			//ѡ����Ѳ�߿���ģʽ
				{
						ccd_Init();							//��ʼ��ELE
				}
				if(Mode>6)						Mode = 0;
				 break;
			case 2:    //���Ѳ��״̬ʱ��˫�����Դ�/�ر��״����ϰ��Ĭ�ϴ�
				if(Mode == ELE_Line_Patrol_Mode)			//���Ѳ�߿���ģʽ��
				{
					Lidar_Detect = !Lidar_Detect;
					if(Lidar_Detect == Lidar_Detect_OFF)
						memset(Dataprocess,0, sizeof(PointDataProcessDef)*225);		//�����״����ϰ������������
				}
				else if(Mode == APP_Control_Mode||Mode ==PS2_Control_Mode){
					RC_Lidar_Avoid_FLAG=!RC_Lidar_Avoid_FLAG;
					if(RC_Lidar_Avoid_FLAG == RC_Avoid_OFF)
						memset(Dataprocess,0, sizeof(PointDataProcessDef)*225);		//�����״����ϰ������������
				}
				break;				 
		}
		
		if(last_mode != Mode)  //�л�ģʽʱ��Ļ����
		{
			last_mode++;
			OLED_Clear();
			if(last_mode>6)
			{
//				OLED_Clear();
				last_mode = 0;
			}
		} 
				
		if(Mode != ELE_Line_Patrol_Mode)
			Buzzer_Alarm(0);
		if(Mode == APP_Control_Mode)          Get_RC();             //Handle the APP remote commands //����APPң������
		else if(Mode == PS2_Control_Mode)     PS2_Control();        //Handle PS2 controller commands //����PS2�ֱ���������
		else if(Mode == Lidar_Avoid_Mode)     Lidar_Avoid();        //Avoid Mode //����ģʽ
		else if(Mode == Lidar_Follow_Mode)    Lidar_Follow();       //Follow Mode //����ģʽ
		else if(Mode == Lidar_Along_Mode)     Lidar_along_wall();   //Along Mode //��ֱ��ģʽ
		else if(Mode == CCD_Line_Patrol_Mode) Get_RC_CCD();	
		else													
		{
			Get_RC_ELE();											
		}					
		//If there is no abnormity in the battery voltage, and the enable switch is in the ON position,
		//and the software failure flag is 0
		//�����ص�ѹ�������쳣������ʹ�ܿ�����ON��λ����������ʧ�ܱ�־λΪ0
		if(Turn_Off(Voltage)==0) 
		 { 			
			 //Speed closed-loop control to calculate the PWM value of each motor, 
			 //PWM represents the actual wheel speed					 
			 //�ٶȱջ����Ƽ�������PWMֵ��PWM��������ʵ��ת��
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);
			 Limit_Pwm(16500) ;
			 //Set different PWM control polarity according to different car models
			 //���ݲ�ͬС���ͺ����ò�ͬ��PWM���Ƽ���
			 switch(Car_Mode)
			 {
					case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  -MOTOR_B.Motor_Pwm, MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Mecanum wheel car       //�����ķ��С��
					case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Omni car                //ȫ����С��
					case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //Ackermann structure car //������С��
					case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Differential car        //���ֲ���С��
					case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //FourWheel car           //������ 
					case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //Tank Car                //�Ĵ���
			 }
		 }
		 //If Turn_Off(Voltage) returns to 1, the car is not allowed to move, and the PWM value is set to 0
		 //���Turn_Off(Voltage)����ֵΪ1������������С�������˶���PWMֵ����Ϊ0
		 else	Set_Pwm(0,0,0,0,0); 
			 	
	}  
}
/**************************************************************************
Function: Assign a value to the PWM register to control wheel speed and direction
Input   : PWM
Output  : none
�������ܣ���ֵ��PWM�Ĵ��������Ƴ���ת���뷽��
��ڲ�����PWM
����  ֵ����
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
	//Forward and reverse control of motor
	//�������ת����
	if(motor_a<0)			PWMA2=16799,PWMA1=16799+motor_a;
	else 	            PWMA1=16799,PWMA2=16799-motor_a;
	
	//Forward and reverse control of motor
	//�������ת����	
	if(motor_b<0)			PWMB1=16799,PWMB2=16799+motor_b;
	else 	            PWMB2=16799,PWMB1=16799-motor_b;
//  PWMB1=10000,PWMB2=5000;

	//Forward and reverse control of motor
	//�������ת����	
	if(motor_c<0)			PWMC2=16799,PWMC1=16799+motor_c;
	else 	            PWMC1=16799,PWMC2=16799-motor_c;
	
	//Forward and reverse control of motor
	//�������ת����
	if(motor_d<0)			PWMD1=16799,PWMD2=16799+motor_d;
	else 	            	PWMD2=16799,PWMD1=16799-motor_d;
	
	//Servo control
	//�������
	Servo_PWM =servo;
}

/**************************************************************************
Function: Limit PWM value
Input   : Value
Output  : none
�������ܣ�����PWMֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
Function: Limiting function
Input   : Value
Output  : none
�������ܣ��޷�����
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
Function: Check the battery voltage, enable switch status, software failure flag status
Input   : Voltage
Output  : Whether control is allowed, 1: not allowed, 0 allowed
�������ܣ�����ص�ѹ��ʹ�ܿ���״̬������ʧ�ܱ�־λ״̬
��ڲ�������ѹ
����  ֵ���Ƿ��������ƣ�1����������0����
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)
			{	                                                
				temp=1;      
				PWMA1=0;PWMA2=0;
				PWMB1=0;PWMB2=0;		
				PWMC1=0;PWMC1=0;	
				PWMD1=0;PWMD2=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
Function: Calculate absolute value
Input   : long int
Output  : unsigned int
�������ܣ������ֵ
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
Function: Incremental PI controller
Input   : Encoder measured value (actual speed), target speed
Output  : Motor PWM
According to the incremental discrete PID formula
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k) represents the current deviation
e(k-1) is the last deviation and so on
PWM stands for incremental output
In our speed control closed loop system, only PI control is used
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)

�������ܣ�����ʽPI������
��ڲ���������������ֵ(ʵ���ٶ�)��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)��������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;    
}
int Incremental_PI_B (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm;
}
int Incremental_PI_C (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias; 
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
int Incremental_PI_D (float Encoder,float Target)
{  
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder; //Calculate the deviation //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;  
	 if(Pwm>16800)Pwm=16800;
	 if(Pwm<-16800)Pwm=-16800;
	 Last_bias=Bias; //Save the last deviation //������һ��ƫ�� 
	 return Pwm; 
}
/**************************************************************************
Function: Processes the command sent by APP through usart 2
Input   : none
Output  : none
�������ܣ���APPͨ������2���͹�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car) //The omnidirectional wheel moving trolley can move laterally //ȫ�����˶�С�����Խ��к����ƶ�
	{
	 switch(Flag_Direction)  //Handle direction control commands //���������������
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		     Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		     Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	   Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		
	 {	
		 //If no direction control instruction is available, check the steering control status
		 //����޷������ָ����ת�����״̬
		 if     (Flag_Left ==1)  Move_Z= PI/2*(RC_Velocity/500); //left rotation  //����ת  
		 else if(Flag_Right==1)  Move_Z=-PI/2*(RC_Velocity/500); //right rotation //����ת
		 else 		               Move_Z=0;                       //stop           //ֹͣ
	 }
	}	
	else //Non-omnidirectional moving trolley //��ȫ���ƶ�С��
	{
	 switch(Flag_Direction) //Handle direction control commands //���������������
	 { 
			case 1:     Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/2;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/2;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/2;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/2;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/2;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/2;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/2; //left rotation  //����ת 
	 else if(Flag_Right==1)  Move_Z=-PI/2; //right rotation //����ת	
	}
	
	//Z-axis data conversion //Z������ת��
	if(Car_Mode==Akm_Car)
	{
		//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		//�������ṹС��ת��Ϊǰ��ת��Ƕ�
		Move_Z=Move_Z*2/9; 
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
		Move_Z=Move_Z*RC_Velocity/500;
	}		
	
	//Unit conversion, mm/s -> m/s
  //��λת����mm/s -> m/s	
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	
	//Control target value is obtained and kinematics analysis is performed
	//�õ�����Ŀ��ֵ�������˶�ѧ����
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
Function: Handle PS2 controller control commands
Input   : none
Output  : none
�������ܣ���PS2�ֱ�����������д���
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_Control(void)
{
   	int LX,LY,RY;
		int Threshold=20; //Threshold to ignore small movements of the joystick //��ֵ������ҡ��С���ȶ���
	static u8 acc_dec_filter = 0;		
	
	if(PS2_ON_Flag==1)
	{
	  //128 is the median.The definition of X and Y in the PS2 coordinate system is different from that in the ROS coordinate system
	  //128Ϊ��ֵ��PS2����ϵ��ROS����ϵ��X��Y�Ķ��岻һ��
		LY=-(PS2_LX-128);  
		LX=-(PS2_LY-128); 
		RY=-(PS2_RX-128); 
		
	  //Ignore small movements of the joystick //����ҡ��С���ȶ���
		if(LX>-Threshold&&LX<Threshold)LX=0; 
		if(LY>-Threshold&&LY<Threshold)LY=0; 
		if(RY>-Threshold&&RY<Threshold)RY=0; 
	
	if(++acc_dec_filter==15)
	{
		acc_dec_filter=0;
	  if (PS2_KEY==11)	    RC_Velocity+=5;  //To accelerate//����
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //To slow down //����	
	}
		if(RC_Velocity<0)   RC_Velocity=0;
	
	  //Handle PS2 controller control commands
	  //��PS2�ֱ�����������д���
		Move_X=LX*RC_Velocity/128; 
		Move_Y=LY*RC_Velocity/128; 
		Move_Z=RY*(PI/2)/128;      
	
	  //Z-axis data conversion //Z������ת��
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		else if(Car_Mode==Akm_Car)
		{
			//Ackermann structure car is converted to the front wheel steering Angle system target value, and kinematics analysis is pearformed
		  //�������ṹС��ת��Ϊǰ��ת��Ƕ�
			Move_Z=Move_Z*2/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z; //The differential control principle series requires this treatment //���ٿ���ԭ��ϵ����Ҫ�˴���
			Move_Z=Move_Z*RC_Velocity/500;
		}	
		 
	  //Unit conversion, mm/s -> m/s
    //��λת����mm/s -> m/s	
		Move_X=Move_X/1000;        
		Move_Y=Move_Y/1000;    
		Move_Z=Move_Z;
	}
		//Control target value is obtained and kinematics analysis is performed
	  //�õ�����Ŀ��ֵ�������˶�ѧ����
		Drive_Motor(Move_X,Move_Y,Move_Z);		 			
} 
/**************************************************************************
�������ܣ�CCDѲ�ߣ��ɼ�3����е����ݲ���ȡ���� 
��ڲ�������
����  ֵ����
**************************************************************************/
void  Get_RC_CCD(void)
{
	static float Bias,Last_Bias;
	float move_z=0;

			//Zebra crossing detected by OpenMV -> stop the car
			if(CCD_Zebra_Stop_Flag)
			{
				Move_X=0;
				Move_Y=0;
				Move_Z=0;
				Last_Bias=0;
				Drive_Motor(0,0,0);
				return;
			}

			Move_X=RC_Velocity_CCD;													//CCDѲ��ģʽ���ٶ�
			Bias=CCD_Median-64;  //��ȡƫ�64ΪѲ�ߵ����ĵ�
	    if(Car_Mode == Omni_Car)
			  move_z=-Bias*Omni_Car_CCD_KP*0.1f-(Bias-Last_Bias)*Omni_Car_CCD_KI*0.1f; //PD���ƣ�ԭ������ʹ��С�����ֿ���Ѳ�ߵ����ĵ�
			else if(Car_Mode == Tank_Car)
				move_z=-Bias*Tank_Car_CCD_KP*0.1f-(Bias-Last_Bias)*Tank_Car_CCD_KI*0.1f;
			else
				move_z=-Bias*CCD_KP*0.1f-(Bias-Last_Bias)*CCD_KI*0.1f;
			Last_Bias=Bias;   //������һ�ε�ƫ��
			
			if(Car_Mode==Mec_Car)															
			{
				Move_Z=move_z*RC_Velocity_CCD/50000;							//���ٿ���ԭ����Ҫ�����˴�����
			}

			else if(Car_Mode==Omni_Car)											
			{
				Move_Z=move_z*RC_Velocity_CCD/21000;							//���ٿ���ԭ����Ҫ�����˴�����
			}
			
			else if(Car_Mode==Akm_Car)												
			{
				Move_Z=move_z/450;																//���ٿ���ԭ����Ҫ�����˴�����
			}
			
			else if(Car_Mode==Diff_Car)		
			{	
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/67000;					//���ٿ���ԭ����Ҫ�����˴�����	
			}
			else if(Car_Mode==Tank_Car)		
			{	
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/50000;					//���ٿ���ԭ����Ҫ�����˴�����	
			}
			else if(Car_Mode==FourWheel_Car)									
			{
				if(Move_X<0) move_z=-move_z;	
				Move_Z=move_z*RC_Velocity_CCD/20100;					//���ٿ���ԭ����Ҫ�����˴�����
			}			
		
			//Z-axis data conversion //Z������ת��	
			//Unit conversion, mm/s -> m/s
			//��λת����mm/s -> m/s
			Move_X=Move_X/1000;
			Move_Z=Move_Z;
			Move_Y=0;
			//Control target value is obtained and kinematics analysis is performed
			//�õ�����Ŀ��ֵ�������˶�ѧ����
			Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
�������ܣ����Ѳ�ߣ��ɼ�3����е����ݲ���ȡ���� 
��ڲ�������
����  ֵ����
**************************************************************************/
void  Get_RC_ELE(void)
{
	static float Bias,Last_Bias;
	float move_z=0;
	
	if(Detect_Barrier() == No_Barrier)
	{
			Move_X=RC_Velocity_ELE;				//���Ѳ��ģʽ���ٶ�
			Bias=100-Sensor;  //��ȡƫ��	
      if(Car_Mode == Omni_Car)		
			  move_z=-Bias* Omni_Car_ELE_KP*0.08f-(Bias-Last_Bias)* Omni_Car_ELE_KI*0.05f; 
			else if(Car_Mode == Tank_Car)
				move_z=-Bias*Tank_Car_ELE_KP*0.1f-(Bias-Last_Bias)*Tank_Car_ELE_KI*0.1f;
			else
				move_z=-Bias* ELE_KP*0.08f-(Bias-Last_Bias)* ELE_KI*0.05f; 
			Last_Bias=Bias; 
		  Buzzer_Alarm(0);

			if(Car_Mode==Mec_Car)															
			{		
				Move_Z=move_z*RC_Velocity_ELE/50000;					//���ٿ���ԭ����Ҫ�����˴�����
			}
			
			else if(Car_Mode==Omni_Car)											
			{
				Move_Z=move_z*RC_Velocity_ELE/10800;					//���ٿ���ԭ����Ҫ�����˴�����
			}
			
			else if(Car_Mode==Diff_Car)		
			{
				if(Move_X<0) move_z=-move_z;			
				Move_Z=move_z*RC_Velocity_ELE/45000;					//���ٿ���ԭ����Ҫ�����˴�����
			}
			
			else if(Car_Mode==Tank_Car)		
			{
				if(Move_X<0) move_z=-move_z;			
				Move_Z=move_z*RC_Velocity_ELE/28000;					//���ٿ���ԭ����Ҫ�����˴�����
			}
			else if(Car_Mode==FourWheel_Car)									
			{
				if(Move_X<0) move_z=-move_z;
				Move_Z=move_z*RC_Velocity_ELE/20100;					//���ٿ���ԭ����Ҫ�����˴�����
			}
			
			else if(Car_Mode==Akm_Car)											
			{
				Move_Z=move_z/450;														//���ٿ���ԭ����Ҫ�����˴�����
			}
		}
	
	else									//���ϰ���
	{
		Buzzer_Alarm(100);				//�����ʹ�ܵ�ʱ�����ϰ��������������
		Move_X = 0;
		Move_Z = 0;
	}

			//Z-axis data conversion //Z������ת��	
			//Unit conversion, mm/s -> m/s
			//��λת����mm/s -> m/s
			Move_X=Move_X/1000;
			Move_Z=Move_Z;
	
			//Control target value is obtained and kinematics analysis is performed
			//�õ�����Ŀ��ֵ�������˶�ѧ����
	    Move_Y=0;
			Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
�������ܣ����ǰ���Ƿ����ϰ���
��ڲ�������
����  ֵ����
**************************************************************************/
u8 Detect_Barrier(void)
{
	int i;
	u8 point_count = 0;
	
	if(Lidar_Detect == Lidar_Detect_ON)
	{
		for(i=0;i<1152;i++)	//����Ƿ����ϰ��� 
		{
			if((Dataprocess[i].angle>300)||(Dataprocess[i].angle<60))
			{
				if(0<Dataprocess[i].distance&&Dataprocess[i].distance<700)//700mm���Ƿ����ϰ���
					point_count++;
		  }
	}
		if(point_count > 0)//���ϰ���
			return Barrier_Detected;
		else
			return No_Barrier;
	}
	else
		return No_Barrier;
}

/**************************************************************************
�������ܣ�С������ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_Avoid(void)
{
	int i = 0; 
	u8 calculation_angle_cnt = 0;	//�����ж�225��������Ҫ�����ϵĵ�
	float angle_sum = 0;			//���Լ����ϰ���λ���������
	u8 distance_count = 0;			//����С��ĳֵ�ļ���
	for(i=0;i<1152;i++)				//����120�ȷ�Χ�ڵľ������ݣ���120�������ҵ�����
	{
		if((Dataprocess[i].angle>300)||(Dataprocess[i].angle<60))  //���ϽǶ���300-60֮��
		{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<Avoid_Distance))	//����С��450mm��Ҫ����,ֻ��Ҫ120�ȷ�Χ�ڵ�
			{
				calculation_angle_cnt++;						 			//�������С�ڱ��Ͼ���ĵ����
				if(Dataprocess[i].angle<60)		
					angle_sum += Dataprocess[i].angle;
				else if(Dataprocess[i].angle>300)
					angle_sum += (Dataprocess[i].angle-360);	//300�ȵ�60��ת��Ϊ-60�ȵ�60��
				if(Dataprocess[i].distance<Avoid_Min_Distance)				//��¼С��200mm�ĵ�ļ���
					distance_count++;
			}
	  }
	}
	Move_X = forward_velocity;
  if(calculation_angle_cnt == 0)//����Ҫ����
	 {
		Move_Z = 0;
	 }
	else                          //������С��200mm��С��������
	{
		if(distance_count>8)
		{
			Move_X = -forward_velocity;
			Move_Z = 0;
		}
		else
		{
			Move_X = 0;
			if(angle_sum > 0)//�ϰ���ƫ��
			{
				if(Car_Mode == Mec_Car)  //����ת����Ҫ��ǰ���ٶȽ���
					Move_X = 0;
				else                     //�������ͱ���ԭ�г���
				  Move_X = forward_velocity;
				
				if(Car_Mode == Akm_Car)
					Move_Z = PI/4;
				else if(Car_Mode == Omni_Car)
					Move_Z=corner_velocity;
				else
				  Move_Z=other_corner_velocity;//��ת
			}
			else //ƫ��
			{
				if(Car_Mode == Mec_Car)
					Move_X = 0;
				else
				  Move_X = forward_velocity;
				
				if(Car_Mode == Akm_Car)
					Move_Z = -PI/4;
				else if(Car_Mode == Omni_Car)
					Move_Z=-corner_velocity;//��ת
				else
					Move_Z=-other_corner_velocity;
			}
	  }
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}


/**************************************************************************
�������ܣ�С������ģʽ 
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_Follow(void)
{
	static u16 cnt = 0;
	int i;
	int calculation_angle_cnt = 0;
	static float angle = 0;				//���ϵĽǶ�
	static float last_angle = 0;		//
	u16 mini_distance = 65535;
	static u8 data_count = 0;			//�����˳�һд���ļ�������
	//��Ҫ�ҳ�������Ǹ���ĽǶ�
	for(i = 0; i < 1152; i++)
	{
			if((0<Dataprocess[i].distance)&&(Dataprocess[i].distance<Follow_Distance))
			{
				calculation_angle_cnt++;
				if(Dataprocess[i].distance<mini_distance)
				{
					mini_distance = Dataprocess[i].distance;
					angle = Dataprocess[i].angle;
				}
			}
	}
	if(angle > 180)  //0--360��ת����0--180��-180--0��˳ʱ�룩
		angle -= 360;
	if((angle-last_angle > 10)||(angle-last_angle < -10))   //��һ����������������10�ȵ���Ҫ���ж�
	{
		if(++data_count > 30)   //����30�βɼ�����ֵ(300ms��)���ϴεıȴ���10�ȣ���ʱ������Ϊ����Чֵ
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	else    //����С��10�ȵĿ���ֱ����Ϊ����Чֵ
	{
		if(++data_count > 10)   //����10�βɼ�����ֵ(100ms��)����ʱ������Ϊ����Чֵ
		{
			data_count = 0;
			last_angle = angle;
		}
	}
	if(calculation_angle_cnt < 8)  //�������С��8�ҵ�cnt>40��ʱ����Ϊ��1600��û�и���Ŀ��
	{
		if(cnt < 40)
			cnt++;
		if(cnt >= 40)
		{
			Move_X = 0;
			Move_Z = 0;
		}
	}
	else
	{
		cnt = 0;
		if(Move_X > 0.06f || Move_X < -0.06f)  //��Move_X���ٶ�ʱ��ת��PID��ʼ����
		{
			if(mini_distance < 700 && (last_angle > 60 || last_angle < -60))
			{
				Move_Z = -0.0098f*last_angle;  //������ƫС�ҽǶȲ�����ֱ�ӿ���ת��
			}
			else
			{
				  Move_Z = -Follow_Turn_PID(last_angle,0);		//ת��PID����ͷ��Զ���Ÿ�����Ʒ
			}
		}
		else
		{
			Move_Z = 0;
		}
		if(angle>150 || angle<-150)  //���С���ں�60����Ҫ�������˶��Լ�����ת��
		{
			Move_X = -Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);
			Move_Z = -0.0098f*last_angle;
		}
		else
		{
		  Move_X = Distance_Adjust_PID(mini_distance, Keep_Follow_Distance);  //���־��뱣����500mm
		}
		Move_X = target_limit_float(Move_X,-amplitude_limiting,amplitude_limiting);   //��ǰ���ٶ��޷�
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}

/**************************************************************************
�������ܣ�С����ֱ��ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
void Lidar_along_wall(void)
{
	static u32 target_distance=0;
	static int i=0;

	u32 distance, last_distance;
	u8 data_count = 0;			//�����˳�һд���ļ�������
	
	Move_X = forward_velocity;  //��ʼ�ٶ�
	
	for(int j=0;j<1152;j++) //225
	{
		if(Dataprocess[j].angle>268 && Dataprocess[j].angle<272)   //ȡ�״��4�ȵĵ�
		{
			if(i==0)
			{
				target_distance=Dataprocess[j].distance;  //�״ﲶ���һ������
				i++;
			}
			 if(Dataprocess[j].distance<(target_distance+limit_distance))//����һ���״��̽�����
			 {
				 data_count++;
				 distance=Dataprocess[j].distance;//ʵʱ����
				 if(distance==0)		distance = last_distance;
				 last_distance = distance;
			 }
		}
	}
	if(Car_Mode == Mec_Car || Car_Mode == Omni_Car)  //ֻ�����ֺ�ȫ�������Move_Y
	{
		Move_Y=Along_Adjust_PID(distance,target_distance);
		Move_X = forward_velocity;
		Move_Z = 0;
	}
	else   //��������ʹ��Move_Z������ֱ��״̬
	{
		Move_Z=Along_Adjust_PID(distance,target_distance);
		Move_X = forward_velocity;
		Move_Y = 0;
	}
	if(data_count == 0)  //��data_count����0��ֻ��ǰ���ٶ�
	{
		Move_Y = 0;
		Move_Z = 0;
	}
	Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
�������ܣ�С��ң�ر��Ϲ���
��ڲ�������
����  ֵ����
**************************************************************************/
void RC_Lidar_Aviod(float *Vx,float *Vy,float *Vz)
{
	int i;
	int avoid_distance=450;//���Ͼ���
	u16 avoid_point_count=0;//��Χ����Ҫ���ϵĵ���
	int left_angle_sum=0, right_angle_sum=0;
	int mini_distance_count=0;//��Сת�����ĵ���
	for(i=0;i<1152;i++)
	{
		if(Dataprocess[i].angle>300||Dataprocess[i].angle<60)
		{//��ǰ���ϽǶ� 300��-60��
			if(0<Dataprocess[i].distance&&Dataprocess[i].distance<avoid_distance)
			{
				avoid_point_count++;//��Ҫ���ϵĵ���
				if(Dataprocess[i].angle<60)
				{
					right_angle_sum+=Dataprocess[i].angle;
				}
				else if(Dataprocess[i].angle>300)
				{
					left_angle_sum+=(Dataprocess[i].angle-360);//300�㵽359��ת��Ϊ-60�㵽-1��
				}
				if(Dataprocess[i].distance<Avoid_Min_Distance)
				{//С����С���Ͼ���
					mini_distance_count++;
				}
			}
		}
	
	}
	if(*Vx>0&&avoid_point_count>8){//��Ҫ����
		if(mini_distance_count>=8)//����С����Сת�����Ҫ��
		{
			*Vx=*Vx*-0.75f;//����
			*Vz=0;
		}
		else
		{
			if(right_angle_sum+left_angle_sum>0)
			{//�������
				*Vx=*Vx*0.5f;//���ϵ�X���ٶ���ǰ���ٶȵ�һ��
				if(Car_Mode==Akm_Car){
					*Vz=(PI/2)*4/9;
				}else{
					*Vz=1;
				}
			}else{//���ұ���
				*Vx=*Vx*0.5f;//���ϵ�X���ٶ���ǰ���ٶȵ�һ��
				if(Car_Mode==Akm_Car){
					*Vz=(-PI/2)*4/9;
				}else{
					*Vz=-1;
				}
				
			}
		}
	}
}

/**************************************************************************
Function: Read the encoder value and calculate the wheel speed, unit m/s
Input   : none
Output  : none
�������ܣ���ȡ��������ֵ�����㳵���ٶȣ���λm/s
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
	  //Retrieves the original data of the encoder
	  //��ȡ��������ԭʼ����
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; 
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

	  //Decide the encoder numerical polarity according to different car models
		//���ݲ�ͬС���ͺž�����������ֵ����
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Omni_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Akm_Car:       Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break;
			case Diff_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
			case FourWheel_Car: Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr=OriginalEncoder.B; Encoder_C_pr= -OriginalEncoder.C;  Encoder_D_pr= -OriginalEncoder.D; break; 
			case Tank_Car:      Encoder_A_pr=OriginalEncoder.A; Encoder_B_pr= -OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; 
		}
		
		//The encoder converts the raw data to wheel speed in m/s
		//������ԭʼ����ת��Ϊ�����ٶȣ���λm/s
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; 
}
/**************************************************************************
Function: Smoothing the three axis target velocity
Input   : Three-axis target velocity
Output  : none
�������ܣ�������Ŀ���ٶ���ƽ������
��ڲ���������Ŀ���ٶ�
����  ֵ����
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;
	
	if(PS2_ON_Flag)
	{
		step=0.05;
	}
	else
	{
		step=0.01;
	}
	
	if	   (vx>0) 	smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9f;
	
	if	   (vy>0)   smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9f;
	
	if	   (vz>0) 	smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9f;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
Function: Floating-point data calculates the absolute value
Input   : float
Output  : The absolute value of the input number
�������ܣ����������ݼ������ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

u32 int_abs(int a)
{
	u32 temp;
	if(a<0) temp=-a;
	else temp = a;
	return temp;
}

/**************************************************************************
Function: Prevent the potentiometer to choose the wrong mode, resulting in initialization error caused by the motor spinning.Out of service
Input   : none
Output  : none
�������ܣ���ֹ��λ��ѡ��ģʽ�����³�ʼ���������������ת����ֹͣʹ��
��ڲ�������
����  ֵ����
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	//If the output is close to full amplitude for 6 times in a row, it is judged that the motor rotates wildly and makes the motor incapacitated
	//�������6�νӽ�����������ж�Ϊ�����ת���õ��ʧ��	
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  
}

/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
�������ܣ���ֱ���״����pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
//��ֱ���״�������pid

float Along_Adjust_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	Integral_bias+=Bias;	                                 			//���ƫ��Ļ���
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == FourWheel_Car)
		OutPut=FourWheel_Along_Distance_KP*Bias/100000+FourWheel_Along_Distance_KI*Integral_bias/100000+FourWheel_Along_Distance_KD*(Bias-Last_Bias)/100000;//λ��ʽPID������
	else if(Car_Mode == Akm_Car)
		OutPut=Akm_Along_Distance_KP*Bias/100000+Akm_Along_Distance_KI*Integral_bias/100000+Akm_Along_Distance_KD*(Bias-Last_Bias)/1000;//λ��ʽPID������
	else if(Car_Mode == Diff_Car || Car_Mode == Tank_Car)
		OutPut=Diff_Along_Distance_KP*Bias/100+Diff_Along_Distance_KI*Integral_bias/100+Diff_Along_Distance_KD*(Bias-Last_Bias);//λ��ʽPID������
	else
	  OutPut=-Along_Distance_KP*Bias/100000-Along_Distance_KI*Integral_bias/100000-Along_Distance_KD*(Bias-Last_Bias)/1000;//λ��ʽPID������
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	if(Turn_Off(Voltage)== 1)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                          	
}


/**************************************************************************
Function: Distance_Adjust_PID
Input   : Current_Distance;Target_Distance
Output  : OutPut
�������ܣ������״����pid
��ڲ���: ��ǰ�����Ŀ�����
����  ֵ�����Ŀ���ٶ�
**************************************************************************/	 	
//�����״�������pid
float Distance_Adjust_PID(float Current_Distance,float Target_Distance)//�������PID
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Distance-Current_Distance;                          	//����ƫ��
	Integral_bias+=Bias;	                                 			//���ƫ��Ļ���
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	OutPut=Distance_KP*Bias/100+Distance_KI*Integral_bias/100+Distance_KD*(Bias-Last_Bias)/100;//λ��ʽPID������
	Last_Bias=Bias;                                       		 			//������һ��ƫ��
	if(Turn_Off(Voltage)== 1)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                          	
}

/**************************************************************************
Function: Follow_Turn_PID
Input   : Current_Angle;Target_Angle
Output  : OutPut
�������ܣ������״�ת��pid
��ڲ���: ��ǰ�ǶȺ�Ŀ��Ƕ�
����  ֵ�����ת���ٶ�
**************************************************************************/	 	
//�����״�ת��pid
float Follow_Turn_PID(float Current_Angle,float Target_Angle)
{
	static float Bias,OutPut,Integral_bias,Last_Bias;
	Bias=Target_Angle-Current_Angle;                         				 //����ƫ��
	Integral_bias+=Bias;	                                 				 //���ƫ��Ļ���
	if(Integral_bias>1000) Integral_bias=1000;
	else if(Integral_bias<-1000) Integral_bias=-1000;
	if(Car_Mode == Akm_Car || Car_Mode == Omni_Car)
		OutPut=(Follow_KP_Akm/100)*Bias+(Follow_KI_Akm/100)*Integral_bias+(Follow_KD_Akm/100)*(Bias-Last_Bias);	//λ��ʽPID������
	else
	  OutPut=(Follow_KP/100)*Bias+(Follow_KI/100)*Integral_bias+(Follow_KD/100)*(Bias-Last_Bias);	//λ��ʽPID������
	Last_Bias=Bias;                                       					 		//������һ��ƫ��
	if(Turn_Off(Voltage)== 1)								//����رգ���ʱ��������
		Integral_bias = 0;
	return OutPut;                                           					 	//���
	
}

