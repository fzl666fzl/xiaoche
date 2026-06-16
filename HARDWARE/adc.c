#include "adc.h"

float Voltage,Voltage_Count,Voltage_All; //Variables related to battery voltage sampling //电池电压采样相关的变量  
const float Revise=0.99;

//CCD相关变量
u16 CCD_Median,CCD_Threshold,ADV[128]={0};

#define CCD_CENTER                      64
#define CCD_EDGE_BEGIN                  5
#define CCD_EDGE_END                    123
#define CCD_MIN_CONTRAST                8
#define CCD_MIN_LINE_WIDTH              4
#define CCD_MAX_LINE_WIDTH              90
#define CCD_LOST_HOLD_FRAMES  75
#define OPENMV_LOST_HOLD_FRAMES          75
#define CCD_MAX_MEDIAN_STEP             64
#define CCD_ZEBRA_MIN_CONTRAST          35
#define CCD_ZEBRA_MIN_RUN_WIDTH         3
#define CCD_ZEBRA_MAX_RUN_WIDTH         14
#define CCD_ZEBRA_CONFIRM_FRAMES        2
#define CCD_ZEBRA_WIDTH_MAX_CHANGE      4
#define CCD_ZEBRA_SIDE_MARGIN           2
#define CCD_ZEBRA_SIDE_WHITE_POINTS     5
#define OPENMV_ONLY_LINE_PATROL          1

u8 CCD_Zebra_Stop_Flag=0;
static u8 Lost_Line_Count=CCD_LOST_HOLD_FRAMES;
static u8 CCD_Last_Line_Type=0;
static u8 OpenMV_Lost_Count=OPENMV_LOST_HOLD_FRAMES;
static u16 OpenMV_Last_Median=CCD_CENTER;
static u8 CCD_Zebra_Confirm_Count=0;
static u16 CCD_Zebra_Last_Main_Width=0;
static u8 CCD_Zebra_Last_Main_Valid=0;

static u8 OpenMV_Use_Line_Data(void);

static u8 CCD_Find_Line(u8 bright_line,u16 *median);
static void CCD_Lost_Line_Process(u16 *last_median);
static u8 CCD_Find_Zebra_Main_Run(u16 threshold,u16 *left,u16 *right,u16 *width);
static u8 CCD_Count_White_Points(u16 begin,u16 end,u16 threshold);
static void CCD_Detect_Zebra(void); 

//电磁巡线相关变量																										
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor,sum;

/**************************************************************************
Function: ADC initializes battery voltage detection
Input   : none
Output  : none
函数功能：ADC初始化电池电压检测
入口参数：无
返回  值：无
**************************************************************************/
void  Adc_Init(void)
{  
		GPIO_InitTypeDef       GPIO_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		ADC_InitTypeDef       ADC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOA时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //使能ADC1时钟


		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;//PB0 通道8
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
		GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化  

		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	 

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
		ADC_CommonInit(&ADC_CommonInitStructure);//初始化

		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
		ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
		ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
		ADC_Cmd(ADC1, ENABLE);//开启AD转换器	 
}		


void  Adc_POWER_Init(void)
{  
		GPIO_InitTypeDef       GPIO_InitStructure;
		ADC_CommonInitTypeDef ADC_CommonInitStructure;
		ADC_InitTypeDef       ADC_InitStructure;

		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOA时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE); //使能ADC1时钟


		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;//PB0 通道8
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
		GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化  

		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,ENABLE);	  //ADC2复位
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC2,DISABLE);	//复位结束	 

		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div6;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
		ADC_CommonInit(&ADC_CommonInitStructure);//初始化

		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
		ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
		ADC_Init(ADC2, &ADC_InitStructure);//ADC初始化
		ADC_Cmd(ADC2, ENABLE);//开启AD转换器	 
}		
/**************************************************************************
Function: The AD sampling
Input   : The ADC channels
Output  : AD conversion results
函数功能：AD采样
入口参数：ADC的通道
返回  值：AD转换结果
**************************************************************************/
u16 Get_Adc(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//设置指定ADC的规则组通道，一个序列，采样时间
	
	//ADC1,ADC通道,采样时间为480周期
	ADC_RegularChannelConfig(ADC1, ch, 1, ADC_SampleTime_480Cycles );
  //Enable the specified ADC1 software transformation startup function	
  //使能指定的ADC1的软件转换启动功能	
	ADC_SoftwareStartConv(ADC1);
	//Wait for the conversion to finish
  //等待转换结束	
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC ));
	//Returns the result of the last ADC1 rule group conversion
	//返回最近一次ADC1规则组的转换结果
	return ADC_GetConversionValue(ADC1);	
}

u16 Get_Adc2(u8 ch)   
{
	//Sets the specified ADC rule group channel, one sequence, and sampling time
	//设置指定ADC的规则组通道，一个序列，采样时间
	
	//ADC2,ADC通道,采样时间为480周期
	ADC_RegularChannelConfig(ADC2, ch, 1, ADC_SampleTime_480Cycles );
  //Enable the specified ADC2 software transformation startup function	
  //使能指定的ADC1的软件转换启动功能	
	ADC_SoftwareStartConv(ADC2);
	//Wait for the conversion to finish
  //等待转换结束	
	while(!ADC_GetFlagStatus(ADC2, ADC_FLAG_EOC ));
	//Returns the result of the last ADC2 rule group conversion
	//返回最近一次ADC1规则组的转换结果
	return ADC_GetConversionValue(ADC2);	
}

/**************************************************************************
Function: Collect multiple ADC values to calculate the average function
Input   : ADC channels and collection times
Output  : AD conversion results
函数功能：采集多次ADC值求平均值函数
入口参数：ADC通道和采集次数
返 回 值：AD转换结果
**************************************************************************/
u16 Get_adc_Average(u8 chn, u8 times)
{
  u32 temp_val=0;
	u8 t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(chn);
		delay_ms(5);
	}
	return temp_val/times;
}

/**************************************************************************
Function: Read the battery voltage
Input   : none
Output  : Battery voltage in mV
函数功能：读取电池电压 
入口参数：无
返回  值：电池电压，单位mv
**************************************************************************/
float Get_battery_volt(void)   
{  
	float Volt;
	
	//The resistance partial voltage can be obtained by simple analysis according to the schematic diagram
	//电阻分压，具体根据原理图简单分析可以得到	
	Volt=Get_Adc2(Battery_Ch)*3.3*11.0*Revise/1.0/4096;	
	return Volt;
}

/**************************************************************************
Function: Adc data acquisition and processing 
Input   : none
Output  : none
函数功能：adc数据采集处理 
入口参数：无
返回  值：无
**************************************************************************/
void adc_task(void *pvParameters)									
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			//The task is run at 50hz
			//此任务以50Hz的频率运行
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ));

			if(Mode  != CCD_Line_Patrol_Mode)
			{
				CCD_Zebra_Confirm_Count=0;
				CCD_Zebra_Stop_Flag=0;
				CCD_Zebra_Last_Main_Valid=0;
				OpenMV_Lost_Count=OPENMV_LOST_HOLD_FRAMES;
				OpenMV_Last_Median=CCD_CENTER;
			}

			if(Mode  == CCD_Line_Patrol_Mode)
			{
#if OPENMV_ONLY_LINE_PATROL
				OpenMV_Use_Line_Data();
#else
				if(!OpenMV_Use_Line_Data())
				{
					RD_TSL();                                                                  //数据采集处理
					Find_CCD_Median ();                                      //===提取中线 
				}
#endif
			}

			else if(Mode  == ELE_Line_Patrol_Mode)
			{
				Sensor_Left=Get_Adc(14);
			  Sensor_Right=Get_Adc(4);
			  Sensor_Middle=Get_Adc(5);
					
			  sum=Sensor_Left*1+Sensor_Middle*100+Sensor_Right*199;		//归一化处理
			  Sensor=sum/(Sensor_Left+Sensor_Middle+Sensor_Right);		//提取中线
			}
		}
}


static u8 OpenMV_Use_Line_Data(void)
{
	if(!OpenMV_Line_Available)
	{
		CCD_Median=CCD_CENTER;
		CCD_Zebra_Stop_Flag=0;
		return 0;
	}

	if(OpenMV_Stop)
	{
		CCD_Median=OpenMV_Median;
		OpenMV_Last_Median=OpenMV_Median;
		CCD_Zebra_Stop_Flag=1;
		return 1;
	}

	if(OpenMV_Lost)
	{
		if(OpenMV_Lost_Count<255) OpenMV_Lost_Count++;
		CCD_Median=OpenMV_Last_Median;
		CCD_Zebra_Stop_Flag=0;
	}
	else
	{
		CCD_Median=OpenMV_Median;
		OpenMV_Last_Median=OpenMV_Median;
		OpenMV_Lost_Count=0;
		CCD_Zebra_Stop_Flag=0;
	}

	return 1;
}
/**************************************************************************
函数功能：线性CCD初始化
入口参数：无
返回  值：无
**************************************************************************/
void  ccd_Init(void)
{    
 //先初始化ADC1通道7 IO口
	   GPIO_InitTypeDef        GPIO_InitStructure;
    ADC_InitTypeDef         ADC_InitStructure;
    ADC_CommonInitTypeDef   ADC_CommonInitStructure;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOC时钟
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);    // 使能ADC1时钟
   
    /*=============================配置ADC对应的GPIO=============================*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PC1
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
		GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化  
		
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//2MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
		GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化
	
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;//普通输出模式
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//2MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
		GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
   
    /*=============================ADC时钟使能=============================*/

    RCC_AHB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);    // ADC1复位
    RCC_AHB2PeriphResetCmd(RCC_APB2Periph_ADC1, DISABLE);   // 复位结束
   
		ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//独立模式
		ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;//两个采样阶段之间的延迟5个时钟
		ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMA失能
		ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//预分频4分频。ADCCLK=PCLK2/4=84/4=21Mhz,ADC时钟最好不要超过36Mhz 
		ADC_CommonInit(&ADC_CommonInitStructure);//初始化
		
		ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
		ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
		ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
		ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
		ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
		ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
		ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
		ADC_Cmd(ADC1, ENABLE);//开启AD转换器	
}	

/**************************************************************************
函数功能：延时
入口参数：无
返回  值：无
**************************************************************************/
void Dly_us(void)
{
   int ii;    
   for(ii=0;ii<30;ii++); 
}
/**************************************************************************
函数功能：CCD数据采集
入口参数：无
返回  值：无
**************************************************************************/
 void RD_TSL(void) 
{
  u8 i=0,tslp=0;
  TSL_CLK=0;
  TSL_SI=0; 
  Dly_us();
      
  TSL_SI=1;
  Dly_us();	
	
  TSL_CLK=0;
  Dly_us();
	
      
  TSL_CLK=1;
	Dly_us(); 
	
  TSL_SI=0;
  Dly_us();
  for(i=0;i<128;i++)
  { 
    TSL_CLK=0; 
    Dly_us();  //调节曝光时间

    ADV[tslp]=(Get_Adc(4))>>4;
    ++tslp;
    TSL_CLK=1;
     Dly_us();

  }  
}
/**************************************************************************
函数功能：线性CCD取中值
入口参数：无
返回  值：无
**************************************************************************/
static u8 CCD_Find_Line(u8 bright_line,u16 *median)
{
	 u16 i,j,Left=0,Right=0,width;
	 u8 edge;

	 for(i = 5;i<118; i++)
	 {
		 if(bright_line)
		 {
			 edge=(ADV[i]<CCD_Threshold &&ADV[i+1]<CCD_Threshold &&ADV[i+2]<CCD_Threshold &&ADV[i+3]>CCD_Threshold &&ADV[i+4]>CCD_Threshold &&ADV[i+5]>CCD_Threshold);
		 }
		 else
		 {
			 edge=(ADV[i]>CCD_Threshold &&ADV[i+1]>CCD_Threshold &&ADV[i+2]>CCD_Threshold &&ADV[i+3]<CCD_Threshold &&ADV[i+4]<CCD_Threshold &&ADV[i+5]<CCD_Threshold);
		 }
		 if(edge)
		 {
			 Left=i+2;
			 break;
		 }
	 }
	 for(j = 118;j>5; j--)
   {
		 if(bright_line)
		 {
			 edge=(ADV[j]>CCD_Threshold &&ADV[j+1]>CCD_Threshold &&ADV[j+2]>CCD_Threshold &&ADV[j+3]<CCD_Threshold &&ADV[j+4]<CCD_Threshold &&ADV[j+5]<CCD_Threshold);
		 }
		 else
		 {
			 edge=(ADV[j]<CCD_Threshold &&ADV[j+1]<CCD_Threshold &&ADV[j+2]<CCD_Threshold &&ADV[j+3]>CCD_Threshold &&ADV[j+4]>CCD_Threshold &&ADV[j+5]>CCD_Threshold);
		 }
		 if(edge)
		 {
		   Right=j+2;
		   break;
		 }
   }
	 if(Left==0 || Right==0 || Right<=Left) return 0;
	 width=Right-Left;
	 if(width<CCD_MIN_LINE_WIDTH || width>CCD_MAX_LINE_WIDTH) return 0;
	 *median=(Left+Right)/2;
	 return 1;
}

static void CCD_Lost_Line_Process(u16 *last_median)
{
	 if(Lost_Line_Count<CCD_LOST_HOLD_FRAMES)
	 {
		 Lost_Line_Count++;
		 CCD_Median=*last_median;
	 }
	 else
	 {
		 CCD_Median=CCD_CENTER;
		 *last_median=CCD_CENTER;
	 }
}

static u8 CCD_Find_Zebra_Main_Run(u16 threshold,u16 *left,u16 *right,u16 *width)
{
	 u16 i,run=0,run_start=0,center,delta,best_delta=128;
	 u8 found=0;

	 for(i=CCD_EDGE_BEGIN;i<CCD_EDGE_END;i++)
	 {
		 if(ADV[i]>threshold)
		 {
			 if(run==0) run_start=i;
			 run++;
		 }
		 else
		 {
			 if(run>=CCD_ZEBRA_MIN_RUN_WIDTH && run<=CCD_ZEBRA_MAX_RUN_WIDTH)
			 {
				 center=(run_start+i-1)/2;
				 delta=(center>CCD_CENTER) ? center-CCD_CENTER : CCD_CENTER-center;
				 if(found==0 || delta<best_delta)
				 {
					 *left=run_start;
					 *right=i-1;
					 *width=run;
					 best_delta=delta;
					 found=1;
				 }
			 }
			 run=0;
		 }
	 }
	 if(run>=CCD_ZEBRA_MIN_RUN_WIDTH && run<=CCD_ZEBRA_MAX_RUN_WIDTH)
	 {
		 center=(run_start+CCD_EDGE_END-1)/2;
		 delta=(center>CCD_CENTER) ? center-CCD_CENTER : CCD_CENTER-center;
		 if(found==0 || delta<best_delta)
		 {
			 *left=run_start;
			 *right=CCD_EDGE_END-1;
			 *width=run;
			 found=1;
		 }
	 }
	 return found;
}

static u8 CCD_Count_White_Points(u16 begin,u16 end,u16 threshold)
{
	 u16 i;
	 u8 count=0;
	 if(begin>=end) return 0;
	 for(i=begin;i<end;i++)
	 {
		 if(ADV[i]>threshold) count++;
	 }
	 return count;
}

static void CCD_Detect_Zebra(void)
{
	 u16 i,value1_max,value1_min,threshold;
	 u16 left=0,right=0,width=0,left_end,right_begin;
	 u8 left_points,right_points,width_stable=0;

	 value1_max=ADV[CCD_EDGE_BEGIN];
	 value1_min=ADV[CCD_EDGE_BEGIN];
	 for(i=CCD_EDGE_BEGIN;i<CCD_EDGE_END;i++)
	 {
		 if(value1_max<=ADV[i]) value1_max=ADV[i];
		 if(value1_min>=ADV[i]) value1_min=ADV[i];
	 }
	 threshold=(value1_max+value1_min)/2;
	 if(value1_max-value1_min<CCD_ZEBRA_MIN_CONTRAST)
	 {
		 CCD_Zebra_Confirm_Count=0;
		 CCD_Zebra_Last_Main_Valid=0;
		 return;
	 }

	 if(CCD_Find_Zebra_Main_Run(threshold,&left,&right,&width)==0)
	 {
		 CCD_Zebra_Confirm_Count=0;
		 CCD_Zebra_Last_Main_Valid=0;
		 return;
	 }

	 if(left>CCD_EDGE_BEGIN+CCD_ZEBRA_SIDE_MARGIN) left_end=left-CCD_ZEBRA_SIDE_MARGIN;
	 else left_end=CCD_EDGE_BEGIN;
	 if(right+CCD_ZEBRA_SIDE_MARGIN+1<CCD_EDGE_END) right_begin=right+CCD_ZEBRA_SIDE_MARGIN+1;
	 else right_begin=CCD_EDGE_END;

	 left_points=CCD_Count_White_Points(CCD_EDGE_BEGIN,left_end,threshold);
	 right_points=CCD_Count_White_Points(right_begin,CCD_EDGE_END,threshold);

	 if(CCD_Zebra_Last_Main_Valid==0)
	 {
		 width_stable=1;
	 }
	 else if(width>CCD_Zebra_Last_Main_Width)
	 {
		 if(width-CCD_Zebra_Last_Main_Width<=CCD_ZEBRA_WIDTH_MAX_CHANGE) width_stable=1;
	 }
	 else
	 {
		 if(CCD_Zebra_Last_Main_Width-width<=CCD_ZEBRA_WIDTH_MAX_CHANGE) width_stable=1;
	 }

	 CCD_Zebra_Last_Main_Width=width;
	 CCD_Zebra_Last_Main_Valid=1;

	 if(width_stable && left_points>=CCD_ZEBRA_SIDE_WHITE_POINTS && right_points>=CCD_ZEBRA_SIDE_WHITE_POINTS)
	 {
		 if(CCD_Zebra_Confirm_Count<CCD_ZEBRA_CONFIRM_FRAMES) CCD_Zebra_Confirm_Count++;
		 if(CCD_Zebra_Confirm_Count>=CCD_ZEBRA_CONFIRM_FRAMES) CCD_Zebra_Stop_Flag=1;
	 }
	 else
	 {
		 CCD_Zebra_Confirm_Count=0;
	 }
}

void  Find_CCD_Median (void)
{ 
	 static u16 Last_CCD_Median=CCD_CENTER;
	 u16 i,value1_max,value1_min;
	 u16 Dark_Median=CCD_CENTER,Bright_Median=CCD_CENTER,Candidate_Median=CCD_CENTER;
	 u16 Dark_Delta,Bright_Delta;
	 u8 Dark_Found,Bright_Found,Candidate_Line_Type=0;
	 //阈值说明：CCD采集回来的128个数据，每个数据单独与阈值进行比较，比阈值大为白色，比阈值小为黑色
	 //动态阈值算法，读取每次采集数据的最大和最小值的平均数作为阈值 
	 value1_max=ADV[CCD_EDGE_BEGIN];  
   for(i=CCD_EDGE_BEGIN;i<CCD_EDGE_END;i++)   //两边各去掉5个点
     {
       if(value1_max<=ADV[i])
       value1_max=ADV[i];
     }
	 value1_min=ADV[CCD_EDGE_BEGIN];  //最小值
   for(i=CCD_EDGE_BEGIN;i<CCD_EDGE_END;i++) 
     {
       if(value1_min>=ADV[i])
       value1_min=ADV[i];
     }
   CCD_Threshold =(value1_max+value1_min)/2;	  //计算出本次中线提取的阈值
	 CCD_Detect_Zebra();
	 if(value1_max-value1_min<CCD_MIN_CONTRAST)
	 {
		 CCD_Lost_Line_Process(&Last_CCD_Median);
		 return;
	 }

	 Dark_Found=CCD_Find_Line(0,&Dark_Median);
	 Bright_Found=CCD_Find_Line(1,&Bright_Median);
	 if(Dark_Found && Bright_Found)
	 {
		 Dark_Delta=(Dark_Median>Last_CCD_Median) ? Dark_Median-Last_CCD_Median : Last_CCD_Median-Dark_Median;
		 Bright_Delta=(Bright_Median>Last_CCD_Median) ? Bright_Median-Last_CCD_Median : Last_CCD_Median-Bright_Median;
		 if(CCD_Last_Line_Type==1 && Bright_Delta<=Dark_Delta)
		 {
			 Candidate_Median=Bright_Median;
			 Candidate_Line_Type=1;
		 }
		 else if(Dark_Delta<=Bright_Delta)
		 {
			 Candidate_Median=Dark_Median;
			 Candidate_Line_Type=0;
		 }
		 else
		 {
			 Candidate_Median=Bright_Median;
			 Candidate_Line_Type=1;
		 }
	 }
	 else if(Dark_Found)
	 {
		 Candidate_Median=Dark_Median;
		 Candidate_Line_Type=0;
	 }
	 else if(Bright_Found)
	 {
		 Candidate_Median=Bright_Median;
		 Candidate_Line_Type=1;
	 }
	 else
	 {
		 CCD_Lost_Line_Process(&Last_CCD_Median);
		 return;
	 }

	 if(Candidate_Median>Last_CCD_Median+CCD_MAX_MEDIAN_STEP)
	 {
		 Candidate_Median=Last_CCD_Median+CCD_MAX_MEDIAN_STEP;
	 }
	 else if(Last_CCD_Median>Candidate_Median+CCD_MAX_MEDIAN_STEP)
	 {
		 Candidate_Median=Last_CCD_Median-CCD_MAX_MEDIAN_STEP;
	 }
	 CCD_Median=Candidate_Median;
	 Last_CCD_Median=Candidate_Median;
	 CCD_Last_Line_Type=Candidate_Line_Type;
}

/**************************************************************************
函数功能：电磁传感器采样初始化
入口参数：无
返回  值：无
**************************************************************************/
void  ele_Init(void)
{    
//	ADC_InitTypeDef ADC_InitStructure; 
	GPIO_InitTypeDef GPIO_InitStructure;
//	ADC_CommonInitTypeDef   ADC_CommonInitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOA,ENABLE);	  //使能GPIOC时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1,ENABLE);	    //使能ADC1通道时钟
                      
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;//PC4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化  
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;//PC4
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//模拟输入
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//不带上下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化
   
//	ADC_DeInit();  //复位ADC1,将外设 ADC1 的全部寄存器重设为缺省值
//	
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,ENABLE);	  //ADC1复位
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1,DISABLE);	//复位结束	
//	/*=============================ADC公共部分配置=============================*/
//    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;      // 独立模式单通道
//    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;   // ADC采样时钟21MHz
//    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;    // DMA失能
//    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;    // 两个采样延时5个Tadcclk,此配置仅在交替模式中生效
//    ADC_CommonInit(&ADC_CommonInitStructure);
//		
//	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12位模式
//	ADC_InitStructure.ADC_ScanConvMode = DISABLE;//非扫描模式	
//	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//关闭连续转换
//	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//禁止触发检测，使用软件触发
//	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//右对齐	
//	ADC_InitStructure.ADC_NbrOfConversion = 1;//1个转换在规则序列中 也就是只转换规则序列1 
//	ADC_Init(ADC1, &ADC_InitStructure);//ADC初始化
//	ADC_Cmd(ADC1, ENABLE);//开启AD转换器
	
	
}	


