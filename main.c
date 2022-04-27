#include "stm32f10x.h"
#include "GPIOLIKE51.h"
#include "adc_usart.h"
#include "adc.h"
#include "i2c.h"
#include "delay.h"

float TEMP_SET=36.0;
float TEMP_CURRENT=36.0;
float TEMP_START=0;
int  V_5v=50000;
float Correct_r=10;
int TR=0;
float DeltaT;
float ERRORT;
int temp_on=0;
int temp_off=0;

float kp=0.00;
float ki=-0.008;
float kd=0;
float DeltaP=0;
float DeltaI=0;
float DeltaD=0;
float T=1;
float T1=0;
float T2=0;
float DeltaT=0.2;

#define  TEMP_ON()      GPIO_SetBits(GPIOC, GPIO_Pin_13)
#define  TEMP_OFF()    GPIO_ResetBits(GPIOC, GPIO_Pin_13)

extern RxTxBuffer_t Data1;


void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE); 						 
//=============================================================================
//LED -> PC13
//=============================================================================			 
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void Delay(uint32_t nCount)
{
  for(; nCount != 0; nCount--);
}

void hardware_init()
{
	SystemInit();
	GPIO_Configuration();
	uart_init(115200);    //串口初始化，波特率115200
	//I2C_Initial();
}


void process_temp(float temp)
{

      //float deltaT=0;
      float k1=0.1;
      float k2=0.1;
      float errort=0;
      float t=0;
      //deltaT=k1*TEMP_SET+k2;
	   
	if(TEMP_START==1)//打开温控
	{
		//Send_data(1);
		errort=TEMP_SET-TEMP_CURRENT;//当前误差

              //如果温度低，开始加热
              if(errort>0)
              {
              	if(temp_on==0) //没加热，开加热
			{
				
				TEMP_ON();//打开加热
				temp_on=1;
				T1=0;
			}
			else
				T1=T1+DeltaT;//更新T1

                      t=kp*T2+ki*errort;//更新加热时间
                      
			//如果没达到加热时间，打开加热
			if(T1<t)
			{
				TEMP_ON();//打开加热
				temp_off=0;
			}

			//达到加热时间，关闭加热
			else
			{
				TEMP_OFF();//关闭加热
				temp_on=0;
				temp_off=1;
			}


		}

	      //如果温度高，关闭加热
		else
		{
			if(temp_off==0) //没有关闭，关闭加热
			{
				TEMP_OFF();
				temp_off=1;
				T2=0;//根据实际情况，是否保留上次时间。
			}
			else
			{
				T2=T2+DeltaT; //累计散热时间
			}

		}
  

	}
	else if(TEMP_START==0)//关温控
	{
		//Send_data(0);
		TEMP_OFF();
	}

}


//处理上位机命令
void Process_uart(void)
{
       s16 data_len;
	u8 head1;
	u8 head2;
	u8 cmd;
  	u8 temp0;
  	u8 temp1;
  	u8 temp2;
  	u8 temp3;
	float   temp;  
	unsigned char  i;   
	void   *pf;  
	
	while(1)  //TODO add a loop buffer 
	{
		if(Data1.RxCount>=Data1.RxIndex) //not loop
		{
			data_len=Data1.RxCount-Data1.RxIndex;
		}
		else
		{
			data_len=BufferMAX-Data1.RxIndex+Data1.RxCount;
		}

		if(data_len>=7)//enough data     // 0x55+0x55+ON/OFF+TEMP
		{
			//data index
			head1=Data1.RxIndex;
			if((head1+1)>=BufferMAX) 
				head2=0;
			else
				head2=head1+1;
			if((head2+1)>=BufferMAX)
				cmd=0;
			else
				cmd=head2+1;
			if((cmd+1)>=BufferMAX)
				temp0=0;
			else
				temp0=cmd+1;
			if((temp0+1)>=BufferMAX)
				temp1=0;
			else
				temp1=temp0+1;
			if((temp1+1)>=BufferMAX)
				temp2=0;
			else
				temp2=temp1+1;
			if((temp2+1)>=BufferMAX)
				temp3=0;
			else
				temp3=temp2+1;
			
			if(Data1.RxBuffer[head1]==0x55&&Data1.RxBuffer[head2]==0x55) //0x55+0x55+cmd+temp
			{
					cmd	=Data1.RxBuffer[cmd];
					pf = &temp;   
					for(i=0;i<4;i++)  
					{  
    						*((u8*)pf+i)=Data1.RxBuffer[temp0+i];       
					}
					
					if(cmd==1)    //start
					{
						TEMP_SET=temp;  //set temp,compute delat
						TEMP_START=1;
						DeltaP=0;
						DeltaI=0;
						DeltaD=(TEMP_SET-TEMP_CURRENT)/T;	

						temp_on=1;
						temp_off=1;
						DeltaT=0.1;
						ERRORT=0;
					}
					else if(cmd==0) //stop
					{
						TEMP_START=0;
					}
					else if(cmd==3) //set temp
					{
						TEMP_SET=temp;;
					}
					Data1.RxIndex+=4; //move to next frame
					if(Data1.RxIndex>=BufferMAX)Data1.RxIndex=(Data1.RxIndex-BufferMAX);
			}
			else
			{
				Data1.RxIndex+=1;//next data
				if(Data1.RxIndex>=BufferMAX)Data1.RxIndex=0;//loop
				continue;
			}
		}
		else //not enough data
		{
			break;			
		}
	}

}


int main(void)
{
	float temp;

	//初始化变量
	DeltaT=0;
  	ERRORT=0;
  	temp_on=0;
  	temp_off=0;
	//初始化硬件
	hardware_init();
	delay_init();

	TEMP_OFF();//关闭温控
	//初始化ad
	adc_init();
	
       while(1)
 	{
           Process_uart(); //process commands
 	    temp=adc_simple();//sample temprature
 	    TEMP_CURRENT=temp;
	    process_temp(temp); //control temp

	    delay_ms(1000);
      	}

}

