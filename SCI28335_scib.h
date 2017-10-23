#include "DSP2833x_Device.h"
/*
SCIStart双口RAM中标识2407已经接受到PC机的串口中断，1代表接受到串口中断，2代表需要完成参数的在线修改
Add_Edit	在线修改参数地址
Add_Val		在线修改参数值
Add_Channel	4个通道的地址，依次存储
Val_Channel	4个通道的值，依次存储
*/

/*#define SCIStart 	(unsigned int *)0x8020
#define Add_Edit 	(unsigned int *)0x8021
#define Add_Val 	(unsigned int *)0x8022
#define Add_Channel 	(unsigned int *)0x8023
#define Val_Channel 	(unsigned int *)0x8033
*/
unsigned int SCIStart;
unsigned int *Add_Edit;
int Add_Val = 0;
unsigned int Add_Channel[4];
unsigned int Val_Channel[4];

/*
串行中断中变量定义
*/

unsigned  int	buffer_length = 24120;//24120;
unsigned  int	BufPointer_PWM;				/*savebuffer中使用，标识保存四个通道数据在BUFFERDATA中位置*/
unsigned  int	BufPointer_SCI;				/*TXINT中标识发送数据在BUFFERDATA中位置*/
unsigned  int	FramePointer;				/*标识每帧发送数据发送地位置*/
unsigned  int	EnableWrite;				/*标识是否将四个通道当前值存入BUFFERDATA中*/
  int	send_char;				/*存储需要发送地字*/
unsigned  int	received_char;				/*存储接收地字*/
unsigned  int	in_frame;				/*标识是否接收到数据头，1 代表接收到*/
unsigned  int	bytecount;				/*标识已经接收到PC机一组数据地位置*/
unsigned  int	ch1_addr;				/*Channel 1 地址*/
unsigned  int	ch2_addr;				/*Channel 2 地址*/
unsigned  int	ch3_addr;				/*Channel 3 地址*/
unsigned  int	ch4_addr;				/*Channel 4 地址*/
unsigned  int	edit_addr;				/*接收到地本组数据中修改地变量地地址*/
  int	edit_val;				/*接收道德本组数据中修改变量地修改值*/
unsigned  int	buffer[24];				/*存储接收到地PC机地一组数据*/
unsigned  int	addr_last;				/*存储上次修改地变量地地址*/
  int	val_last;				/*存储上次修改地变量地值*/
unsigned  int	SampleCounter;				/*标识两个采样点之间间隔点数*/
unsigned  int	Clock;					/*标识时间，使用在计算单次触发开始时间*/
unsigned  int	SampleTime;				/*采样点数*/
unsigned  int	MultiSingle;				/*标识多次、单次触发，1 代表多次*/
unsigned  int	StartTime;				/*标识单次触发开始时间*/
//unsigned  int	BUFFERDATA[160];			/*四个通道值存储数组*/
unsigned  int	change_online;				/*在线修改变量使用，标识已经通知C33 有变量修改保持时间，2个中断*/
unsigned  int	S01_K = 1000;				/*单次触发计算开始时间使用，代表0.1s*/
unsigned  int	S01;					/*单次触发计算开始时间*/  
unsigned  int   storePIVR;

//unsigned  int   timess;                  /*calculat interrupt times*/

#pragma	DATA_SECTION(BUFFERDATA,"data");
//unsigned  int   BUFFERDATA[24120];
unsigned  int   BUFFERDATA[24120];
/*
以下为2407侧程序
*/

/*====================================串口初始化程序=========================================*/
/*		在文件DSP28Sci.c和文件interrupt.c中实现,在文件aci_time.c中设置和调用	     */
/*===========================================================================================*/

#pragma CODE_SECTION(DATATODUALRAM, "ramfuncs")
void	DATATODUALRAM(void)				/*在RXINT接收到全部数据后调用，将数据送给C33*/
{
	*Add_Channel = ch1_addr;
	*(Add_Channel+1) = ch2_addr;
	*(Add_Channel+2) = ch3_addr;
	*(Add_Channel+3) = ch4_addr; 
//	timess = timess + 1;				/*calculate interrupt times*/
}

#pragma CODE_SECTION(SCOPE_INT, "ramfuncs")
void	SCOPE_INT(void)
{
	edit_addr = 0x0000;
	addr_last = 0x0000;
	edit_val = 0x0000;
	val_last = 0x0000;
	ch1_addr = 0x0000;
	ch2_addr = 0x0000;
	ch3_addr = 0x0000;
	ch4_addr = 0x0000;
	Clock = 0x0000;					
	MultiSingle = 0x0001;
	SampleTime = 0x0001;
	StartTime = 0x0001;
	SCIStart = 1;
//	timess = 0;                          /*time*/
}

#pragma CODE_SECTION(SCOPE_RST, "ramfuncs")
void 	SCOPE_RST(void)
{
	BufPointer_SCI = 0x0000;			/*标识SCITX中发送位置*/
	FramePointer = 0x0000;
	EnableWrite = 0x0001;
	SampleCounter = 0x0000;
	in_frame = 0x0000;
	bytecount = 0x0000;
	change_online = 0x0000;
	BufPointer_PWM = 0x0000;
	S01 = 0;
}

/*================================Scope串行接收子程序=======================================*/
#pragma CODE_SECTION(scib_rx_isr, "ramfuncs")
interrupt void scib_rx_isr(void)
{
	received_char = ScibRegs.SCIRXBUF.all;
	if(received_char == 0x00FA)			/*判断数据头*/
	{
		in_frame = 0x0001;
		bytecount = 0x0000;
	}
	else
	{
		if(in_frame == 0x0000)			/*在接收到数据头地时候才执行else中地程序*/
		{}
		else
		{
			buffer[bytecount] = received_char;
			bytecount = bytecount + 0x0001;
		}
		
		if(bytecount == 24)				/*下面地程序需要确认PC机发过来地数据格式才能确认*/
		{
			asm(" CLRC SXM");
			in_frame = 0x0000;
			ch1_addr = buffer[0];
			ch1_addr = ch1_addr + (buffer[1]<<4);
			ch1_addr = ch1_addr + (buffer[2]<<8);
			ch1_addr = ch1_addr + (buffer[3]<<12);
			
			ch2_addr = buffer[4];
			ch2_addr = ch2_addr + (buffer[5]<<4);
			ch2_addr = ch2_addr + (buffer[6]<<8);
			ch2_addr = ch2_addr + (buffer[7]<<12);
			
			ch3_addr = buffer[8];
			ch3_addr = ch3_addr + (buffer[9]<<4);
			ch3_addr = ch3_addr + (buffer[10]<<8);
			ch3_addr = ch3_addr + (buffer[11]<<12);
			
			ch4_addr = buffer[12];
			ch4_addr = ch4_addr + (buffer[13]<<4);
			ch4_addr = ch4_addr + (buffer[14]<<8);
			ch4_addr = ch4_addr + (buffer[15]<<12);
			
			addr_last = edit_addr;
			
			edit_addr = buffer[16];
			edit_addr = edit_addr + (buffer[17]<<4);
			edit_addr = edit_addr + (buffer[18]<<8);
			edit_addr = edit_addr + (buffer[19]<<12);

			val_last = edit_val;
			
			edit_val = buffer[20];
			edit_val = edit_val + (buffer[21]<<4);
			edit_val = edit_val + (buffer[22]<<8);
			edit_val = edit_val + (buffer[23]<<12);
			
			if(edit_addr == 0xFF2A)							/*应该对发来的数据为零零时进行一定的处理*/
			{
				if(addr_last != edit_addr || val_last != edit_val)
				{
					SampleTime = edit_val;
					SCOPE_RST();
					Clock = 0;
				}
			}
			else
			{
				if(edit_addr == 0xFF2B)
				{
					if(addr_last != edit_addr || val_last != edit_val)
					{
						MultiSingle = edit_val;
						SCOPE_RST();
						Clock = 0;
					}
				}
				else
				{
					if(edit_addr == 0xFF2C)
					{
						if(addr_last != edit_addr || val_last != edit_val)
						{
							StartTime = edit_val;
							SCOPE_RST();
							Clock = 0;
						}
					}
					else
					{
						if(addr_last != edit_addr || val_last != edit_val)		/*需要添加SCIStart ＝ 2保持两个周期地标识变量*/
						{   
							if(edit_addr == 0 && edit_val == 0)
							{}
							else
							{
								*Add_Edit = edit_addr;
								Add_Val = edit_val;
								SCIStart = 2;
								change_online = 2;
								}
						}
					}
				}
			}

			
			asm(" SETC SXM");
			
			DATATODUALRAM();				/*将四个通道地地址送到C33*/
		}
		else
		{}
	}
	 PieCtrlRegs.PIEACK.bit.ACK9=1;
}

/*=====================Scope串行发送子程序======================================*/	
/*Frame结构:	head;   (data1;data2;data3;data4);(data1;data2;data3;data4);	*/
/*			(data1;data2;data3;data4);			 	*/
/*		共26个字节.分别对应FramePointer的0至25.			 	*/
/*==============================================================================*/
#pragma CODE_SECTION(scib_tx_isr, "ramfuncs")
interrupt void scib_tx_isr(void)
{
	if(BufPointer_SCI == buffer_length && FramePointer%2 == 0)			/*FramePointer = 26*/
	{
		if(MultiSingle == 1)
		{
			EnableWrite = 0x0001;
		}
		BufPointer_SCI = 0x0000;
		
	}
	else
	{
		if(FramePointer >= 26)							/*帧结束，FramePointer = 0*/
		{
			FramePointer = 0;
		}
		
		if(FramePointer < 2)							/*发送数据头*/
		{
			if(FramePointer == 0)
			{

				ScibRegs.SCITXBUF = 0x00FF;	//
			}
			if(FramePointer == 1)
			{
				
				ScibRegs.SCITXBUF = 0x007F;//
			}
			FramePointer = FramePointer + 1;
		}
		else
		{
			if(FramePointer % 2 == 0)
			{
				send_char = BUFFERDATA[BufPointer_SCI];
				//send_char = 10;/////////for test
				BufPointer_SCI = BufPointer_SCI + 1;
				ScibRegs.SCITXBUF = send_char&0x00FF;				/*发送数据低位字节和高位字节要重新写，确认一下*/

			}
			else
			{
			
				ScibRegs.SCITXBUF = (send_char&0xFF00)>>8;			/*发送高位字节*/

			}
			FramePointer = FramePointer + 1;
				
		}
		
	}                              
	 PieCtrlRegs.PIEACK.bit.ACK9=1;
	 
/*	ScibRegs.SCITXBUF = 0x00FF;             */  
}

/*============================================scope初始化程序=======================================*/
#pragma CODE_SECTION(SCOPEINITNOLOOP, "ramfuncs")
void	SCOPEINITNOLOOP(void)
{
	SCOPE_INT();
	SCOPE_RST();
}

/*====================================2407保存四个通道的数据到BUFFERDATA============================*/
#pragma CODE_SECTION(SAVETOBUFFER, "ramfuncs")
void	SAVETOBUFFER(void)				/*保存四个通道数据，在TIMER定时中断中调用堇丛从贑33，通过读DUAL RAM*/
{
	if(change_online > 0)				/*需要重新确认change_online等于1还是2，觉得应该是1，现在是2*/
	{
		change_online = change_online - 1;
	}
	else
	{
		SCIStart = 1;
	}
	if(MultiSingle == 1 || (MultiSingle == 0 && Clock >= StartTime))       /*Clock一直在增加，故采用>=*/
	{
		if(EnableWrite == 1)
		{
			SampleCounter = SampleCounter + 1;
			if(SampleCounter == SampleTime)
			{
				SampleCounter = 0x0000;
				BUFFERDATA[BufPointer_PWM] = *Val_Channel;
				BufPointer_PWM = BufPointer_PWM + 1;
				BUFFERDATA[BufPointer_PWM] = *(Val_Channel + 1);
				BufPointer_PWM = BufPointer_PWM + 1;
				BUFFERDATA[BufPointer_PWM] = *(Val_Channel + 2);
				BufPointer_PWM = BufPointer_PWM + 1;
				BUFFERDATA[BufPointer_PWM] = *(Val_Channel + 3);
				BufPointer_PWM = BufPointer_PWM + 1;
				if(BufPointer_PWM <= buffer_length - 1)			/*当一组数据未存满时，不进行else中的变量复位*/
				{}
				else
				{
					BufPointer_PWM = 0x0000;
					EnableWrite = 0x0000;
					ScibRegs.SCITXBUF = 0x00FF;
				}
			}
		}
		
	}
	else
	{
		SCOPE_RST();
	}
}

/*===================================单次触发中开始采集数据计时程序======================================*/
#pragma CODE_SECTION(INC_CLOCK, "ramfuncs")
void	INC_CLOCK(void)
{
	S01 = S01 + 1;
	if(S01 == S01_K)
	{
		S01 = 0;
		Clock = Clock + 1;
	}
}


