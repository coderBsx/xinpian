#include <reg51.H>
#include "LCD12864.h"
#include "1302.h"
#include "60S2EEPROM.h"		
#include <absacc.h>		  //???
#include <string.h>		  //???
#define  AA  3000000
//站点   GPS			  
sbit key1=P1^0;
sbit key2=P1^1;
sbit key3=P1^2;
sbit key4=P1^3;
sbit key5=P1^4;
sbit key6=P1^5;
sbit key7=P1^6;
sbit key8=P1^7;		//定义按键IO
sbit led0=P3^4;
sbit led1=P3^5;
sbit led2=P3^6;
sbit led3=P3^7;	 	//定义指示灯IO
sbit Music_Busy=P3^2;	  //定义
sbit Tishi=P3^3;	  //定义
bit key1_flag=0;
bit key2_flag=0;
bit key3_flag=0;
bit key4_flag=0;
bit key5_flag=0;
bit key6_flag=0;
bit key7_flag=0;
bit key8_flag=0;    //定义按键位变量
typedef struct {
ulong	shangxing_JD_da[15];
ulong shangxing_WD_da[15];	
ulong xiaxing_JD_da[15];	
ulong xiaxing_WD_da[15];		
}gps_data;
typedef union
 {
     gps_data Vehicle_Obj;    
     uchar gps_Bytes[sizeof(gps_data)];    /* ?1??? */
 }gps_shuju;
gps_shuju xdata gps_cun;
uchar flag_cun=0;

uchar Station_Count=1;	 
sbit Busy=P3^2;
bit position=0;
bit Display_Reversal=0;
uchar state=0;		//显示变量
bit s0=0;		    //数据闪烁变量
uchar ms=0;			//定时器用到的变量
//发现两个Bug一个是自动下，到终点站没有自动切换上行、下行
//一个是  自动下 上行站名和语音不照应
uchar sec=0;
uchar sec1=0;
bit memory_flag=0;
uchar Sound=25;			//音量大小变量
uchar Station=10;		//车站总数变量
bit Mode=0;			    //等于0代表固化    等于1代表自定义
bit A_M=0;
bit Upstream_Down=1;
uchar count=0;
uint JD_Difference=100;	 //34.800340，113.499447
uint WD_Difference=100;
uchar *pp;
bit Sound_flag=1;
uchar code xiaxing1[] ="  测试下行Ａ站  ";
uchar code xiaxing2[] ="  测试下行Ｂ站　";
uchar code xiaxing3[] ="  测试下行Ｃ站　";
uchar code xiaxing4[] ="  测试下行Ｄ站　";
uchar code xiaxing5[] ="  测试下行Ｅ站　";
uchar code xiaxing6[] ="  测试下行Ｆ站　";
uchar code xiaxing7[] ="  测试下行Ｇ站　";
uchar code xiaxing8[] ="  测试下行Ｈ站　";
uchar code xiaxing9[] ="  测试下行Ｉ站　";
uchar code xiaxing10[]="  测试下行Ｊ站　";
uchar code xiaxing11[]="    测试上行11  ";
uchar code xiaxing12[]="    测试上行12  ";
uchar code xiaxing13[]="    测试上行13  ";
uchar code xiaxing14[]="    测试上行14  ";
uchar code xiaxing15[]="    测试上行15  ";
uchar code shangxing1[] ="  测试下行Ｊ站　";
uchar code shangxing2[] ="  测试下行Ｉ站　";
uchar code shangxing3[] ="  测试下行Ｈ站　";
uchar code shangxing4[] ="  测试下行Ｇ站　";
uchar code shangxing5[] ="  测试下行Ｆ站　";
uchar code shangxing6[] ="  测试下行Ｅ站　";
uchar code shangxing7[] ="  测试下行Ｄ站　";
uchar code shangxing8[] ="  测试下行Ｃ站　";
uchar code shangxing9[] ="  测试下行Ｂ站　";
uchar code shangxing10[]="  测试下行Ａ站　";
uchar code shangxing11[]="    测试下行11  ";
uchar code shangxing12[]="    测试下行12  ";
uchar code shangxing13[]="    测试下行13  ";
uchar code shangxing14[]="    测试下行14  ";
uchar code shangxing15[]="    测试下行15  ";
uchar xdata A_dat[20];	 //暂存从GPS提取的经纬度数据
uchar xdata B_dat[20];	 //暂存从GPS提取的经纬度数据
uchar xdata GPS_dat[100];//暂存GPS模块返回的数据
uchar subscript;		 //串口数据计数变量
uchar GPS_time=0;		    //检测GPS是否接收到有效数据变量
float latitude，longitude;   //latitude是暂存纬度， longitude是暂存经度数据，这两个数据，是上面的暂存数组计算得到
unsigned long WD_A，JD_B;  //这两个变量，是最终现实与进行比较的经纬度数据变量

bit GPS_Write=0;		    //是否开启GPS校时更新时间数据标志位
uchar xdata NowTime[7]=0;	//从GPS返回的数据里面提取出时间数据
bit memory_GPS_flag=1;
uchar Sec_set=0;
uchar Time_Calibration=0;   //校准时间标志位
void memory()
 {
   if(memory_flag) 
    {
	  memory_flag=0;
	  IapEraseSector(0x08000);
	  IapProgramByte(0x08000，Sound);		       //记录音量大小变量
	  IapProgramByte(0x08001，Station);	       //记录车站总数
	  if(GPS_Write)IapProgramByte(0x08002，1);   //记录自动校时标志  GPS_Write=1，就记录为1
      else IapProgramByte(0x08002，0);           //记录自动校时标志  GPS_Write=0，就记录为0
	  if(Mode)IapProgramByte(0x08003，1);        //记录固化还是自定义  Mode=1，就记录为1
      else IapProgramByte(0x08003，0);           //记录固化还是自定义  Mode=0，就记录为0  
	  if(A_M)IapProgramByte(0x08004，1);         //记录手动自动模式  A_M=1，就记录为1
      else IapProgramByte(0x08004，0);           //记录手动自动模式  A_M=0，就记录为0  
	  if(Upstream_Down) IapProgramByte(0x08005，1);  //记录上行下行  Station_Count=1，就记录为1
      else              IapProgramByte(0x08005，0);  //记录上行下行  Station_Count=0，就记录为0   

//	  IapProgramByte(0x08006，gps_cun.Vehicle_Obj.shangxing_JD_da[0]/65536/256);
//	  IapProgramByte(0x08006，gps_cun.Vehicle_Obj.shangxing_JD_da[0]/65536%256);
//	  IapProgramByte(0x08006，gps_cun.Vehicle_Obj.shangxing_JD_da[0]%256);

	}
 }
void read_memory()
 {
   	Sound=IapReadByte(0x08000);	   //读取的记录音量大小变量
//	Station=IapReadByte(0x08001);   //读取的记录车站总数

	if(Sound>30||Station>15) 	   //如果读取出来的数据不对，就进行初始化
	 {
	   Sound=25;
	   Station=5;
	 }
    if(IapReadByte(0x08002)!=0&&IapReadByte(0x08002)!=1) //读取GPS校时标志位
	 {
	   	GPS_Write=1;								   //如果读取不对，默认打开
	 } else  GPS_Write=IapReadByte(0x08002);			   //如果对，进行赋值
    if(IapReadByte(0x08003)!=0&&IapReadByte(0x08003)!=1) //读取存储的
	 {
	  	Mode=0;
	 }else  Mode=IapReadByte(0x08003);

    if(IapReadByte(0x08004)!=0&&IapReadByte(0x08004)!=1) 
	 {
	  	A_M=0;
	 }else  A_M=IapReadByte(0x08004);

    if(IapReadByte(0x08005)!=0&&IapReadByte(0x08005)!=1) 
	 {
	  	Upstream_Down=0;
	 }else  Upstream_Down=IapReadByte(0x08005);

	 

     memory_flag=1;									   //将读取的数据再次存储

 }

void memory_GPS()
 {
   	  unsigned char i=0;
	  if(memory_GPS_flag)
	   {	
	        memory_GPS_flag=0;
			IapEraseSector(0x08200);
			for(i=0;i<60;i++)		    	   
			 {
				if(i<15) 
				 {
					IapProgramByte(0x08200+i*4，gps_cun.Vehicle_Obj.shangxing_JD_da[i]/16777216%256);
					IapProgramByte(0x08200+i*4+1，gps_cun.Vehicle_Obj.shangxing_JD_da[i]/65536%256);
					IapProgramByte(0x08200+i*4+2，gps_cun.Vehicle_Obj.shangxing_JD_da[i]/256%256);
					IapProgramByte(0x08200+i*4+3，gps_cun.Vehicle_Obj.shangxing_JD_da[i]%256);
				 }
				 else if(i<30) 
				  {
					IapProgramByte(0x08200+i*4，gps_cun.Vehicle_Obj.shangxing_WD_da[i-15]/16777216%256);
					IapProgramByte(0x08200+i*4+1，gps_cun.Vehicle_Obj.shangxing_WD_da[i-15]/65536%256);
					IapProgramByte(0x08200+i*4+2，gps_cun.Vehicle_Obj.shangxing_WD_da[i-15]/256%256);
					IapProgramByte(0x08200+i*4+3，gps_cun.Vehicle_Obj.shangxing_WD_da[i-15]%256);
				  }
				 else if(i<45) 
				  {			IapProgramByte(0x08200+i*4，gps_cun.Vehicle_Obj.xiaxing_JD_da[i-30]/16777216%256);					IapProgramByte(0x08200+i*4+1，gps_cun.Vehicle_Obj.xiaxing_JD_da[i-30]/65536%256);					IapProgramByte(0x08200+i*4+2，gps_cun.Vehicle_Obj.xiaxing_JD_da[i-30]/256%256);					IapProgramByte(0x08200+i*4+3，gps_cun.Vehicle_Obj.xiaxing_JD_da[i-30]%256);
				  }
				 else if(i<60) 
				  {
					IapProgramByte(0x08200+i*4，gps_cun.Vehicle_Obj.xiaxing_WD_da[i-45]/16777216%256);
	IapProgramByte(0x08200+i*4+1，gps_cun.Vehicle_Obj.xiaxing_WD_da[i-45]/65536%256);			IapProgramByte(0x08200+i*4+2，gps_cun.Vehicle_Obj.xiaxing_WD_da[i-45]/256%256);			IapProgramByte(0x08200+i*4+3，gps_cun.Vehicle_Obj.xiaxing_WD_da[i-45]%256);
				  }
			 }
	   }
 }

void read_GPS()
 {   
	   pp=gps_cun.gps_Bytes;
	   IapReadSector(0x08200， sizeof(gps_data)， pp);
//   unsigned char i=0;
//   unsigned long flsh1=0，flsh2=0，flsh3=0，flsh4=0;
//   for(i=0;i<60;i++)
//    {
//	  if(i<15) 			
//	   {
//		 flsh1=IapReadByte(0x08200+i*4);
//		 flsh2=IapReadByte(0x08200+i*4+1);
//		 flsh3=IapReadByte(0x08200+i*4+2);
//    	 flsh4=IapReadByte(0x08200+i*4+3);

//	   	 gps_cun.Vehicle_Obj.shangxing_JD_da[i]=flsh1*16777216+flsh2*65536+flsh3*256+flsh4;
//	   }
//	   else if(i<30) 
//	   {
//		 flsh1=IapReadByte(0x08200+i*4);
//		 flsh2=IapReadByte(0x08200+i*4+1);
//		 flsh3=IapReadByte(0x08200+i*4+2);
//    	 flsh4=IapReadByte(0x08200+i*4+3);
//	   	 gps_cun.Vehicle_Obj.shangxing_WD_da[i-15]=flsh1*16777216+flsh2*65536+flsh3*256+flsh4;
//	   }
//	   else if(i<45) 
//	   {
//		 flsh1=IapReadByte(0x08200+i*4);
//		 flsh2=IapReadByte(0x08200+i*4+1);
//		 flsh3=IapReadByte(0x08200+i*4+2);
//    	 flsh4=IapReadByte(0x08200+i*4+3);
//		 gps_cun.Vehicle_Obj.xiaxing_JD_da[i-30]=flsh1*16777216+flsh2*65536+flsh3*256+flsh4;
//	   } 
//	   else if(i<60) 
//	   {
//		 flsh1=IapReadByte(0x08200+i*4);
//		 flsh2=IapReadByte(0x08200+i*4+1);
//		 flsh3=IapReadByte(0x08200+i*4+2);
//    	 flsh4=IapReadByte(0x08200+i*4+3);

//		 gps_cun.Vehicle_Obj.xiaxing_WD_da[i-45]=flsh1*16777216+flsh2*65536+flsh3*256+flsh4;
//	   } 
//	}
 }
void Uart1Data(uchar dat) 	// 串口发送一个字节数据
{
	SBUF=dat;
	while(!TI);
	TI=0;	
}
void UartData_Byte(uchar *byte)	//串口发送一串数据
{
	while(*byte != '\0')
	{
	  Uart1Data(*byte++);
	}
}

void delay(uint dat) 
 {
   while(dat--);
 }0
uchar verify_GPSdat(uchar *dat)//读取服务器返回的数据
{
  uchar i=0;
  while(*dat != 0){
    if(*dat != GPS_dat[i]){
      return 0;
    }
    i++;
    dat++;
  }
  GPS_dat[0]=0;
  return 1;
}

void Send_Hex(unsigned char *p，unsigned char num)
{
   	while(num--)   //剩余发送的字符数
	{
        SBUF = *p; //将要发送的数据赋给串口缓冲寄存器
		while(!TI);//等待发送结束
		TI = 0;    //软件清零
		p++;       //指针加一
	}	
}

void DoSum(unsigned char *Str，unsigned char len)//校验位计算
{
	unsigned int xorsum = 0;
	unsigned char i;

	for(i=1;i<len;i++)
	{
		xorsum = xorsum + Str[i];
	}
	xorsum = 0 - xorsum;
	*(Str+i)     = (unsigned char)(xorsum >> 8);
	*(Str+i+1)   = (unsigned char)(xorsum & 0x00ff);
}


#ifndef _LCD12864_H_
#define _LCD12864_H_
 #define uchar unsigned char  //宏定义
#define uint unsigned int 
#define ulong unsigned long 
sbit LCD12864_CS=P0^2;							 //12864控制I/O口
sbit LCD12864_SID=P0^1;
sbit LCD12864_CLK=P0^0;
/**********************************************************
#：函数名：SendWrite(uchar dat)
#：函数功能：发送8位数据给LCD12864
#：函数参数：dat位发送的数据变量
***********************************************************/
void SendWrite(uchar dat)
{
	uchar i;
	for(i=0;i<8;i++)
	{
		LCD12864_CLK=0;
		dat=dat<<1;
		LCD12864_SID=CY;
		LCD12864_CLK=1;
		LCD12864_CLK=0;
	}
}
/**********************************************************
#：函数名：uchar LCD12864_Read(void)
#：函数功能：读取LCD12864中的数据
#：函数参数：函数返回读取的数据内容
***********************************************************/
uchar LCD12864_Read(void)
{
	uchar i,dat1,dat2;
	dat1=dat2=0;
	for(i=0;i<8;i++)
	{
		dat1=dat1<<1;
		LCD12864_CLK = 0;
		LCD12864_CLK = 1;                
		LCD12864_CLK = 0;
		if(LCD12864_SID) dat1++;
	}
	for(i=0;i<8;i++)
	{
		dat2=dat2<<1;
		LCD12864_CLK = 0;
		LCD12864_CLK = 1;
		LCD12864_CLK = 0;
		if(LCD12864_SID) dat2++;
	}
	return ((0xf0&dat1)+(0x0f&dat2));
}
/**********************************************************
#：函数名：LCD12864_Busy( void )
#：函数功能：判忙函数
#：函数参数：无
***********************************************************/
void LCD12864_Busy( void )
{
	do SendWrite(0xfc);     //11111,RW(1),RS(0),0
	while(0x80&LCD12864_Read());
}

/**********************************************************
#：函数名：void LCD12864_write(bit cmd,uchar dat)
#：函数功能：向屏发送命令/数据 带发送数据
#：函数参数：cmd标志发送数据、命令，0为命令，1位数据；   dat 位数据内容
***********************************************************/
void LCD12864_write(bit cmd,uchar dat)  
{
	LCD12864_CS = 1;
	LCD12864_Busy();
	if(cmd==0) SendWrite(0xf8);
	else SendWrite(0xfa);          //11111,RW(0),RS(1),0
	SendWrite(0xf0&dat);
	SendWrite(0xf0&dat<<4);
	LCD12864_CS = 0;
}
/**********************************************************
#：函数名：void LCD12864_writebyte(uchar *prointer)
#：函数功能：指针发送显示数据
#：函数参数：prointer位指针内容
***********************************************************/
void LCD12864_writebyte(uchar *prointer)			
{
    while(*prointer!='\0')
    {
        LCD12864_write(1,*prointer);
        prointer++;
    }
}
/******************************************************************
                         lcd初始化函数
*******************************************************************/
void LCD12864_init(void)
{
     LCD12864_write(0,0x30);
     LCD12864_write(0,0x03);
     LCD12864_write(0,0x0c);
     LCD12864_write(0,0x01);
     LCD12864_write(0,0x06);
}
/**********************************************************
#：函数名：void LCD12864_pos(uchar x,y)
#：函数功能：设置屏幕显示的位置
#：函数参数：X，Y，为显示的坐标   X位行数据，Y位列数据
***********************************************************/
void LCD12864_pos(uchar x,y)
{
	switch(x)
	{
		case 0:
			x=0x80;break;
		case 1:
			x=0x90;break;
		case 2:
			x=0x88;break;
		case 3:
			x=0x98;break;
		default:
			x=0x80;
	}
	y=y&0x07;
	LCD12864_write(0,0x30);
	LCD12864_write(0,y+x);
	LCD12864_write(0,y+x);
}
/**********************************************************
#：函数名：void LCD12864_Qing( void )
#：函数功能：清除屏幕显示的你内容
#：函数参数：X，Y，为显示的坐标   X位行数据，Y位列数据
***********************************************************/
void LCD12864_Qing( void )
{
	unsigned char i;
	LCD12864_write(0,0x30);
	LCD12864_write(0,0x80);
	for(i=0;i<64;i++)
	LCD12864_write(1,0x20);
	LCD12864_pos(0,0);	    
}
#endif


#ifndef _1302_H_
#define _1302_H_
  // #include <intrins.h>
#define uchar unsigned char
#define uint unsigned int   
code uchar table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //月修正数据表
uchar xdata time_data[7]=0;
uchar xdata time_data_1[7]=0;
uchar xdata time_data_2[7]=0;
uchar xdata time_data_3[7]=0;
uchar xdata time_data_4[7]=0;
//=====================================================================================
//=====================================================================================
//=====================================================================================
/*******************************************************************************
* 函 数 名         : Ds1302Write
* 函数功能		   : 向DS1302命令（地址+数据）
* 输    入         : addr,dat
* 输    出         : 无
*******************************************************************************/
sbit DSIO=P2^1;
sbit RST=P2^2;
sbit SCLK=P2^0;
uchar Conver_week(uchar year,uchar month,uchar day)
{//c=0 为21世纪,c=1 为19世纪 输入输出数据均为BCD数据
    uchar p1,p2,week;
    year+=0x64;  //如果为21世纪,年份数加100
    p1=year/0x4;  //所过闰年数只算1900年之后的
    p2=year+p1;
    p2=p2%0x7;  //为节省资源,先进行一次取余,避免数大于0xff,避免使用整型数据
    p2=p2+day+table_week[month-1];
    if (year%0x4==0&&month<3)p2-=1;
    week=p2%0x7;
	return week;
}
void ds1302write(uchar addr, uchar dat)
{
	uchar n;
	RST = 0;
	//_nop_();
	SCLK = 0;//先将SCLK置低电平。
	//_nop_();
	RST = 1; //然后将RST(CE)置高电平。
	//_nop_();
	for (n=0; n<8; n++)//开始传送八位地址命令
	{
		DSIO = addr & 0x01;//数据从低位开始传送
		addr >>= 1;
		SCLK = 1;//数据在上升沿时，DS1302读取数据
		//_nop_();
		SCLK = 0;
		//_nop_();
	}
	for (n=0; n<8; n++)//写入8位数据
	{
		DSIO = dat & 0x01;
		dat >>= 1;
		SCLK = 1;//数据在上升沿时，DS1302读取数据
		//_nop_();
		SCLK = 0;
		//_nop_();	
	}	
	RST = 0;//传送数据结束
	//_nop_();
}
/*******************************************************************************
* 函 数 名         : Ds1302Read
* 函数功能		   : 读取一个地址的数据
* 输    入         : addr
* 输    出         : dat
*******************************************************************************/
uchar ds1302read(uchar addr)
{
	uchar n,dat,dat1;
	RST = 0;
	//_nop_();
	SCLK = 0;//先将SCLK置低电平。
	//_nop_();
	RST = 1;//然后将RST(CE)置高电平。
	//_nop_();
	for(n=0; n<8; n++)//开始传送八位地址命令
	{
		DSIO = addr & 0x01;//数据从低位开始传送
		addr >>= 1;
		SCLK = 1;//数据在上升沿时，DS1302读取数据
		//_nop_();
		SCLK = 0;//DS1302下降沿时，放置数据
		//_nop_();
	}
	//_nop_();
	for(n=0; n<8; n++)//读取8位数据
	{
		dat1 = DSIO;//从最低位开始接收
		dat = (dat>>1) | (dat1<<7);
		SCLK = 1;
		//_nop_();
		SCLK = 0;//DS1302下降沿时，放置数据
		//_nop_();
	}
	RST = 0;
	//_nop_();	//以下为DS1302复位的稳定时间,必须的。
	SCLK = 1;
	//_nop_();
	DSIO = 0;
	//_nop_();
	DSIO = 1;
	//_nop_();
	return dat;	
}
#endif