#include <reg51.H>
#include "LCD12864.h"
#include "1302.h"
#include "60S2EEPROM.h"		
#include <absacc.h>		  //???
#include <string.h>		  //???
#define  AA  3000000
//վ��   GPS			  
sbit key1=P1^0;
sbit key2=P1^1;
sbit key3=P1^2;
sbit key4=P1^3;
sbit key5=P1^4;
sbit key6=P1^5;
sbit key7=P1^6;
sbit key8=P1^7;		//���尴��IO
sbit led0=P3^4;
sbit led1=P3^5;
sbit led2=P3^6;
sbit led3=P3^7;	 	//����ָʾ��IO
sbit Music_Busy=P3^2;	  //����
sbit Tishi=P3^3;	  //����
bit key1_flag=0;
bit key2_flag=0;
bit key3_flag=0;
bit key4_flag=0;
bit key5_flag=0;
bit key6_flag=0;
bit key7_flag=0;
bit key8_flag=0;    //���尴��λ����
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
uchar state=0;		//��ʾ����
bit s0=0;		    //������˸����
uchar ms=0;			//��ʱ���õ��ı���
//��������Bugһ�����Զ��£����յ�վû���Զ��л����С�����
//һ����  �Զ��� ����վ������������Ӧ
uchar sec=0;
uchar sec1=0;
bit memory_flag=0;
uchar Sound=25;			//������С����
uchar Station=10;		//��վ��������
bit Mode=0;			    //����0����̻�    ����1�����Զ���
bit A_M=0;
bit Upstream_Down=1;
uchar count=0;
uint JD_Difference=100;	 //34.800340��113.499447
uint WD_Difference=100;
uchar *pp;
bit Sound_flag=1;
uchar code xiaxing1[] ="  �������У�վ  ";
uchar code xiaxing2[] ="  �������У�վ��";
uchar code xiaxing3[] ="  �������У�վ��";
uchar code xiaxing4[] ="  �������У�վ��";
uchar code xiaxing5[] ="  �������У�վ��";
uchar code xiaxing6[] ="  �������У�վ��";
uchar code xiaxing7[] ="  �������У�վ��";
uchar code xiaxing8[] ="  �������У�վ��";
uchar code xiaxing9[] ="  �������У�վ��";
uchar code xiaxing10[]="  �������У�վ��";
uchar code xiaxing11[]="    ��������11  ";
uchar code xiaxing12[]="    ��������12  ";
uchar code xiaxing13[]="    ��������13  ";
uchar code xiaxing14[]="    ��������14  ";
uchar code xiaxing15[]="    ��������15  ";
uchar code shangxing1[] ="  �������У�վ��";
uchar code shangxing2[] ="  �������У�վ��";
uchar code shangxing3[] ="  �������У�վ��";
uchar code shangxing4[] ="  �������У�վ��";
uchar code shangxing5[] ="  �������У�վ��";
uchar code shangxing6[] ="  �������У�վ��";
uchar code shangxing7[] ="  �������У�վ��";
uchar code shangxing8[] ="  �������У�վ��";
uchar code shangxing9[] ="  �������У�վ��";
uchar code shangxing10[]="  �������У�վ��";
uchar code shangxing11[]="    ��������11  ";
uchar code shangxing12[]="    ��������12  ";
uchar code shangxing13[]="    ��������13  ";
uchar code shangxing14[]="    ��������14  ";
uchar code shangxing15[]="    ��������15  ";
uchar xdata A_dat[20];	 //�ݴ��GPS��ȡ�ľ�γ������
uchar xdata B_dat[20];	 //�ݴ��GPS��ȡ�ľ�γ������
uchar xdata GPS_dat[100];//�ݴ�GPSģ�鷵�ص�����
uchar subscript;		 //�������ݼ�������
uchar GPS_time=0;		    //���GPS�Ƿ���յ���Ч���ݱ���
float latitude��longitude;   //latitude���ݴ�γ�ȣ� longitude���ݴ澭�����ݣ����������ݣ���������ݴ��������õ�
unsigned long WD_A��JD_B;  //��������������������ʵ����бȽϵľ�γ�����ݱ���

bit GPS_Write=0;		    //�Ƿ���GPSУʱ����ʱ�����ݱ�־λ
uchar xdata NowTime[7]=0;	//��GPS���ص�����������ȡ��ʱ������
bit memory_GPS_flag=1;
uchar Sec_set=0;
uchar Time_Calibration=0;   //У׼ʱ���־λ
void memory()
 {
   if(memory_flag) 
    {
	  memory_flag=0;
	  IapEraseSector(0x08000);
	  IapProgramByte(0x08000��Sound);		       //��¼������С����
	  IapProgramByte(0x08001��Station);	       //��¼��վ����
	  if(GPS_Write)IapProgramByte(0x08002��1);   //��¼�Զ�Уʱ��־  GPS_Write=1���ͼ�¼Ϊ1
      else IapProgramByte(0x08002��0);           //��¼�Զ�Уʱ��־  GPS_Write=0���ͼ�¼Ϊ0
	  if(Mode)IapProgramByte(0x08003��1);        //��¼�̻������Զ���  Mode=1���ͼ�¼Ϊ1
      else IapProgramByte(0x08003��0);           //��¼�̻������Զ���  Mode=0���ͼ�¼Ϊ0  
	  if(A_M)IapProgramByte(0x08004��1);         //��¼�ֶ��Զ�ģʽ  A_M=1���ͼ�¼Ϊ1
      else IapProgramByte(0x08004��0);           //��¼�ֶ��Զ�ģʽ  A_M=0���ͼ�¼Ϊ0  
	  if(Upstream_Down) IapProgramByte(0x08005��1);  //��¼��������  Station_Count=1���ͼ�¼Ϊ1
      else              IapProgramByte(0x08005��0);  //��¼��������  Station_Count=0���ͼ�¼Ϊ0   

//	  IapProgramByte(0x08006��gps_cun.Vehicle_Obj.shangxing_JD_da[0]/65536/256);
//	  IapProgramByte(0x08006��gps_cun.Vehicle_Obj.shangxing_JD_da[0]/65536%256);
//	  IapProgramByte(0x08006��gps_cun.Vehicle_Obj.shangxing_JD_da[0]%256);

	}
 }
void read_memory()
 {
   	Sound=IapReadByte(0x08000);	   //��ȡ�ļ�¼������С����
//	Station=IapReadByte(0x08001);   //��ȡ�ļ�¼��վ����

	if(Sound>30||Station>15) 	   //�����ȡ���������ݲ��ԣ��ͽ��г�ʼ��
	 {
	   Sound=25;
	   Station=5;
	 }
    if(IapReadByte(0x08002)!=0&&IapReadByte(0x08002)!=1) //��ȡGPSУʱ��־λ
	 {
	   	GPS_Write=1;								   //�����ȡ���ԣ�Ĭ�ϴ�
	 } else  GPS_Write=IapReadByte(0x08002);			   //����ԣ����и�ֵ
    if(IapReadByte(0x08003)!=0&&IapReadByte(0x08003)!=1) //��ȡ�洢��
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

	 

     memory_flag=1;									   //����ȡ�������ٴδ洢

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
					IapProgramByte(0x08200+i*4��gps_cun.Vehicle_Obj.shangxing_JD_da[i]/16777216%256);
					IapProgramByte(0x08200+i*4+1��gps_cun.Vehicle_Obj.shangxing_JD_da[i]/65536%256);
					IapProgramByte(0x08200+i*4+2��gps_cun.Vehicle_Obj.shangxing_JD_da[i]/256%256);
					IapProgramByte(0x08200+i*4+3��gps_cun.Vehicle_Obj.shangxing_JD_da[i]%256);
				 }
				 else if(i<30) 
				  {
					IapProgramByte(0x08200+i*4��gps_cun.Vehicle_Obj.shangxing_WD_da[i-15]/16777216%256);
					IapProgramByte(0x08200+i*4+1��gps_cun.Vehicle_Obj.shangxing_WD_da[i-15]/65536%256);
					IapProgramByte(0x08200+i*4+2��gps_cun.Vehicle_Obj.shangxing_WD_da[i-15]/256%256);
					IapProgramByte(0x08200+i*4+3��gps_cun.Vehicle_Obj.shangxing_WD_da[i-15]%256);
				  }
				 else if(i<45) 
				  {			IapProgramByte(0x08200+i*4��gps_cun.Vehicle_Obj.xiaxing_JD_da[i-30]/16777216%256);					IapProgramByte(0x08200+i*4+1��gps_cun.Vehicle_Obj.xiaxing_JD_da[i-30]/65536%256);					IapProgramByte(0x08200+i*4+2��gps_cun.Vehicle_Obj.xiaxing_JD_da[i-30]/256%256);					IapProgramByte(0x08200+i*4+3��gps_cun.Vehicle_Obj.xiaxing_JD_da[i-30]%256);
				  }
				 else if(i<60) 
				  {
					IapProgramByte(0x08200+i*4��gps_cun.Vehicle_Obj.xiaxing_WD_da[i-45]/16777216%256);
	IapProgramByte(0x08200+i*4+1��gps_cun.Vehicle_Obj.xiaxing_WD_da[i-45]/65536%256);			IapProgramByte(0x08200+i*4+2��gps_cun.Vehicle_Obj.xiaxing_WD_da[i-45]/256%256);			IapProgramByte(0x08200+i*4+3��gps_cun.Vehicle_Obj.xiaxing_WD_da[i-45]%256);
				  }
			 }
	   }
 }

void read_GPS()
 {   
	   pp=gps_cun.gps_Bytes;
	   IapReadSector(0x08200�� sizeof(gps_data)�� pp);
//   unsigned char i=0;
//   unsigned long flsh1=0��flsh2=0��flsh3=0��flsh4=0;
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
void Uart1Data(uchar dat) 	// ���ڷ���һ���ֽ�����
{
	SBUF=dat;
	while(!TI);
	TI=0;	
}
void UartData_Byte(uchar *byte)	//���ڷ���һ������
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
uchar verify_GPSdat(uchar *dat)//��ȡ���������ص�����
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

void Send_Hex(unsigned char *p��unsigned char num)
{
   	while(num--)   //ʣ�෢�͵��ַ���
	{
        SBUF = *p; //��Ҫ���͵����ݸ������ڻ���Ĵ���
		while(!TI);//�ȴ����ͽ���
		TI = 0;    //�������
		p++;       //ָ���һ
	}	
}

void DoSum(unsigned char *Str��unsigned char len)//У��λ����
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
 #define uchar unsigned char  //�궨��
#define uint unsigned int 
#define ulong unsigned long 
sbit LCD12864_CS=P0^2;							 //12864����I/O��
sbit LCD12864_SID=P0^1;
sbit LCD12864_CLK=P0^0;
/**********************************************************
#����������SendWrite(uchar dat)
#���������ܣ�����8λ���ݸ�LCD12864
#������������datλ���͵����ݱ���
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
#����������uchar LCD12864_Read(void)
#���������ܣ���ȡLCD12864�е�����
#�������������������ض�ȡ����������
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
#����������LCD12864_Busy( void )
#���������ܣ���æ����
#��������������
***********************************************************/
void LCD12864_Busy( void )
{
	do SendWrite(0xfc);     //11111,RW(1),RS(0),0
	while(0x80&LCD12864_Read());
}

/**********************************************************
#����������void LCD12864_write(bit cmd,uchar dat)
#���������ܣ�������������/���� ����������
#������������cmd��־�������ݡ����0Ϊ���1λ���ݣ�   dat λ��������
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
#����������void LCD12864_writebyte(uchar *prointer)
#���������ܣ�ָ�뷢����ʾ����
#������������prointerλָ������
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
                         lcd��ʼ������
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
#����������void LCD12864_pos(uchar x,y)
#���������ܣ�������Ļ��ʾ��λ��
#������������X��Y��Ϊ��ʾ������   Xλ�����ݣ�Yλ������
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
#����������void LCD12864_Qing( void )
#���������ܣ������Ļ��ʾ��������
#������������X��Y��Ϊ��ʾ������   Xλ�����ݣ�Yλ������
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
code uchar table_week[12]={0,3,3,6,1,4,6,2,5,0,3,5}; //���������ݱ�
uchar xdata time_data[7]=0;
uchar xdata time_data_1[7]=0;
uchar xdata time_data_2[7]=0;
uchar xdata time_data_3[7]=0;
uchar xdata time_data_4[7]=0;
//=====================================================================================
//=====================================================================================
//=====================================================================================
/*******************************************************************************
* �� �� ��         : Ds1302Write
* ��������		   : ��DS1302�����ַ+���ݣ�
* ��    ��         : addr,dat
* ��    ��         : ��
*******************************************************************************/
sbit DSIO=P2^1;
sbit RST=P2^2;
sbit SCLK=P2^0;
uchar Conver_week(uchar year,uchar month,uchar day)
{//c=0 Ϊ21����,c=1 Ϊ19���� ����������ݾ�ΪBCD����
    uchar p1,p2,week;
    year+=0x64;  //���Ϊ21����,�������100
    p1=year/0x4;  //����������ֻ��1900��֮���
    p2=year+p1;
    p2=p2%0x7;  //Ϊ��ʡ��Դ,�Ƚ���һ��ȡ��,����������0xff,����ʹ����������
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
	SCLK = 0;//�Ƚ�SCLK�õ͵�ƽ��
	//_nop_();
	RST = 1; //Ȼ��RST(CE)�øߵ�ƽ��
	//_nop_();
	for (n=0; n<8; n++)//��ʼ���Ͱ�λ��ַ����
	{
		DSIO = addr & 0x01;//���ݴӵ�λ��ʼ����
		addr >>= 1;
		SCLK = 1;//������������ʱ��DS1302��ȡ����
		//_nop_();
		SCLK = 0;
		//_nop_();
	}
	for (n=0; n<8; n++)//д��8λ����
	{
		DSIO = dat & 0x01;
		dat >>= 1;
		SCLK = 1;//������������ʱ��DS1302��ȡ����
		//_nop_();
		SCLK = 0;
		//_nop_();	
	}	
	RST = 0;//�������ݽ���
	//_nop_();
}
/*******************************************************************************
* �� �� ��         : Ds1302Read
* ��������		   : ��ȡһ����ַ������
* ��    ��         : addr
* ��    ��         : dat
*******************************************************************************/
uchar ds1302read(uchar addr)
{
	uchar n,dat,dat1;
	RST = 0;
	//_nop_();
	SCLK = 0;//�Ƚ�SCLK�õ͵�ƽ��
	//_nop_();
	RST = 1;//Ȼ��RST(CE)�øߵ�ƽ��
	//_nop_();
	for(n=0; n<8; n++)//��ʼ���Ͱ�λ��ַ����
	{
		DSIO = addr & 0x01;//���ݴӵ�λ��ʼ����
		addr >>= 1;
		SCLK = 1;//������������ʱ��DS1302��ȡ����
		//_nop_();
		SCLK = 0;//DS1302�½���ʱ����������
		//_nop_();
	}
	//_nop_();
	for(n=0; n<8; n++)//��ȡ8λ����
	{
		dat1 = DSIO;//�����λ��ʼ����
		dat = (dat>>1) | (dat1<<7);
		SCLK = 1;
		//_nop_();
		SCLK = 0;//DS1302�½���ʱ����������
		//_nop_();
	}
	RST = 0;
	//_nop_();	//����ΪDS1302��λ���ȶ�ʱ��,����ġ�
	SCLK = 1;
	//_nop_();
	DSIO = 0;
	//_nop_();
	DSIO = 1;
	//_nop_();
	return dat;	
}
#endif