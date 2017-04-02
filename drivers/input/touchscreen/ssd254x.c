//#include <mach/gpio.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <asm/io.h>
#include <asm/types.h>
#include <asm/uaccess.h>
#include <linux/timer.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/input-polldev.h>
#include <linux/earlysuspend.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <linux/input/mt.h>
#include <linux/wakelock.h>
#include <linux/irq.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */

#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/input.h>
#include <linux/input/mt.h>

#include "../../arch/arm/mach-imx/hardware.h"

#define TP_CHR "tp_chr"
#define TYPE_NAME "ssd254x-ts"


static int probe_flag=0;

static struct i2c_client *this_client;
static ssize_t tp_read(struct file *file, char __user *buf, size_t count,
                loff_t *offset);
static ssize_t tp_write(struct file *file, const char __user *buf,
                size_t count, loff_t *offset);

static const struct file_operations tp_fops = {
        .read           = tp_read,
        .write          = tp_write,
};

static struct miscdevice misc = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = TP_CHR,
    .fops  = &tp_fops,
};

#define CONFIG_TOUCHSCREEN_SSL_DEBUG	
#undef  CONFIG_TOUCHSCREEN_SSL_DEBUG   //linan

#define ProtocolB
//#define JITTER_FILTER

#define DEVICE_ID_REG                 2
#define VERSION_ID_REG                3
#define EVENT_STATUS                  0x79
#define FINGER01_REG                  0x7c
int Ssd_Timer_flag;//Ssd_Timer1,Ssd_Timer2,Ssd_Timer_flag;

struct ChipSetting {
	char No;
	char Reg;
	char Data1;
	char Data2;
};

static int version;
//static bool normal_touch = true;
static bool sensitivity_flag = true;

#include "ssd254x.h"

void deviceReset(struct i2c_client *client);
void deviceResume(struct i2c_client *client);
void deviceSuspend(struct i2c_client *client);
void SSD254xdeviceInit(struct i2c_client *client); 

//static int ssd254x_ts_open(struct input_dev *dev);
//static void ssd254x_ts_close(struct input_dev *dev);
static int ssd254x_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int ssd254x_ts_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ssd254x_ts_early_suspend(struct early_suspend *h);
static void ssd254x_ts_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */

static enum hrtimer_restart ssd254x_ts_timer(struct hrtimer *timer);
static irqreturn_t ssd254x_ts_isr(int irq, void *dev_id);
static struct workqueue_struct *ssd254x_wq;

#define I2C_CTPM_ADDRESS 0x48

//#define	ON_TOUCH_INT IMX_GPIO_NR(4, 8)//	( IRQ_SIRQ0 )    //GPIO :set the interrupt 
#define ON_TOUCH_RST IMX_GPIO_NR(6, 10)//   ASOC_GPIO_PORTB(3)

//#define FILTER_POINT
#ifdef FILTER_POINT
#define FILTER_MAX	6
#endif

static u32 id_sign[FINGERNO] = {0};
static u8 id_state_flag[FINGERNO] = {0};
static u8 id_state_old_flag[FINGERNO] = {0};
static u16 x_old[FINGERNO] = {0};
static u16 y_old[FINGERNO] = {0};
static u16 x_new = 0;
static u16 y_new = 0;

extern int get_config(const char *key, char *buff, int len);
struct ft5x06_cfg_xml {
    unsigned int sirq;
    unsigned int i2cNum;
    unsigned int i2cAddr;
    unsigned int xMax;
    unsigned int yMax;
    unsigned int rotate; 
}; 
static struct ft5x06_cfg_xml cfg_xml;
//TP POWER
volatile int current_val = 0;
#define SSD254X_POWER_ID	("sensor28")//("touchPannel_power")
#define SSD254X_POWER_MIN_VOL	(3300000)
#define SSD254X_POWER_MAX_VOL	(3300000)


static struct regulator *tp_regulator = NULL;

static inline void regulator_deinit(struct regulator *);
static struct regulator *regulator_init(const char *, int, int);
static inline void disable_power(struct regulator *);


static inline void disable_power(struct regulator *power)
{
        regulator_disable(power);
}

static inline void regulator_deinit(struct regulator *power)
{
    regulator_disable(power);    
    regulator_put(power);
}

static struct regulator *regulator_init(const char *name, int minvol, int maxvol)
{
    struct regulator *power;

    power = regulator_get(NULL,"sensor28");// name);
    if (IS_ERR(power)) {
        return NULL;
    }
    
    if (regulator_set_voltage(power, minvol, maxvol)) {
        regulator_deinit(power);
        return NULL;
    }
    regulator_enable(power);
    return (power);
}

//----- motinlu add end-----
static struct i2c_client *save_client;

struct ssl_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct hrtimer timer;
	struct work_struct  ssl_work;
#ifdef	CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif 

	int irq;
	int use_irq;
	int FingerNo;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];

	int Resolution;
	int EventStatus;
	int FingerDetect;

	int sFingerX[FINGERNO];
	int sFingerY[FINGERNO];
	int pFingerX[FINGERNO];
	int pFingerY[FINGERNO];
};

int ReadRegister(struct i2c_client *client,uint8_t reg,int ByteNo)
{
	unsigned char buf[4];
	struct i2c_msg msg[2];
	int ret;

	memset(buf, 0xFF, sizeof(buf));
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	//msg[0].scl_rate = 400 * 1000;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ByteNo;
	msg[1].buf = buf;
	//msg[1].scl_rate = 400 * 1000;

	ret = i2c_transfer(client->adapter, msg, 2);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<0)	printk("		ReadRegister: i2c_transfer Error !\n");
	else		printk("		ReadRegister: i2c_transfer OK !\n");
	#endif
//	printk("buf[0]=0x%02x,buf[1]=0x%02x,buf[2]=0x%02x,buf[3]=0x%02x\n",buf[0],buf[1],buf[2],buf[3]);
	if(ByteNo==1) return (int)((unsigned int)buf[0]<<0);
	if(ByteNo==2) return (int)((unsigned int)buf[1]<<0)|((unsigned int)buf[0]<<8);
	if(ByteNo==3) return (int)((unsigned int)buf[2]<<0)|((unsigned int)buf[1]<<8)|((unsigned int)buf[0]<<16);
	if(ByteNo==4) return (int)((unsigned int)buf[3]<<0)|((unsigned int)buf[2]<<8)|((unsigned int)buf[1]<<16)|(buf[0]<<24);
	
	return 0;
}

void WriteRegister(struct i2c_client *client,uint8_t Reg,unsigned char Data1,unsigned char Data2,int ByteNo)
{	
	struct i2c_msg msg;
	unsigned char buf[4];
	int ret;

	buf[0]=Reg;
	buf[1]=Data1;
	buf[2]=Data2;
	buf[3]=0;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ByteNo+1;
	msg.buf = (char *)buf;
	//msg.scl_rate = 400 * 1000;
	ret = i2c_transfer(client->adapter, &msg, 1);

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret<0)	printk("		WriteRegister: i2c_master_send Error !\n");
	else		printk("		WriteRegister: i2c_master_send OK !\n");
	#endif
}

void SSD254xdeviceInit(struct i2c_client *client)
{	
	int i;
	int regVal;
	deviceReset(client);
	for(i=0;i<sizeof(ssd254x_cfgTable)/sizeof(ssd254x_cfgTable[0]);i++)
                {
                        WriteRegister(  client,ssd254x_cfgTable[i].Reg,
                                ssd254x_cfgTable[i].Data1,ssd254x_cfgTable[i].Data2,
                                ssd254x_cfgTable[i].No);
                }
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	for(i=0;i<sizeof(ssd254x_cfgTable)/sizeof(ssd254x_cfgTable[0]);i++)
	{
		regVal = ReadRegister(client, ssd254x_cfgTable[i].Reg,2);
		printk("Reg: 0x%X  = 0x%04X \n",ssd254x_cfgTable[i].Reg, regVal);
	}
	#endif
	
	mdelay(100);
}

void deviceReset(struct i2c_client *client)
{
	
	int i;
	for(i=0;i<sizeof(Reset)/sizeof(Reset[0]);i++)
	{
		WriteRegister(	client,Reset[i].Reg,
				Reset[i].Data1,Reset[i].Data2,
				Reset[i].No);
	}
	mdelay(100);   
}

void deviceResume(struct i2c_client *client)
{
	if (client == NULL) {
		return;
	}
	gpio_direction_output(ON_TOUCH_RST, 1);

    mdelay(5);
    gpio_direction_output(ON_TOUCH_RST, 0);
    mdelay(10);
    gpio_direction_output(ON_TOUCH_RST, 1);
    mdelay(100);
	SSD254xdeviceInit(client);
	//suspend_flag = false;
}

void deviceSuspend(struct i2c_client *client)
{	
if(client == NULL)
        return;
	//suspend_flag = true;
	WriteRegister(	client,Suspend[0].Reg,
				Suspend[0].Data1,Suspend[0].Data2,
				Suspend[0].No);
	msleep(100);
	
	WriteRegister(	client,Suspend[1].Reg,
				Suspend[1].Data1,Suspend[1].Data2,
				Suspend[1].No);
	
}

#ifdef JITTER_FILTER

// using previous [iFilterLen] of points to determine whether there is jitter

const int iFilterLen = 10;	// number of previous points to be used
const int iDirChange = 3;	// number of direction change to be recognized as "jitter"

// the following is the best to assume a circular buffer (less computation), but moving buffer would be easy to use (less code)
// I assume a moving buffer in pseudo code
int xBuffer[FINGERNO][FINGERNO]={{4095}};
int yBuffer[FINGERNO][FINGERNO]={{4095}};


#define DirPos	1
#define DirNeg	-1
#define Dir0	0

#define Jitter_Width 30
#define FilterOn_Cnt 15

int jitter_width = Jitter_Width;
int filteron_cnt = FilterOn_Cnt;
int move_length = 20;
int finger_num = 1;

int PprevX[FINGERNO] ={4095};
int PprevY[FINGERNO] ={4095};

int filter_on_flag[FINGERNO] = {0}; 
int filter_on_count[FINGERNO] ={0};
int finger_status =0;

void ssd254x_jitter_buffer_clean(int i)
{
	int j; 
	for (j = 0; j < iFilterLen; j++) {
			xBuffer[i][j] = 4095;
			yBuffer[i][j] = 4095;
		}
	filter_on_count[i]=0;
}

void ssd254x_jitter_filter(unsigned short i, unsigned short* x_pos, unsigned short* y_pos)
{
    int j;
    // count direction change
    int DirCntX = 0, DirCntY = 0;
    int dirX, dirY;
    int temp=0;
    int xPos = 0, yPos = 0;
    int numPoint = 0;


	int dot_value; 
	int a_xoffset, a_yoffset, b_xoffset, b_yoffset, a_lenght, b_lenght;
	int pre_offsetX, pre_offsetY;

	if (*x_pos == 4095 && *y_pos == 4095) {
		// reset buffer
		// set xBuffer[i][] = 4095
		// set yBuffer[i][] = 4095
		for (j = 0; j < iFilterLen; j++) {
			xBuffer[i][j] = 4095;
			yBuffer[i][j] = 4095;
		}
	}
	else {
		// new valid point for finger i received
		// add to buffer
		// shift x[i][] y[i][] to higher index

		if((*x_pos == xBuffer[i][0]) && (*y_pos == yBuffer[i][0]))
		{
		//	SSL_DEBUG(" skip same point\n");
		}
		else
		{
			
			for (j = 0; j < iFilterLen-1; j++) {
			xBuffer[i][iFilterLen - j - 1] = xBuffer[i][iFilterLen - j - 2];
			yBuffer[i][iFilterLen - j - 1] = yBuffer[i][iFilterLen - j - 2];
			}
			xBuffer[i][0] = *x_pos;
			yBuffer[i][0] = *y_pos;
		}

		if(xBuffer[i][1]==4095 || xBuffer[i][2] == 4095)
		{
			PprevX[i] =*x_pos;
			PprevY[i] =*y_pos;
			return;
		}

    a_xoffset = xBuffer[i][0]- xBuffer[i][1];
		a_yoffset = yBuffer[i][0]- yBuffer[i][1];
		b_xoffset = xBuffer[i][1]- xBuffer[i][2];
		b_yoffset = yBuffer[i][1]- yBuffer[i][2];
		a_lenght  = int_sqrt(a_xoffset*a_xoffset + a_yoffset*a_yoffset);
		b_lenght  = int_sqrt(b_xoffset*b_xoffset + b_yoffset*b_yoffset);

		pre_offsetX = xBuffer[i][0] - PprevX[i];
		pre_offsetY = yBuffer[i][0] - PprevY[i];
		

    dot_value = (a_xoffset* b_xoffset + a_yoffset * b_yoffset)*10 /(a_lenght * b_lenght);

		if(dot_value < 0 && finger_status > finger_num)
		{  
			filter_on_count[i]=0; 
			
			//SSL_DEBUG(" TUNING point  = %d, finger =%d \n", dot_value, i);
			goto FILTER_ON; 
		}
		else if (filter_on_flag[i] ==1)
		{
			if(int_sqrt(pre_offsetX * pre_offsetX + pre_offsetY * pre_offsetY) > jitter_width )
			{
				goto FILTER_OFF;
			}
			if(dot_value > 0 )
			{
				filter_on_count[i]++; 
				if(filter_on_count[i]>filteron_cnt)
				{
					//SSL_DEBUG("=====filter on count over = %d, finger =%d \n", dot_value, i);
					goto FILTER_OFF; 
				}
			}
	//		SSL_DEBUG(" within jitter:	 = %d, width =%d \n", dot_value, int_sqrt(pre_offsetX * pre_offsetX + pre_offsetY * pre_offsetY));
			goto FILTER_ON; 
		}
			            
      FILTER_OFF:
			filter_on_flag[i] = 0; 
			filter_on_count[i]=0;
			PprevX[i] = *x_pos;
			PprevY[i] = *y_pos;
			
	//		SSL_DEBUG(" 	FITER OFFFFF: [ %d , %d ]\n",PprevX[i],PprevY[i]);
			return ; 
      FILTER_ON:
	  	
			filter_on_flag[i] = 1; 
			*x_pos = (unsigned short)PprevX[i];
			*y_pos = (unsigned short)PprevY[i];
			
	//		SSL_DEBUG(" 	FITER ONNNN: [%d , %d] from [%d, %d] \n",*x_pos ,*y_pos, xBuffer[i][0], yBuffer[i][0]);
			return ; 
	}
	
}

#endif	 // JITTER_FILTER



#ifdef FILTER_POINT
static void filter_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;
	u16 filter_step_x = 0, filter_step_y = 0;
	
	id_sign[id] = id_sign[id] + 1;
	if(id_sign[id] == 1)
	{
		x_old[id] = x;
		y_old[id] = y;
	}
	
	x_err = x > x_old[id] ? (x -x_old[id]) : (x_old[id] - x);
	y_err = y > y_old[id] ? (y -y_old[id]) : (y_old[id] - y);

	if( (x_err > FILTER_MAX && y_err > FILTER_MAX/3) || (x_err > FILTER_MAX/3 && y_err > FILTER_MAX) )
	{
		filter_step_x = x_err;
		filter_step_y = y_err;
	}
	else
	{
		if(x_err > FILTER_MAX)
			filter_step_x = x_err; 
		if(y_err> FILTER_MAX)
			filter_step_y = y_err;
	}

	if(x_err <= 2*FILTER_MAX && y_err <= 2*FILTER_MAX)
	{
		filter_step_x >>= 2; 
		filter_step_y >>= 2;
	}
	else if(x_err <= 3*FILTER_MAX && y_err <= 3*FILTER_MAX)
	{
		filter_step_x >>= 1; 
		filter_step_y >>= 1;
	}	
	else if(x_err <= 4*FILTER_MAX && y_err <= 4*FILTER_MAX)
	{
		filter_step_x = filter_step_x*3/4; 
		filter_step_y = filter_step_y*3/4;
	}	
	
	x_new = x > x_old[id] ? (x_old[id] + filter_step_x) : (x_old[id] - filter_step_x);
	y_new = y > y_old[id] ? (y_old[id] + filter_step_y) : (y_old[id] - filter_step_y);

	x_old[id] = x_new;
	y_old[id] = y_new;
}
#else
static void record_point(u16 x, u16 y , u8 id)
{
	u16 x_err =0;
	u16 y_err =0;

	id_sign[id]=id_sign[id]+1;
	
	if(id_sign[id]==1){
		x_old[id]=x;
		y_old[id]=y;
	}

	x = (x_old[id] + x)/2;
	y = (y_old[id] + y)/2;
		
	if(x>x_old[id]){
		x_err=x -x_old[id];
	}
	else{
		x_err=x_old[id]-x;
	}

	if(y>y_old[id]){
		y_err=y -y_old[id];
	}
	else{
		y_err=y_old[id]-y;
	}

	if( (x_err > 3 && y_err > 1) || (x_err > 1 && y_err > 3) ){
		x_new = x;     x_old[id] = x;
		y_new = y;     y_old[id] = y;
	}
	else{
		if(x_err > 3){
			x_new = x;     x_old[id] = x;
		}
		else
			x_new = x_old[id];
		if(y_err> 3){
			y_new = y;     y_old[id] = y;
		}
		else
			y_new = y_old[id];
	}

	if(id_sign[id]==1){
		x_new= x_old[id];
		y_new= y_old[id];
	}
	
}
#endif

static int x1[FINGERNO] = {4095,4095,4095,4095,4095,4095,4095,4095,4095,4095};
static int y1[FINGERNO] = {4095,4095,4095,4095,4095,4095,4095,4095,4095,4095};


static void ssd254x_ts_work(struct work_struct *work)
{
	int i;
	unsigned short xpos=0, ypos=0, width=0;
	int FingerInfo;
	int EventStatus;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];
	//int Ssd_Timer;

	struct ssl_ts_priv *ssl_priv = container_of(work,struct ssl_ts_priv,ssl_work);
	//printk("%s\n",__FUNCTION__);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_work!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif

	EventStatus = ReadRegister(ssl_priv->client,EVENT_STATUS,2)>>4;
	//printk(KERN_INFO "EventStatus=%d\n",EventStatus); 


  #ifdef JITTER_FILTER
	  finger_status = 0;
	  for(i=0;i<ssl_priv->FingerNo;i++)
        {
	  	    if((EventStatus>>i)&0x1)
		      finger_status++;
	      }
  #endif	


	ssl_priv->FingerDetect=0;
	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		if((EventStatus>>i)&0x1)
		{
			 FingerInfo=ReadRegister(ssl_priv->client,FINGER01_REG+i,4);
			ypos = ((FingerInfo>>0)&0xF00)|((FingerInfo>>16)&0xFF);
			xpos = ((FingerInfo>>4)&0xF00)|((FingerInfo>>24)&0xFF);
			width= FingerInfo & 0x0FF;	
	//printk(KERN_INFO "[%d]:xpos=%d,ypos=%d\n",i,xpos,ypos); //linan
/*
			if (xpos<5)
				{xpos=5;
			    }	 
			if (ypos<5)
				{ypos=5;
				}
			if (xpos>SCREEN_MAX_X-5)
				{xpos=SCREEN_MAX_X-5;
				}
			if (ypos>SCREEN_MAX_Y-5)
				{ypos=SCREEN_MAX_Y-5;
				}
*/
			
			if(xpos!=0xFFF)
			{
				ssl_priv->FingerDetect++;
			}
			else 
			{
				// This part is to avoid asyn problem when the finger leaves
//				printk("		ssd254x_ts_work: Correct %x\n",EventStatus);
				EventStatus=EventStatus&~(1<<i);
				//clrFlag=1;
			}
		}
		else
		{
			xpos=ypos=0xFFF;
			width=0;
			//clrFlag=1;
		}
		
		
	  #ifdef JITTER_FILTER
			  ssd254x_jitter_filter(i, &xpos, &ypos);
		#endif	
		
		
		//if((abs(ssl_priv->FingerX[i]-xpos) >5) || (abs(ssl_priv->FingerY[i]-ypos)>5))//linan
		//{
			FingerX[i]=xpos;
			FingerY[i]=ypos;
			FingerP[i]=width;
		//}
		//else
		//{
		//	FingerX[i]=ssl_priv->FingerX[i];
		//	FingerY[i]=ssl_priv->FingerY[i];
		//	FingerP[i]=width;
		//}
	}

        //modified by Gavin 20140113
	//for(i = 0; i <= FINGERNO; i ++){
	//	if(ssl_priv->FingerDetect==0)
	//		id_sign[i] = 0;	
	//}
        //modified by Gavin 20140113
				
	if(ssl_priv->use_irq==1) enable_irq(ssl_priv->irq);
	if(ssl_priv->use_irq==2)
	{
		if(ssl_priv->FingerDetect==0) enable_irq(ssl_priv->irq);
		else{
			hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
			//printk("ssl_priv->FingerDetect=%d\n",ssl_priv->FingerDetect);
		}
	}
	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		xpos= 1280 - FingerX[i];
		ypos= 800 - FingerY[i];
		width=FingerP[i];
		
                //modified by Gavin 20140113
		//#ifdef FILTER_POINT
		//	filter_point(FingerX[i], FingerY[i] ,i);
		//#else
		//	record_point(FingerX[i], FingerY[i] , i);
		//#endif
		//xpos=x_new;
		//ypos=y_new;
                //modified by Gavin 20140113		

		if(xpos!=0xFFF)
		{
			if (sensitivity_flag)
			{
				WriteRegister(ssl_priv->client,0x34,0xC6,0x40,2);
			}
			sensitivity_flag = false;
                   

                      #ifdef ProtocolB
                        input_mt_slot(ssl_priv->input, i);
                        input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);
                        input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, xpos);
                        input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, ypos);
                        input_report_abs(ssl_priv->input, ABS_MT_WIDTH_MAJOR, width); 
                        input_report_abs(ssl_priv->input, ABS_MT_PRESSURE, width);
                        input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, width);                       
                        
                      #else
                        input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, 1);
                        input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, xpos);
                        input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, ypos);
                        input_report_abs(ssl_priv->input, ABS_MT_WIDTH_MAJOR, width);
                        input_report_abs(ssl_priv->input, ABS_MT_PRESSURE, 1);
                        input_report_abs(ssl_priv->input, BTN_TOUCH,1);
			            input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);
                        input_mt_sync(ssl_priv->input);
                      #endif

			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			if(i==0) printk("		ssd254x_ts_work: X = %d , Y = %d, W = %d\n",xpos,ypos,width);
			#endif
			 //printk("         <%d>      ssd254x_ts_work: X = %d , Y = %d, W = %d\n",i, xpos,ypos,width);
			x1[i] = xpos;
			y1[i] = ypos;
		}
		else if(ssl_priv->FingerX[i]!=0xFFF)
		{			
		  #ifdef JITTER_FILTER
					ssd254x_jitter_buffer_clean(i);
			#endif
				
			WriteRegister(ssl_priv->client,0x34,0xC6,0x64,2);
			sensitivity_flag = true;

                      #ifdef ProtocolB
                        input_mt_slot(ssl_priv->input, i);
			                  input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, -1);
                        input_mt_report_slot_state(ssl_priv->input, MT_TOOL_FINGER, false);
                      #else
                        input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, x1[i]);
                        input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, y1[i]);
			                  input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);
			                  input_report_abs(ssl_priv->input, ABS_MT_WIDTH_MAJOR, width);
			                  input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, 0);
			                  input_report_abs(ssl_priv->input, ABS_MT_PRESSURE, 0);
			                  input_report_abs(ssl_priv->input, BTN_TOUCH,0);
			                  input_mt_sync(ssl_priv->input);
			                #endif

			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			if(i==0) printk("	release	ssd254x_ts_work: X = %d , Y = %d, W = %d\n",xpos,ypos,width);
			#endif
			//printk("       release <%d> ssd254x_ts_work: X = %d , Y = %d, W = %d\n",i, x1[i],y1[i],width);
		}
		ssl_priv->FingerX[i]=FingerX[i];
		ssl_priv->FingerY[i]=FingerY[i];
		ssl_priv->FingerP[i]=width;

	}		
	ssl_priv->EventStatus=EventStatus;
		
	input_sync(ssl_priv->input);
}

static int ssd254x_ts_probe(struct i2c_client *client,const struct i2c_device_id *idp)
{
	struct ssl_ts_priv *ssl_priv;
	struct input_dev *ssl_input;
	int error;
	int i;
	save_client = client;

	if(probe_flag == 0 )
	{
	probe_flag =1;

	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_probe!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: need I2C_FUNC_I2C\n");
		#endif
		return -ENODEV;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: i2c Check OK!\n");
		printk("		ssd254x_ts_probe: i2c_client name : %s\n",client->name);
		#endif
	}

	ssl_priv = kzalloc(sizeof(*ssl_priv), GFP_KERNEL);
	if (!ssl_priv)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: kzalloc Error!\n");
		#endif
		error=-ENODEV;
		goto	err0;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: kzalloc OK!\n");
		#endif
	}
	dev_set_drvdata(&client->dev, ssl_priv);
	
	ssl_input = input_allocate_device();
	if (!ssl_input)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: input_allocate_device Error\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: input_allocate_device OK\n");
		#endif
	}

        ssl_input->evbit[0] = BIT_MASK (EV_SYN)|BIT_MASK (EV_ABS)|BIT_MASK (BTN_TOUCH);//mult touch      
	ssl_input->name = client->name;
	ssl_input->phys = "mt";
	ssl_input->id.bustype = BUS_I2C;
	ssl_input->id.vendor = 0xABCD;
        ssl_input->id.product = 0xBEEE;
//	ssl_input->id.vendor  = 0x2878; // Modify for Vendor ID
//	ssl_input->dev.parent = &client->dev;
	//ssl_input->open = ssd254x_ts_open;
	//ssl_input->close = ssd254x_ts_close;

	input_set_drvdata(ssl_input, ssl_priv);
	ssl_priv->client = client;
	ssl_priv->input = ssl_input;
	ssl_priv->use_irq = ENABLE_INT;
//	ssl_priv->irq = ON_TOUCH_INT;
	ssl_priv->irq = client->irq;
	ssl_priv->FingerNo=FINGERNO;
	ssl_priv->Resolution=64;

	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		ssl_priv->FingerP[i]=0;
		// For Finger Check Swap
		ssl_priv->sFingerX[i]=0xFFF;
		ssl_priv->sFingerY[i]=0xFFF;

		// For Adaptive Running Average
		ssl_priv->pFingerX[i]=0xFFF;
		ssl_priv->pFingerY[i]=0xFFF;
	}



#if 1
	/*******************reset the IC**********************/
	printk("reset the gpio6_pc2\n");		
	gpio_direction_output(ON_TOUCH_RST, 1);
	mdelay(5);
	gpio_direction_output(ON_TOUCH_RST, 0);
	mdelay(10);
	gpio_direction_output(ON_TOUCH_RST, 1);
	mdelay(5);
	/***************add by hjc***************/
#endif
	
	deviceReset(client);
	printk("SSL Touchscreen I2C Address: 0x%02X\n",client->addr);
	ssl_input->id.product = ReadRegister(client, DEVICE_ID_REG,2);
	ssl_input->id.version = ReadRegister(client,VERSION_ID_REG,2);

	printk("SSL Touchscreen Device ID  : 0x%04X\n",ssl_input->id.product);
	printk("SSL Touchscreen Version ID : 0x%04X\n",ssl_input->id.version);

	version = ssl_input->id.version;
	SSD254xdeviceInit(client);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("		ssd254x_ts_probe: %04XdeviceInit OK!\n",ssl_input->id.product);
	#endif

	if(ssl_priv->input->id.product==0x2543)		ssl_priv->Resolution=64;
	else if(ssl_priv->input->id.product==0x2541)	ssl_priv->Resolution=64;
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: ssl_input->id.product Error\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}

	input_set_abs_params(ssl_input, ABS_MT_TRACKING_ID, 0,FINGERNO, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_WIDTH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_PRESSURE,0, 255, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_X,  0,SCREEN_MAX_X, 0, 0);
	input_set_abs_params(ssl_input, ABS_MT_POSITION_Y,  0,SCREEN_MAX_Y, 0, 0);

        #ifdef ProtocolB
        __set_bit(EV_ABS, ssl_input->evbit);
        __set_bit(EV_KEY, ssl_input->evbit);
        __set_bit(EV_REP, ssl_input->evbit);
        __set_bit(INPUT_PROP_DIRECT, ssl_input->propbit);
        input_mt_init_slots(ssl_input, (FINGERNO+1),0);//TES added flag
        #endif

	INIT_WORK(&ssl_priv->ssl_work, ssd254x_ts_work);
	error = input_register_device(ssl_input);
	if(error)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: input_register_device input Error!\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: input_register_device input OK!\n");
		#endif
	}

	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2))
	{
		ssl_priv->irq = client->irq;//ON_TOUCH_INT;
		error = request_irq(ssl_priv->irq, ssd254x_ts_isr, IRQF_TRIGGER_LOW, client->name,ssl_priv);  
		if(error)
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd254x_ts_probe: request_irq Error!\n");
			#endif
			error=-ENODEV;
			goto err2;
		}
		else
		{
			#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
			printk("		ssd254x_ts_probe: request_irq OK!\n");
			#endif
		}	
	}

	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2))
	{
		hrtimer_init(&ssl_priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ssl_priv->timer.function = ssd254x_ts_timer;
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_probe: timer_init OK!\n");
		#endif
	}

#ifdef	CONFIG_HAS_EARLYSUSPEND
	ssl_priv->early_suspend.suspend = ssd254x_ts_early_suspend;
	ssl_priv->early_suspend.resume  = ssd254x_ts_late_resume;
	ssl_priv->early_suspend.level   = EARLY_SUSPEND_LEVEL_BLANK_SCREEN-2;
	register_early_suspend(&ssl_priv->early_suspend);
#endif
	}
	return 0;

err2:	input_unregister_device(ssl_input);
err1:	input_free_device(ssl_input);
	kfree(ssl_priv);
err0:	dev_set_drvdata(&client->dev, NULL);
	return error;
}

//static int ssd254x_ts_open(struct input_dev *dev)
//{
//	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
//	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
//	printk("+-----------------------------------------+\n");
//	printk("|	ssd254x_ts_open!                  |\n");
//	printk("+-----------------------------------------+\n");
//	#endif	
//	deviceResume(ssl_priv->client);
//	printk("%s\n",__FUNCTION__);
////HJC ADD
//	gpio_direction_output(ON_TOUCH_RST, 0);
//	mdelay(5);
//	gpio_direction_output(ON_TOUCH_RST, 1);
//	//mdelay(5);
//	
//	deviceReset(ssl_priv->client);
//	SSD254xdeviceInit(ssl_priv->client);
////HJC ADD	
//	if(ssl_priv->use_irq) enable_irq(ssl_priv->irq);
//	else hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
//	return 0;
//}

//static void ssd254x_ts_close(struct input_dev *dev)
//{
//	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
//	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
//	printk("+-----------------------------------------+\n");
//	printk("|	ssd254x_ts_close!                 |\n");
//	printk("+-----------------------------------------+\n");
//	#endif
//	deviceSuspend(ssl_priv->client);
//	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
////	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) free_irq(ssl_priv->irq, ssl_priv);
//}

static int ssd254x_ts_resume(struct i2c_client *client)
{
	int ret;
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_resume!                |\n");
	printk("+-----------------------------------------+\n");
	#endif
		tp_regulator = regulator_init(SSD254X_POWER_ID,current_val,current_val);
		ret = gpio_request(ON_TOUCH_RST, TYPE_NAME);
		if ( ret ) 
	    {
			ret = -EINVAL;
			printk("###ssd254x_ts: request gpio error!\n");
			return ret;
	    }
		Ssd_Timer_flag = 0;
	#if 0	
		deviceResume(client);	
	#else
		gpio_direction_output(ON_TOUCH_RST, 0);
		mdelay(5);
		gpio_direction_output(ON_TOUCH_RST, 1);
		mdelay(2);
		enable_irq(ssl_priv->irq);//add by hjc//
			
		deviceReset(client);
		SSD254xdeviceInit(client);
	#endif
	
	if(ssl_priv->use_irq) enable_irq(ssl_priv->irq);
	else hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

static int ssd254x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	printk("%s,ssl_priv->use_irq=%d\n",__FUNCTION__,ssl_priv->use_irq);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_suspend!               |\n");
	printk("+-----------------------------------------+\n");
	#endif	

	#if 0
		deviceSuspend(client);	
	#else
	gpio_direction_output(ON_TOUCH_RST, 0);//add by hjc
	mdelay(5);
	gpio_direction_output(ON_TOUCH_RST, 1);//add by hjc
	mdelay(2);
	#endif

	disable_irq(ssl_priv->irq);  //add by hjc////
	cancel_work_sync(&ssl_priv->ssl_work);//add by hjc
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) disable_irq(ssl_priv->irq);	
		
    gpio_free(ON_TOUCH_RST);
     if ( tp_regulator )
    {
        current_val = regulator_get_voltage(tp_regulator);
        printk("current_val is %d \n",current_val);    
        //disable_power(tp_regulator);
        regulator_deinit(tp_regulator);
    }
	return 0;
}

void  in_sleep_ssd254x(void) {
printk("SSD254x in sleep\n");
	deviceSuspend(save_client);
}

void  out_sleep_ssd254x(void) {
printk("SSD254x out sleep\n");
        deviceResume(save_client);
}

#ifdef	CONFIG_HAS_EARLYSUSPEND
static void ssd254x_ts_late_resume(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_late_resume!           |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd254x_ts_resume(ssl_priv->client);
}
static void ssd254x_ts_early_suspend(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_early_suspend!         |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd254x_ts_suspend(ssl_priv->client, PMSG_SUSPEND);
}
#endif

static int ssd254x_ts_remove(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);	
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_remove !               |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	if((ssl_priv->use_irq==0)||(ssl_priv->use_irq==2)) hrtimer_cancel(&ssl_priv->timer);
	//if((ssl_priv->use_irq==1)||(ssl_priv->use_irq==2)) free_irq(ssl_priv->irq, ssl_priv);
	input_unregister_device(ssl_priv->input);
	input_free_device(ssl_priv->input);
	kfree(ssl_priv);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static irqreturn_t ssd254x_ts_isr(int irq, void *dev_id)
{
	struct ssl_ts_priv *ssl_priv = dev_id;
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_isr!                   |\n");
	printk("+-----------------------------------------+\n");
	#endif	
	//printk("%s\n",__FUNCTION__);
	disable_irq_nosync(ssl_priv->irq);
	queue_work(ssd254x_wq, &ssl_priv->ssl_work);
	return IRQ_HANDLED;
}

static enum hrtimer_restart ssd254x_ts_timer(struct hrtimer *timer)
{
	struct ssl_ts_priv *ssl_priv = container_of(timer, struct ssl_ts_priv, timer);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_timer!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	queue_work(ssd254x_wq, &ssl_priv->ssl_work);
	if(ssl_priv->use_irq==0) hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static const struct i2c_device_id ssd254x_ts_id[] = {
	{ TYPE_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ssd254x_ts_id);

static struct i2c_board_info i2c_board_info[] = {
	{
		I2C_BOARD_INFO(TYPE_NAME, I2C_CTPM_ADDRESS),
		.platform_data = NULL,
	},
};

static struct i2c_driver ssd254x_ts_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = TYPE_NAME,
	},
	.probe = ssd254x_ts_probe,
	.remove = ssd254x_ts_remove,
	.id_table = ssd254x_ts_id,
};

static char banner[] __initdata = KERN_INFO "SSL Touchscreen driver, (c) 2011 Solomon Systech Ltd.\n";
static int __init ssd254x_ts_init(void)
{
	int ret;
	struct i2c_adapter *adapter;
	struct i2c_client *client;
	
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG	
	printk("+-----------------------------------------+\n");
	printk("|	SSL_ts_init!                      |\n");
	printk("+-----------------------------------------+\n");
	#endif
	
	tp_regulator = regulator_init(SSD254X_POWER_ID,SSD254X_POWER_MIN_VOL, SSD254X_POWER_MAX_VOL);
	if ( !tp_regulator ) {
		ret = -EINVAL;
	}
	
	printk("\n##ssd254x_ts_init init... \n");
#if 0 //TES
	ret = get_config("ctp.xMax", (char *)(&cfg_xml.xMax), sizeof(unsigned int));
	if (ret != 0) {
		printk(KERN_ERR "get xMax %d fail\n", cfg_xml.xMax);
		return ret;
	}else{  	   
	    printk("get xMax %d ok\n", cfg_xml.xMax);
	}
	
    ret |= get_config("ctp.yMax", (char *)(&cfg_xml.yMax), sizeof(unsigned int));
	if (ret != 0) {
		printk(KERN_ERR "get yMax %d fail\n", cfg_xml.yMax);
		return ret;
	}else{  	   
	    printk("get yMax %d ok\n", cfg_xml.yMax);
	}
	
	ret |= get_config("ctp.rotate", (char *)(&cfg_xml.rotate), sizeof(unsigned int));
	if (ret != 0) {
		printk(KERN_ERR "get rotate %d fail\n", cfg_xml.rotate);
		return ret;
	}else{  	   
	    printk("get rotate %d ok\n", cfg_xml.rotate);
	}
    
#endif

	
	ret = gpio_request(ON_TOUCH_RST, TYPE_NAME);
	if ( ret ) 
    {
		ret = -EINVAL;
		printk("###ssd254x_ts: request gpio error!\n");
		return ret;
    }
    
	printk(banner);
	ssd254x_wq = create_singlethread_workqueue("ssd254x_wq");
	if (!ssd254x_wq)
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_init: create_singlethread_workqueue Error!\n");
		#endif
		return -ENOMEM;
	}
	else
	{
		#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
		printk("		ssd254x_ts_init: create_singlethread_workqueue OK!\n");
		#endif
	}
	
    struct ssl_ts_priv *Ssd254x_ts;    
	Ssd254x_ts = kzalloc(sizeof(*Ssd254x_ts), GFP_KERNEL);
	//Ssd254x_ts->i2c_address = I2C_CTPM_ADDRESS;

	adapter = i2c_get_adapter(1);
	if ( !adapter ) {
		printk("Unable to get i2c adapter on bus%d\n",1);
		return -ENODEV;
	}

	client = i2c_new_device(adapter, i2c_board_info);
	//g_i2c_client = client;
	printk("register client:%p\n", client);
	//i2c_put_adapter(adapter);
	if (!client) {
		printk("Unable to create i2c device on bus %d\n",1);
		return -ENODEV;
	}

	Ssd254x_ts->client = client;
	i2c_set_clientdata(client, Ssd254x_ts);

	ret=i2c_add_driver(&ssd254x_ts_driver);
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	if(ret) printk("		ssd254x_ts_init: i2c_add_driver Error! \n");
	else    printk("		ssd254x_ts_init: i2c_add_driver OK! \n");
	#endif
	misc_register(&misc);
	return ret;
}

static void __exit ssd254x_ts_exit(void)
{
	#ifdef CONFIG_TOUCHSCREEN_SSL_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd254x_ts_exit!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif
	i2c_del_driver(&ssd254x_ts_driver);
	misc_deregister(&misc);
	if (ssd254x_wq) {
		destroy_workqueue(ssd254x_wq);
	}
	gpio_free(ON_TOUCH_RST);
	if ( tp_regulator ) {
        regulator_disable(tp_regulator);
        regulator_put(tp_regulator);
    }
	probe_flag=0;
}

static ssize_t tp_read(struct file *file, char __user *buf, size_t count,
                loff_t *offset)
{
    char *kbuf;
    uint8_t reg;
    int  ByteNo;
    int readValue;
    int i;

    kbuf = kmalloc(count,GFP_KERNEL);

    if(copy_from_user(kbuf,buf,1))
    {
        printk("no enough memory!\n");
        return -1;
    }

    reg = (uint8_t)kbuf[0];
    ByteNo = count;

//              printk("[%s]===ReadRegister:reg=0x%02x,ByteNo=%d============\n",__func__,reg,ByteNo);
    readValue = ReadRegister(save_client, reg, ByteNo);
//              printk("[%s]===readValue:0x%04x============\n",__func__,readValue);
    for(i = 0;i < ByteNo;i++)
    {
            kbuf[i] = (readValue>>(8*i)) & 0xff;
//                      printk("[%s]===read:0x%02x============\n",__func__,kbuf[i]);
    }

    if(copy_to_user(buf,kbuf,count))
    {
        printk("no enough memory!\n");
        return -1;
    }
    kfree(kbuf);

	return count;
}

static ssize_t tp_write(struct file *file, const char __user *buf,
                size_t count, loff_t *offset)
{
    char *kbuf;

    kbuf = kmalloc(count,GFP_KERNEL);

    if(copy_from_user(kbuf,buf,count))
    {
        printk("no enough memory!\n");
        return -1;
    }
//    printk("<1>spi write!,count=%d,buf=0x%02x,0x%02x,0x%02x,0x%02x\n",count,kbuf[0],kbuf[1],kbuf[2],kbuf[3]);

    //gpio reset
    if(kbuf[1] == 0x04)
    {
        gpio_direction_output(ON_TOUCH_RST,0);
        msleep(1);
        gpio_direction_output(ON_TOUCH_RST,1);
    }

    WriteRegister(  save_client,kbuf[1],kbuf[2],kbuf[3],kbuf[0]);

    if(kbuf[1] == 0x04)
    {
    	msleep(100);
    }
	kfree(kbuf);

	return count;
}

module_init(ssd254x_ts_init);
module_exit(ssd254x_ts_exit);

MODULE_AUTHOR("Solomon Systech Ltd - Gavin Liu");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ssd254x Touchscreen Driver");


