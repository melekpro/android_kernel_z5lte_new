/*
 * DW97141AF voice coil motor driver
 *
 *
 */

#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/fs.h>

#include "lens_info.h"


#define AF_DRVNAME "DW97141AF_DRV"
#define AF_I2C_SLAVE_ADDR        0x18

#define AF_DEBUG
#ifdef AF_DEBUG
#define LOG_INF(format, args...) pr_debug(AF_DRVNAME " [%s] " format, __FUNCTION__, ##args)
#else
#define LOG_INF(format, args...)
#endif


static struct i2c_client * g_pstAF_I2Cclient = NULL;

static int *g_pAF_Opened;
static spinlock_t *g_pAF_SpinLock;


static long g_i4MotorStatus = 0;
static long g_i4Dir = 0;
static unsigned long g_u4AF_INF = 0;
static unsigned long g_u4AF_MACRO = 1023;
static unsigned long g_u4TargetPosition = 0;
static unsigned long g_u4CurrPosition    = 0;

static int g_sr = 3;

static int s4AF_ReadReg(unsigned short *a_pu2Result)
{
    int  i4RetValue = 0;
    char pBuff[2];

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

    i4RetValue = i2c_master_recv(g_pstAF_I2Cclient, pBuff , 2);

    if (i4RetValue < 0)
    {
        LOG_INF("I2C read failed!! \n");
        return -1;
    }

   *a_pu2Result = (((u16)(pBuff[0] & 0x03)) << 8) + pBuff[1];

    return 0;
}

static int s4AF_WriteReg(u16 a_u2Data)
{
    int  i4RetValue = 0;

	char puSendCmd[2] = {(char)(((a_u2Data >> 8) & 0x03) | 0xc4), (char)(a_u2Data & 0xff)};

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;

     i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd, 2);

    if (i4RetValue < 0)
    {
        LOG_INF("I2C send failed!! \n");
        return -1;
    }

    return 0;
}

static inline int getAFInfo(__user stAF_MotorInfo *pstMotorInfo)
{
	stAF_MotorInfo stMotorInfo;

    stMotorInfo.u4MacroPosition   = g_u4AF_MACRO;
    stMotorInfo.u4InfPosition      = g_u4AF_INF;
    stMotorInfo.u4CurrentPosition = g_u4CurrPosition;
    stMotorInfo.bIsSupportSR      = 1;

    if (g_i4MotorStatus == 1)    {stMotorInfo.bIsMotorMoving = 1;}
    else                        {stMotorInfo.bIsMotorMoving = 0;}

    if (*g_pAF_Opened >= 1)    {stMotorInfo.bIsMotorOpen = 1;}
    else                        {stMotorInfo.bIsMotorOpen = 0;}

    if(copy_to_user(pstMotorInfo , &stMotorInfo , sizeof(stAF_MotorInfo)))
    {
        LOG_INF("copy to user failed when getting motor information \n");
    }

	return 0;
}

static inline int moveAF(unsigned long a_u4Position)
{
	int ret = 0;
	
	if((a_u4Position > g_u4AF_MACRO) || (a_u4Position < g_u4AF_INF))
	{
		LOG_INF("out of range \n");
		return -EINVAL;
	}
	
	if (*g_pAF_Opened == 1)
	{
		unsigned short InitPos;
		ret = s4AF_ReadReg(&InitPos);
	
		if(ret == 0)
		{
			LOG_INF("Init Pos %6d \n", InitPos);
	
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = (unsigned long)InitPos;
			spin_unlock(g_pAF_SpinLock);
	
		}
		else
		{
			spin_lock(g_pAF_SpinLock);
			g_u4CurrPosition = 0;
			spin_unlock(g_pAF_SpinLock);
		}
	
		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 2;
		spin_unlock(g_pAF_SpinLock);
	}
	
	if (g_u4CurrPosition < a_u4Position)
	{
		spin_lock(g_pAF_SpinLock);
		g_i4Dir = 1;
		spin_unlock(g_pAF_SpinLock);
	}
	else if (g_u4CurrPosition > a_u4Position)
	{
		spin_lock(g_pAF_SpinLock);
		g_i4Dir = -1;
		spin_unlock(g_pAF_SpinLock);
	}
	else
	{
		return 0;
	}
	
	spin_lock(g_pAF_SpinLock);
	g_u4TargetPosition = a_u4Position;
	spin_unlock(g_pAF_SpinLock);
	
	//LOG_INF("move [curr] %d [target] %d\n", g_u4CurrPosition, g_u4TargetPosition);
	
	spin_lock(g_pAF_SpinLock);
	g_sr = 3;
	g_i4MotorStatus = 0;
	spin_unlock(g_pAF_SpinLock);
	
	if(s4AF_WriteReg((unsigned short)g_u4TargetPosition) == 0)
	{
		spin_lock(g_pAF_SpinLock);
		g_u4CurrPosition = (unsigned long)g_u4TargetPosition;
		spin_unlock(g_pAF_SpinLock);
	}
	else
	{
		LOG_INF("set I2C failed when moving the motor \n");
		spin_lock(g_pAF_SpinLock);
		g_i4MotorStatus = -1;
		spin_unlock(g_pAF_SpinLock);
	}
	return 0;
}

static inline int setAFInf(unsigned long a_u4Position)
{
    spin_lock(g_pAF_SpinLock);
    g_u4AF_INF = a_u4Position;
    spin_unlock(g_pAF_SpinLock);
    return 0;
}

static inline int setAFMacro(unsigned long a_u4Position)
{
    spin_lock(g_pAF_SpinLock);
    g_u4AF_MACRO = a_u4Position;
    spin_unlock(g_pAF_SpinLock);
    return 0;
}

////////////////////////////////////////////////////////////////
long DW97141AF_Ioctl(struct file *a_pstFile, unsigned int a_u4Command, unsigned long a_u4Param)
{
	long i4RetValue = 0;
	
	switch (a_u4Command) 
	{
		case AFIOC_G_MOTORINFO:
			i4RetValue = getAFInfo((__user stAF_MotorInfo *) (a_u4Param));
			break;
		
		case AFIOC_T_MOVETO:
			i4RetValue = moveAF(a_u4Param);
			break;
	
		case AFIOC_T_SETINFPOS:
			i4RetValue = setAFInf(a_u4Param);
			break;
	
		case AFIOC_T_SETMACROPOS:
			i4RetValue = setAFMacro(a_u4Param);
			break;
	
		default:
			LOG_INF("No CMD\n");
			i4RetValue = -EPERM;
			break;
	}
	return i4RetValue;
}

//Main jobs:
// 1.Deallocate anything that "open" allocated in private_data.
// 2.Shut down the device on last close.
// 3.Only called once on last time.
// Q1 : Try release multiple times.
int DW97141AF_Release(struct inode *a_pstInode, struct file *a_pstFile)
{
    LOG_INF("Start \n");

	if (*g_pAF_Opened == 2)
    {
        g_sr = 5;
        s4AF_WriteReg(200);
        msleep(10);
        s4AF_WriteReg(100);
        msleep(10);    
    }

	if (*g_pAF_Opened)
    {
        LOG_INF("Free \n");

		spin_lock(g_pAF_SpinLock);
		*g_pAF_Opened = 0;
		spin_unlock(g_pAF_SpinLock);
    }

    LOG_INF("End \n");

    return 0;
}

void DW97141AF_SetI2Cclient(struct i2c_client *pstAF_I2Cclient, spinlock_t *pAF_SpinLock, int *pAF_Opened)
{

	int  i4RetValue = 0;

    char puSendCmd1[2]={(char)(0x94),(char)(0x64)};//A point=100d 
    char puSendCmd2[2]={(char)(0x9C),(char)(0x8B)};//B point=139d 
    char puSendCmd3[2]={(char)(0xA4),(char)(0xE4)};//A-B point step mode:200us/7Lsd 
    char puSendCmd4[2]={(char)(0x8C),(char)(0x63)};//105Hz,Fastest mode
	
	g_pstAF_I2Cclient = pstAF_I2Cclient;
	g_pAF_SpinLock = pAF_SpinLock;
	g_pAF_Opened = pAF_Opened;

	g_pstAF_I2Cclient->addr = AF_I2C_SLAVE_ADDR;

	g_pstAF_I2Cclient->addr = g_pstAF_I2Cclient->addr >> 1;
	
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd1, 2);	
    if (i4RetValue < 0) 
    {
        LOG_INF("[DW97141AF] I2C send failed!! \n");
        return;
    }
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd2, 2);
    if (i4RetValue < 0) 
    {
        LOG_INF("[DW97141AF] I2C send failed!! \n");
        return;
    }
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd3, 2);
    if (i4RetValue < 0) 
    {
        LOG_INF("[DW97141AF] I2C send failed!! \n");
        return;
    }
    i4RetValue = i2c_master_send(g_pstAF_I2Cclient, puSendCmd4, 2);
    if (i4RetValue < 0) 
    {
        LOG_INF("[DW97141AF] I2C send failed!! \n");
        return;
    }
}
