#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"

#define OLED_I2C_ADDRESS       (0x3CU)
#define OLED_WIDTH             (128U)
#define OLED_HEIGHT            (64U)
#define OLED_I2C_WAIT_TIMEOUT_COUNT  (100000U)
#define OLED_I2C_IDLE_GRACE_COUNT    (16U)
#define OLED_I2C_BUS_CLEAR_PULSES    (9U)
#define OLED_I2C_BUS_CLEAR_WAIT_COUNT (10000U)
#define OLED_I2C_BUS_CLEAR_DELAY_CYCLES (320U)
#define OLED_I2C_LINE_PINS \
    (GPIO_OLED_SDA_PIN | GPIO_OLED_SCL_PIN)
#define OLED_I2C_ERROR_STATUS        (DL_I2C_CONTROLLER_STATUS_ERROR | \
    DL_I2C_CONTROLLER_STATUS_ARBITRATION_LOST)

u8 OLED_GRAM[144][8];
extern void delay_ms(uint32_t ms);
static volatile uint8_t g_oledError;
static uint8_t g_oledInitActive;
static uint8_t g_oledRecoverActive;

/*
 * 作用：给 I2C bus clear 提供很短的 GPIO 时序间隔。
 * 使用场景：手动拉低/释放 SCL、SDA 时，保证外部上拉和 OLED 有反应时间。
 */
static void OLED_BusClearDelay(void)
{
    delay_cycles(OLED_I2C_BUS_CLEAR_DELAY_CYCLES);
}

/*
 * 作用：把 PA0/PA1 临时切成 GPIO 开漏输出，并释放到高电平。
 * 使用场景：I2C 控制器异常后，手动打 SCL 脉冲释放被卡住的从机。
 */
static void OLED_ConfigBusClearGPIO(void)
{
    DL_GPIO_setPins(GPIO_OLED_SDA_PORT, OLED_I2C_LINE_PINS);
    DL_GPIO_initDigitalOutputFeatures(GPIO_OLED_IOMUX_SDA,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_ENABLE);
    DL_GPIO_initDigitalOutputFeatures(GPIO_OLED_IOMUX_SCL,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_DRIVE_STRENGTH_LOW, DL_GPIO_HIZ_ENABLE);
    DL_GPIO_enableOutput(GPIO_OLED_SDA_PORT, OLED_I2C_LINE_PINS);
    OLED_BusClearDelay();
}

/*
 * 作用：把 PA0/PA1 切回 I2C0 复用功能。
 * 使用场景：bus clear 完成后重新启用硬件 I2C 控制器。
 */
static void OLED_ConfigPeripheralPins(void)
{
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_OLED_IOMUX_SDA, GPIO_OLED_IOMUX_SDA_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initPeripheralInputFunctionFeatures(
        GPIO_OLED_IOMUX_SCL, GPIO_OLED_IOMUX_SCL_FUNC,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_DISABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SDA);
    DL_GPIO_enableHiZ(GPIO_OLED_IOMUX_SCL);
}

/*
 * 作用：等待某根 I2C 线被上拉释放为高电平。
 * 使用场景：判断 SCL 是否仍被外设拉低，避免 bus clear 自己死等。
 */
static uint8_t OLED_WaitLineHigh(uint32_t pin)
{
    uint32_t timeout = OLED_I2C_BUS_CLEAR_WAIT_COUNT;

    while (timeout > 0U) {
        if ((DL_GPIO_readPins(GPIO_OLED_SDA_PORT, pin) & pin) != 0U) {
            return 1U;
        }
        --timeout;
    }
    return 0U;
}

/*
 * 作用：执行 I2C bus clear。
 * 使用场景：OLED/I2C 超时、NACK 或仲裁丢失后，手动发送 9 个 SCL 脉冲，
 *          再生成一个 STOP，尽量把被卡住的 OLED 从机释放出来。
 */
static uint8_t OLED_BusClear(void)
{
    uint8_t index;

    OLED_ConfigBusClearGPIO();
    DL_GPIO_setPins(GPIO_OLED_SDA_PORT, OLED_I2C_LINE_PINS);
    if (OLED_WaitLineHigh(GPIO_OLED_SCL_PIN) == 0U) {
        return 0U;
    }

    for (index = 0U; index < OLED_I2C_BUS_CLEAR_PULSES; ++index) {
        DL_GPIO_clearPins(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
        OLED_BusClearDelay();
        DL_GPIO_setPins(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
        if (OLED_WaitLineHigh(GPIO_OLED_SCL_PIN) == 0U) {
            return 0U;
        }
        OLED_BusClearDelay();
    }

    /* 生成 STOP：SDA 低 -> SCL 高 -> SDA 高。 */
    DL_GPIO_clearPins(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN);
    OLED_BusClearDelay();
    DL_GPIO_setPins(GPIO_OLED_SCL_PORT, GPIO_OLED_SCL_PIN);
    if (OLED_WaitLineHigh(GPIO_OLED_SCL_PIN) == 0U) {
        return 0U;
    }
    OLED_BusClearDelay();
    DL_GPIO_setPins(GPIO_OLED_SDA_PORT, GPIO_OLED_SDA_PIN);
    OLED_BusClearDelay();

    return ((DL_GPIO_readPins(GPIO_OLED_SDA_PORT, OLED_I2C_LINE_PINS) &
        OLED_I2C_LINE_PINS) == OLED_I2C_LINE_PINS) ? 1U : 0U;
}

/*
 * 作用：复位当前 OLED I2C 传输状态。
 * 使用场景：I2C 等待超时、NACK、仲裁丢失后释放控制器，避免继续卡在同一次传输。
 */
static void OLED_ResetTransfer(void)
{
    DL_I2C_resetControllerTransfer(OLED_INST);
    DL_I2C_startFlushControllerTXFIFO(OLED_INST);
    DL_I2C_stopFlushControllerTXFIFO(OLED_INST);
    DL_I2C_startFlushControllerRXFIFO(OLED_INST);
    DL_I2C_stopFlushControllerRXFIFO(OLED_INST);
}

/*
 * 作用：记录 OLED/I2C 错误并复位本次传输。
 * 使用场景：OLED 未接好、SCL/SDA 松动、地址无应答或总线异常。
 * 说明：这里不直接重入 OLED_Init，恢复动作由 OLED_WR_Byte 统一触发。
 */
static void OLED_SetError(void)
{
    g_oledError = 1U;
    OLED_ResetTransfer();
}

/*
 * 作用：等待 I2C 状态达到期望值。
 * 使用场景：OLED_WR_Byte 中等待控制器空闲。
 * 说明：所有等待都带超时，不能写死循环。
 */
static uint8_t OLED_WaitStatus(uint32_t mask, uint32_t expected)
{
    uint32_t timeout = OLED_I2C_WAIT_TIMEOUT_COUNT;
    uint32_t status;

    while (timeout > 0U) {
        status = DL_I2C_getControllerStatus(OLED_INST);
        if ((status & OLED_I2C_ERROR_STATUS) != 0U) {
            OLED_SetError();
            return 0U;
        }
        if ((status & mask) == expected) {
            return 1U;
        }
        --timeout;
    }

    OLED_SetError();
    return 0U;
}

/*
 * 作用：等待一次 OLED I2C 发送结束。
 * 使用场景：OLED_WR_Byte 启动传输后等待 STOP 完成。
 * 说明：即使 OLED 线松，也只会置错误标志，不会把程序卡死。
 */
static uint8_t OLED_WaitTransferDone(void)
{
    uint32_t timeout = OLED_I2C_WAIT_TIMEOUT_COUNT;
    uint32_t idleGrace = OLED_I2C_IDLE_GRACE_COUNT;
    uint32_t status;
    uint8_t sawBusy = 0U;

    while (timeout > 0U) {
        status = DL_I2C_getControllerStatus(OLED_INST);
        if ((status & OLED_I2C_ERROR_STATUS) != 0U) {
            OLED_SetError();
            return 0U;
        }
        if ((status & (DL_I2C_CONTROLLER_STATUS_BUSY |
            DL_I2C_CONTROLLER_STATUS_BUSY_BUS)) != 0U) {
            sawBusy = 1U;
        }
        if ((status & DL_I2C_CONTROLLER_STATUS_IDLE) != 0U) {
            if (sawBusy != 0U) {
                return 1U;
            }
            if (idleGrace == 0U) {
                return 1U;
            }
            --idleGrace;
        }
        --timeout;
    }

    OLED_SetError();
    return 0U;
}

uint8_t OLED_HasError(void)
{
    return g_oledError;
}

/*
 * 作用：对 OLED I2C 总线做一次有限恢复。
 * 使用场景：等待超时、NACK、SCL/SDA 被拉住后，先手动 bus clear，
 *          再恢复 I2C 控制器和 OLED 初始化序列。
 * 返回值：1 表示恢复成功，0 表示总线仍异常。
 */
uint8_t OLED_TryRecover(void)
{
    uint8_t recovered;

    if (g_oledRecoverActive != 0U) {
        return 0U;
    }

    g_oledRecoverActive = 1U;
    OLED_ResetTransfer();
    DL_I2C_disableController(OLED_INST);

    recovered = OLED_BusClear();
    OLED_ConfigPeripheralPins();
    SYSCFG_DL_OLED_init();

    if (recovered != 0U) {
        g_oledError = 0U;
        if (g_oledInitActive == 0U) {
            OLED_Init();
            recovered = (g_oledError == 0U) ? 1U : 0U;
        }
    }

    if (recovered == 0U) {
        g_oledError = 1U;
    }
    g_oledRecoverActive = 0U;
    return recovered;
}

void OLED_ClearError(void)
{
    g_oledError = 0U;
    OLED_ResetTransfer();
}

//反显函数
void OLED_ColorTurn(u8 i)
{
	if(i==0) OLED_WR_Byte(0xA6,OLED_CMD);//正常显示
	if(i==1) OLED_WR_Byte(0xA7,OLED_CMD);//反色显示
}

//屏幕旋转180度
void OLED_DisplayTurn(u8 i)
{
	if(i==0)
	{
		OLED_WR_Byte(0xC8,OLED_CMD);//正常显示
		OLED_WR_Byte(0xA1,OLED_CMD);
	}
	if(i==1)
	{
		OLED_WR_Byte(0xC0,OLED_CMD);//反转显示
		OLED_WR_Byte(0xA0,OLED_CMD);
	}
}

/*
 * 作用：只尝试发送一次 OLED 字节，不在内部递归恢复。
 * 使用场景：OLED_WR_Byte 的正常发送和恢复后的单次重试。
 */
static uint8_t OLED_WriteByteOnce(uint8_t dat, uint8_t mode)
{
    uint8_t txData[2];

    if (g_oledError != 0U) {
        return 0U;
    }
    
    // 控制字节: 0x00为命令, 0x40为数据
    txData[0] = mode ? 0x40 : 0x00; 
    txData[1] = dat;

    // 1. 等待 I2C 彻底空闲，带超时保护
    if (!OLED_WaitStatus(DL_I2C_CONTROLLER_STATUS_IDLE,
        DL_I2C_CONTROLLER_STATUS_IDLE)) {
        return 0U;
    }
    
    // 2. 将 2 个字节填入发送 FIFO
    if (DL_I2C_fillControllerTXFIFO(OLED_INST, txData, 2) != 2U) {
        OLED_SetError();
        return 0U;
    }
    
    // 3. 启动传输
    DL_I2C_startControllerTransfer(OLED_INST, OLED_I2C_ADDRESS, DL_I2C_CONTROLLER_DIRECTION_TX, 2);
    
    // 4. 等待 I2C 回到空闲状态，代表本次传输结束
    return OLED_WaitTransferDone();
}

void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    if (g_oledError != 0U) {
        return;
    }

    if (OLED_WriteByteOnce(dat, mode) != 0U) {
        return;
    }

    if (g_oledRecoverActive != 0U) {
        return;
    }

    if (OLED_TryRecover() != 0U) {
        (void)OLED_WriteByteOnce(dat, mode);
    }
}

//开启OLED显示 
void OLED_DisPlay_On(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
	OLED_WR_Byte(0x14,OLED_CMD);//开启电荷泵
	OLED_WR_Byte(0xAF,OLED_CMD);//点亮屏幕
}

//关闭OLED显示 
void OLED_DisPlay_Off(void)
{
	OLED_WR_Byte(0x8D,OLED_CMD);//电荷泵使能
	OLED_WR_Byte(0x10,OLED_CMD);//关闭电荷泵
	OLED_WR_Byte(0xAE,OLED_CMD);//关闭屏幕
}

//更新显存到OLED	
void OLED_Refresh(void)
{
	u8 i,n;
    if (g_oledError != 0U) {
        return;
    }
	for(i=0;i<8;i++)
	{
	   OLED_WR_Byte(0xb0+i,OLED_CMD); //设置行起始地址
	   OLED_WR_Byte(0x00,OLED_CMD);   //设置低列起始地址
	   OLED_WR_Byte(0x10,OLED_CMD);   //设置高列起始地址
	   for(n=0;n<128;n++)
       {
         if (g_oledError != 0U) {
             return;
         }
		 OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
       }
	}
}

//清屏函数
void OLED_Clear(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
	   for(n=0;n<128;n++)
		{
			 OLED_GRAM[n][i]=0;//清除所有数据
		}
	}
	OLED_Refresh();//更新显示
}

//画点 
void OLED_DrawPoint(u8 x,u8 y)
{
	u8 i,m,n;
	i=y/8;
	m=y%8;
	n=1<<m;
	OLED_GRAM[x][i]|=n;
}

//清除一个点
void OLED_ClearPoint(u8 x,u8 y)
{
	u8 i,m,n;
	i=y/8;
	m=y%8;
	n=1<<m;
	OLED_GRAM[x][i]=~OLED_GRAM[x][i];
	OLED_GRAM[x][i]|=n;
	OLED_GRAM[x][i]=~OLED_GRAM[x][i];
}

//画线
void OLED_DrawLine(u8 x1,u8 y1,u8 x2,u8 y2)
{
	u8 i,k,k1,k2;
	if((x2>128)||(y2>64)||(x1>x2)||(y1>y2))return;
	if(x1==x2)    //画竖线
	{
		for(i=0;i<(y2-y1);i++) OLED_DrawPoint(x1,y1+i);
	}
	else if(y1==y2)   //画横线
	{
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1);
	}
	else      //画斜线
	{
		k1=y2-y1;
		k2=x2-x1;
		k=k1*10/k2;
		for(i=0;i<(x2-x1);i++) OLED_DrawPoint(x1+i,y1+i*k/10);
	}
}

//画圆
void OLED_DrawCircle(u8 x,u8 y,u8 r)
{
	int a = 0, b = r, num;
	while(2 * b * b >= r * r)      
	{
		OLED_DrawPoint(x + a, y - b);
		OLED_DrawPoint(x - a, y - b);
		OLED_DrawPoint(x - a, y + b);
		OLED_DrawPoint(x + a, y + b);
		OLED_DrawPoint(x + b, y + a);
		OLED_DrawPoint(x + b, y - a);
		OLED_DrawPoint(x - b, y - a);
		OLED_DrawPoint(x - b, y + a);
		
		a++;
		num = (a * a + b * b) - r*r;
		if(num > 0) { b--; a--; }
	}
}

//显示字符
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size1)
{
	u8 i,m,temp,size2,chr1;
	u8 y0=y;
	size2=(size1/8+((size1%8)?1:0))*(size1/2);  
	chr1=chr-' ';  
	for(i=0;i<size2;i++)
	{
		if(size1==12) {temp=asc2_1206[chr1][i];} 
		else if(size1==16) {temp=asc2_1608[chr1][i];} 
		else if(size1==24) {temp=asc2_2412[chr1][i];} 
		else return;
		for(m=0;m<8;m++)           
		{
			if(temp&0x80)OLED_DrawPoint(x,y);
			else OLED_ClearPoint(x,y);
			temp<<=1;
			y++;
			if((y-y0)==size1)
			{
				y=y0;
				x++;
				break;
			}
		}
	}
}

//显示字符串
void OLED_ShowString(u8 x,u8 y,u8 *chr,u8 size1)
{
	while((*chr>=' ')&&(*chr<='~'))
	{
		OLED_ShowChar(x,y,*chr,size1);
		x+=size1/2;
		if(x>128-size1)  //换行
		{
			x=0;
			y+=size1; // 修复了原代码的y+=2的bug
		}
		chr++;
	}
}

void OLED_ShowLine(u8 line, const char *text, u8 size1)
{
	u8 y;

	if ((text == NULL) || (size1 == 0U)) {
		return;
	}
	y = (u8)(line * size1);
	if (y >= OLED_HEIGHT) {
		return;
	}
	OLED_ShowString(0U, y, (u8 *)text, size1);
}

//m^n
u32 OLED_Pow(u8 m,u8 n)
{
	u32 result=1;
	while(n--) result*=m;
	return result;
}

//显示数字
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size1)
{
	u8 t,temp;
	for(t=0;t<len;t++)
	{
		temp=(num/OLED_Pow(10,len-t-1))%10;
		if(temp==0) OLED_ShowChar(x+(size1/2)*t,y,'0',size1);
		else OLED_ShowChar(x+(size1/2)*t,y,temp+'0',size1);
	}
}

//显示汉字
void OLED_ShowChinese(u8 x,u8 y,u8 num,u8 size1)
{
	u8 i,m,n=0,temp,chr1;
	u8 x0=x,y0=y;
	u8 size3=size1/8;
	while(size3--)
	{
		chr1=num*size1/8+n;
		n++;
		for(i=0;i<size1;i++)
		{
			if(size1==16) {temp=Hzk1[chr1][i];}
			else if(size1==24) {temp=Hzk2[chr1][i];}
			else if(size1==32) {temp=Hzk3[chr1][i];}
			else if(size1==64) {temp=Hzk4[chr1][i];}
			else return;
						
			for(m=0;m<8;m++)
			{
				if(temp&0x01)OLED_DrawPoint(x,y);
				else OLED_ClearPoint(x,y);
				temp>>=1;
				y++;
			}
			x++;
			if((x-x0)==size1) {x=x0;y0=y0+8;}
			y=y0;
		}
	}
}

//配置写入数据的起始位置
void OLED_WR_BP(u8 x,u8 y)
{
    if (g_oledError != 0U) {
        return;
    }
	OLED_WR_Byte(0xb0+y,OLED_CMD);//设置行起始地址
	OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	OLED_WR_Byte((x&0x0f)|0x01,OLED_CMD);
}

//显示图片
void OLED_ShowPicture(u8 x0,u8 y0,u8 x1,u8 y1,u8 BMP[])
{
	u32 j=0;
	u8 x=0,y=0;
	if(y%8==0)y=0;
	else y+=1;
	for(y=y0;y<y1;y++)
	{
		 OLED_WR_BP(x0,y);
		 for(x=x0;x<x1;x++)
		 {
             if (g_oledError != 0U) {
                 return;
             }
			 OLED_WR_Byte(BMP[j],OLED_DATA);
			 j++;
		 }
	}
}

//OLED的初始化
void OLED_Init(void)
{
    g_oledInitActive = 1U;
	// 4针OLED没有RST引脚，直接延时等待屏幕内部RC电路上电复位完成
	delay_ms(100);
    OLED_ClearError();
	
	OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
	OLED_WR_Byte(0x00,OLED_CMD);//---set low column address
	OLED_WR_Byte(0x10,OLED_CMD);//---set high column address
	OLED_WR_Byte(0x40,OLED_CMD);//--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
	OLED_WR_Byte(0x81,OLED_CMD);//--set contrast control register
	OLED_WR_Byte(0xCF,OLED_CMD);// Set SEG Output Current Brightness
	OLED_WR_Byte(0xA1,OLED_CMD);//--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
	OLED_WR_Byte(0xC8,OLED_CMD);//Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
	OLED_WR_Byte(0xA6,OLED_CMD);//--set normal display
	OLED_WR_Byte(0xA8,OLED_CMD);//--set multiplex ratio(1 to 64)
	OLED_WR_Byte(0x3f,OLED_CMD);//--1/64 duty
	OLED_WR_Byte(0xD3,OLED_CMD);//-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
	OLED_WR_Byte(0x00,OLED_CMD);//-not offset
	OLED_WR_Byte(0xd5,OLED_CMD);//--set display clock divide ratio/oscillator frequency
	OLED_WR_Byte(0x80,OLED_CMD);//--set divide ratio, Set Clock as 100 Frames/Sec
	OLED_WR_Byte(0xD9,OLED_CMD);//--set pre-charge period
	OLED_WR_Byte(0xF1,OLED_CMD);//Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
	OLED_WR_Byte(0xDA,OLED_CMD);//--set com pins hardware configuration
	OLED_WR_Byte(0x12,OLED_CMD);
	OLED_WR_Byte(0xDB,OLED_CMD);//--set vcomh
	OLED_WR_Byte(0x40,OLED_CMD);//Set VCOM Deselect Level
	OLED_WR_Byte(0x20,OLED_CMD);//-Set Page Addressing Mode (0x00/0x01/0x02)
	OLED_WR_Byte(0x02,OLED_CMD);//
	OLED_WR_Byte(0x8D,OLED_CMD);//--set Charge Pump enable/disable
	OLED_WR_Byte(0x14,OLED_CMD);//--set(0x10) disable
	OLED_WR_Byte(0xA4,OLED_CMD);// Disable Entire Display On (0xa4/0xa5)
	OLED_WR_Byte(0xA6,OLED_CMD);// Disable Inverse Display On (0xa6/a7) 
	OLED_WR_Byte(0xAF,OLED_CMD);
	OLED_Clear();
    g_oledInitActive = 0U;
}
