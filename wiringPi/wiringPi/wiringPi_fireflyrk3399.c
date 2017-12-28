#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>
#include <poll.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <time.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>

#include "wiringPi.h"
#include "wiringPi_fireflyrk3399.h"
#include "../version.h"

#define  HARDWARE_RASPBERRYPI    0
#define  HARDWARE_FIREFLY_RK3399 1

// Revision FireflyRK3399:
int pinToGpioFireflyRK3399 [64] =
{
    66, 156, 67, 68, 149, 51, 155, 69,	// From the Original Wiki - GPIO 0 through 7:	wpi  0 -  7
    71, 72,				// I2C  - SDA1, SCL1				wpi  8 -  9
    42, 45,				// SPI  - CE0, (CE0)    wpi 10 - 11
    40, 39, 41, 				// SPI  - MOSI, MISO, SCLK			wpi 12 - 14
    148, 147,				// UART - Tx, Rx				wpi 15 - 16
    -1, -1, -1, -1,			// Rev 2: New GPIOs 8 though 11			wpi 17 - 20
    123, 5, 8, 125, 127,			// B+						wpi 21, 22, 23, 24, 25
    54, 55, 157, 152,			// B+						wpi 26, 27, 28, 29
    150, 57,				// B+						wpi 30, 31
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 47
    -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,	// ... 63
} ;


// physToGpio:
//	Take a physical pin (1 through 26) and re-map it to the BCM_GPIO pin
//	Cope for 2 different board revisions here.
//	Also add in the P5 connector, so the P5 pins are 3,4,5,6, so 53,54,55,56

int physToGpioFireflyRK3399[64] =
{
    -1,		// 0
    -1, -1,	// 1, 2
    71, -1,
    72, -1,
    69, 148,
    -1, 147,
    66, 156,
    67, -1,
    68, 149,
    -1, 51,
    40, -1,
    39, 155,
    41, 42,
    -1, 45,	// 25, 26

    // B+
    150,57,
    123, -1,
    5, 54,
    8, -1,
    125, 55,
    127, 157,
    -1, 152,

    // the P5 connector on the Rev 2 boards:

    -1, -1,
    -1, -1,
    -1, -1,
    -1, -1,
    -1, -1,
    -1, -1,
    -1, -1,
    -1, -1,
    -1, -1,
    -1, -1,
    -1, -1,
} ;


typedef struct _RK3399FunctionConfig {
    int muxFunction;
    int *pins;
    int *pinsFunction;
}RK3399FUNCTIONCONFIG;

RK3399FUNCTIONCONFIG gpioFunctionConfig_RK3399[] = {
    {
        .muxFunction = RK_GPIO_GPIO,
        .pins = (int []){0},
        .pinsFunction = (int []){0}
    },
    {
        .muxFunction = RK_GPIO_I2C,
        .pins = (int []){71,72},
        .pinsFunction = (int []){RK_FUNC_2, RK_FUNC_2}
    },
    {
        .muxFunction = RK_GPIO_SPI,
        .pins = (int []){39,40,41,42},
        .pinsFunction = (int []){RK_FUNC_2, RK_FUNC_2, RK_FUNC_2, RK_FUNC_2}
    },
    {
        .muxFunction = RK_GPIO_UART,
        .pins = (int []){147,148},
        .pinsFunction = (int []){RK_FUNC_1, RK_FUNC_1}
    },
    {
        .muxFunction = RK_GPIO_PWM,
        .pins = (int []){150},
        .pinsFunction = (int []){RK_FUNC_1}
    },
};

static const unsigned int GRFBASE_RK3399 = 0xff770000;   // 
static const unsigned int PMUBASE_RK3399 = 0xff320000;   // bank 0 1
static const unsigned int CLKBASE_RK3399 = 0xff760000;
static const unsigned int GRFMUXOFFSET_RK3399 = 0xe000;   // 
static const unsigned int PMUMUXOFFSET_RK3399 = 0x00;   // bank 0 1
static const unsigned int GRFDRVOFFSET_RK3399 = 0xe100;   // 
static const unsigned int PMUDRVOFFSET_RK3399 = 0x80;   // bank 0 1
static const unsigned int CLKENOFFSET_RK3399  = 0x037c/4;

#define    IOMUX_WIDTH_2BIT     0
#define    IOMUX_WIDTH_4BIT     1

static uint32_t* rk3399_grfRegBase;
static uint32_t* rk3399_pmugrfRegBase;
static uint32_t* rk3399_clkcruRegBase;

static uint32_t* rk3399_gpioRegMap[5];
static const uint32_t rk3399_gpioRegBase[] = 
{
    0xff720000,
    0xff730000,
    0xff780000,
    0xff788000,
    0xff790000,
};

int rk3399_initRegMap()
{
    int  i, fd ;

    if ((fd = open ("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC) ) < 0)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: get devmem failed: %s\n", strerror (errno)) ;

    rk3399_grfRegBase = (uint32_t*)mmap(0, 0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, GRFBASE_RK3399);
    if (rk3399_grfRegBase == MAP_FAILED)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (rk3399Grf) failed: %s\n", strerror (errno)) ;

    rk3399_pmugrfRegBase = (uint32_t*)mmap(0, 0x1000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, PMUBASE_RK3399);
    if (rk3399_pmugrfRegBase == MAP_FAILED)
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (rk3399PMU) failed: %s\n", strerror (errno)) ;

    rk3399_clkcruRegBase = (uint32_t*)mmap(0,0x10000, PROT_READ|PROT_WRITE, MAP_SHARED, fd, CLKBASE_RK3399);
    if (rk3399_clkcruRegBase == MAP_FAILED) 
        return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap (rk3399CLK) failed: %s\n", strerror (errno)) ;

    for(i=0; i<5; i++)
    {
        rk3399_gpioRegMap[i] = (uint32_t*)mmap(0,0x0100, PROT_READ|PROT_WRITE, MAP_SHARED, fd, rk3399_gpioRegBase[i]);
        if(rk3399_gpioRegMap[i] == MAP_FAILED)
           return wiringPiFailure (WPI_ALMOST, "wiringPiSetup: mmap(gpiobase) failed: %s\n", strerror (errno)) ;
    }

    return 0;
}

//===============TODO========================
//check if the level is set correct
//===========================================
int rk3399_gpio_set_direction(int pin, int input)
{
    int bank_num = pin / 32;
    int offset = pin % 32;
    uint32_t *reg = rk3399_gpioRegMap[bank_num];
    int data = *(reg + GPIO_SWPORT_DDR);

    data &= ~BIT(offset);
    if (input)
        data |= BIT(offset);

    *(reg + GPIO_SWPORT_DDR) = data;
}

static int rk3399_checkGpioFunctionSupport(int rk3399gpio, int function)
{
    uint8_t i;
    RK3399FUNCTIONCONFIG* config;

    config = &(gpioFunctionConfig_RK3399[function]);
    for(i=0; i<ARRAY_SIZE(config->pins); i++)
    {
        if (rk3399gpio == config->pins[i])
            return 0;
    }
    return -1;
}

static void rk3399_updateMuxRegBit(int gpio, int function)
{
    uint32_t *regbase , *clkbase;
    uint32_t regOffset, clkOffset;
    uint8_t bankIndex, pinGroup, pinIndex, iomuxWidth, iomType;
    unsigned int bit, data, rmask, mask, orig, tmp;

    bankIndex = gpio / 32;
    pinGroup = (gpio % 32) / 8;
    pinIndex = gpio % 8;

    if ((bankIndex == 0) || (bankIndex == 1)) {
        regbase = rk3399_pmugrfRegBase;
        regOffset = PMUMUXOFFSET_RK3399;
    } else {
        regbase = rk3399_grfRegBase;
        regOffset = GRFMUXOFFSET_RK3399;

	clkbase = rk3399_clkcruRegBase;
	clkOffset = CLKENOFFSET_RK3399;
        *(clkbase + clkOffset) = 0x00380180;
    }

    iomType = IOMUX_WIDTH_2BIT;

    if (iomType  == IOMUX_WIDTH_4BIT ) {
       iomuxWidth = 8;
       mask = 0xf;
    } else  {
       iomuxWidth = 4;
       mask = 0x3;
    }

    if(bankIndex == 2)
       bankIndex = 0;
    else if(bankIndex == 3)
       bankIndex = 1;
    else if(bankIndex == 4) 
       bankIndex = 2;
    regOffset += bankIndex * 4 * iomuxWidth + pinGroup * iomuxWidth;

    bit = pinIndex * 2;
    data = (mask << (bit + 16));
    rmask = data | (data >> 16);
    data |= (function & mask) << bit;

    orig = *(regbase + regOffset);
    tmp = orig & ~rmask;
    tmp |= data & rmask;
    *(regbase + regOffset) = tmp;
}
int rk3399_gpio_enable_function(int functionmode)
{
    uint8_t i;
    RK3399FUNCTIONCONFIG* config;

    config = &(gpioFunctionConfig_RK3399[functionmode]);
    for( i=0; i<ARRAY_SIZE(config->pins); i++) {
        rk3399_updateMuxRegBit(config->pins[i],config->pinsFunction[i]);
    }
    return 0;
}

int rk3399_gpio_set_gpiomode(int gpio)
{
    rk3399_updateMuxRegBit(gpio, RK_FUNC_GPIO);
    return 0;
}

static const int rk3399_bank_pull_type[][4] =
{
    {
	 PULL_TYPE_IO_1V8_ONLY,
         PULL_TYPE_IO_1V8_ONLY,
         PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
    },
    {
	 PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
    },
    {
         PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
	PULL_TYPE_IO_1V8_ONLY,
         PULL_TYPE_IO_1V8_ONLY,
    },
    {
	 PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
    },
    {
	 PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
         PULL_TYPE_IO_DEFAULT,
    },
};

static const int rk3399_pull_list[PULL_TYPE_MAX][4] = {
	{
		PIN_CONFIG_BIAS_DISABLE,
		PIN_CONFIG_BIAS_PULL_UP,
		PIN_CONFIG_BIAS_PULL_DOWN,
		PIN_CONFIG_BIAS_BUS_HOLD
	},
	{
		PIN_CONFIG_BIAS_DISABLE,
		PIN_CONFIG_BIAS_PULL_DOWN,
		PIN_CONFIG_BIAS_DISABLE,
		PIN_CONFIG_BIAS_PULL_UP
	},
};

static void rk3399_calc_pull_reg_and_bit(int bank_num,
					 int pin_num, uint32_t **regmap,
					 int *reg, uint8_t *bit)
{
	/* The bank0:16 and bank1:32 pins are located in PMU */
	if ((bank_num == 0) || (bank_num == 1)) {
		*regmap = rk3399_pmugrfRegBase;
		*reg = RK3399_PULL_PMU_OFFSET;

		*reg += bank_num * RK3399_PULL_BANK_STRIDE;

		*reg += ((pin_num / RK3399_PULL_PINS_PER_REG) * 4);
		*bit = pin_num % RK3399_PULL_PINS_PER_REG;
		*bit *= RK3399_PULL_BITS_PER_PIN;
	} else {
		*regmap = rk3399_grfRegBase;
		*reg = RK3399_PULL_GRF_OFFSET;

		/* correct the offset, as we're starting with the 3rd bank */
		*reg -= 0x20;
		*reg += bank_num * RK3399_PULL_BANK_STRIDE;
		*reg += ((pin_num / RK3399_PULL_PINS_PER_REG) * 4);

		*bit = (pin_num % RK3399_PULL_PINS_PER_REG);
		*bit *= RK3399_PULL_BITS_PER_PIN;
	}
}

#if 0
static enum rockchip_pin_drv_type rk3399_calc_drv_reg_and_bit(
				       int bank_num,
				       int pin_num, uint32_t **regmap,
				       int *reg, uint8_t *bit)
{
	int drv_num = (pin_num / 8);

	/*  The bank0:16 and bank1:32 pins are located in PMU */
	if ((bank_num == 0) || (bank_num == 1))
		*regmap = rk3399_pmugrfRegBase;
	else
		*regmap = rk3399_grfRegBase;

	*reg = bank->drv[drv_num].offset;
	if ((bank->drv[drv_num].drv_type == DRV_TYPE_IO_1V8_3V0_AUTO) ||
	    (bank->drv[drv_num].drv_type == DRV_TYPE_IO_3V3_ONLY))
		*bit = (pin_num % 8) * 3;
	else
		*bit = (pin_num % 8) * 2;

	return DRV_TYPE_IO_DEFAULT;
}
#endif

/*
 * pullUpDownCtrl:
 *	Control the internal pull-up/down resistors on a GPIO pin
 *	The Arduino only has pull-ups and these are enabled by writing 1
 *	to a port when in input mode - this paradigm doesn't quite apply
 *	here though.
 *********************************************************************************
 */

void rk3399_pullUpDnControl (int pin, int pud)
{
    unsigned int i, bank_num, pin_num;
    int ret, pull_type;
    int orig, tmp, data, rmask, reg;
    uint8_t bit;
    uint32_t *regmap;

    bank_num = pin / 32;
    pin_num = pin % 32;
    pull_type = rk3399_bank_pull_type[bank_num][pin_num / 8];
    rk3399_calc_pull_reg_and_bit(bank_num, pin_num, &regmap, &reg, &bit);
    ret = -1;
    for (i = 0; i < ARRAY_SIZE(rk3399_pull_list[pull_type]);
        i++) {
        if (rk3399_pull_list[pull_type][i] == pud) {
            ret = i;
            break;
        }
    }

    if(ret < 0)
        return;

    data = ((1 << RK3399_PULL_BITS_PER_PIN) - 1) << (bit + 16);
    rmask = data | (data >> 16);
    data |= (ret << bit);

    orig = *(regmap + reg);
    tmp = orig & ~rmask;
    tmp |= data & rmask;
    *(regmap + reg) = tmp;
    return;
}

/*
 * digitalRead:
 *	Read the value of a given Pin, returning HIGH or LOW
 *********************************************************************************
 */
int rk3399_digitalRead (int pin)
{
    int bank_num = pin / 32;
    int offset = pin % 32;
    uint32_t *reg = rk3399_gpioRegMap[bank_num];
    int data = *(reg + GPIO_EXT_PORT);
   
    data >>=pin;
    data &=1; 
    if (data != 0)
        return HIGH ;
    else
        return LOW ;
}

/*
 * digitalWrite:
 *	Set an output bit
 *********************************************************************************
 */
void rk3399_digitalWrite (int pin, int value)
{
    int bank_num = pin / 32;
    int offset = pin % 32;
    uint32_t *reg = rk3399_gpioRegMap[bank_num];
    int data = *(reg + GPIO_SWPORT_DR);
    int data2 = *(reg + GPIO_SWPORT_DDR);

    data &= ~BIT(offset);
    if (value){
        data |= BIT(offset);
    }
    printf("\n");
    *(reg + GPIO_SWPORT_DR) = data;
}

#if 0
//=============TODO==============================
//pwm support need complate
//===============================================
void rk3399_pinMode (int pin, int mode)
{
    switch (mode) 
    {
        case INPUT:
            rk3399_gpio_set_gpiomode(pin);
            rk3399_gpio_set_direction(pin,INPUT);
            break;
        case OUTPUT:
            rk3399_gpio_set_gpiomode(pin);
            rk3399_gpio_set_direction(pin,OUTPUT);
            break;
        case SOFT_PWM_OUTPUT:
            softPwmCreate (origPin, 0, 100) ;
            break;
        case SOFT_TONE_OUTPUT:
            softToneCreate (origPin) ;
            break;
        case PWM_OUTPUT:
            rk3399_gpio_enable_function(RK_GPIO_PWM);
            break;
        case PWM_TONE_OUTPUT:
            rk3399_gpio_enable_function(RK_GPIO_PWM);
            pwmSetMode(PWM_MODE_MS);
            break;
        case GPIO_CLK:
            break;
    }
}
#endif

void rk3399_export_gpio(int exportFd, int gpio)
{
    struct stat st;
    char path[30];
    sprintf(path,"/sys/class/gpio/gpio%d",gpio);

	if(stat(path, &st) < 0)
    {
        char numStr[5];
        sprintf(numStr,"%d\n",gpio);
        lseek (exportFd, 0L, SEEK_SET);
        write(exportFd, numStr, strlen(numStr));
    }
}

//export pwm1 device
void rk3399_export_pwm()
{
	struct stat st;
	if(stat("/sys/class/pwm/pwmchip1/pwm0", &st) < 0)
    {
        int sysPwmFd = -1;
        sysPwmFd = open("/sys/class/pwm/pwmchip1/export", O_RDWR);
        if (sysPwmFd != -1)
        {
            lseek (sysPwmFd, 0L, SEEK_SET);
            write(sysPwmFd, "0\n", 2);
            delayMicroseconds(200);
            close(sysPwmFd);
        }
    }
}

void rk3399_set_pwm_enable(int enable)
{
    int sysPwmFd = -1;

    rk3399_export_pwm();

    sysPwmFd = open("/sys/class/pwm/pwmchip1/pwm0/enable", O_RDWR);
    if (sysPwmFd != -1)
    {
        lseek (sysPwmFd, 0L, SEEK_SET);
        if (enable)
            write(sysPwmFd, "1\n", 2);
        else
            write(sysPwmFd, "0\n", 2);
        close(sysPwmFd);
    }
}

void rk3399_set_pwm_duty(uint32_t duty_ns)
{
    int sysPwmFd = -1;
    char dutyStr[20];

    rk3399_export_pwm();

    sysPwmFd = open("/sys/class/pwm/pwmchip1/pwm0/duty_cycle", O_RDWR);
    if (sysPwmFd != -1)
    {
        lseek (sysPwmFd, 0L, SEEK_SET);
        sprintf(dutyStr, "%d\n", duty_ns);
        write(sysPwmFd, dutyStr, strlen(dutyStr));
        close(sysPwmFd);
    }
}

void rk3399_set_pwm_period(uint32_t period_ns)
{
    int sysPwmFd = -1;
    char periodStr[20];

    rk3399_export_pwm();

    sysPwmFd = open("/sys/class/pwm/pwmchip1/pwm0/period", O_RDWR);
    if (sysPwmFd != -1)
    {
        lseek (sysPwmFd, 0L, SEEK_SET);
        sprintf(periodStr, "%d\n", period_ns);
        write(sysPwmFd, periodStr, strlen(periodStr));
        close(sysPwmFd);
    }
}

void rk3399_set_pwm_duty_balance()
{
    int sysPwmFd = -1;
    char periodStr[20];

    rk3399_export_pwm();

    sysPwmFd = open("/sys/class/pwm/pwmchip1/pwm0/period", O_RDWR);
    if (sysPwmFd != -1)
    {
        lseek (sysPwmFd, 0L, SEEK_SET);
        read (sysPwmFd, periodStr, 20);
        char* leftover;
        uint32_t period_ns = strtoul(periodStr, &leftover, 10);
        uint32_t duty_ns = period_ns / 2;
        rk3399_set_pwm_duty(duty_ns);
        close(sysPwmFd);
    }
}

