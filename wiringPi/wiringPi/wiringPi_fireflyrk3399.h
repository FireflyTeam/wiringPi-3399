#ifndef __WIRWIRINGPI_FIREFLYRK3399_H__
#define __WIRWIRINGPI_FIREFLYRK3399_H__

#define RK3399_PULL_GRF_OFFSET		(0xe040 / 4)
#define RK3399_PULL_PMU_OFFSET		(0x40 / 4)
#define RK3399_DRV_3BITS_PER_PIN	3

#define RK3399_PULL_BITS_PER_PIN	2
#define RK3399_PULL_PINS_PER_REG	8
#define RK3399_PULL_BANK_STRIDE		16

#define RK_FUNC_GPIO	0
#define RK_FUNC_1	1
#define RK_FUNC_2	2
#define RK_FUNC_3	3
#define RK_FUNC_4	4

enum rk_gpio_function {
    RK_GPIO_GPIO,
    RK_GPIO_I2C,
    RK_GPIO_SPI,
    RK_GPIO_UART,
    RK_GPIO_PWM
};

/**
 * enum type index corresponding to rockchip_perpin_drv_list arrays index.
 */
enum rockchip_pin_drv_type {
	DRV_TYPE_IO_DEFAULT = 0,
	DRV_TYPE_IO_1V8_OR_3V0,
	DRV_TYPE_IO_1V8_ONLY,
	DRV_TYPE_IO_1V8_3V0_AUTO,
	DRV_TYPE_IO_3V3_ONLY,
	DRV_TYPE_IO_WIDE_LEVEL,
	DRV_TYPE_IO_NARROW_LEVEL,
	DRV_TYPE_MAX
};

/**
 * enum type index corresponding to rockchip_pull_list arrays index.
 */
enum rockchip_pin_pull_type {
	PULL_TYPE_IO_DEFAULT = 0,
	PULL_TYPE_IO_1V8_ONLY,
	PULL_TYPE_MAX
};

/* GPIO control registers */
/*
#define GPIO_SWPORT_DR		0x00
#define GPIO_SWPORT_DDR		0x04
#define GPIO_INTEN		0x30
#define GPIO_INTMASK		0x34
#define GPIO_INTTYPE_LEVEL	0x38
#define GPIO_INT_POLARITY	0x3c
#define GPIO_INT_STATUS		0x40
#define GPIO_INT_RAWSTATUS	0x44
#define GPIO_DEBOUNCE		0x48
#define GPIO_PORTS_EOI		0x4c
#define GPIO_EXT_PORT		0x50
#define GPIO_LS_SYNC		0x60
*/


#define GPIO_SWPORT_DR		0x00
#define GPIO_SWPORT_DDR		(0x04 / 4)
#define GPIO_INTEN		(0x30 / 4)
#define GPIO_INTMASK		(0x34 / 4)
#define GPIO_INTTYPE_LEVEL	(0x38 / 4)
#define GPIO_INT_POLARITY	(0x3c / 4)
#define GPIO_INT_STATUS		(0x40 / 4)
#define GPIO_INT_RAWSTATUS	(0x44 / 4)
#define GPIO_DEBOUNCE		(0x48 / 4)
#define GPIO_PORTS_EOI		(0x4c / 4)
#define GPIO_EXT_PORT		(0x50 / 4)
#define GPIO_LS_SYNC		(0x60 / 4)



enum pin_config_param {
	PIN_CONFIG_BIAS_BUS_HOLD,
	PIN_CONFIG_BIAS_DISABLE,
	PIN_CONFIG_BIAS_HIGH_IMPEDANCE,
	PIN_CONFIG_BIAS_PULL_DOWN,
	PIN_CONFIG_BIAS_PULL_PIN_DEFAULT,
	PIN_CONFIG_BIAS_PULL_UP,
	PIN_CONFIG_DRIVE_OPEN_DRAIN,
	PIN_CONFIG_DRIVE_OPEN_SOURCE,
	PIN_CONFIG_DRIVE_PUSH_PULL,
	PIN_CONFIG_DRIVE_STRENGTH,
	PIN_CONFIG_INPUT_DEBOUNCE,
	PIN_CONFIG_INPUT_ENABLE,
	PIN_CONFIG_INPUT_SCHMITT,
	PIN_CONFIG_INPUT_SCHMITT_ENABLE,
	PIN_CONFIG_LOW_POWER_MODE,
	PIN_CONFIG_OUTPUT,
	PIN_CONFIG_POWER_SOURCE,
	PIN_CONFIG_SLEW_RATE,
	PIN_CONFIG_END = 0x7FFF,
};

#define BIT(nr)         (1UL << (nr))
#define ARRAY_SIZE(_A) (sizeof(_A) / sizeof((_A)[0]))


extern int pinToGpioFireflyRK3399 [64];
extern int physToGpioFireflyRK3399[64];

extern int rk3399_initRegMap();
extern int rk3399_gpio_set_direction(int rk3399gpio, int input);
extern int rk3399_gpio_enable_function(int functionmode);
extern int rk3399_gpio_set_gpiomode(int gpio);
extern void rk3399_pullUpDnControl (int pin, int pud);
extern int rk3399_digitalRead (int pin);
extern void rk3399_digitalWrite (int pin, int value);
extern void rk3399_export_gpio(int exportFd, int gpio);
extern void rk3399_export_pwm();
extern void rk3399_set_pwm_enable(int enable);
extern void rk3399_set_pwm_duty(uint32_t duty_ns);
extern void rk3399_set_pwm_period(uint32_t period_ns);
extern void rk3399_set_pwm_duty_balance();

#ifdef __cplusplus
}
#endif

#endif
