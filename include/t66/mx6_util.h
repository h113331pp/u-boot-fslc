#include <common.h>
#include <asm/arch/imx-regs.h>
#include <asm/gpio.h>
#include <asm/io.h>

/* gpio and gpio based interrupt handling */
#define GPIO_DR                 0x00
#define GPIO_GDIR               0x04

#define mdelay(t)	({unsigned long msec=(t); while (msec--) { udelay(1000);}})

/* inform CPLD the operation state */
typedef enum {
	SHUTDOWN = 3,	/* 3 rising edge */
	PHY_ON,			/* 4 rising edge */
	PHY_OFF,		/* 5 rising edge */
	LAST_STATE,		/* 6 rising edge */
} last_state;

typedef enum {
	ATRUST_T66,
	ATRUST_T67,
} tc_model;

typedef enum {
	EVT,
	DVT,
	PVT,
} pcb_version;

#define POWER_BTN 			IMX_GPIO_NR(2, 23)

#define PWR_2V5 			IMX_GPIO_NR(3, 21)
#define LED_ORANGE_EVT		IMX_GPIO_NR(1, 1)
#define LED_ORANGE 			IMX_GPIO_NR(3, 22)
#define LED_BLUE 			IMX_GPIO_NR(7, 13)

#define OC_USB_HUB2_PORT1	IMX_GPIO_NR(3, 25)
#define OC_USB_HUB2_PORT2	IMX_GPIO_NR(2, 25)
#define OC_USB_HUB1_PORT1	IMX_GPIO_NR(3, 24)
#define OC_USB_HUB1_PORT2	IMX_GPIO_NR(3, 23)
#define OC_USB_WIFI			IMX_GPIO_NR(4, 6)
#define USB_HUB1_RESET		IMX_GPIO_NR(5, 2)
#define USB_HUB2_RESET		IMX_GPIO_NR(2, 24)

#define USB_PORT1_POWER		IMX_GPIO_NR(6, 9)
#define USB_PORT2_POWER		IMX_GPIO_NR(6, 14)
#define USB_PORT3_POWER		IMX_GPIO_NR(1, 5)
#define USB_PORT4_POWER		IMX_GPIO_NR(1, 2)
#define USB_WIFI_POWER		IMX_GPIO_NR(1, 7)

#define PCIE_RESET			IMX_GPIO_NR(4, 5)
#define ETH_PHY_RESET		IMX_GPIO_NR(1, 25)
#define EMMC_RESET			IMX_GPIO_NR(6, 11)
#define EMMC_WRITE_PROTECT	IMX_GPIO_NR(3, 26)
#define BUTTON_DETECT		IMX_GPIO_NR(7, 12)
#define AUDIO_POWER			IMX_GPIO_NR(4, 10)

#define LASTSTATE			IMX_GPIO_NR(1, 8)
#define SOFT_OFF			IMX_GPIO_NR(4, 14)
#define CPLD_ALIVE_DETECT	IMX_GPIO_NR(1, 27)

#define PCB_DET1			IMX_GPIO_NR(2, 4)
#define PCB_DET0			IMX_GPIO_NR(2, 5)

#define PROJECT_DET0		IMX_GPIO_NR(2, 3)
#define PROJECT_DET1		IMX_GPIO_NR(2, 2)
#define PROJECT_DET2		IMX_GPIO_NR(2, 1)

#define AUTO_SDN_EVT		IMX_GPIO_NR(3, 31)
#define AUTO_SDN			IMX_GPIO_NR(4, 15)

#define CPLD_RSESERVED2		IMX_GPIO_NR(1, 30)

#define VGA_HOTPLUG			IMX_GPIO_NR(4, 7)
#define HDMI_INTERRUPT		IMX_GPIO_NR(4, 8)
#define HEADPHONE_DET		IMX_GPIO_NR(4, 9)
#define MICROPHONE_DET		IMX_GPIO_NR(1, 9)

/* define t66 gpio direction */
#define GPIO_OUTPUT 1
#define GPIO_INPUT 0

/* define t66 gpio value */
#define GPIO_HIGH 1
#define GPIO_LOW 0

/*
 * t66_gpio_read_direction
 *
 * input:
 *   bus, group of GPIO
 *   port, pin of GPIO
 *
 * for example bus "2" means GPIO'2'-3, port '3' mean GPIO0-'3'
 *
 *  output:
 *     1 - GPIO's direction is output
 *     0 - GPIO's direction is input
 */
unsigned int t66_gpio_read_direction(int bus, int port);

/*
 * t66_gpio_read_value
 *
 * input:
 *   bus, group of GPIO
 *   port, pin of GPIO
 *
 * for example bus "2" means GPIO'2'-3, port '3' mean GPIO0-'3'
 *
 *  output:
 *     1 - GPIO's value is high
 *     0 - GPIO's value is low
 */
unsigned int t66_gpio_read_value(int bus, int port);

/*
 * t66_gpio_write_direction
 *
 * input:
 *   bus, group of GPIO
 *   port, pin of GPIO
 *   high, 1 -> set direction output; 0 -> set direction input
 *
 */
void t66_gpio_write_direction(int bus, int port, int high);

/*
 * t66_gpio_write_value
 *
 * input:
 *   bus, group of GPIO
 *   port, pin of GPIO
 *   high, 1 -> set value high; 0 ->set value low
 *
 */
void t66_gpio_write_value(int bus, int port, int high);

/*
 * t66_power_led: setup power led
 *
 * input:
 *   level, 1 -> blue led ON, orange led OFF
 *          0 -> blue led OFF, orange led ON
 */
void t66_power_led(int level);

/*
 * t66_softoff: turn off t66
 */
void t66_power_off(void);

/*
 * pwr_pressed: detect if power button is pressed
 */
int pwr_pressed(void);

/*
 * toggle last state
 */
void t66_toggle_last_state(int num_pulse);

/*
 * set softoff pin to low|high
 *
 * input:
 *   level, 1 -> set value high; 0 ->set value low
 */
void t66_set_softoff_pin(int level);

/*
 * pull reset pin of audio codec
 */
void t66_reset_audio(void);

/*
 * pull reset pin of lan chip
 */
void t66_reset_ethernet(void);

/*
 * pull reset pin of PCIe
 */
void t66_reset_pcie(void);

/*
 * using GPIO_1_27 tell CPLD t66 still alive(by pull it low),
 * or CPLD will shutdownt 66 then power-on it again
 */
void t66_cpld_alive_response(void);

/*
 * pull reset pin of lan chip
 */
void t66_reset_emmc(void);

int t66_get_project_version(void);
int t66_get_pcb_version(void);
void t66_set_disp0_src(void);
void t66_test(void);
void t66_enable_usb_hub(int enable);
