#include <t66/mx6_util.h>

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
 *     1 - GPIO's direction is output(means you can write this pin)
 *     0 - GPIO's direction is input(means you can ONLY read this pin)
 */
unsigned int t66_gpio_read_direction(int bus, int port)
{
	u32 reg;
	u32 mask = 0x00000001;
	u32 gpio_addr;
	gpio_addr = GPIO1_BASE_ADDR + ( 0x4000 * (bus - 1));

	reg = readl(gpio_addr + GPIO_GDIR);
	reg = (reg >> port) & mask;

	return reg;
}

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
unsigned int t66_gpio_read_value(int bus, int port)
{
	u32 reg;
	u32 mask = 0x00000001;
	u32 gpio_addr;
	gpio_addr = GPIO1_BASE_ADDR + ( 0x4000 * (bus - 1));

	reg = readl(gpio_addr + GPIO_DR);
	reg = (reg >> port) & mask;
	return reg;
}

/*
 * t66_gpio_write_direction
 *
 * input:
 *   bus, group of GPIO
 *   port, pin of GPIO
 *   high, 1 -> set direction output; 0 -> set direction input
 *     1 - GPIO's direction is output(means you can write this pin)
 *     0 - GPIO's direction is input(means you can ONLY read this pin)
 *
 */
void t66_gpio_write_direction(int bus, int port, int high)
{
	u32 reg;
	u32 mask = 0x00000001;
	u32 gpio_addr;
	gpio_addr = GPIO1_BASE_ADDR + ( 0x4000 * (bus - 1));

	reg = readl(gpio_addr + GPIO_GDIR);

	if(high)
		reg |= (mask << port);
	else
		reg &= ~(mask << port);
	writel(reg, gpio_addr + GPIO_GDIR);
}

/*
 * t66_gpio_write_value
 *
 * input:
 *   bus, group of GPIO
 *   port, pin of GPIO
 *   high, 1 -> set value high; 0 ->set value low
 *
 */
void t66_gpio_write_value(int bus, int port, int high)
{
	u32 reg;
	u32 mask = 0x00000001;
	u32 gpio_addr;
	gpio_addr = GPIO1_BASE_ADDR + ( 0x4000 * (bus - 1));

	reg = readl(gpio_addr + GPIO_DR);
	if(high)
		reg |= (mask << port);
	else
		reg &= ~(mask << port);
	writel(reg, gpio_addr + GPIO_DR);
}

/*
 * t66_power_led
 *
 * input:
 *   level, 1 -> blue led ON, orange led OFF
 *          0 -> blue led OFF, orange led ON
 */
void t66_power_led(int level)
{
	if(level)
	{
		gpio_direction_output(LED_ORANGE, GPIO_LOW);
		gpio_direction_output(LED_ORANGE_EVT, GPIO_LOW);
		gpio_direction_output(LED_BLUE, GPIO_HIGH);
	}
	else
	{
		gpio_direction_output(LED_BLUE, GPIO_LOW);
		gpio_direction_output(LED_ORANGE, GPIO_HIGH);
		gpio_direction_output(LED_ORANGE_EVT, GPIO_HIGH);
	}
}

/*
 * t66_softoff: turn off t66
 */
void t66_power_off(void)
{
	printf("t66 power off now...\n");
	t66_set_softoff_pin(GPIO_LOW);
	t66_toggle_last_state(SHUTDOWN);
	mdelay(500);
}

void t66_cpld_alive_response(void)
{
	gpio_direction_output(CPLD_ALIVE_DETECT, GPIO_LOW);
}

/* need pull softoff pin to low when communicate with CPLD */
void t66_set_softoff_pin(int level)
{

	gpio_direction_output(SOFT_OFF, GPIO_HIGH);
	mdelay(20);
	if( level == GPIO_LOW )
	{
		gpio_direction_output(SOFT_OFF, GPIO_LOW);
		mdelay(20);
	}
}

/* toggle last state */
void t66_toggle_last_state(int num_pulse)
{
	int i=0;
	int PULSE_TIME = 20;

	gpio_direction_output(LASTSTATE, GPIO_HIGH);

	for (i=0; i <= num_pulse; i++) {
		/* debug message, needed removed later */
		//printf("last state pulse count: %d\n", i);
		mdelay(PULSE_TIME);
		gpio_direction_output(LASTSTATE, GPIO_LOW);
		mdelay(PULSE_TIME);
		gpio_direction_output(LASTSTATE, GPIO_HIGH);
	}
	printf("finish toggle last state\n");
}

void t66_reset_emmc(void)
{
	gpio_direction_output(EMMC_RESET, GPIO_HIGH);
	mdelay(4);
	gpio_direction_output(EMMC_RESET, GPIO_LOW);
	mdelay(4);
	gpio_direction_output(EMMC_RESET, GPIO_HIGH);
}

void t66_reset_audio(void)
{
	gpio_direction_output(AUDIO_POWER, GPIO_HIGH);
	mdelay(4);
	gpio_direction_output(AUDIO_POWER, GPIO_LOW);
	mdelay(4);
	gpio_direction_output(AUDIO_POWER, GPIO_HIGH);
}

void t66_reset_ethernet(void)
{
	gpio_direction_output(ETH_PHY_RESET, GPIO_HIGH);
	mdelay(4);
	gpio_direction_output(ETH_PHY_RESET, GPIO_LOW);
	mdelay(4);
	gpio_direction_output(ETH_PHY_RESET, GPIO_HIGH);
}

void t66_reset_pcie(void)
{
	printf("Reseting PCIe\n");
	gpio_direction_output(PCIE_RESET, GPIO_HIGH);
	mdelay(1);
	gpio_direction_output(PCIE_RESET, GPIO_LOW);
	mdelay(1);
	gpio_direction_output(PCIE_RESET, GPIO_HIGH);
}

void t66_enable_usb_hub(int enable)
{
	if (enable) {
		gpio_direction_output(USB_PORT1_POWER, GPIO_HIGH);
		gpio_direction_output(USB_PORT2_POWER, GPIO_HIGH);
		gpio_direction_output(USB_PORT3_POWER, GPIO_HIGH);
		gpio_direction_output(USB_PORT4_POWER, GPIO_HIGH);
	} else {
		gpio_direction_output(USB_PORT1_POWER, GPIO_LOW);
		gpio_direction_output(USB_PORT2_POWER, GPIO_LOW);
		gpio_direction_output(USB_PORT3_POWER, GPIO_LOW);
		gpio_direction_output(USB_PORT4_POWER, GPIO_LOW);
	}
}


/* get t66 model from GPIO2-4, GPIO2-5 */
int t66_get_pcb_version(void)
{
	gpio_direction_input(PCB_DET0);
	gpio_direction_input(PCB_DET1);

	return gpio_get_value(PCB_DET1) << 1 | gpio_get_value(PCB_DET0);
}

/* get project name from GPIO2-1, GPIO2-2, GPIO2-3 */
int t66_get_project_version(void)
{
	int project = 0;
	gpio_direction_input(PROJECT_DET0);
	gpio_direction_input(PROJECT_DET1);
	gpio_direction_input(PROJECT_DET2);

	project = gpio_get_value(PROJECT_DET2) << 2
				| gpio_get_value(PROJECT_DET1) << 1
				| gpio_get_value(PROJECT_DET0);

	return project;
}

void t66_set_disp0_src(void)
{
	u32 disp_type=0;
	int project = t66_get_project_version();

	/*a66 only have hdmi interface*/
	if (project == 2)
		setenv("disp0_src", "hdmi");
	else
	{
		disp_type = t66_gpio_read_value(3, 21);
		if (disp_type == 1)
			setenv("disp0_src", "hdmi");
		else
			setenv("disp0_src", "vga");
	}
}

int pwr_pressed(void)
{
	gpio_direction_input(BUTTON_DETECT);
	return gpio_get_value(BUTTON_DETECT);
}
