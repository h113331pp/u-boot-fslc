#include <common.h>
#include <command.h>
#include <malloc.h>

#include <t66/mx6_util.h>

static int do_gpio ( cmd_tbl_t *cmdtp, int flag, int argc, char *argv[])
{
	char *mode;
	mode = argv[1];
	unsigned int bus;
	unsigned int port;
	int value;
	int result;

	if (argc < 2) {
		cmd_usage(cmdtp);
		return 0;
	}

	if (strcmp(mode, "probe") == 0) {
		if (argc == 5) {
		bus = simple_strtoul(argv[3], NULL, 10);
		port = simple_strtoul(argv[4], NULL, 10);
			if (strcmp(argv[2], "direction") == 0) {
				result = t66_gpio_read_direction(bus, port);
				if (result)
					printf("output\n");
				else
					printf("input\n");

				return 0;
			}
			else if (strcmp(argv[2], "value") == 0) {
				result = t66_gpio_read_value(bus, port);
				if (result)
					printf("high\n");
				else
					printf("low\n");

				return 0;
			}
		}

	}

	if (strcmp(mode, "set") == 0) {
		if (argc == 6) {
		bus = simple_strtoul(argv[3], NULL, 10);
		port = simple_strtoul(argv[4], NULL, 10);
		value = simple_strtoul(argv[5], NULL, 10);
			if (strcmp(argv[2], "direction") == 0) {
				t66_gpio_write_direction(bus, port, value);

				return 0;
			}
			else if (strcmp(argv[2], "value") == 0) {
				t66_gpio_write_value(bus, port, value);

				return 0;
			}
		}
	}

	if (strcmp(mode, "softoff") == 0) {
		t66_power_off();
		return 0;
	}

	if (strcmp(mode, "led") == 0) {
		if (argc == 3) {
			value = simple_strtoul(argv[2], NULL, 10);
			t66_power_led(value);
			return 0;
		}
	}

	if (strcmp(mode, "project") == 0) {
		int project = t66_get_project_version();

		switch (project) {
			case 0:
				printf("t66\n");
				break;
			case 2:
				printf("a66\n");
				break;
			default:
				printf("Undetermined project for %d\n", project);
		}
		return 0;
	}

	if (strcmp(mode, "pcb") == 0) {
		int ver = t66_get_pcb_version();

		switch (ver) {
			case 0:
				printf("EVT\n");
				break;
			case 1:
				printf("DVT\n");
				break;
			case 2:
				printf("PVT\n");
				break;
			default:
				printf("Undetermined board version\n");
		}
		return 0;
	}

	if (strcmp(mode, "setdisp0") == 0) {
		t66_set_disp0_src();
		printf("setting u-boot env disp0_src done\n");
		return 0;
	}

	cmd_usage(cmdtp);
	return 0;
}

static int do_checkatb(void)
{
	/* 2 sec passed since power button pressed */
	int c;
	char *current_value = NULL;
	int bootmode = 0;

	current_value = getenv("userbutton");
	if (strncmp(current_value, "1", 1) != 0 )
	{
		if (pwr_pressed() == 1)
		{
			for (c=0; c<1800; c++)
				udelay(1000);
			/* 4 sec passed since power button pressed */
			if (pwr_pressed() == 1 )
			{
				/* process userbutton */
				bootmode = 1;
			}
		}

		setenv("userbutton", simple_itoa(bootmode) );
	}
	
	return 0;
}


U_BOOT_CMD(
	gpio,	CONFIG_SYS_MAXARGS,	1,	do_gpio,
	"GPIO test interface",
	"[option] \n"
	"    - probe direction/value bus port: probe specified direction/value \n"
	"    - set direction/value bus port 1/0: set specified direction/value to 1/0\n"
	"    - sofoff: turn off t66\n"
	"    - led 1/0: setup power led, 1-> blue led; 0-> orange led\n"
	"    - project: show project edition\n"
	"    - pcb: show pcb version\n"
	"    - setdisp0: auto set u-boot env disp0_src"
);

U_BOOT_CMD(
	checkatb,	CONFIG_SYS_MAXARGS,	1,	do_checkatb,
	"detect ATB mode",
	"detect powerbutton press for 4 sec to toggle user flag\n"
);
