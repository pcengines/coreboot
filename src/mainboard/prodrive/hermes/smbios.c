/* SPDX-License-Identifier: GPL-2.0-only */

#include <smbios.h>
#include <types.h>

#include "variants/baseboard/include/eeprom.h"

const char *smbios_system_serial_number(void)
{
	const size_t offset = offsetof(struct eeprom_layout, system_serial_number);
	static char serial_no[HERMES_SERIAL_NUMBER_LENGTH] = { 0 };

	if (eeprom_read_buffer(serial_no, offset, sizeof(serial_no)) == 0)
		return serial_no;
	else
		return CONFIG_MAINBOARD_SERIAL_NUMBER;
}

const char *smbios_mainboard_serial_number(void)
{
	const size_t offset = offsetof(struct eeprom_layout, board_serial_number);
	static char serial_no[HERMES_SERIAL_NUMBER_LENGTH] = { 0 };

	if (eeprom_read_buffer(serial_no, offset, sizeof(serial_no)) == 0)
		return serial_no;
	else
		return CONFIG_MAINBOARD_SERIAL_NUMBER;
}
