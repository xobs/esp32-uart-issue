/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2011  Black Sphere Technologies Ltd.
 * Written by Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "driver/gpio.h"
#include "esp_ota_ops.h"
#include "nvs_flash.h"

#include "config.h"
#include "ota-tftp.h"

void app_main(void)
{

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

#ifdef CONFIG_LED_GPIO
	gpio_reset_pin(CONFIG_LED_GPIO);
	gpio_set_direction(CONFIG_LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_LED_GPIO, 1);
#endif

	wifi_manager_start();

	uart_init();

	ota_tftp_init_server(69, 4);

	// Wait two seconds for the system to stabilize before confirming the
	// new firmware image works. This gives us time to ensure the new
	// environment works well.
	vTaskDelay(pdMS_TO_TICKS(2000));
	esp_ota_mark_app_valid_cancel_rollback();
}
