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

#include <assert.h>
#include <sys/time.h>
#include <sys/unistd.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "dhcpserver/dhcpserver.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"
#include "lwip/api.h"
#include "lwip/tcp.h"

#include "esp_ota_ops.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/uart.h"

#include "ota-tftp.h"

#include "config.h"

#define TAG "bmp-esp32"
nvs_handle h_nvs_conf;

void app_main(void)
{
	gpio_reset_pin(CONFIG_LED_GPIO);
	gpio_set_direction(CONFIG_LED_GPIO, GPIO_MODE_OUTPUT);
	gpio_set_level(CONFIG_LED_GPIO, 1);

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(nvs_open("config", NVS_READWRITE, &h_nvs_conf));

	wifi_manager_start();

	uart_init();

	ota_tftp_init_server(69, 4);

	ESP_LOGI(__func__, "Free heap %d", esp_get_free_heap_size());

	// Wait two seconds for the system to stabilize before confirming the
	// new firmware image works. This gives us time to ensure the new
	// environment works well.
	vTaskDelay(pdMS_TO_TICKS(2000));
	esp_ota_mark_app_valid_cancel_rollback();
}
