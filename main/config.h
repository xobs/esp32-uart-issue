#ifndef __CONFIG_H__
#define __CONFIG_H__

void uart_init(void);
void wifi_manager_start(void);

#define CONFIG_BLACKMAGIC_HOSTNAME "uart"

#define CONFIG_UART_TX_GPIO 18
#define CONFIG_UART_RX_GPIO 22
#define CONFIG_LED_GPIO 13

#define CONFIG_ESP_WIFI_SSID "Omicron Persei 8"
#define CONFIG_ESP_WIFI_PASSWORD "987654321"
#define CONFIG_ESP_WIFI_SSID2 ""
#define CONFIG_ESP_WIFI_PASSWORD2 ""
#define CONFIG_ESP_WIFI_SSID3 ""
#define CONFIG_ESP_WIFI_PASSWORD3 ""
#define CONFIG_ESP_WIFI_SSID4 ""
#define CONFIG_ESP_WIFI_PASSWORD4 ""

#define CONFIG_UART_BAUD 460800

#define CONFIG_TARGET_UART_IDX 1

#endif /* __CONFIG_H__ */