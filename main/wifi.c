#include <string.h>

#include "esp_adc_cal.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/uart.h"
#include "soc/uart_reg.h"
#include "driver/adc.h"
#include "driver/gpio.h"

#include "config.h"

// case SYSTEM_EVENT_STA_START:
static void esp_system_event_sta_start(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    esp_wifi_connect();
}
// case SYSTEM_EVENT_STA_CONNECTED:
static void esp_system_event_sta_connected(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_event_sta_connected_t *event = event_data;
    ESP_LOGI("WIFI", "connected:%s", event->ssid);
#ifdef CONFIG_BLACKMAGIC_HOSTNAME
    tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, CONFIG_BLACKMAGIC_HOSTNAME);
    ESP_LOGI("WIFI", "setting hostname:%s", CONFIG_BLACKMAGIC_HOSTNAME);
#endif
}
// case SYSTEM_EVENT_STA_GOT_IP:
static void esp_system_event_sta_got_ip(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ip_event_got_ip_t *ip = event_data;
    ESP_LOGI("WIFI", "Associated IP address: " IPSTR, IP2STR(&ip->ip_info.ip));
    // xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
}

// case SYSTEM_EVENT_STA_DISCONNECTED:
static void esp_system_event_sta_disconnected(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_event_sta_disconnected_t *info = event_data;
    ESP_LOGE("WIFI", "Disconnect reason : %d", info->reason);
    // if (info->disconnected.reason == WIFI_REASON_BASIC_RATE_NOT_SUPPORT) {
    //     /*Switch to 802.11 bgn mode */
    //     //esp_wifi_set_protocol(ESP_IF_WIFI_STA, WIFI_PROTOCAL_11B | WIFI_PROTOCAL_11G | WIFI_PROTOCAL_11N);
    // }
    esp_wifi_connect();
    // xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
}

// case SYSTEM_EVENT_AP_STACONNECTED:
static void esp_system_event_ap_staconnected(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_event_ap_staconnected_t *event = event_data;
    ESP_LOGI("WIFI", "station:" MACSTR " join, AID=%d",
             MAC2STR(event->mac),
             event->aid);
}
// case SYSTEM_EVENT_AP_STADISCONNECTED:
static void esp_system_event_ap_stadisconnected(void *event_handler_arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    wifi_event_ap_stadisconnected_t *event = event_data;
    ESP_LOGI("WIFI", "station:" MACSTR " leave, AID=%d",
             MAC2STR(event->mac),
             event->aid);
}

static esp_err_t wifi_fill_sta_config(wifi_config_t *wifi_config)
{
    ESP_LOGI(__func__, "wifi_fill_sta_config begun.");

    memset(wifi_config, 0, sizeof(*wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(__func__, "wifi_fill_sta_config started wifi.");
    // ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(4*2)); // 2 mA (in units of 0.25 mA)
    ESP_LOGI(__func__, "wifi_fill_sta_config set max tx power.");
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_START, esp_system_event_sta_start, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_CONNECTED, esp_system_event_sta_connected, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, esp_system_event_sta_disconnected, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, esp_system_event_sta_got_ip, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, esp_system_event_ap_staconnected, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, esp_system_event_ap_stadisconnected, NULL, NULL));
    while (1)
    {
        uint16_t i;
        wifi_scan_config_t scan_config = {0};
        ESP_ERROR_CHECK(esp_wifi_scan_start(&scan_config, true));
        uint16_t num_aps = 0;
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&num_aps));
        wifi_ap_record_t scan_results[num_aps];
        ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&num_aps, scan_results));

        for (i = 0; i < num_aps; i++)
        {
            if ((strlen(CONFIG_ESP_WIFI_SSID) > 0) && !strcmp((char *)scan_results[i].ssid, CONFIG_ESP_WIFI_SSID))
            {
                // DEBUG_WARN("Connecting to %s\n", CONFIG_ESP_WIFI_SSID);
                strncpy((char *)wifi_config->sta.ssid, CONFIG_ESP_WIFI_SSID, sizeof(wifi_config->sta.ssid));
                strncpy((char *)wifi_config->sta.password, CONFIG_ESP_WIFI_PASSWORD, sizeof(wifi_config->sta.password));
                ESP_ERROR_CHECK(esp_wifi_stop());
                return ESP_OK;
            }
            if ((strlen(CONFIG_ESP_WIFI_SSID2) > 0) && !strcmp((char *)scan_results[i].ssid, CONFIG_ESP_WIFI_SSID2))
            {
                strncpy((char *)wifi_config->sta.ssid, CONFIG_ESP_WIFI_SSID2, sizeof(wifi_config->sta.ssid));
                strncpy((char *)wifi_config->sta.password, CONFIG_ESP_WIFI_PASSWORD2, sizeof(wifi_config->sta.password));
                // DEBUG_WARN("Connecting to %s (password: %s)\n", wifi_config->sta.ssid, wifi_config->sta.password);
                ESP_ERROR_CHECK(esp_wifi_stop());
                return ESP_OK;
            }
            if ((strlen(CONFIG_ESP_WIFI_SSID3) > 0) && !strcmp((char *)scan_results[i].ssid, CONFIG_ESP_WIFI_SSID3))
            {
                // DEBUG_WARN("Connecting to %s\n", CONFIG_ESP_WIFI_SSID3);
                strncpy((char *)wifi_config->sta.ssid, CONFIG_ESP_WIFI_SSID3, sizeof(wifi_config->sta.ssid));
                strncpy((char *)wifi_config->sta.password, CONFIG_ESP_WIFI_PASSWORD3, sizeof(wifi_config->sta.password));
                ESP_ERROR_CHECK(esp_wifi_stop());
                return ESP_OK;
            }
            if ((strlen(CONFIG_ESP_WIFI_SSID4) > 0) && !strcmp((char *)scan_results[i].ssid, CONFIG_ESP_WIFI_SSID4))
            {
                // DEBUG_WARN("Connecting to %s\n", CONFIG_ESP_WIFI_SSID4);
                strncpy((char *)wifi_config->sta.ssid, CONFIG_ESP_WIFI_SSID4, sizeof(wifi_config->sta.ssid));
                strncpy((char *)wifi_config->sta.password, CONFIG_ESP_WIFI_PASSWORD4, sizeof(wifi_config->sta.password));
                ESP_ERROR_CHECK(esp_wifi_stop());
                return ESP_OK;
            }
        }
    }
}

void wifi_init_sta(void)
{
    uint8_t mac_address[8];
    esp_err_t result;

    ESP_LOGI(__func__, "wifi_init_sta begun");

    esp_netif_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config;

    result = esp_base_mac_addr_get(mac_address);

    if (result == ESP_ERR_INVALID_MAC)
    {
        ESP_LOGE(__func__, "base mac address invalid.  reading from fuse.");
        ESP_ERROR_CHECK(esp_efuse_mac_get_default(mac_address));
        ESP_ERROR_CHECK(esp_base_mac_addr_set(mac_address));
        ESP_LOGE(__func__, "base mac address configured.");
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    wifi_fill_sta_config(&wifi_config);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MAX_MODEM));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_max_tx_power(4 * 2)); // 2 mA (in units of 0.25 mA)

    ESP_LOGI(__func__, "wifi_init_sta finished.");
}

void wifi_manager_start(void)
{
    wifi_init_sta();
}