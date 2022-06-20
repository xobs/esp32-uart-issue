#include <stdint.h>
#include <string.h>

#include "esp_log.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "hal/uart_hal.h"
#include "soc/uart_reg.h"
#include "soc/uart_periph.h"

#include "config.h"

#define DEFAULT_UART_MASK UART_RXFIFO_FULL_INT_ENA_M | UART_FRM_ERR_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M
static int uart_mode = 0;
static const uint32_t bulk_mode_size = 126;

#if CONFIG_TARGET_UART_IDX == 0
#define TARGET_UART_DEV UART0
#define PERIPH_UART_MODULE PERIPH_UART0_MODULE
#define PERIPH_UART_IRQ ETS_UART0_INTR_SOURCE
#elif CONFIG_TARGET_UART_IDX == 1
#define TARGET_UART_DEV UART1
#define PERIPH_UART_MODULE PERIPH_UART1_MODULE
#define PERIPH_UART_IRQ ETS_UART1_INTR_SOURCE
#elif CONFIG_TARGET_UART_IDX == 2
#define TARGET_UART_DEV UART2
#define PERIPH_UART_MODULE PERIPH_UART2_MODULE
#define PERIPH_UART_IRQ ETS_UART2_INTR_SOURCE
#else
#error "No target UART defined"
#endif

static uart_isr_handle_t console_isr_handle;

// UART statistics counters
uint32_t uart_overrun_cnt;
uint32_t uart_frame_error_cnt;
uint32_t uart_queue_full_cnt;
uint32_t uart_rx_count;
uint32_t uart_tx_count;
uint32_t uart_irq_count;

static void IRAM_ATTR uart_byte_mode(void)
{
	if (uart_mode == 1)
	{
		return;
	}
	uart_mode = 1;
#ifdef CONFIG_LED_GPIO
	gpio_set_level(CONFIG_LED_GPIO, 1);
#endif
	uart_ll_set_rx_tout(&TARGET_UART_DEV, 20);
	uart_ll_set_rxfifo_full_thr(&TARGET_UART_DEV, 2);
	uart_ll_ena_intr_mask(&TARGET_UART_DEV, DEFAULT_UART_MASK);
	uart_ll_clr_intsts_mask(&TARGET_UART_DEV, UART_INTR_RXFIFO_TOUT);
}

static void IRAM_ATTR uart_bulk_mode(void)
{
	if (uart_mode == 2)
	{
		return;
	}
	uart_mode = 2;
#ifdef CONFIG_LED_GPIO
	gpio_set_level(CONFIG_LED_GPIO, 0);
#endif
	// Symbol length is:
	//      - 1 start bit
	//		- 8 data bits
	//		- 1 stop bit
	const uint32_t symbol_length = 10;

	// Switch back to byte mode if we haven't received a full buffer
	uart_ll_set_rx_tout(&TARGET_UART_DEV, symbol_length * (bulk_mode_size + 1));

	// Fire an interrupt if we get 16 bytes
	uart_ll_set_rxfifo_full_thr(&TARGET_UART_DEV, bulk_mode_size);

	// Enable the TOUT flag
	uart_ll_ena_intr_mask(&TARGET_UART_DEV, DEFAULT_UART_MASK | UART_RXFIFO_TOUT_INT_ENA);
	uart_ll_clr_intsts_mask(&TARGET_UART_DEV, UART_INTR_RXFIFO_TOUT);
}

static void IRAM_ATTR console_isr(void *param)
{
	uint32_t uart_intr_status = uart_ll_get_intsts_mask(&TARGET_UART_DEV);
	int bytes_read = 0;

	uart_irq_count += 1;

	if ((uart_intr_status & UART_INTR_RXFIFO_TOUT) || (uart_intr_status & UART_INTR_RXFIFO_FULL) ||
		(uart_intr_status & UART_INTR_CMD_CHAR_DET))
	{
		while (TARGET_UART_DEV.mem_rx_status.wr_addr != TARGET_UART_DEV.mem_rx_status.rd_addr)
		{
			bytes_read += 1;
			char c = (*((volatile uint32_t *)UART_FIFO_REG(CONFIG_TARGET_UART_IDX)));
			(void)c;
		}
		uart_rx_count += bytes_read;

		if (uart_intr_status & UART_INTR_RXFIFO_TOUT)
		{
			uart_byte_mode();
		}
		else if (bytes_read >= 2)
		{
			uart_bulk_mode();
		}
	}

	if (uart_intr_status & UART_INTR_RXFIFO_OVF)
	{
		uart_ll_rxfifo_rst(&TARGET_UART_DEV);
		uart_overrun_cnt += 1;
	}

	if (uart_intr_status & UART_INTR_FRAM_ERR)
	{
		uart_frame_error_cnt += 1;
		uart_ll_rxfifo_rst(&TARGET_UART_DEV);
	}

	uart_ll_clr_intsts_mask(&TARGET_UART_DEV, uart_intr_status);
}

static void uart_hw_init(void)
{
	uart_config_t uart_config;
	memset(&uart_config, 0, sizeof(uart_config));
	uart_config.baud_rate = CONFIG_UART_BAUD;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

	ESP_ERROR_CHECK(uart_param_config(CONFIG_TARGET_UART_IDX, &uart_config));
	ESP_ERROR_CHECK(uart_set_pin(CONFIG_TARGET_UART_IDX, CONFIG_UART_TX_GPIO, CONFIG_UART_RX_GPIO,
								 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

	// The ESP32 ISR doesn't handle continuous streams of large amounts of data.
	// Remove their ISR and use our own.
	uart_ll_disable_intr_mask(&TARGET_UART_DEV, UART_LL_INTR_MASK);
	uart_ll_clr_intsts_mask(&TARGET_UART_DEV, UART_LL_INTR_MASK);
	ESP_ERROR_CHECK(esp_intr_alloc(PERIPH_UART_IRQ, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, console_isr,
								   NULL, &console_isr_handle));
	uart_byte_mode();
}

static void IRAM_ATTR uart_rx_task(void *parameters)
{
	(void)parameters;

	uart_hw_init();

	while (1)
	{
		vTaskDelay(portMAX_DELAY);

		// Ordinarily, this would contain code to unload a queue that was filled in by the
		// UART top-half.
	}
}

void uart_init(void)
{
#if !defined(CONFIG_TARGET_UART_NONE)
	ESP_LOGI(__func__, "configuring UART%d for target", CONFIG_TARGET_UART_IDX);

	// Start UART tasks
	xTaskCreatePinnedToCore(uart_rx_task, "uart_rx_task", 4096, NULL, 1, NULL, 1);
#endif
}