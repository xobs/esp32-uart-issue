#include <stdint.h>

#include "esp_attr.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "driver/periph_ctrl.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/semphr.h"
#include "hal/uart_hal.h"
#include "nvs_flash.h"
#include "soc/uart_reg.h"
#include "soc/uart_periph.h"
#include "esp_spi_flash.h"

#include "sdkconfig.h"

#include "config.h"

#include "lwip/sockets.h"

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

#define UART_CONTEX_INIT_DEF(uart_num)                                                                      \
	{                                                                                                       \
		.hal.dev = UART_LL_GET_HW(uart_num), .spinlock = portMUX_INITIALIZER_UNLOCKED, .hw_enabled = false, \
	}

typedef struct
{
	uart_hal_context_t hal; /*!< UART hal context*/
	portMUX_TYPE spinlock;
	bool hw_enabled;
} uart_context_t;

static uart_context_t uart_context[UART_NUM_MAX] = {
	UART_CONTEX_INIT_DEF(UART_NUM_0),
	UART_CONTEX_INIT_DEF(UART_NUM_1),
#if UART_NUM_MAX > 2
	UART_CONTEX_INIT_DEF(UART_NUM_2),
#endif
};

#define UART_ENTER_CRITICAL_ISR(mux) portENTER_CRITICAL_ISR(mux)
#define UART_EXIT_CRITICAL_ISR(mux) portEXIT_CRITICAL_ISR(mux)
#define UART_ENTER_CRITICAL(mux) portENTER_CRITICAL(mux)
#define UART_EXIT_CRITICAL(mux) portEXIT_CRITICAL(mux)

struct
{
	volatile uint16_t m_get_idx;
	volatile uint16_t m_put_idx;
	uint8_t m_entry[16384];
	SemaphoreHandle_t sem;
} uart_msg_queue;

void IRAM_ATTR uart_write_all(const uint8_t *data, int len)
{
#ifdef UART_USE_DMA_WRITE
	uart_dma_write(UHCI_INDEX, data, len);
#else
	while (len > 0)
	{
		while (!uart_ll_is_tx_idle(&TARGET_UART_DEV))
		{
		}
		uint16_t fill_len = uart_ll_get_txfifo_len(&TARGET_UART_DEV);
		if (fill_len > len)
		{
			fill_len = len;
		}
		len -= fill_len;
		if (fill_len > 0)
		{
			uart_ll_write_txfifo(&TARGET_UART_DEV, data, fill_len);
		}
		data += fill_len;
	}
#endif
}

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
	// uart_context_t *uart = param;
	portBASE_TYPE HPTaskAwoken = pdFALSE;

	uint32_t uart_intr_status = uart_ll_get_intsts_mask(&TARGET_UART_DEV);

	// while (uart_intr_status != 0) {
	int bytes_read = 0;
	uart_irq_count += 1;

	if ((uart_intr_status & UART_INTR_RXFIFO_TOUT) || (uart_intr_status & UART_INTR_RXFIFO_FULL) ||
		(uart_intr_status & UART_INTR_CMD_CHAR_DET))
	{
		// bytes_read = uart_ll_get_rxfifo_len(&TARGET_UART_DEV);
		while (TARGET_UART_DEV.mem_rx_status.wr_addr != TARGET_UART_DEV.mem_rx_status.rd_addr)
		{
			bytes_read += 1;
			char c = (*((volatile uint32_t *)UART_FIFO_REG(CONFIG_TARGET_UART_IDX)));
			(void)c;
		}
		uart_rx_count += bytes_read;

		if (uart_intr_status & UART_INTR_RXFIFO_TOUT)
		{
			// UART_ENTER_CRITICAL_ISR(&uart->spinlock);
			uart_byte_mode();
			// UART_EXIT_CRITICAL_ISR(&uart->spinlock);
		}
		else if (bytes_read >= 2)
		{
			// UART_ENTER_CRITICAL_ISR(&uart->spinlock);
			uart_bulk_mode();
			// UART_EXIT_CRITICAL_ISR(&uart->spinlock);
		}

		// 			// UART_ENTER_CRITICAL_ISR(&uart->spinlock);
		// 			while (bytes_read > 0) {
		// 				bytes_read -= 1;

		// 				// if (CBUF_IsFull(uart_msg_queue)) {
		// 				// 	uart_queue_full_cnt += 1;
		// 				// } else {
		// 				// 	CBUF_Push(uart_msg_queue, c);
		// 				// }
		// #ifdef CONFIG_COMPILER_OPTIMIZATION_PERF
		// 				__asm__ __volatile__("nop");
		// #endif
		// 			}
		// UART_EXIT_CRITICAL_ISR(&uart->spinlock);
		portBASE_TYPE shouldWake = pdFALSE;
		// xSemaphoreGiveFromISR(uart_msg_queue.sem, &shouldWake);
		if (shouldWake)
		{
			HPTaskAwoken = pdTRUE;
		}
		// uart_ll_clr_intsts_mask(&TARGET_UART_DEV, UART_INTR_RXFIFO_TOUT | UART_INTR_RXFIFO_FULL);
	}

	if (uart_intr_status & UART_INTR_RXFIFO_OVF)
	{
		// When fifo overflows, we reset the fifo.
		// UART_ENTER_CRITICAL_ISR(&uart->spinlock);
		uart_ll_rxfifo_rst(&TARGET_UART_DEV);
		uart_overrun_cnt += 1;
		// UART_EXIT_CRITICAL_ISR(&uart->spinlock);
		// uart_ll_clr_intsts_mask(&TARGET_UART_DEV, UART_INTR_RXFIFO_OVF);
	}

	if (uart_intr_status & UART_INTR_FRAM_ERR)
	{
		uart_frame_error_cnt += 1;
		// UART_ENTER_CRITICAL_ISR(&uart->spinlock);
		uart_ll_rxfifo_rst(&TARGET_UART_DEV);
		// UART_EXIT_CRITICAL_ISR(&uart->spinlock);
		// uart_ll_clr_intsts_mask(&TARGET_UART_DEV, UART_INTR_FRAM_ERR);
	}

	// else {
	uart_ll_clr_intsts_mask(&TARGET_UART_DEV, uart_intr_status);
	// }

	// 	uart_intr_status = uart_ll_get_intsts_mask(&TARGET_UART_DEV);
	// }

	if (HPTaskAwoken == pdTRUE)
	{
		portYIELD_FROM_ISR();
	}
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

	uart_sclk_t src_clk;
	uart_ll_get_sclk(&TARGET_UART_DEV, &src_clk);
	ESP_LOGE(__func__, "Existing sclk src: %d", src_clk);
	uart_ll_set_sclk(&TARGET_UART_DEV, UART_SCLK_APB);

	// The ESP32 ISR doesn't handle continuous streams of large amounts of data.
	// Remove their ISR and use our own.
	uart_ll_disable_intr_mask(&TARGET_UART_DEV, UART_LL_INTR_MASK);
	uart_ll_clr_intsts_mask(&TARGET_UART_DEV, UART_LL_INTR_MASK);
	ESP_ERROR_CHECK(esp_intr_alloc(PERIPH_UART_IRQ, ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_LEVEL1, console_isr,
								   &uart_context[CONFIG_TARGET_UART_IDX], &console_isr_handle));

	const uart_intr_config_t uart_intr = {
		.intr_enable_mask = UART_RXFIFO_FULL_INT_ENA_M | UART_FRM_ERR_INT_ENA_M | UART_RXFIFO_OVF_INT_ENA_M
		/*|UART_TXFIFO_EMPTY_INT_ENA_M*/,
		.rx_timeout_thresh = 2,
		.txfifo_empty_intr_thresh = 1,
		.rxfifo_full_thresh = 1,
	};
	ESP_ERROR_CHECK(uart_intr_config(CONFIG_TARGET_UART_IDX, &uart_intr));
	uart_byte_mode();

	uart_set_wakeup_threshold(CONFIG_TARGET_UART_IDX, 3);
	esp_sleep_enable_uart_wakeup(CONFIG_TARGET_UART_IDX);
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
	// xTaskCreate(net_uart_task, "net_uart_task", 6 * 1024, NULL, 1, NULL);
#endif
}