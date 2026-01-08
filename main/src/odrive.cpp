#include "odrive.h"

#include <stdlib.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char* TAG = "ODrive";

ODrive::ODrive()
	: uart_num_(UART_NUM_1), tx_io_num_(-1), rx_io_num_(-1), baudrate_(115200), rx_buffer_size_(1024), rx_cb_(nullptr), rx_cb_ctx_(nullptr), framing_enabled_(false), framing_start_byte_(0xAA), rx_task_handle_(nullptr)
{
}

bool ODrive::init(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baudrate, int rx_buffer_size)
{
	uart_num_ = uart_num;
	tx_io_num_ = tx_io_num;
	rx_io_num_ = rx_io_num;
	baudrate_ = baudrate;
	rx_buffer_size_ = rx_buffer_size;

	uart_config_t uart_config = {};
	uart_config.baud_rate = baudrate_;
	uart_config.data_bits = UART_DATA_8_BITS;
	uart_config.parity = UART_PARITY_DISABLE;
	uart_config.stop_bits = UART_STOP_BITS_1;
	uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
	uart_config.source_clk = UART_SCLK_APB;

	esp_err_t err = uart_param_config(uart_num_, &uart_config);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_param_config failed: %d", err);
		return false;
	}

	err = uart_set_pin(uart_num_, tx_io_num_, rx_io_num_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_set_pin failed: %d", err);
		return false;
	}

	err = uart_driver_install(uart_num_, rx_buffer_size_ * 2, 0, 0, NULL, 0);
	if (err != ESP_OK) {
		ESP_LOGE(TAG, "uart_driver_install failed: %d", err);
		return false;
	}

	return true;
}

bool ODrive::start()
{
	if (rx_task_handle_ != nullptr) return true;

	BaseType_t r = xTaskCreate([](void* arg){ ((ODrive*)arg)->rxTaskEntry(arg); }, "odrive_rx", 4096, this, tskIDLE_PRIORITY + 5, (TaskHandle_t*)&rx_task_handle_);
	if (r != pdPASS) {
		ESP_LOGE(TAG, "Failed to create rx task");
		rx_task_handle_ = nullptr;
		return false;
	}
	return true;
}

void ODrive::stop()
{
	if (rx_task_handle_) {
		vTaskDelete((TaskHandle_t)rx_task_handle_);
		rx_task_handle_ = nullptr;
	}
	uart_driver_delete(uart_num_);
}

int ODrive::send(const uint8_t* data, size_t len)
{
	if (!data || len == 0) return 0;
	int written = uart_write_bytes(uart_num_, (const char*)data, len);
	return written;
}

int ODrive::sendFramed(uint8_t start_byte, const uint8_t* payload, size_t payload_len)
{
	if (!payload && payload_len > 0) return -1;
	std::vector<uint8_t> buf;
	buf.reserve(2 + payload_len);
	buf.push_back(start_byte);
	buf.push_back((uint8_t)payload_len);
	if (payload_len) buf.insert(buf.end(), payload, payload + payload_len);
	return send(buf.data(), buf.size());
}

void ODrive::setRxCallback(odrive_rx_cb_t cb, void* ctx)
{
	rx_cb_ = cb;
	rx_cb_ctx_ = ctx;
}

void ODrive::enableFraming(bool enabled, uint8_t start_byte)
{
	framing_enabled_ = enabled;
	framing_start_byte_ = start_byte;
}

void ODrive::rxTaskEntry(void* arg)
{
	ODrive* self = (ODrive*)arg;
	self->rxTask();
	vTaskDelete(NULL);
}

void ODrive::rxTask()
{
	std::vector<uint8_t> buffer;
	buffer.reserve(rx_buffer_size_);

	uint8_t* read_buf = (uint8_t*)malloc(rx_buffer_size_);
	if (!read_buf) {
		ESP_LOGE(TAG, "Failed to allocate read buffer");
		return;
	}

	while (true) {
		int len = uart_read_bytes(uart_num_, read_buf, rx_buffer_size_, pdMS_TO_TICKS(100));
		if (len > 0) {
			if (!framing_enabled_) {
				if (rx_cb_) rx_cb_(read_buf, len, rx_cb_ctx_);
			} else {
				// append and process frames
				buffer.insert(buffer.end(), read_buf, read_buf + len);
				// process loop
				while (buffer.size() >= 2) {
					// find start byte at front
					if (buffer[0] != framing_start_byte_) {
						// drop until we find start
						auto it = std::find(buffer.begin(), buffer.end(), framing_start_byte_);
						if (it == buffer.end()) {
							buffer.clear();
							break;
						} else {
							buffer.erase(buffer.begin(), it);
							if (buffer.size() < 2) break;
						}
					}

					uint8_t payload_len = buffer[1];
					if (buffer.size() < (size_t)(2 + payload_len)) break; // wait for rest

					// got full frame
					if (rx_cb_) rx_cb_(buffer.data() + 2, payload_len, rx_cb_ctx_);

					// erase consumed bytes
					buffer.erase(buffer.begin(), buffer.begin() + 2 + payload_len);
				}
			}
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}

	free(read_buf);
}