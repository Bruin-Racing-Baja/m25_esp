#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <vector>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "driver/gpio.h"
#include <odrive.h>
#include <gpio_wrapper.h>
#include <constants.h>

#include <input_output/button.h>
#include <input_output/led.h>
#include <input_output/centerlock_limit_switch.h>

using std::vector;

#define TWAI_SENDER_TX_GPIO     GPIO_NUM_5
#define TWAI_SENDER_RX_GPIO     GPIO_NUM_4
#define TWAI_QUEUE_DEPTH        10
#define TWAI_BITRATE            250000

// Message IDs
#define TWAI_DATA_ID            0x100
#define TWAI_HEARTBEAT_ID       0x7FF
#define TWAI_DATA_LEN           1000

static const char *TAG = "twai_sender";

typedef struct {
    twai_frame_t frame;
    uint8_t data[TWAI_FRAME_MAX_LEN];
} twai_sender_data_t;

Button button_a(DAQ_BUTTON_A_PIN);
Button button_b(CONTROLS_BUTTON_4_PIN);

// Transmission completion callback
static IRAM_ATTR bool twai_sender_tx_done_callback(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    if (!edata->is_tx_success) {
        ESP_EARLY_LOGW(TAG, "Failed to transmit message, ID: 0x%X", edata->done_tx_frame->header.id);
    }
    return false; // No task wake required
}

// Bus error callback
static IRAM_ATTR bool twai_sender_on_error_callback(twai_node_handle_t handle, const twai_error_event_data_t *edata, void *user_ctx)
{
    ESP_EARLY_LOGW(TAG, "TWAI node error: 0x%x", edata->err_flags.val);
    return false; // No task wake required
}

static IRAM_ATTR void button_a_callback(void * params)
{
    button_a.button_isr();
    ESP_EARLY_LOGW(TAG, "Button A state: %d", button_a.read_button_state());
}


static IRAM_ATTR void button_b_callback(void * params)
{
    button_b.button_isr();
}


extern "C" void app_main(void)
{    
    vector<int> led_pins = {DAQ_LED_1_PIN};
    LED daq_leds(led_pins);

    attachInterrupt(CONTROLS_BUTTON_4_PIN, button_b_callback, InterruptMode::LOW_LEVEL);

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_EARLY_LOGW(TAG, "C: %d\n", digitalRead(CONTROLS_BUTTON_4_PIN));
    }
}
