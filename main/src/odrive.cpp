#include "odrive.h"
#include "esp_log.h"
#include <string.h>

static const char* TAG = "ODrive";

ODrive::ODrive()
    : tx_pin_(GPIO_NUM_NC)
    , rx_pin_(GPIO_NUM_NC)
    , bitrate_(500000)
    , rx_task_handle_(nullptr)
    , heartbeat_cb_(nullptr)
    , heartbeat_ctx_(nullptr)
    , encoder_cb_(nullptr)
    , encoder_ctx_(nullptr)
    , iq_cb_(nullptr)
    , iq_ctx_(nullptr)
{
}

ODrive::~ODrive()
{
    stop();
}

bool ODrive::init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t bitrate)
{
    tx_pin_ = tx_pin;
    rx_pin_ = rx_pin;
    bitrate_ = bitrate;

    // Configure TWAI timing based on bitrate
    twai_timing_config_t timing_config;
    
    switch (bitrate) {
        case 1000000:
            timing_config = TWAI_TIMING_CONFIG_1MBITS();
            break;
        case 500000:
            timing_config = TWAI_TIMING_CONFIG_500KBITS();
            break;
        case 250000:
            timing_config = TWAI_TIMING_CONFIG_250KBITS();
            break;
        case 125000:
            timing_config = TWAI_TIMING_CONFIG_125KBITS();
            break;
        default:
            ESP_LOGE(TAG, "Unsupported bitrate: %lu", bitrate);
            return false;
    }

    // Configure filter to accept all messages
    twai_filter_config_t filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Configure general settings
    twai_general_config_t general_config = TWAI_GENERAL_CONFIG_DEFAULT(
        tx_pin, rx_pin, TWAI_MODE_NORMAL);
    general_config.rx_queue_len = 20;
    general_config.tx_queue_len = 10;

    // Install TWAI driver
    esp_err_t err = twai_driver_install(&general_config, &timing_config, &filter_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(err));
        return false;
    }

    ESP_LOGI(TAG, "TWAI driver installed (TX: GPIO%d, RX: GPIO%d, %lu bps)", 
             tx_pin, rx_pin, bitrate);

    return true;
}

bool ODrive::start()
{
    // Start TWAI driver
    esp_err_t err = twai_start();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI: %s", esp_err_to_name(err));
        return false;
    }

    // Create RX task
    BaseType_t result = xTaskCreate(
        rx_task_entry,
        "odrive_can_rx",
        4096,
        this,
        tskIDLE_PRIORITY + 5,
        &rx_task_handle_
    );

    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create RX task");
        twai_stop();
        return false;
    }

    ESP_LOGI(TAG, "ODrive CAN started");
    return true;
}

void ODrive::stop()
{
    if (rx_task_handle_) {
        vTaskDelete(rx_task_handle_);
        rx_task_handle_ = nullptr;
    }

    twai_stop();
    twai_driver_uninstall();
    ESP_LOGI(TAG, "ODrive CAN stopped");
}

uint32_t ODrive::build_can_id(uint8_t node_id, uint16_t cmd_id)
{
    // ODrive CAN ID format: (node_id << 5) | cmd_id
    return ((uint32_t)node_id << 5) | (cmd_id & 0x1F);
}

void ODrive::send_can_msg(uint32_t can_id, const uint8_t* data, uint8_t len, bool remote)
{
    twai_message_t msg;
    msg.identifier = can_id;
    msg.data_length_code = len;
    msg.extd = 0;  
    msg.rtr = remote ? 1 : 0;   
    msg.ss = 0;
    msg.self = 0;
    msg.dlc_non_comp = 0;

    if (data && len > 0) {
        memcpy(msg.data, data, len);
    }

    esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(100));
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to send CAN message: %s", esp_err_to_name(err));
    }
}

void ODrive::set_axis_state(uint8_t node_id, odrive_axis_state_t state)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_AXIS_STATE);
    uint32_t state_val = (uint32_t)state;
    send_can_msg(can_id, (uint8_t*)&state_val, 4);
    ESP_LOGI(TAG, "Set axis state: node=%d, state=%d", node_id, state);
}

void ODrive::set_controller_mode(uint8_t node_id, odrive_control_mode_t ctrl_mode, odrive_input_mode_t input_mode)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_CONTROLLER_MODE);
    uint8_t data[8];
    memcpy(&data[0], &ctrl_mode, 4);
    memcpy(&data[4], &input_mode, 4);
    send_can_msg(can_id, data, 8);
    ESP_LOGI(TAG, "Set controller mode: node=%d, ctrl=%d, input=%d", node_id, ctrl_mode, input_mode);
}

void ODrive::set_input_pos(uint8_t node_id, float pos, int16_t vel_ff, int16_t torque_ff)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_INPUT_POS);
    uint8_t data[8];
    memcpy(&data[0], &pos, 4);
    memcpy(&data[4], &vel_ff, 2);
    memcpy(&data[6], &torque_ff, 2);
    send_can_msg(can_id, data, 8);
}

void ODrive::set_input_vel(uint8_t node_id, float vel, float torque_ff)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_INPUT_VEL);
    uint8_t data[8];
    memcpy(&data[0], &vel, 4);
    memcpy(&data[4], &torque_ff, 4);
    send_can_msg(can_id, data, 8);
}

void ODrive::set_input_torque(uint8_t node_id, float torque)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_INPUT_TORQUE);
    send_can_msg(can_id, (uint8_t*)&torque, 4);
}

void ODrive::set_limits(uint8_t node_id, float vel_limit, float current_limit)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_LIMITS);
    uint8_t data[8];
    memcpy(&data[0], &vel_limit, 4);
    memcpy(&data[4], &current_limit, 4);
    send_can_msg(can_id, data, 8);
    ESP_LOGI(TAG, "Set limits: node=%d, vel=%.2f, current=%.2f", node_id, vel_limit, current_limit);
}

void ODrive::set_pos_gain(uint8_t node_id, float pos_gain)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_POS_GAIN);
    send_can_msg(can_id, (uint8_t*)&pos_gain, 4);
}

void ODrive::set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain)
{
    uint32_t can_id = build_can_id(node_id, CAN_SET_VEL_GAINS);
    uint8_t data[8];
    memcpy(&data[0], &vel_gain, 4);
    memcpy(&data[4], &vel_integrator_gain, 4);
    send_can_msg(can_id, data, 8);
}

void ODrive::estop(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_ESTOP);
    send_can_msg(can_id, nullptr, 0);
    ESP_LOGW(TAG, "E-STOP sent to node %d", node_id);
}

void ODrive::clear_errors(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_CLEAR_ERRORS);
    send_can_msg(can_id, nullptr, 0);
    ESP_LOGI(TAG, "Clear errors: node=%d", node_id);
}

void ODrive::reboot(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_REBOOT);
    send_can_msg(can_id, nullptr, 0);
    ESP_LOGI(TAG, "Reboot: node=%d", node_id);
}

void ODrive::request_encoder_est(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_GET_ENCODER_ESTIMATES);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_iq(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_GET_IQ);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_bus_voltage_current(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_GET_BUS_VOLTAGE_CURRENT);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::request_temperature(uint8_t node_id)
{
    uint32_t can_id = build_can_id(node_id, CAN_GET_TEMPERATURE);
    send_can_msg(can_id, nullptr, 0);
}

void ODrive::set_heartbeat_callback(odrive_heartbeat_cb_t cb, void* ctx)
{
    heartbeat_cb_ = cb;
    heartbeat_ctx_ = ctx;
}

void ODrive::set_encoder_callback(odrive_encoder_cb_t cb, void* ctx)
{
    encoder_cb_ = cb;
    encoder_ctx_ = ctx;
}

void ODrive::set_iq_callback(odrive_iq_cb_t cb, void* ctx)
{
    iq_cb_ = cb;
    iq_ctx_ = ctx;
}

void ODrive::rx_task_entry(void* arg)
{
    ODrive* self = (ODrive*)arg;
    self->rx_task();
    vTaskDelete(NULL);
}

void ODrive::rx_task()
{
    twai_message_t rx_msg;
    
    while (true) {
        esp_err_t err = twai_receive(&rx_msg, pdMS_TO_TICKS(100));
        
        if (err == ESP_OK) {
            process_msg(rx_msg);
        } else if (err == ESP_ERR_TIMEOUT) {
            // Normal timeout, continue
        } else {
            ESP_LOGW(TAG, "TWAI receive error: %s", esp_err_to_name(err));
        }
    }
}

void ODrive::process_msg(const twai_message_t& msg)
{
    // Extract node ID and command ID from CAN ID
    uint8_t node_id = (msg.identifier >> 5) & 0x3F;
    uint16_t cmd_id = msg.identifier & 0x1F;

    switch (cmd_id) {
        case CAN_HEARTBEAT:
            parse_heartbeat(node_id, msg.data, msg.data_length_code);
            break;
            
        case CAN_GET_ENCODER_ESTIMATES:
            parse_encoder_estimates(node_id, msg.data, msg.data_length_code);
            break;
            
        case CAN_GET_IQ:
            parse_iq(node_id, msg.data, msg.data_length_code);
            break;
            
        default:
            // Unhandled message type
            break;
    }
}

void ODrive::parse_heartbeat(uint8_t node_id, const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    uint32_t error;
    uint8_t state;
    
    memcpy(&error, &data[0], 4);
    state = data[4];
    // data[5-7] contain procedure result and trajectory done flag
    
    if (heartbeat_cb_) {
        heartbeat_cb_(node_id, error, state, heartbeat_ctx_);
    }
}

void ODrive::parse_encoder_estimates(uint8_t node_id, const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    float pos, vel;
    memcpy(&pos, &data[0], 4);
    memcpy(&vel, &data[4], 4);
    
    if (encoder_cb_) {
        encoder_cb_(node_id, pos, vel, encoder_ctx_);
    }
}

void ODrive::parse_iq(uint8_t node_id, const uint8_t* data, uint8_t len)
{
    if (len < 8) return;
    
    float iq_setpoint, iq_measured;
    memcpy(&iq_setpoint, &data[0], 4);
    memcpy(&iq_measured, &data[4], 4);
    
    if (iq_cb_) {
        iq_cb_(node_id, iq_setpoint, iq_measured, iq_ctx_);
    }
}