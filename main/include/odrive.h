#ifndef ODRIVE_H
#define ODRIVE_H

#include <stdint.h>
#include <stddef.h>
#include "driver/uart.h"

typedef void (*odrive_rx_cb_t)(const uint8_t* data, size_t len, void* ctx);

class ODrive {
public:
    ODrive();
    ~ODrive();

    // Initialize the driver (does not start RX task)
    // Returns true on success.
    bool init(uart_port_t uart_num, int tx_io_num, int rx_io_num, int baudrate = 115200, int rx_buffer_size = 1024);

    // Start and stop background RX handling
    bool start();
    void stop();

    // Send raw bytes over the configured transport
    // Returns number of bytes written or -1 on error
    int send(const uint8_t* data, size_t len);

    // Simple framed send: prepend start byte and length
    int sendFramed(uint8_t start_byte, const uint8_t* payload, size_t payload_len);

    // Register receive callback
    void setRxCallback(odrive_rx_cb_t cb, void* ctx);

    // Configure framing behaviour for RX (default: raw passthrough)
    void enableFraming(bool enabled, uint8_t start_byte = 0xAA);

private:
    uart_port_t uart_num_;
    int tx_io_num_;
    int rx_io_num_;
    int baudrate_;
    int rx_buffer_size_;
    odrive_rx_cb_t rx_cb_;
    void* rx_cb_ctx_;
    bool framing_enabled_;
    uint8_t framing_start_byte_;
    void* rx_task_handle_;

    static void rxTaskEntry(void* arg);
    void rxTask();
};

#endif // ODRIVE_H