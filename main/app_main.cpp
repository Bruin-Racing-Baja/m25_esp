#include <stdio.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h" 
#include "freertos/task.h"

#include "odrive.h"
#include "ecvt_controller.h"
#include "centerlock_controller.h"
#include "telemetry.h"
#include "gpio_wrapper.h"

extern "C" void app_main(void)
{
    printf("Hello world!\n");
    
    // Example usage of ODrive class (UART)
    static ODrive odrv;

    // Replace these pins with your board's UART TX/RX pins
    const int tx_pin = 17; // placeholder
    const int rx_pin = 16; // placeholder

    if (!odrv.init(UART_NUM_1, tx_pin, rx_pin, 115200, 1024)) {
        printf("ODrive init failed\n");
        return;
    }

    // simple callback that prints received frames
    void odrive_rx_cb(const uint8_t* data, size_t len, void* ctx) {
        printf("ODrive RX (%u bytes): ", (unsigned)len);
        for (size_t i = 0; i < len; ++i) printf("%02X ", data[i]);
        printf("\n");
    }

    odrv.setRxCallback(odrive_rx_cb, nullptr);
    odrv.enableFraming(true, 0xAA);
    odrv.start();

    // Send a test framed packet after startup
    const uint8_t payload[] = { 0x01, 0x02, 0x03 };
    odrv.sendFramed(0xAA, payload, sizeof(payload));
}
