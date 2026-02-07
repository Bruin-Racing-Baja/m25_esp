#include "telemetry.h"

Telemetry::Telemetry()
{
    data_mutex = xSemaphoreCreateMutex();
    sequence_number = 0;
}

bool Telemetry::lock(TickType_t timeout_ticks = portMAX_DELAY) {
    return xSemaphoreTake(data_mutex, timeout_ticks) == pdTRUE;
}

void Telemetry::unlock() {
    xSemaphoreGive(data_mutex);
} 

void Telemetry::send_data() {
    TelemetryPacket packet;
    packet.start_byte_1 = 0xFA;
    packet.start_byte_2 = 0xCE;
    packet.packet_type = 0x01; 
    packet.sequence_number = sequence_number++;

    lock();

    memcpy(&packet.payload, &data, sizeof(VehicleData));
    unlock();
    packet.checksum = esp_rom_crc32_le(0, (uint8_t*)&packet, sizeof(TelemetryPacket) - sizeof(packet.checksum));

    uart_write_bytes(UART_NUM_0, (const char*)&packet, sizeof(TelemetryPacket));
}