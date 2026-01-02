#include "constants.h"
#include "esp_clk.h"

uint32_t CPU_HZ = 0; 

void init_constants() {
    CPU_HZ = esp_clk_cpu_freq();
}