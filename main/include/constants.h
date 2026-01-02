#ifndef CONSTANTS_H
#define CONSTANTS_H
#include <stdint.h>

/* Units */
constexpr float SECONDS_PER_MINUTE = 60.0; /* s / min */
constexpr float MS_PER_SECOND = 1.0e3;     /* ms / s */ 
constexpr float US_PER_SECOND = 1.0e6;     /* us / s */
constexpr float SECONDS_PER_MS = 1.0e-3;   /* s / ms */
constexpr float SECONDS_PER_US = 1.0e-6;   /* s / us */

constexpr float MM_PER_INCH = 25.4;              /* mm / inch */
constexpr float INCHES_PER_MM = 1 / MM_PER_INCH; /* inch / mm */

constexpr float FEET_PER_MILE = 5280.0; /* feet / mile */
constexpr float INCH_PER_FEET = 12.0;   /* inch / feet */

/* External Constants */
void init_constants(); 

extern uint32_t CPU_HZ; /* CPU Frequency (Hz) */

#endif // CONSTANTS_H