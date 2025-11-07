#ifndef __CORE1_MAIN_H__
#define __CORE1_MAIN_H__

#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/multicore.h"

#include "core1_structs.h"

#define FIFO_HEADER_UINT16 0xABAB

void core1_entry(void);


#endif