#include "pico/stdlib.h"


const uint8_t no_sd_width = 10;
const uint8_t no_sd_height = 8;

const uint8_t no_sd_bitmap [] = 
{
// 'noSD', 10x8px
0x55, 0x80, 0x81, 0x40, 0x81, 0x80, 0x41, 0x00, 0x41, 0x2a
};

const bitmap no_sd = {(uint8_t *) no_sd_bitmap, no_sd_width, no_sd_height};