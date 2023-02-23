#include "pico/stdlib.h"


const uint8_t sd_width = 10;
const uint8_t sd_height = 8;

const uint8_t sd_bitmap [] = 
{
// 'SD', 10x8px
0xff, 0x81, 0x95, 0x4b, 0x81, 0x9f, 0x53, 0x4d, 0x41, 0x3e
};

const bitmap sd = {(uint8_t *) sd_bitmap, sd_width, sd_height};