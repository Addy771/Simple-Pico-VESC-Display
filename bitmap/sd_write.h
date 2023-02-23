#include "pico/stdlib.h"


const uint8_t sd_wr_width = 10;
const uint8_t sd_wr_height = 8;

const uint8_t sd_wr_bitmap [] = 
{
// 'SD_writing', 10x8px
0x00, 0x3d, 0x43, 0x07, 0x00, 0x00, 0xe0, 0xc2, 0xbc, 0x00
};

const bitmap sd_wr = {(uint8_t *) sd_wr_bitmap, sd_wr_width, sd_wr_height};