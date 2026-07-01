#ifndef NV_FLASH_H
#define NV_FLASH_H

#include <hardware/flash.h>
#include "pico/sync.h"


#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE) // Start of last sector of flash
#define BLOCK_SIZE 64       // Size in bytes reserved for stored data structure. Must be power of 2 and big enough for data struct

#define WRITE_MARKER 0xAA   // Value to store in flash to indicate where a data block starts. At least one bit must be cleared.
#define MAX_PAGES (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)
#define MAX_BLOCKS (FLASH_PAGE_SIZE / BLOCK_SIZE)


typedef struct
{
    uint8_t hour = 0;
    uint8_t minute = 0;
}   time_of_day_t;

typedef struct 
{
    uint8_t write_mark = WRITE_MARKER;      // KEEP IN FRONT
    uint8_t disp_brightness = 1;
    uint8_t load_modes = 0;
    uint8_t flags = 0;
    uint32_t log_num = 0;
    float odometer = 0.0f;    
    float trip_a = 0.0f;
    float trip_b = 0.0f;
    uint8_t user_vesc_can_id = 0;
    uint8_t user_disp_can_id = 0;
    uint8_t speed_bar_max = 0;
    uint8_t spare_0 = 0;
    time_of_day_t day_start = {9, 0};
    time_of_day_t night_start = {17, 0};
} nv_flash_struct;


typedef enum
{
    COMM_USE_CAN = (1 << 0),
    BL_MODE_B0 = (1 << 1),
    BL_MODE_B1 = (1 << 2)
} nv_flags;


class nv_flash_storage
{
    private:
        mutex_t *write_lock;

    public:
        nv_flash_struct data;
        void init(mutex_t *flash_write_lock);
        void store_data();
        uint8_t block_id;
        int8_t page_id;

};

#endif /* NV_FLASH_H */
