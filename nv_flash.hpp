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
    uint8_t write_mark = WRITE_MARKER;
    uint8_t disp_brightness = 0;
    uint32_t log_num = 0;
    float odometer = 0.0;
    float total_watt_hours_used = 0.0;
    float total_watt_hours_charged = 0.0;
} nv_flash_struct;


class nv_flash_storage
{
    private:
        mutex_t *write_lock;

    public:
        nv_flash_struct data;
        nv_flash_storage(mutex_t *flash_write_lock);
        void store_data();
        void reset_data();
        uint8_t block_id;
        int8_t page_id;

};

extern mutex_t flash_lock;
extern nv_flash_storage nv_settings;

#endif /* NV_FLASH_H */
