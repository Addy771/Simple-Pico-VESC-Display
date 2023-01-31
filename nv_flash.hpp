/*

PICO_FLASH_SIZE_BYTES # The total size of the RP2040 flash, in bytes
FLASH_SECTOR_SIZE     # The size of one sector, in bytes (the minimum amount you can erase)
FLASH_PAGE_SIZE       # The size of one page, in bytes (the mimimum amount you can write)

flash_range_erase(uint32_t flash_offs, size_t count);
flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);

*/

#include <hardware/flash.h>


#define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE) // Start of last sector of flash
#define BLOCK_SIZE 64       // Size in bytes reserved for stored data structure. Must be power of 2 and big enough for data struct

#define WRITE_MARKER 0xAA   // Value to store in flash to indicate where a data block starts. At least one bit must be cleared.
#define MAX_PAGES (FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE)
#define MAX_BLOCKS (FLASH_PAGE_SIZE / BLOCK_SIZE)


typedef struct 
{
    uint8_t write_mark;
    uint8_t disp_brightness;
} nv_flash_struct;


class nv_flash_storage
{
    private:
        mutex_t *write_lock;

    public:
        nv_flash_struct data;
        nv_flash_storage(mutex_t *flash_write_lock);
        void store_data();
        uint8_t block_id;
        int8_t page_id;

};

