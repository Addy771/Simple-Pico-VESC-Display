/*  */
#include <cstdlib>
#include <hardware/flash.h>
#include <hardware/sync.h>
#include <pico/sync.h>
#include <pico/stdlib.h>
#include <pico/malloc.h>
#include <string.h>
#include "nv_flash.hpp"



/*

PICO_FLASH_SIZE_BYTES # The total size of the RP2040 flash, in bytes
FLASH_SECTOR_SIZE     # The size of one sector, in bytes (the minimum amount you can erase)
FLASH_PAGE_SIZE       # The size of one page, in bytes (the mimimum amount you can write)

flash_range_erase(uint32_t flash_offs, size_t count);
flash_range_program(uint32_t flash_offs, const uint8_t *data, size_t count);
offsetof()

*/


/// @brief A non-volatile data manager which uses the code flash memory for storage
/// @param flash_write_lock A mutex which will be used to suspend execution on the other processor core
nv_flash_storage::nv_flash_storage(mutex_t *flash_write_lock)
{
    uint8_t *flash_byte;
    uint address;

    // Store the mutex we'll use when doing flash writes
    write_lock = flash_write_lock;

    // Probe the NV storage sector to see if data exists already
    page_id = -1;

    for (uint8_t current_page = 0; current_page < MAX_PAGES; current_page++)
    {
        address = XIP_BASE + FLASH_TARGET_OFFSET + (current_page * FLASH_PAGE_SIZE);

        for (uint8_t current_block = 0; current_block < MAX_BLOCKS; current_block++)
        {
            flash_byte = (uint8_t *) address + (current_block * BLOCK_SIZE);

            // If the marker is present, store the page and block ID
            if (*flash_byte == WRITE_MARKER)
            {
                page_id = current_page;
                block_id = current_block;
            }
            else
            {   
                // Write marker is missing, stop searching
                break;
            }
        }
    }

    
    // If data is stored in the flash, read it into the class data structure
    if (page_id != -1)
    {
        address = XIP_BASE + FLASH_TARGET_OFFSET + (page_id * FLASH_PAGE_SIZE) + (block_id * BLOCK_SIZE);
        memcpy(&data, address, sizeof(nv_flash_struct));
    }
    else
    {
        // Otherwise, initialize the data structure with default values
        data.write_mark = WRITE_MARKER;
        data.disp_brightness = 0;
    }
}


/// @brief Write the data structure into the non-volatile flash memory
void nv_flash_storage::store_data()
{
    uint saved_interrupts;

    mutex_enter_blocking(write_lock);   // Enter mutex to stop other core from doing flash reads
    saved_interrupts = save_and_disable_interrupts();   // Disable interrupts to make sure other core doesn't run any code

    // Figure out the page and block to write to next
    if (page_id < MAX_PAGES - 1)
    {
        if (block_id < MAX_BLOCKS - 1)
        {
            block_id++;
        }
        else
        {
            block_id = 0;
            page_id++;      // When chunks roll over, move to the next page
        }
    }
    else
    {
        page_id = 0;        
        // When page rolls over to the beginning we need to erase the whole sector

        flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);

    }


    uint8_t *page_buffer;
    page_buffer = (uint8_t *) malloc(FLASH_PAGE_SIZE);

    // Fill page with 1's so that unused data areas don't get written
    memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);

    // Copy the data structure into the buffer at the appropriate location
    memcpy(page_buffer + block_id*BLOCK_SIZE, &data, BLOCK_SIZE);

    // Write out the data
    flash_range_program(FLASH_TARGET_OFFSET + page_id*FLASH_PAGE_SIZE, page_buffer, FLASH_PAGE_SIZE);

    restore_interrupts(saved_interrupts);
    mutex_exit(write_lock);     // Release lock now that flash erase/write is complete

    free(page_buffer);

}