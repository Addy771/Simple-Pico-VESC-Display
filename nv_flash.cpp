/*  */
#include <cstdlib>
#include <hardware/flash.h>
// #include <hardware/sync.h>
// #include <pico/sync.h>
#include <pico/stdlib.h>
#include <pico/flash.h>
// #include <pico/malloc.h>
#include <string.h>
#include "nv_flash.hpp"


/// @brief A non-volatile data manager which uses the code flash memory for storage
void nv_flash_storage::init(void)
{
    uint8_t *flash_byte;
    uint address;

    static_assert(sizeof(data) <= BLOCK_SIZE, "Non-volatile storage data exceeds block size! Increase BLOCK_SIZE.");


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
}


/// @brief Wrapper to init flash safety code on the core that isn't doing flash writes
void nv_flash_storage::alt_core_init(void)
{
    flash_safe_execute_core_init();
}


/// @brief Performs the flash erase/program operations
static void nv_flash_storage::flash_operations(void)
{
    uint8_t page_buffer[FLASH_PAGE_SIZE];

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


    // Fill page with 1's so that unused data areas don't get written
    memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);

    // Copy the data structure into the buffer at the appropriate location
    memcpy(page_buffer + block_id*BLOCK_SIZE, &data, BLOCK_SIZE);

    // Write out the data
    flash_range_program(FLASH_TARGET_OFFSET + page_id*FLASH_PAGE_SIZE, page_buffer, FLASH_PAGE_SIZE);
}


/// @brief Write the data structure into the non-volatile flash memory
void nv_flash_storage::store_data()
{
    int rc;

    // When safe, run flash_operations() with maximum timeout for enter/exit
    rc = flash_safe_execute(&nv_flash_storage::static_wrapper, this, UINT32_MAX);

    hard_assert(rc == PICO_OK);
}


/// @brief Wrapper needed to match flash_safe_execute's expected signature
static void nv_flash_storage::static_wrapper(void *Param)
{
    // Cast void param back to the class type
    nv_flash_storage* instance = static_cast<nv_flash_storage*>(Param);

    instance->flash_operations();
}

