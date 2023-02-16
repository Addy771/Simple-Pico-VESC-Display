#include <cstdlib>
#include <pico/stdlib.h>
#include <string.h>
#include "log.hpp"
#include "pico-oled/pico-oled.hpp"
#include "Button-debouncer/button_debounce.h"

#include "sd_card.h"
#include "ff.h"
#include "f_util.h"
#include "hw_config.h"
#include "hw_def.h"


extern pico_oled display;
extern Debounce debouncer;


uint8_t get_next_log_name(char *filename)
{
    // If there's a valid source of time, timestamp names should be used

    // Otherwise, check what log files exist already
}



uint8_t init_filesystem()
{
    sd_card_t *pSD = sd_get_by_num(0);

    // Try to mount SD card
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);

    // display.print(FRESULT_str(fr));
    // display.print("\r\n");


    // switch (fr)
    // {
    //     case FR_OK:
    //         display.print("FAT FILESYSTEM MOUNTED\n");
    //         break;

    //     case FR_NO_FILESYSTEM:
    //         display.print("NO FILESYSTEM DETECTED\n");
    //         break;

    //     case FR_NOT_READY:
    //         display.print("SD CARD NOT FOUND\n");
    // }

    // display.render();

    // Check if formatting is needed
    // FR_NO_FILESYSTEM
    if (fr == FR_NO_FILESYSTEM)
    {
        display.draw_boxed_text("SD CARD NEEDS FORMATTING.\nPRESS LEFT TO CANCEL\nPRESS RIGHT TO FORMAT", 1, 1, 8, 20);
        display.render();
        display.fill(0);
        sleep_ms(500);


        while (1)
        {
            // Left button pressed?
            if (debouncer.read(PB_LEFT_GPIO))
            {
                // Abort, return status
                f_unmount(pSD->pcName);
                return 1;
            }

            // Right button pressed? 
            if (debouncer.read(PB_RIGHT_GPIO))
            {
                // Format SD card

                BYTE work[FF_MAX_SS];   // Work area for formatting

                // unmount first
                f_unmount(pSD->pcName);

                // Create FAT volume with default parameters
                fr = f_mkfs("", 0, work, sizeof(work));

                if (fr != FR_OK)
                {
                    // Failed to format, do something
                    f_unmount(pSD->pcName);
                    return 2;
                }

                // Set volume label (Can't use in this fatfs implementation)
                //f_setlabel("SPVD SD");


                // Mount once again so the filesystem can be used
                fr = f_mount(&pSD->fatfs, pSD->pcName, 1);

                if (fr != FR_OK)
                {
                    // Still can't use the SD card??
                    f_unmount(pSD->pcName);
                    return 3;
                }

                break;  
            }
        }

    }
    else if (fr == FR_NOT_READY)
    {
        // No SD card found
        f_unmount(pSD->pcName);
        return 4;
    }


    // Check if enough free space exists on the SD card??
    DWORD free_clust;
    uint32_t free_mb;
    FATFS *fs_ptr = &pSD->fatfs;

    fr = f_getfree(pSD->pcName, &free_clust, &fs_ptr);

    if (fr != FR_OK)
    {
        // Couldn't get free space
        f_unmount(pSD->pcName);
        return 5;
    }
    else
    {
        free_mb = (free_clust * pSD->fatfs.csize) / (2 * 1024); // sectors / 2 = KB, /1024 = MB
        display.print_num("FREE SPACE: %dMB\n", free_mb);   

        // Warn user if free space is low
        if (free_mb < MIN_FREE_MB)
        {
            display.print("WARNING! SD CARD NEARLY FULL\n");
        }
    }

    // Create log file directory if it doesn't exist
    // FR_NO_PATH, FR_NO_FILE
    char log_path[30] = "";
    strcat(log_path, pSD->pcName);
    strcat(log_path, "/logs");

    fr = f_mkdir(log_path);

    if (fr != FR_OK && fr != FR_EXIST)
    {
        // Couldn't make log dir
        f_unmount(pSD->pcName);
        return 6;
    }


    // Iterate through files in log dir
    DIR log_dir;
    FILINFO f_info;
    uint16_t log_num = 0;

    DBG_PRINT("Checking files in directory \'%s\'\n", log_path);

    fr = f_opendir(&log_dir, log_path);
    
    if (fr != FR_OK)
    {
        // Couldn't open log dir
        f_unmount(pSD->pcName);
        return 7;        
    }

    while (1)
    {
        fr = f_readdir(&log_dir, &f_info);

        if (fr != FR_OK || f_info.fname[0] == 0)
            break;  // Break on error or end of directory


        // If directory item is a FILE
        if ((f_info.fattrib & AM_DIR) == 0)
        {
            char *log_num_str;
            uint8_t prefix_len = strlen(LOG_PREFIX);

            // If the start of the filename matches the logfile prefix
            if (strncmp(LOG_PREFIX, f_info.fname, prefix_len) == 0)
            {
                log_num_str = f_info.fname + prefix_len;
                char *end_char = strchr(log_num_str, '.');

                if (end_char != NULL)
                {
                    *end_char = 0;
                    uint16_t log_file_num = atoi(log_num_str);
                    
                    DBG_PRINT("<LOG n=%d > ", log_file_num); 

                    // Record the log num if it's the highest we've seen
                    if (log_file_num > log_num)
                        log_num = log_file_num;
                    
                }              
            }

            DBG_PRINT("FILE: %s\n", f_info.fname);
        }

    }
    
    DBG_PRINT("The next log number should be %d", log_num + 1);

    f_unmount(pSD->pcName);
    return 0;
}