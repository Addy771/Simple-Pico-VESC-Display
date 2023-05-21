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

#include "bitmap/sd.h"
#include "bitmap/noSD.h"
#include "bitmap/sd_write.h"


extern pico_oled display;
extern Debounce debouncer;

uint8_t sd_status = SD_NOT_PRESENT;

const char csv_head[] = "ms_today,input_voltage,temp_mos_max,temp_mos_1,temp_mos_2,temp_mos_3,"
                        "temp_motor,current_motor,current_in,d_axis_current,q_axis_current,"
                        "erpm,duty_cycle,amp_hours_used,amp_hours_charged,watt_hours_used,"
                        "watt_hours_charged,tachometer,tachometer_abs,encoder_position,fault_code,"
                        "vesc_id,d_axis_voltage,q_axis_voltage,input_power,speed_kph,adc1_decoded,adc2_decoded,\n";

char log_filename[30] = "";

log_data_t data_pt;


FRESULT init_filesystem()
{
    sd_card_t *pSD = sd_get_by_num(0);

    sd_status = SD_NOT_PRESENT; // Status will be updated if init is successful, otherwise it remains not present

    // Try to mount SD card
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);

    // display.print(FRESULT_str(fr));

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

    log_num++;
    DBG_PRINT("The next log number should be %d\n", log_num);

    // Put together log filename if we didn't do this before
    if (strlen(log_filename) == 0)
    {
        strcat(log_filename, log_path);
        strcat(log_filename, "/");
        strcat(log_filename, LOG_PREFIX);
        itoa(log_num, (char *) (log_filename + strlen(log_filename)), 10);
        strcat(log_filename, ".csv");
    }

    DBG_PRINT("Filename: %s\n", log_filename);

    f_unmount(pSD->pcName);

    sd_status = SD_PRESENT;
    return 0;
}


/// @brief Create next log file and set the CSV columns
/// @return FRESULT of file operations
FRESULT create_log_file()
{
    sd_card_t *pSD = sd_get_by_num(0);
    FIL log_fil;

    // Try to mount SD card
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);    

    if (fr != FR_OK)
    {
        sd_status = SD_ERROR;
        DBG_PRINT("SD mount failed during write attempt!\n%s\n", FRESULT_str(fr));
        return fr;
    }

    fr = f_open(&log_fil, log_filename, FA_WRITE | FA_CREATE_ALWAYS);

    if (fr != FR_OK)
    {
        sd_status = SD_ERROR;
        DBG_PRINT("SD open failed during write attempt!\n%s\n", FRESULT_str(fr));
        return fr;
    }    

    uint32_t bytes_written;

    fr = f_write(&log_fil, csv_head, strlen(csv_head), &bytes_written);

    if (fr != FR_OK || bytes_written != strlen(csv_head))
    {
        sd_status = SD_ERROR;
        DBG_PRINT("SD write failed!\n%s\n", FRESULT_str(fr));
        DBG_PRINT("%d/%d bytes written.\n", bytes_written, sizeof(csv_head));
        return fr;
    }        

    f_close(&log_fil);
    f_unmount(pSD->pcName);

    return 0;

}


/// @brief 
/// @return 
FRESULT append_data_pt()
{
    sd_card_t *pSD = sd_get_by_num(0);
    FIL log_fil;

    // Try to mount SD card
    FRESULT fr = f_mount(&pSD->fatfs, pSD->pcName, 1);    

    if (fr != FR_OK)
    {
        sd_status = SD_ERROR;
        DBG_PRINT("f_mount() failed during write attempt!\n%s\n", FRESULT_str(fr));
        return fr;
    }

    fr = f_open(&log_fil, log_filename, FA_WRITE | FA_OPEN_APPEND);

    if (fr != FR_OK)
    {
        sd_status = SD_ERROR;
        DBG_PRINT("f_open() failed during write attempt!\n%s\n", FRESULT_str(fr));
        return fr;
    }    

    // Write out one row worth of data
    uint16_t bytes_written;
    f_printf(&log_fil, "%d,%f,%f,%f,%f,%f,", data_pt.ms_today, data_pt.v_in, data_pt.temp_mos, data_pt.temp_mos_1, data_pt.temp_mos_2, data_pt.temp_mos_3);
    f_printf(&log_fil, "%f,%f,%f,%f,%f,", data_pt.temp_motor, data_pt.current_motor, data_pt.current_in, data_pt.id, data_pt.iq);
    f_printf(&log_fil, "%f,%f,%f,%f,%f,", data_pt.rpm, data_pt.duty_now, data_pt.amp_hours, data_pt.amp_hours_charged, data_pt.watt_hours);
    f_printf(&log_fil, "%f,%d,%d,%f,%d,", data_pt.watt_hours_charged, data_pt.tachometer, data_pt.tachometer_abs, data_pt.position, data_pt.fault_code);
    bytes_written = f_printf(&log_fil, "%d,%f,%f,%f,%f,%f,%f,\n", data_pt.vesc_id, data_pt.vd, data_pt.vq, data_pt.p_in, data_pt.speed_kph, data_pt.adc1_decoded, data_pt.adc2_decoded);

    if (bytes_written < 0)
    {
        sd_status = SD_ERROR;
        DBG_PRINT("f_printf() failed during write attempt!\n");
        return FR_DISK_ERR;
    }    


    f_close(&log_fil);
    f_unmount(pSD->pcName);

    return 0;
}


/// @brief Draw the SD icon according to the SD status
/// @param x screen x coordinate to draw at
/// @param y screen y coordinate to draw at
void draw_SD_status(uint8_t x, uint8_t y)
{
    switch(sd_status)
    {
        case SD_NOT_PRESENT:
        case SD_ERROR:
            display.draw_bmp(no_sd_bitmap, no_sd_width, no_sd_height, x, y);
            break;

        case SD_PRESENT:
            display.draw_bmp(sd_bitmap, sd_width, sd_height, x, y);
            break;

        case SD_WRITING:
            display.draw_bmp(sd_wr_bitmap, sd_wr_width, sd_wr_height, x, y);
            break;
    }
}