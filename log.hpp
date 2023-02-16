
//#define DBG_PRINT (fmt, args...)
#define DBG_PRINT printf


#define MIN_FREE_MB 100     // If the SD has less free space than this, the user should be warned
#define LOG_PREFIX "log_"   // Numbered log filename prefix


uint8_t get_next_log_name(char *filename);
uint8_t init_filesystem();