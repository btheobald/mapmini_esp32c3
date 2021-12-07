#include "io_posix.h"
#include <esp_log.h>

static const char *TAG = "io";

int init_buffer(fb_handler * fbh, char * filename) {
    fbh->buffer_pos = 0;
    fbh->bytes_read = 0;

    fbh->fp = fopen(filename, "rb");
    if(fbh->fp == NULL) {
        ESP_LOGI(TAG, "Couldn't open %s", filename);

        return 1;
    }

    load_buffer(fbh); 

    return 0;
}

void load_buffer(fb_handler * fbh) {
    fbh->buffer_pos = 0; // Reset buffer offset
    fbh->bytes_read = fread(fbh->buffer_ptr, 1, FILE_READ_BUFFER_SIZE, fbh->fp);
}

uint16_t get_remaining_bytes(fb_handler * fbh) {
    return (fbh->bytes_read - fbh->buffer_pos);
}

void relative_reset_buffer(fb_handler * fbh, uint16_t seek) {
    fbh->buffer_pos = 0;
    fseek(fbh->fp, -seek, SEEK_CUR);
    load_buffer(fbh);
}