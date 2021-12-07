#include <string.h>
#include <math.h>

#include "map.h"
#include "parse.h"
#include "way.h"
#include "memory.h"
#include <unistd.h>

#include "hagl.h"
#include "hagl_hal.h"
#include "thick.h"
#include "rgb332.h"
#include "aa.h"

#include <esp_log.h>

#define MAPSFORGE_MAGIC_STRING "mapsforge binary OSM"

static const char *TAG = "map";

int long2tilex(double lon, int z) { 
	return (int)(floor((lon + 180.0) / 360.0 * (1 << z))); 
}

int lat2tiley(double lat, int z) { 
    double latrad = lat * M_PI/180.0;
	return (int)(floor((1.0 - asinh(tan(latrad)) / M_PI) / 2.0 * (1 << z))); 
}

float tilex2long(int x, int z) 
{
	return x / (float)(1 << z) * 360.0 - 180;
}

float tiley2lat(int y, int z) 
{
	float n = M_PI - 2.0 * M_PI * y / (float)(1 << z);
	return 180.0 / M_PI * atan(0.5 * (exp(n) - exp(-n)));
}

void g_draw_way(way_prop * way, color_t colour, uint8_t layer, int16_t xo, int16_t yo, float rot, uint16_t size) {

    float cos_pre = cosf(rot);
    float sin_pre = sinf(rot);

    if(way->data[0].block[0].nodes > 1) {
      
        color_t cl = rgb332(0,0,0);
        uint8_t th = 1;

        for(uint8_t t = 0; t < way->n_tags; t++) {
            switch(((uint8_t*)way->tag_ids)[t]) {
                case 26: // Pedestrian
                case 13: // Steps
                    cl = rgb332(0xE5,0xE0,0xC2);
                    th = 1;
                    goto tag_found;
                case 3: // Footway
                case 4: // Path
                    cl = rgb332(0xAA,0x00,0x00);
                    th = 1;
                    goto tag_found;
                case 2: // Track
                    cl = rgb332(0xFF,0xFA,0xF2);
                    th = 1;
                    goto tag_found;                    
                case 14: // Cycleway
                    cl = rgb332(0xFF,0xF2,0xDE);
                    th = 1;
                    goto tag_found;
                case 32: // Bridleway
                    cl = rgb332(0xD3,0xCB,0x98);
                    th = 1;
                    goto tag_found;           
                case 0: // Service
                    cl = rgb332(0xFF,0xFF,0xFF);
                    th = 1;
                    goto tag_found;          
                case 28: // Construction
                    cl = rgb332(0xD0,0xD0,0xD0);
                    th = 1;
                    goto tag_found;    
                case 64: // Road
                    cl = rgb332(0xD0,0xD0,0xD0);
                    th = 2;
                    goto tag_found;              
                case 1: // Residential
                case 6: // Unclassified
                case 30: // Living Street
                    cl = rgb332(0xFF,0xFF,0xFF);
                    th = 2;
                    goto tag_found;         
                case 8:  // Tertiary
                case 35: // Tertiary Link
                    cl = rgb332(0xFF,0xFF,0x90);
                    th = 3;
                    goto tag_found;    
                case 12: // Secondary
                case 34: // Secondary Link
                    cl = rgb332(0xBB,0x85,0x0F);
                    th = 3;
                    goto tag_found;
                case 7: // Primary
                    cl = rgb332(0xFE,0x85,0x0C);
                    th = 4;
                    goto tag_found;
                case 27: // Primary Link
                    cl = rgb332(0xFE,0x85,0x0C);
                    th = 3;
                    goto tag_found;   
                case 11: // Trunk
                    cl = rgb332(0x80,0x00,0x40);
                    th = 4;
                    goto tag_found;
                case 24: // Trunk Link
                    cl = rgb332(0x80,0x00,0x40);
                    th = 3;
                    goto tag_found;    
                case 21: // Motorway
                    cl = rgb332(0x40,0x00,0x00);
                    th = 3;
                    goto tag_found;
                case 23: // Motorway Link
                    cl = rgb332(0x40,0x00,0x00);
                    th = 3;
                    goto tag_found;  
                //default:
                    ////ESP_LOGI(TAG,"Tag %d Not Found: %hu\n", t, way->tag_ids[t]);
            } 
        }
        tag_found:

        for(int i = 0; i < (way->data[0].block[0].nodes-1); i++) {
            int16_t xt0 = xo+way->data[0].block[0].coords[i].x-DISPLAY_WIDTH/2;
            int16_t yt0 = yo+way->data[0].block[0].coords[i].y-DISPLAY_HEIGHT/2;

            int16_t xt1 = xo+way->data[0].block[0].coords[i+1].x-DISPLAY_WIDTH/2;
            int16_t yt1 = yo+way->data[0].block[0].coords[i+1].y-DISPLAY_HEIGHT/2;

            if(cl != 0) draw_varthick_line( xt0*cos_pre-yt0*sin_pre+DISPLAY_WIDTH/2, 
                                            yt0*cos_pre+xt0*sin_pre+DISPLAY_HEIGHT/2,
                                            xt1*cos_pre-yt1*sin_pre+DISPLAY_WIDTH/2, 
                                            yt1*cos_pre+xt1*sin_pre+DISPLAY_HEIGHT/2, th, cl);

            /*if(cl != 0) hagl_draw_line( xt0*cos_pre-yt0*sin_pre+DISPLAY_WIDTH/2, 
                                            yt0*cos_pre+xt0*sin_pre+DISPLAY_HEIGHT/2,
                                            xt1*cos_pre-yt1*sin_pre+DISPLAY_WIDTH/2, 
                                            yt1*cos_pre+xt1*sin_pre+DISPLAY_HEIGHT/2, cl);*/

            /*if(cl != 0) hagl_draw_line(way->data[0].block[0].coords[i].x, 
                                       way->data[0].block[0].coords[i].y,
                                       way->data[0].block[0].coords[i+1].x,
                                       way->data[0].block[0].coords[i+1].y, cl);*/
        }
    }   
}

int load_map(arena_t* a0, char* filename, way_prop** way_list_ptr, uint32_t x_in, uint32_t y_in, uint32_t z_in, int16_t xo, int16_t yo, uint16_t st, float rot, float size) {
    fb_handler fbh;
    if(init_buffer(&fbh, filename)) {
        //ESP_LOGI(TAG,"Failed to init file buffer\n\r");
        return 1;
    }

    ////ESP_LOGI(TAG,"Read in %d Bytes\n\r", fbh.bytes_read);

    if(memcmp(fbh.buffer_ptr, MAPSFORGE_MAGIC_STRING, 20)) {
        //ESP_LOGI(TAG,"Not a valid .MAP file!\n\r");
        return -1;
    } else {
        ////ESP_LOGI(TAG,"Valid .MAP: %s\n\r", fbh.buffer_ptr);
    }

    fbh.buffer_pos += 20;

    mapsforge_file_header hdr;

    hdr.header_size = get_uint32(&fbh);
    //ESP_LOGI(TAG,"Header Size:%d\n\r", hdr.header_size);
    hdr.file_version = get_uint32(&fbh);
    //ESP_LOGI(TAG,"File Version:%d\n\r", hdr.file_version);
    hdr.file_size = get_uint64(&fbh);
    //ESP_LOGI(TAG,"File Size:%uMB\n\r", (uint8_t)(hdr.file_size/1000000));
    hdr.file_creation = get_int64(&fbh);
    //ESP_LOGI(TAG,"File Created:%llu\n\r", (uint64_t)hdr.file_creation/1000);
    hdr.bounding_box[0] = get_int32(&fbh);
    hdr.bounding_box[1] = get_int32(&fbh);
    hdr.bounding_box[2] = get_int32(&fbh);
    hdr.bounding_box[3] = get_int32(&fbh);
    //ESP_LOGI(TAG,"Bounding Box:\n\r");
    //ESP_LOGI(TAG,"\t[0]:%7.3f\n\r\t[1]:%7.3f\n\r\t[2]:%7.3f\n\r\t[3]:%7.3f\n\r", (float)hdr.bounding_box[0]/1000000, (float)hdr.bounding_box[1]/1000000, (float)hdr.bounding_box[2]/1000000, (float)hdr.bounding_box[3]/1000000);
    hdr.tile_size = get_uint16(&fbh);
    //ESP_LOGI(TAG,"Tile Size:\t%hhu\n\r", hdr.tile_size);

    uint8_t str_len = get_uint8(&fbh);
    get_string(&fbh, hdr.projection,str_len);
    //ESP_LOGI(TAG,"Projection:\t%s\n\r", hdr.projection);

    hdr.flags = get_uint8(&fbh);

    if(hdr.flags & 0x40) {
        hdr.init_lat_long[0] = get_uint32(&fbh);
        hdr.init_lat_long[1] = get_uint32(&fbh);
        //ESP_LOGI(TAG,"Start Position:\n\r");
        //ESP_LOGI(TAG,"\t[0]:%7.3f\n\r\t[1]:%7.3f\n\r", (float)hdr.init_lat_long[0]/1000000, (float)hdr.init_lat_long[1]/1000000);
    } else {
        hdr.init_lat_long[0] = 0;
        hdr.init_lat_long[1] = 0;
    }

    if(hdr.flags & 0x20) {
        hdr.init_zoom = get_uint8(&fbh);    
        //ESP_LOGI(TAG,"Start Zoom:\t%u\n\r", hdr.init_zoom);
    } else {
        hdr.init_zoom = 0;
    }

    if(hdr.flags & 0x10) {
        str_len = get_uint8(&fbh);
        get_string(&fbh, hdr.lang_pref,str_len);
        //ESP_LOGI(TAG,"Language:\t%s\n\r", hdr.lang_pref); 
    } else {
        hdr.lang_pref[0] = '\0';
    }

    if(hdr.flags & 0x08) {
        str_len = get_uint8(&fbh);
        get_string(&fbh, hdr.comment, str_len);    
        //ESP_LOGI(TAG,"Comment:\t%s\n\r", fbh.buffer_pos, hdr.comment);
    } else {
        hdr.comment[0] = '\0';
    }

    if(hdr.flags & 0x04) {
        str_len = get_uint8(&fbh);
        get_string(&fbh, hdr.created_by, str_len);
        //ESP_LOGI(TAG,"Created By:\t%s\n\n\r", hdr.created_by);
    } else {
        hdr.created_by[0] = '\0';
    }

    hdr.n_poi_tags = get_uint16(&fbh);

    //ESP_LOGI(TAG,"# of POI Tags:\t%hhu\n\r", hdr.n_poi_tags);

    for(int poi_id = 0; poi_id < hdr.n_poi_tags; poi_id++) {
        str_len = get_uint8(&fbh);
        get_string(&fbh, hdr.poi_tag_names[poi_id], str_len);
        //ESP_LOGI(TAG,"\t[%d]: %d : %s\n", poi_id, str_len, hdr.poi_tag_names[poi_id]);
    }

    hdr.n_way_tags = get_uint16(&fbh);

    //ESP_LOGI(TAG,"# of Way Tags:\t%d\n\r", hdr.n_way_tags);
 
    for(int way_id = 0; way_id < hdr.n_way_tags; way_id++) {
        str_len = get_uint8(&fbh);
        get_string(&fbh, hdr.way_tag_names[way_id], str_len);
        //ESP_LOGI(TAG,"\t[%d]: %s\n\r", way_id, hdr.way_tag_names[way_id]);
    }

    hdr.n_zoom_intervals = get_uint8(&fbh);

    //ESP_LOGI(TAG,"\n# Zoom Intervals:\t%u\n\r", hdr.n_zoom_intervals);

    for(int zoom_id = 0; zoom_id < hdr.n_zoom_intervals; zoom_id++) {
        hdr.zoom_conf[zoom_id].base_zoom = get_uint8(&fbh);
        hdr.zoom_conf[zoom_id].min_zoom = get_uint8(&fbh);
        hdr.zoom_conf[zoom_id].max_zoom = get_uint8(&fbh);
        hdr.zoom_conf[zoom_id].sub_file = get_uint64(&fbh);
        hdr.zoom_conf[zoom_id].sub_file_size = get_uint64(&fbh);

        hdr.zoom_conf[zoom_id].n_tiles_x = (long2tilex(((double)hdr.bounding_box[3])/1000000, hdr.zoom_conf[zoom_id].base_zoom) - long2tilex(((double)hdr.bounding_box[1])/1000000, hdr.zoom_conf[zoom_id].base_zoom)) + 1;
        hdr.zoom_conf[zoom_id].n_tiles_y = (lat2tiley(((double)hdr.bounding_box[0])/1000000, hdr.zoom_conf[zoom_id].base_zoom) - lat2tiley(((double)hdr.bounding_box[2])/1000000, hdr.zoom_conf[zoom_id].base_zoom)) + 1;

        //ESP_LOGI(TAG,"Zoom Interval [%d]:\n\r", zoom_id);
        //ESP_LOGI(TAG,"\tBase Zoom: %u\n\r", hdr.zoom_conf[zoom_id].base_zoom);
        //ESP_LOGI(TAG,"\tMax Zoom: %u\n\r", hdr.zoom_conf[zoom_id].max_zoom);
        //ESP_LOGI(TAG,"\tMin Zoom: %u\n\r", hdr.zoom_conf[zoom_id].min_zoom);
        //ESP_LOGI(TAG,"\tSub-file Start: %llu\n\r", hdr.zoom_conf[zoom_id].sub_file);
        //ESP_LOGI(TAG,"\tSub-file Size: %fMB\n\r", (float)hdr.zoom_conf[zoom_id].sub_file_size/1000000);
        //ESP_LOGI(TAG,"\t# of Tiles in X: %u\n\r", hdr.zoom_conf[zoom_id].n_tiles_x);
        //ESP_LOGI(TAG,"\t# of Tiles in Y: %u\n\r", hdr.zoom_conf[zoom_id].n_tiles_y);
        //ESP_LOGI(TAG,"\t# of Tiles: %u\n\r", hdr.zoom_conf[zoom_id].n_tiles_y * hdr.zoom_conf[zoom_id].n_tiles_x );
        //ESP_LOGI(TAG,"\tOSM Base Tile Origin: %d/%d/%d\n\r", hdr.zoom_conf[zoom_id].base_zoom, long2tilex(((double)hdr.bounding_box[1])/1000000, hdr.zoom_conf[zoom_id].base_zoom), lat2tiley(((double)hdr.bounding_box[2])/1000000, hdr.zoom_conf[zoom_id].base_zoom));
    }
    
    int z_ds;
    for(z_ds = 0; z_ds < hdr.n_zoom_intervals; z_ds++)
        if((z_in > hdr.zoom_conf[z_ds].min_zoom) && \
           (z_in < hdr.zoom_conf[z_ds].max_zoom)) break;

    //ESP_LOGI(TAG,"Zoom Interval:%d\n\r", z_ds);    

    uint32_t x_ds = x_in - long2tilex(((double) hdr.bounding_box[1])/1000000, hdr.zoom_conf[z_ds].base_zoom);
    uint32_t y_ds = y_in - lat2tiley(((double) hdr.bounding_box[2])/1000000, hdr.zoom_conf[z_ds].base_zoom);

    uint32_t t_lookup = ((y_ds*hdr.zoom_conf[z_ds].n_tiles_x) + x_ds);

    const uint64_t addr_mask =  0x7fffffffffULL;
    const uint64_t water_mask = 0x8000000000ULL;

    file_seek(&fbh, hdr.zoom_conf[z_ds].sub_file+(t_lookup*5));
    uint64_t addr_lookup = get_varint(&fbh, 5);
    uint64_t offset_lookup = addr_lookup & addr_mask;

    if(addr_lookup & water_mask) {
      //ESP_LOGI(TAG,"Only Water\n");
      //ESP_LOGI(TAG,"Arena: %d/%d\n\r", arena_free(&a0), ARENA_DEFAULT_SIZE);
      file_close(&fbh);
      return 0;
    }
        
    //ESP_LOGI(TAG,"%u/%d/%d -> %lu, %llu, %llu\n\r", hdr.zoom_conf[z_ds].base_zoom, x_in, y_in, t_lookup, offset_lookup, addr_lookup&water_mask);
    
    file_seek(&fbh, hdr.zoom_conf[z_ds].sub_file+offset_lookup);

    uint16_t pois[22] = {0};
    uint16_t ways[22] = {0};

    //ESP_LOGI(TAG,"Z\tPOIs\tWays\n\r");
    for(int z = hdr.zoom_conf[z_ds].min_zoom; z <= hdr.zoom_conf[z_ds].max_zoom; z++) {
        pois[z] = get_vbe_uint(&fbh);
        ways[z] = get_vbe_uint(&fbh);
        //ESP_LOGI(TAG,"%d\t%d\t%d\n\r", z, pois[z], ways[z]);
    }
    //ESP_LOGI(TAG,"Zoom Table End\n\r");
    
    uint32_t first_way_offset = get_vbe_uint(&fbh);
    uint32_t first_way_file_addr = hdr.zoom_conf[z_ds].sub_file + \
                                         offset_lookup + \
                                         fbh.buffer_pos + \
                                         first_way_offset;

    //ESP_LOGI(TAG,"First Way Offset: %lu - %lu\n\r", first_way_offset, first_way_file_addr);
    file_seek(&fbh, first_way_file_addr);                                   

    int ways_to_draw = ways[12];//+ways[13]+ways[14]+ways[15];
    *way_list_ptr = arena_malloc(a0, sizeof(way_prop)*ways_to_draw);
    uint32_t way_size = 0;
    
    double lon = tilex2long(x_in,z_in);
    double lat = tiley2lat(y_in,z_in);
  
    double lon1 = tilex2long(x_in+1,z_in);
    double lat1 = tiley2lat(y_in+1,z_in);
  
    double londiff = fabs(lon1-lon);
    double latdiff = fabs(lat1-lat);
  
    //ESP_LOGI(TAG,"tile   origin: %f, %f\n", lat, lon);
    //ESP_LOGI(TAG,"tile + origin: %f, %f\n", lat1, lon1);
    //ESP_LOGI(TAG,"tile differences: %f, %f\n", fabs(lat1-lat), fabs(lon1-lon));
    
    int x_pix = lon_to_x(londiff*1000000, 1);
    int y_pix = lat_to_y(latdiff*1000000, 1);
    float x_mercator = ((float)x_pix/y_pix);
    float fit_scale = y_pix/size;
    int x_fit = lon_to_x(londiff*1000000, x_mercator*fit_scale);
    int y_fit = lat_to_y(latdiff*1000000, fit_scale);
    
    //ESP_LOGI(TAG,"scale diff to: %d, %d\n", y_pix, x_pix);
    //ESP_LOGI(TAG,"scale factors: %f, %f (%f)\n", fit_scale, x_mercator*fit_scale, x_mercator);
    //ESP_LOGI(TAG,"fit diff tile: %d, %d\n", y_fit, x_fit);
      
    for(int w = 0; w < ways_to_draw; w++) {
        uint8_t rtn = get_way(*way_list_ptr+w,&fbh,a0, st, fit_scale, x_mercator);
        if(rtn) { // Ignore way
          if(w > 0) w--; 
          ways_to_draw--;  
        }
    }

    file_close(&fbh);

    /*for(int w = 0; w < ways_to_draw; w++) {
        if(st & testway[w].subtile_bitmap)
            g_draw_way(&testway[w], 0, testway[w].tag_ids[0], xo+DISPLAY_WIDTH/2, yo+DISPLAY_HEIGHT/2, rot, size);
    }*/

    //ESP_LOGI(TAG,"Size of Ways: %d\n\r", way_size);
    
    //ESP_LOGI(TAG,"Arena: %d/%d\n\r", arena_free(&a0), ARENA_DEFAULT_SIZE);



    return ways_to_draw;
}
