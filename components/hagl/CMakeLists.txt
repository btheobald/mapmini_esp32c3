idf_component_register(
    SRCS "src/bitmap.c" "src/clip.c" "src/fontx.c" "src/hagl.c"  "src/hsl.c"  "src/rgb565.c"  "src/rgb888.c"  "src/tjpgd.c"  "src/hagl_hal.c" "src/ssd1283a.c" "src/thick.c" "src/aa.c"
    INCLUDE_DIRS "./include" "../../main" 
)
add_definitions("-DHAGL_INCLUDE_SDKCONFIG_H")