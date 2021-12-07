#include "spi_bus.h"
#include "driver/spi_common.h"

volatile spi_bus_config_t buscfg;

void spi_bus_init() {
    esp_err_t ret;
    buscfg.sclk_io_num=PIN_NUM_CLK;
    buscfg.mosi_io_num=PIN_NUM_MOSI;
    buscfg.miso_io_num=PIN_NUM_MISO;
    buscfg.quadwp_io_num=-1;
    buscfg.quadhd_io_num=-1;
    buscfg.max_transfer_sz=(128*128);
    buscfg.flags=SPICOMMON_BUSFLAG_IOMUX_PINS|SPICOMMON_BUSFLAG_NATIVE_PINS|SPICOMMON_BUSFLAG_SCLK|SPICOMMON_BUSFLAG_MOSI;

    //Initialize the SPI bus
    ret=spi_bus_initialize(SPI_HOST, &buscfg, DMA_CHAN);
    ESP_ERROR_CHECK(ret);
}