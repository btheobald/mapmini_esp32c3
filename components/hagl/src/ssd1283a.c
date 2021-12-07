#include "spi_bus.h"
#include "ssd1283a.h"
#include <stdlib.h>
#include <string.h>

static const uint16_t SSD1283A_Init[] = {
    // Power Configuration
    SSD1283_POWER_1,            __bswap16(P1_DOT | P1_DCY(DCY_FOSC_DIV32) | P1_BTH(BTH_12V) | P1_AP(6)),            
    SSD1283_POWER_2,            __bswap16(P2_PU(PU_X6)),  
    SSD1283_POWER_3,            __bswap16(P3_VRH(9)),
    SSD1283_POWER_4,            __bswap16(P4_VCOMG | P4_VDV(17)),    
    SSD1283_OTP_VCOM_1,         __bswap16(0x0006),
    SSD1283_OTP_VCOM_1,         __bswap16(0x0001),
    SSD1283_OTP_VCOM_2,         __bswap16(0xFFFE),
    SSD1283_FURTHER_BIAS,       __bswap16(0x0570), 
    
    // Timing
    SSD1283_OSC_FREQ,           __bswap16(OF_OSCR(8)), // 520khz
    SSD1283_H_PORCH,            __bswap16(HP_XL(129) | HP_HBP(4)),
    SSD1283_DRIVER_OUTPUT,      __bswap16(DO_MUX(129) | DO_REV),
    SSD1283_GATE_SCAN_START,    __bswap16(GS_GSP(2)),
    SSD1283_FRAME_CYCLE,        __bswap16(FC_SRTN | FC_SDIV),

    // Display Mode
    SSD1283_ENTRY_MODE,         __bswap16(EM_DFM(DFM_RGB232) | EM_OEDEF | EM_ID(ID_HI_VI)),
    SSD1283_DISPLAY,            __bswap16(DS_GON | DS_DTE | DS_D1 | DS_D0 | DS_8CM),
    
    // Display Enable
    SSD1283_OSC_START,          __bswap16(OS_OSCEN),     
};

static const uint16_t SSD1283A_Fill[] = {
    SSD1283_H_POS, __bswap16(HP_HSA(2) | HP_HEA(131)),
    SSD1283_V_POS, __bswap16(VP_VSA(0) | VP_VEA(129)),
    SSD1283_RAM_ADDR, __bswap16(2),
};

spi_device_handle_t spi_lcd;
spi_device_interface_config_t devcfg;

void lcd_cmd(const uint8_t cmd)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(spi_transaction_t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&cmd;               //The data is the cmd itself
    gpio_set_level( PIN_NUM_DC, 0 );
    ret=spi_device_polling_transmit(spi_lcd, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void lcd_data(const uint8_t *data, int len)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;             //no need to send anything
    memset(&t, 0, sizeof(spi_transaction_t));       //Zero out the transaction
    t.length=len*8;                 //Len is in bytes, transaction length is in bits.
    t.tx_buffer=data;               //Data
    gpio_set_level( PIN_NUM_DC, 1 );
    ret=spi_device_transmit(spi_lcd, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

//Initialize the display
void lcd_init()
{   
    esp_err_t ret;
    devcfg.clock_speed_hz=40000000;           //Clock out at 40 MHz
    devcfg.mode=2;                            //SPI mode 0
    devcfg.spics_io_num=PIN_NUM_CS;           //CS pin
    devcfg.queue_size=1;                      //We want to be able to queue 7 transactions at a time

    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(SPI_HOST, &devcfg, &spi_lcd);
    ESP_ERROR_CHECK(ret);  

    gpio_reset_pin( PIN_NUM_DC );
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_level( PIN_NUM_DC, 0 );

    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);

    for(int cmd = 0; cmd < sizeof(SSD1283A_Init)/2; cmd+=2) {
        lcd_cmd(SSD1283A_Init[cmd]);
        lcd_data((uint8_t*)&SSD1283A_Init[cmd+1], 2);
    }

    for(int cmd = 0; cmd < sizeof(SSD1283A_Fill)/2; cmd+=2) {
        lcd_cmd(SSD1283A_Fill[cmd]);
        lcd_data((uint8_t*)&SSD1283A_Fill[cmd+1], 2);
    }

    gpio_set_level(PIN_NUM_BCKL, 1);
}

void display_update(uint8_t* fb) {
    lcd_cmd(SSD1283_RAM_DATA);
    lcd_data(fb, SSD1283_XS*SSD1283_YS);
}