#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

void lcd_cmd(const uint8_t cmd);
void lcd_data(const uint8_t *data, int len);
void lcd_init();
void spi_init();
void display_update(uint8_t* fb);

#define _BVS(S) (1<<S)           // Bit Value Set (Shift)
#define _MVS(S,V,M) ((V&M)<<S)  // Masked Value Set (Shift, Value, Mask)

#define M_2B 0x03
#define M_3B 0x07
#define M_4B 0x0F
#define M_5B 0x1F
#define M_6B 0x3F
#define M_7B 0xEF
#define M_8B 0xFF

#define SSD1283_CMD_DELAY       0xFF

#define SSD1283_DEV_CODE        0x00
#define SSD1283_OSC_START       0x00
    #define OS_OSCEN _BVS(0) // Enable oscillator

#define SSD1283_DRIVER_OUTPUT   0x01
    #define DO_MUX(V)   _MVS(0,V,M_8B)  // Number of Lines for LCD (8b)
    #define DO_RL       _BVS(8)         // Source driver output shift direction
    #define DO_TB       _BVS(9)         // Gate driver output shift direction
    #define DO_SM       _BVS(10)        // Scanning oder
    #define DO_BGR      _BVS(11)        // RGB or BGR (HW override)
    #define DO_CAD      _BVS(12)        // Retention capacitor POR/Gate (HW override)
    #define DO_REV      _BVS(13)        // Reverse char and graphics display

#define SSD1283_LCD_DRIVE_AC    0x02
    #define LD_NW(V)    _MVS(0,V,M_7B)  // N-line inversion lines (7b)
    #define LD_XOR      _BVS(8)         // Odd/even frame-select and n-line inversion
    #define LD_BC       _BVS(9)         // Frame inversion / n-line inversion
    #define LD_FLD      _BVS(11)        // Display 3-field interlace

#define SSD1283_ENTRY_MODE      0x03
    #define EM_LG(V)    _MVS(0,V,M_3B)  // See compare registers (3b) (Arithmetic)
    #define EM_AM       _BVS(3)         // Auto Address Horizontal/Vertical
        #define AM_HORIZONTAL   0
        #define AM_VERTICAL     1
    #define EM_ID(V)    _MVS(4,V,M_2B)  // Auto Address Increment/Decrement (2b)
        #define ID_HD_VD        0
        #define ID_HI_VD        1
        #define ID_HD_VI        2
        #define ID_HI_VI        3
    #define EM_TY(V)    _MVS(6,V,M_2B)  // 16-bit IF RGB666 Packing (A/B/C)
        #define TY_A_RGBRGB     0
        #define TY_B_RGXB       1
        #define TY_C_RGBX       2
    #define EM_DMODE(V) _MVS(8,V,M_2B)  // Disply from RAM or 'generic input', 1x Reserved
        #define DMODE_RAM               0
        #define DMODE_GENERIC           1
        #define DMODE_WINDOW_RAM        2
        #define DMODE_WINDOW_GENERIC    3
    #define EM_WMODE    _BVS(10)        // Normal data bus / generic interface
    #define EM_OEDEF    _BVS(11)        // DMode=1x (1C/1D defines window) / OE Defines window
    #define EM_TRANS    _BVS(12)        // DMode=1x Transparent display allowed
    #define EM_DFM(V)   _MVS(13,V,M_2B) // 0 = RGB232, 2 = RGB666, 3 = RGB565
        #define DFM_RGB565      3
        #define DFM_RGB666      2
        #define DFM_GS_UNKNOWN  1
        #define DFM_RGB232      0
    #define EM_VSMODE   _BVS(15)        // Frame frequency dependent on VSYNC when DSMODE = 0          

#define SSD1283_COMPARE_1       0x04    // NA in external display interface mode (EM_LG = 000)
    #define C1_CPG(V)   _MVS(2,V,M_6B)  // Compares Green
    #define C1_CPR(V)   _MVS(10,V,M_6B) // Compares Red

#define SSD1283_COMPARE_2       0x05
    #define C1_CPB(V)   _MVS(2,V,M_6B)  // Compares Blue

#define SSD1283_DISPLAY         0x07
    #define DS_D0       _BVS(0)         // D0 = Internal display
    #define DS_D1       _BVS(1)         // D1 = Display On/off
    #define DS_DTE      _BVS(4)         // GON = 1 DTE = 0 VGOFF / GON = 1 DTE = 1 Selected Gate VGH
    #define DS_GON      _BVS(5)         // GON = 0 Gate off VGH
    #define DS_8CM       _BVS(3)        // 8-color mode
    #define DS_SPT      _BVS(8)         // Div/2 LCD drive
    #define DS_VLE1     _BVS(9)         // S1 Vertical scroll from VLE1
    #define DS_VLE2     _BVS(10)        // S2 Vertical scroll from VLE2
    #define DS_PT(V)    _MVS(11,V,M_2B) // Normalise source output of non displayed partial display

// Frame frequency calculated by
// 520khz / (2^(DIV+1) x RTN+16 x (MUX+VBP+VFP+3))
// Default: 520k / (2x28x140) = 66Hz

#define SSD1283_FRAME_CYCLE     0x0B
    #define FC_RTN(V)   _MVS(0,V,M_4B)  // Clocks per line, plus 16
    #define FC_SRTN     _BVS(4)         // Auto RTN when SRTN = 0
    #define FC_SDIV     _BVS(5)         // Auto DIV when SDIV = 0
    #define FC_DIV(V)   _MVS(8,V,M_2B)  // Divide 520khz clock by 2/4/8/16
        #define EQ_DIV2  0
        #define EQ_DIV4  1
        #define EQ_DIV8  2
        #define EQ_DIV16 3
    #define FC_EQ(V)    _MVS(10,V,M_2B) // Equalising period in clocks
        #define EQ_NONE  0
        #define EQ_1_CLK 1
        #define EQ_2_CLK 2
        #define EQ_3_CLK 3
    #define FC_SDT(V)   _MVS(12,V,M_2B) // Gate output signal delay in clocks
        #define SDT_1_CLK 0
        #define SDT_2_CLK 1
        #define SDT_3_CLK 2
        #define SDT_4_CLK 3
    #define FC_NO(V)    _MVS(14,V,M_2B) // Gate non-overlap 0/4/6/8
        #define NO_0_CLK 0
        #define NO_4_CLK 1
        #define NO_6_CLK 2
        #define NO_8_CLK 3

#define SSD1283_POWER_1         0x10
    #define P1_SLP      _BVS(0)         // Driver sleep mode
    #define P1_AP(V)    _MVS(1,V,M_3B)  // Op-amp power least-max
    #define P1_DC(V)    _MVS(4,V,M_2B)  // Step-up clock div 1/2/4/8
        #define DC_FOSC 0
        #define DC_FOSC_DIV2 1
        #define DC_FOSC_DIV4 2
        #define DC_FOSC_DIV8 3
    #define P1_BTH(V)   _MVS(9,V,M_3B)  // Step-up factor for VGH 8/9/10/11/12/13/14/Unreg (Max 12)
        #define BTH_8V  0
        #define BTH_9V  1
        #define BTH_10V 2
        #define BTH_11V 3
        #define BTH_12V 4
    #define P1_DCY(V)   _MVS(12,V,M_3B) // Step-up clock for HVO DCY2 = fline x8/x4/x2/x1 / fosc/ 4/8/16/32
        #define DCY_FLINE_X8    0
        #define DCY_FLINE_X4    1
        #define DCY_FLINE_X2    2
        #define DCY_FLINE       3
        #define DCY_FOSC_DIV4   4
        #define DCY_FOSC_DIV8   5
        #define DCY_FOSC_DIV16  6
        #define DCY_FOSC_DIV32  7
    #define P1_DOT      _BVS(15)        // DCDC uses on-chip / dotclock

#define SSD1283_POWER_2         0x11
    #define P2_PU(V)    _MVS(3,V,M_2B)  // VGH from VCI step-up x3/x4/x5/x6
        #define PU_X3   0
        #define PU_X4   1
        #define PU_X5   2
        #define PU_X6   3

// VRH3 VRH2 VRH1 VRH0 V63SH=0          V63SH=1 
// 0    0    0    0    Vref x 1.330 
// 0    0    0    1    Vref x 1.450 
// 0    0    1    0    Vref x 1.550 
// 0    0    1    1    Vref x 1.650 
// 0    1    0    0    Vref x 1.750 
// 0    1    0    1    Vref x 1.800 
// 0    1    1    0    Vref x 1.850 
// 0    1    1    1    Stopped 
// 1    0    0    0    Vref x 1.900     Vref x 2.03 
// 1    0    0    1    Vref x 2.175     Vref x 2.25 
// 1    0    1    0    Vref x 2.325     Vref x 2.40 
// 1    0    1    1    Vref x 2.475     Vref x 2.55 
// 1    1    0    0    Vref x 2.625 
// 1    1    0    1    Vref x 2.700 
// 1    1    1    0    Vref x 2.775 
// 1    1    1    1    Stopped

#define SSD1283_POWER_3         0x12
    #define P3_VRH(V)   _MVS(4,V,M_4B)  // VLCD63 Amplitude magnification 
    #define P3_V63SH    _BVS(7)         // V63SH 1.33-2.2775
    #define P3_SX263B   _BVS(8)         // Short VCIX2 to VLCD63 for 8-colour mode

#define SSD1283_POWER_4         0x13
    #define P4_VDV(V)   _MVS(8,V,M_5B)  // VLCD63 Amplify from 0.54-1.17 (0.03-step)
    #define P4_VCOMG    _BVS(13)        // Can set at any level / fixed at hi-z

#define SSD1283_POWER_5         0x1E
    #define P5_VCM(V)   _MVS(0,V,M_6B)  // VCOMH 0.36-0.99 x VLCD63
    #define P5_NOTP     _BVS(7)         // VCM valid over VCMR

#define SSD1283_POWER_6         0x1F     // OTP
    #define P6_VCMR(V)   _MVS(0,V,M_6B)  // VCOMH 0.36-0.99 x VLCD63

#define SSD1283_H_PORCH         0x16
    #define HP_HBP(V)   _MVS(0,V,M_6B)  // Dotclock front porch 2-65
    #define HP_XL(V)    _MVS(8,V,M_8B)  // Number of valid pixels per line 1-132

#define SSD1283_V_PORCH         0x17
    #define HP_VBP(V)   _MVS(0,V,M_7B)  // Dotclock back porch 1-128
    #define HP_VFP(V)   _MVS(8,V,M_7B)  // Dotclock front porch 1-128

#define SSD1283_RAM_ADDR        0x21    // Set access address (15b)
#define SSD1283_RAM_DATA        0x22    // Read/Write to current address (17b)

#define SSD1283_RAM_WR_MASK_1   0x23    
    #define M1_WMG(V)   _MVS(2,V,M_6B)  // Write mask (Green)
    #define M1_WMR(V)   _MVS(10,V,M_6B) // Write mask (Red)

#define SSD1283_RAM_WR_MASK_2   0x24
    #define M2_WMB(V)   _MVS(2,V,M_6B)  // Write mask (Blue)

#define SSD1283_OTP_VCOM_1      0x28
#define SSD1283_OTP_VCOM_2      0x29
#define SSD1283_GAMMA_1         0x30
#define SSD1283_GAMMA_2         0x31
#define SSD1283_GAMMA_3         0x32
#define SSD1283_GAMMA_4         0x33
#define SSD1283_GAMMA_5         0x34
#define SSD1283_GAMMA_6         0x35
#define SSD1283_GAMMA_7         0x36
#define SSD1283_GAMMA_8         0x37
#define SSD1283_GAMMA_9         0x38
#define SSD1283_GAMMA_10        0x39

#define SSD1283_GATE_SCAN_START 0x40
    #define GS_GSP(V)   _MVS(0,V,M_8B)  // Scan start (0-131)

#define SSD1283_V_SCROLL        0x41
    #define VS_VL1(V)   _MVS(0,V,M_8B)  // Scroll length for 1st screen
    #define VS_VL2(V)   _MVS(8,V,M_8B)  // Scroll length for 2nd screen

#define SSD1283_1ST_DRIVE_POS   0x42
    #define FD_SS1(V)   _MVS(0,V,M_8B)  // Drive start for 1st screen
    #define FD_SE1(V)   _MVS(8,V,M_8B)  // Drive end for 1st screen

#define SSD1283_2ND_DRIVE_POS   0x43
    #define SD_SS2(V)   _MVS(0,V,M_8B)  // Drive start for 2st screen
    #define SD_SE2(V)   _MVS(8,V,M_8B)  // Drive end for 2st screen

#define SSD1283_H_POS           0x44
    #define HP_HSA(V)   _MVS(0,V,M_8B)  // Horizontal Start
    #define HP_HEA(V)   _MVS(8,V,M_8B)  // Horizontal End

#define SSD1283_V_POS           0x45
    #define VP_VSA(V)   _MVS(0,V,M_8B)  // Vertical Start
    #define VP_VEA(V)   _MVS(8,V,M_8B)  // Vertical End

#define SSD1283_FURTHER_BIAS    0x27
    #define FB_CS(V)    _MVS(3,V,M_3B)  // Further bias least-maximum    

#define SSD1283_OSC_FREQ        0x2C
    #define OF_OSCR(V)  _MVS(12,V,M_4B)  // Further bias least-maximum

#define SSD1283_XS 130
#define SSD1283_YS 130

/*static const uint16_t SSD1283A_Init[] = {
    SSD1283_POWER_1,        0x2F8E,//(P1_DOT | P1_DCY(DCY_FLINE) | P1_BTH(BTH_12V) | P1_AP(6)),            
    SSD1283_POWER_2,        0x000C,//(P2_PU(PU_X6)),  
    SSD1283_DISPLAY,        (DS_GON | DS_D0),      
    SSD1283_FURTHER_BIAS,   0x0057F,//(FB_CS(6)),      
    SSD1283_OSC_START,      (OS_OSCEN),     
    SSD1283_CMD_DELAY, 100, 
    SSD1283_DISPLAY,        (DS_VLE1 | DS_GON | DS_D1 | DS_D0),
    SSD1283_CMD_DELAY, 30,  
    SSD1283_DISPLAY,        (DS_VLE1 | DS_GON | DS_DTE | DS_D1 | DS_D0),
    SSD1283_DRIVER_OUTPUT,  (DO_REV | DO_RL | DO_MUX(131)),
    SSD1283_ENTRY_MODE,     (EM_DFM(DFM_RGB565) | EM_OEDEF | EM_ID(ID_HI_VI)),
    0x2F, 0xFFFF,
    SSD1283_OSC_FREQ,       (OF_OSCR(8)), // 520khz
    SSD1283_LCD_DRIVE_AC,   (LD_BC | LD_XOR),
    SSD1283_FRAME_CYCLE,    0x580C,//(FC_NO(NO_4_CLK) | (SDT_2_CLK) | FC_EQ(EQ_2_CLK) | FC_RTN(12)),
    SSD1283_POWER_3,        0x0609,//(P3_VRH(9)),
    SSD1283_POWER_4,        (P4_VCOMG | P4_VDV(17)), 
};*/