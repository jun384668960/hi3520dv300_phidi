/*
 *
 * History: 
 *      2018-7-20 create this file
 */ 

#ifndef _INC_WM8960
#define  _INC_WM8960	

/*R25 (19h) Power Mgmt (1)*/
#define VMIDSEL_OFF             (00)
#define VMIDSEL_PLAYBACK_RECORD (01)
#define VMIDSEL_STANDBY         (10)
#define VMIDSEL_FAST_STARTUP    (11)

#define MASK_VMIDSEL (0x3 << 7)
#define MASK_VREF (1 << 6)
#define MASK_AINL (1 << 5)
#define MASK_RINR (1 << 4)
#define MASK_ADCL (1 << 3)
#define MASK_ADCR (1 << 2)
#define MASK_MICB (1 << 1)

enum Vmid_Divider {
    Vmid_disabled = 0,
    Vmid_divider_2x50kOhm,
    Vmid_divider_2x250kOhm,
    Vmid_divider_2x5kOhm,
};

#define VMID_FOR_PLAYBACK_RECORD (Vmid_divider_2x50kOhm)
#define VMID_FOR_LOWPOWER_STANDBY (Vmid_divider_2x250kOhm)
#define VMID_FOR_FAST_STARTUP (Vmid_divider_2x5kOhm)

#define VMIDSEL(x)    (x << 7)
#define POWER_VREF(x) (x << 6)
#define POWER_AINL(x) (x << 5)
#define POWER_RINR(x) (x << 4)
#define POWER_ADCL(x) (x << 3)
#define POWER_ADCR(x) (x << 2)
#define POWER_MICB(x) (x << 1)

#define DIGENB     (1 << 0)
#define MASTER_CLOCK_DISABLE (DIGENB)

#define LD2LO (1 << 8)
#define LI2LO (1 << 7)
#define LI2LOVOL_MAX (000)//0db
#define LI2LOVOL_MIN (111)//-21db
#define LI2LOVOL (LI2LOVOL_MAX << 4)//3dB steps

#define RD2LO (1 << 8)
#define RI2LO (1 << 7)
#define RI2LOVOL_MAX (000)//0db
#define RI2LOVOL_MIN (111)//-21db
#define RI2LOVOL (LI2LOVOL_MAX << 4)//3dB steps

#if 1
    #define     IN2LR_2_LEFT_ADC_CTRL 0x0
    #define     IN2LR_2_RIGTH_ADC_CTRL 0x1
    #define     IN1L_2_LEFT_ADC_CTRL 0x2
    #define     IN1R_2_RIGHT_ADC_CTRL 0x3
    #define     PGAL_2_HPLOUT_VOL_CTRL 0x4
    #define     PGAR_2_HPLOUT_VOL_CTRL 0x5
    #define     DACL1_2_HPLOUT_VOL_CTRL 0x6
    #define     DACR1_2_HPLOUT_VOL_CTRL 0x7
    #define     HPLOUT_OUTPUT_LEVEL_CTRL 0x8 
    #define     PGAL_2_HPLCOM_VOL_CTRL 0x9
    #define     PGAR_2_HPLCOM_VOL_CTRL 0xa
    #define     DACL1_2_HPLCOM_VOL_CTRL 0xb
    #define     DACR1_2_HPLCOM_VOL_CTRL 0xc
    #define     HPLCOM_OUTPUT_LEVEL_CTRL 0xd
    #define     PGAR_2_HPROUT_VOL_CTRL 0xf
    #define     DACR1_2_HPROUT_VOL_CTRL 0x10
    #define     HPROUT_OUTPUT_LEVEL_CTRL 0x11
    #define     PGAR_2_HPRCOM_VOL_CTRL 0x12
    #define     DACR1_2_HPRCOM_VOL_CTRL 0X13
    #define     HPRCOM_OUTPUT_LEVEL_CTRL 0x14
    #define     PGAL_2_LEFT_LOP_VOL_CTRL 0x15
    #define     DACL1_2_LEFT_LOP_VOL_CTRL 0x16
    #define     LEFT_LOP_OUTPUT_LEVEL_CTRL 0x17
    #define     PGAR_2_RIGHT_LOP_VOL_CTRL 0x18
    #define     DACR1_2_RIGHT_LOP_VOL_CTRL 0x19
    #define     RIGHT_LOP_OUTPUT_LEVEL_CTRL 0x20
    #define     SET_ADC_SAMPLE 0x21
    #define     SET_DAC_SAMPLE 0x22
    #define     SET_DATA_LENGTH 0x23
    #define     SET_CTRL_MODE 0x24
    #define     LEFT_DAC_VOL_CTRL 0x25
    #define     RIGHT_DAC_VOL_CTRL 0x26
    #define     LEFT_DAC_POWER_SETUP 0x27
    #define     RIGHT_DAC_POWER_SETUP 0x28
    #define     DAC_OUT_SWITCH_CTRL 0x29
    #define     LEFT_ADC_PGA_CTRL 0x30
    #define     RIGHT_ADC_PGA_CTRL 0x31
    #define     WM8960_REG_DUMP 0x32
    #define     SOFT_RESET 0x33
    #define     SET_TRANSFER_MODE 0x34
    #define     SET_SERIAL_DATA_OFFSET 0X35

    /*
    0: ADC Fs = Fsref/1     48		44
    1: ADC Fs = Fsref/1.5	32	
    2: ADC Fs = Fsref/2		24		22
    3: ADC Fs = Fsref/2.5	20
    4: ADC Fs = Fsref/3		16
    5: ADC Fs = Fsref/3.5	13.7
    6: ADC Fs = Fsref/4		12		11
    7: ADC Fs = Fsref/4.5	10.6
    8: ADC Fs = Fsref/5		9.6
    9: ADC Fs = Fsref/5.5
    a: ADC Fs = Fsref / 6	8
    */
    #define     AC31_SET_8K_SAMPLERATE	                0xa
    #define     AC31_SET_12K_SAMPLERATE	                0x6
    #define     AC31_SET_16K_SAMPLERATE	                0x4
    #define     AC31_SET_24K_SAMPLERATE	                0x2
    #define     AC31_SET_32K_SAMPLERATE	                0x1
    #define     AC31_SET_48K_SAMPLERATE	                0x0

    #define     AC31_SET_11_025K_SAMPLERATE	                0x6
    #define     AC31_SET_22_05K_SAMPLERATE	                0x2
    #define     AC31_SET_44_1K_SAMPLERATE	                0x0

    #define     AC31_SET_SLAVE_MODE                      0
    #define     AC31_SET_MASTER_MODE                     1

    #define     AC31_SET_16BIT_WIDTH                      0
    #define     AC31_SET_20BIT_WIDTH                      1
    #define     AC31_SET_24BIT_WIDTH                      2
    #define     AC31_SET_32BIT_WIDTH                      3		




    typedef enum Audio_In_
    {
        IN1L =0,
        IN1R =1,
        IN2L =2,
        IN2R =3,
    }Audio_In;
    typedef enum Audio_Out_
    {
        LINE_OUT_R=0,
        LINE_OUT_L,
        HPL,
        HPR,
    }Audio_Out;
    typedef struct 
    {
        unsigned int chip_num:3;
        unsigned int audio_in_out:2;
        unsigned int if_mute_route:1;
        unsigned int if_powerup:1;
        unsigned int input_level:7;
        unsigned int sample:4;
        unsigned int if_44100hz_series:1;
        unsigned int data_length:2;
        unsigned int ctrl_mode:1;
        unsigned int dac_path:2;
        unsigned int trans_mode:2;
        unsigned int reserved :6;
        unsigned int data_offset:8;
        unsigned int sampleRate;
    }Audio_Ctrl;
#endif

/* WM8960 register space */


#if 1 //from kernel/.../codec/wm8960.h
    #define WM8960_CACHEREGNUM  56

		enum input_pga_vol {
			INPUT_PGA_VOL_DB_n17dot25 = 0x0,
			INPUT_PGA_VOL_DB_n16dot5,
			INPUT_PGA_VOL_DB_n15dot75,
			INPUT_PGA_VOL_DB_n15,
			INPUT_PGA_VOL_DB_n14dot25,
			INPUT_PGA_VOL_DB_n13dot5,
			INPUT_PGA_VOL_DB_n12dot75,
			INPUT_PGA_VOL_DB_n12,
			INPUT_PGA_VOL_DB_n11dot25,
			INPUT_PGA_VOL_DB_n10dot5,
			INPUT_PGA_VOL_DB_n9dot75,
			INPUT_PGA_VOL_DB_n9,
			INPUT_PGA_VOL_DB_n8dot25,
			INPUT_PGA_VOL_DB_n7dot5,
			INPUT_PGA_VOL_DB_n6dot75,
			INPUT_PGA_VOL_DB_n6,
			INPUT_PGA_VOL_DB_n5dot25,
			INPUT_PGA_VOL_DB_n4dot5,
			INPUT_PGA_VOL_DB_n3dot75,
			INPUT_PGA_VOL_DB_n3,
			INPUT_PGA_VOL_DB_n2dot25,
			INPUT_PGA_VOL_DB_n1dot5,
			INPUT_PGA_VOL_DB_n0dot75,
			INPUT_PGA_VOL_DB_0,
			INPUT_PGA_VOL_DB_0dot75,
			INPUT_PGA_VOL_DB_1dot5,
			INPUT_PGA_VOL_DB_2dot25,
			INPUT_PGA_VOL_DB_3,
			INPUT_PGA_VOL_DB_3dot75,
			INPUT_PGA_VOL_DB_4dot5,
			INPUT_PGA_VOL_DB_5dot25,
			INPUT_PGA_VOL_DB_6,
			INPUT_PGA_VOL_DB_6dot75,
			INPUT_PGA_VOL_DB_7dot5,
			INPUT_PGA_VOL_DB_8dot25,
			INPUT_PGA_VOL_DB_9,
			INPUT_PGA_VOL_DB_9dot75,
			INPUT_PGA_VOL_DB_10dot5,
			INPUT_PGA_VOL_DB_11dot25,
			INPUT_PGA_VOL_DB_12,
			INPUT_PGA_VOL_DB_12dot75,
			INPUT_PGA_VOL_DB_13dot5,
			INPUT_PGA_VOL_DB_14dot25,
			INPUT_PGA_VOL_DB_15,
			INPUT_PGA_VOL_DB_15dot75,
			INPUT_PGA_VOL_DB_16dot5,
			INPUT_PGA_VOL_DB_17dot25,
			INPUT_PGA_VOL_DB_18,
			INPUT_PGA_VOL_DB_18dot75,
			INPUT_PGA_VOL_DB_19dot5,
			INPUT_PGA_VOL_DB_20dot25,
			INPUT_PGA_VOL_DB_21,
			INPUT_PGA_VOL_DB_21dot75,
			INPUT_PGA_VOL_DB_22dot5,
			INPUT_PGA_VOL_DB_23dot25,
			INPUT_PGA_VOL_DB_24,
			INPUT_PGA_VOL_DB_24dot75,
			INPUT_PGA_VOL_DB_25dot5,
			INPUT_PGA_VOL_DB_26dot25,
			INPUT_PGA_VOL_DB_27,
			INPUT_PGA_VOL_DB_27dot75,
			INPUT_PGA_VOL_DB_28dot5,
			INPUT_PGA_VOL_DB_29dot25,
			INPUT_PGA_VOL_DB_30,
		};

        // Input PGA Analogue Mute
        #define SWITCH_INMUTE(x) (x << 7)
        #define MASK_INMUTE (1 << 7)
        // Input PGA Volume Update
        #define SWITCH_IPVU(x) (x << 8)
        #define MASK_IPVU (1 << 8)
        //Input PGA Zero Cross Detector
        #define SWITCH_LIZC(x) (x << 6)
        #define MASK_LIZC (1 << 6)
        // Input PGA Volume Control 
        #define INVOL_VALUE(x) (x)// 0.75dB steps
        #define INVOL_MASK (0x3f)
    #define WM8960_LINVOL       0x0
    #define WM8960_RINVOL       0x1

//Headphone Volume Update
#define SWITCH_OUT1VU(x) (x << 8)
#define MASK_OUT1VU (1 << 8)
// zero cross enable        
#define SWITCH_O1ZC(x) (x << 7)
#define MASK_O1ZC (1 << 7)
// LOUT1 Volume        
#define SWITCH_OUT1VOL(x) (x + 0x79)//-73DB <= x <= 6db mute: x < -73db
#define MASK_OUT1VOL (0x7f << 0)
    #define WM8960_LOUT1        0x2
    #define WM8960_ROUT1        0x3
    #define WM8960_CLOCK1       0x4
// ADC High Pass Filter Disable        
#define SWITCH_ADCHPD(x) (x)
#define MASK_ADCHPD (1)
    #define WM8960_DACCTL1      0x5
    #define WM8960_DACCTL2      0x6
enum Master_Slave_Mode_Control {
    digital_slave_mode = 0,
    digital_master_mode = 1,
};
#define SWITCH_MS(x) (x << 6)
#define MASK_MS (1 << 6)
enum Audio_Data_Word_Length {
    Audio_Data_16bits = 0,
    Audio_Data_20bits,
    Audio_Data_24bits,
    Audio_Data_32bits,
};
#define SWITCH_WL(x) (x << 2)
#define MASK_WL (3 << 2)
enum Audio_Data_Format_Select {
    Audio_Data_Format_Right_justified = 0,
    Audio_Data_Format_Left_justified,
    Audio_Data_Format_I2S,
    Audio_Data_Format_DSP_Mode,
};
#define SWITCH_FORMAT(x) (x)
#define MASK_FORMAT (3)
    #define WM8960_IFACE1       0x7
    #define WM8960_CLOCK2       0x8
enum ADCLRC_GPIO1_Pin_Function_Select {
    ADCLRC_GPIO1_Pin_ADCLRC = 0,
    ADCLRC_GPIO1_Pin_GPIO = 1,
};
#define SWITCH_ALRCGPIO(x) (x << 6)
#define MASK_ALRCGPIO (1 << 6)
#define SWITCH_LOOPBACK(x) (x)
#define MASK_LOOPBACK (1)
    #define WM8960_IFACE2       0x9
    #define WM8960_LDAC     0xa
    #define WM8960_RDAC     0xb

    #define WM8960_RESET        0xf
    #define WM8960_3D       0x10
    #define WM8960_ALC1     0x11
    #define WM8960_ALC2     0x12
    #define WM8960_ALC3     0x13
    #define WM8960_NOISEG       0x14

#define ADCVOL_MUTE (0)// -97db <= x <= 30db mute: x= -98db
#define ADCVOL(x) ((x * 2) + 0xc3)// 0.5dB steps -97db <= x <= 30db mute: x= -98db
#define MASK_ADCVOL (0xff)
#define SWITCH_ADCVU(x) (x << 8)
#define MASK_ADCVU (1 << 8)
    #define WM8960_LADC     0x15
    #define WM8960_RADC     0x16
// Enables Slow Clock for Volume Update Timeout and Jack Detect Debounce 
#define SWITCH_TOEN(x) (x)
#define MASK_TOEN (1)
    #define WM8960_ADDCTL1      0x17

//Headphone Switch Enable
#define SWITCH_HPSWEN(x) (x << 6)
#define MASK_HPSWEN (1 << 6)
// Headphone Switch Polarity
// 0 = HPDETECT high = headphone
// 1 = HPDETECT high = speaker
#define SWITCH_HPSWPOL(x) (x << 5)
#define MASK_HPSWPOL (1 << 5)
//Selects disable mode for ADCLRC and DACLRC (Master mode)
//0 = ADCLRC disabled when ADC (Left and Right) disabled; DACLRC disabled when DAC (Left and Right) disabled. 
//1 = ADCLRC and DACLRC disabled only when ADC (Left and Right) and DAC (Left and Right) are disabled. 
#define SWITCH_LRCM(x) (x << 2)
#define MASK_LRCM (1 << 2)
    #define WM8960_ADDCTL2      0x18
    #define WM8960_POWER1       0x19
#define LOUT1_MASK (1 << 6)        
#define ROUT1_MASK (1 << 5)        
#define POWER_LOUT1(x) (x << 6)        
#define POWER_ROUT1(x) (x << 5)        
    #define WM8960_POWER2       0x1a
    #define WM8960_ADDCTL3      0x1b
    #define WM8960_APOP1        0x1c
    #define WM8960_APOP2        0x1d

        //Input PGA to Input BoostMixer
        #define MASK_MIC2B (1 << 3)
        #define SWITCH_MIC2B(x) (x << 3)
        // Connect LINPUT1 to inverting input of Left Input PGA 
        #define MASK_MN1 (1 << 8)
        #define SWITCH_MN1(x) (x << 8)
        // Connect LINPUT2 to non-inverting input of Left Input PGA 
        #define SWITCH_MP2(x) (x << 6)
        #define MASK_MP2 (1 << 6)
        // Connect LINPUT3 to non-inverting input of Left Input PGA 
        #define SWITCH_MP3(x) (x << 7)
        #define MASK_MP3 (1 << 7)
    enum Input_PGA_Boost_Gain {
        Input_PGA_Boost_Gain_0dB = 0x0,
        Input_PGA_Boost_Gain_13dB,
        Input_PGA_Boost_Gain_20dB,
        Input_PGA_Boost_Gain_29dB,
    };
    #define MICBOOST_MASK (111 << 4)
    #define MICBOOST_GAIN(x) (x << 4)
    #define WM8960_LINPATH      0x20
    #define WM8960_RINPATH      0x21

    #define WM8960_LOUTMIX      0x22

    #define WM8960_ROUTMIX      0x25
    #define WM8960_MONOMIX1     0x26
    #define WM8960_MONOMIX2     0x27
    #define WM8960_LOUT2        0x28
    #define WM8960_ROUT2        0x29
    #define WM8960_MONO     0x2a
    enum input3_Boost_Mixer_gain {
        LINE_input_Boost_gain_MUTE =0,
        LINE_input_Boost_gain_DB_n12,
        LINE_input_Boost_gain_DB_n9,
        LINE_input_Boost_gain_DB_n6,
        LINE_input_Boost_gain_DB_n3,
        LINE_input_Boost_gain_DB_0,
        LINE_input_Boost_gain_DB_3,
        LINE_input_Boost_gain_DB_6,
    };
    #define IN3BOOST_MASK (111 << 4)
    #define IN3BOOST_GAIN(x) (x << 4)
    #define WM8960_INBMIX1      0x2b
    #define WM8960_INBMIX2      0x2c

#define B2O_MASK (1 << 7)
#define B2O_ENABLE(x) (x << 7)   
enum Input_Boost_to_Output_Volume {
    B2O_VOL_DB_0 = 0x0,
    B2O_VOL_DB_n3,
    B2O_VOL_DB_n6,
    B2O_VOL_DB_n9,
    B2O_VOL_DB_n12,
    B2O_VOL_DB_n15,
    B2O_VOL_DB_n18,
    B2O_VOL_DB_n21,
};
#define B2OVOL_MASK (0x7 << 4)
#define B2OVOL(x) (x << 4)
    #define WM8960_BYPASS1      0x2d
    #define WM8960_BYPASS2      0x2e
#define LMIC_MASK  (1 << 5)        
#define RMIC_MASK  (1 << 4)        
#define LOMIX_MASK (1 << 3)        
#define ROMIX_MASK (1 << 2)        
#define POWER_LMIC(x)  (x << 5)        
#define POWER_RMIC(x)  (x << 4)        
#define POWER_LOMIX(x) (x << 3)        
#define POWER_ROMIX(x) (x << 2)        
    #define WM8960_POWER3       0x2f
enum Microphone_Bias_Voltage_Control {
    Mic_Bias_Voltage_0dot9_AVDD = 0,
    Mic_Bias_Voltage_0dot65_AVDD = 1,
};
#define MBSEL_MASK (1 << 0)
#define MBSEL(x) (x << 0)
enum GPIO_Function_Select{
    GPIO_Function_Jack_detect_input = 0x0,
    GPIO_Function_Reserved,
    GPIO_Function_Temperature_ok,
    GPIO_Function_Debounced_jack_detect_output,
    GPIO_Function_SYSCLK_output,
    GPIO_Function_PLL_lock,
    GPIO_Function_Logic_0,
    GPIO_Function_Logic_1,  
};
#define GPIOSEL_MASK (0x7 << 4)
#define GPIOSEL(x) (x << 0)
enum Headphone_Switch_Input_Select {
    HP_SWITCH_SEL_GPIO1 = 0x0,//or ox1,Requires ADCLRC pin to be configured as a GPIO
    HP_SWITCH_SEL_JD2 = 0x2,
    HP_SWITCH_SEL_JD3 = 0x3,
};
#define HPSEL_MASK (0x3 << 2)
#define HPSEL(x) (x << 0)
    #define WM8960_ADDCTL4      0x30
    #define WM8960_CLASSD1      0x31

    #define WM8960_CLASSD3      0x33
    #define WM8960_PLL1     0x34
    #define WM8960_PLL2     0x35
    #define WM8960_PLL3     0x36
    #define WM8960_PLL4     0x37

    #define MAX_REGISTER (WM8960_PLL4)
    /*
     * WM8960 Clock dividers
     */
    #define WM8960_SYSCLKDIV        0
    #define WM8960_DACDIV           1
    #define WM8960_OPCLKDIV         2
    #define WM8960_DCLKDIV          3
    #define WM8960_TOCLKSEL         4

    #define WM8960_SYSCLK_DIV_1     (0 << 1)
    #define WM8960_SYSCLK_DIV_2     (2 << 1)

    #define WM8960_SYSCLK_MCLK      (0 << 0)
    #define WM8960_SYSCLK_PLL       (1 << 0)

    #define WM8960_DAC_DIV_1        (0 << 3)
    #define WM8960_DAC_DIV_1_5      (1 << 3)
    #define WM8960_DAC_DIV_2        (2 << 3)
    #define WM8960_DAC_DIV_3        (3 << 3)
    #define WM8960_DAC_DIV_4        (4 << 3)
    #define WM8960_DAC_DIV_5_5      (5 << 3)
    #define WM8960_DAC_DIV_6        (6 << 3)

    #define WM8960_DCLK_DIV_1_5     (0 << 6)
    #define WM8960_DCLK_DIV_2       (1 << 6)
    #define WM8960_DCLK_DIV_3       (2 << 6)
    #define WM8960_DCLK_DIV_4       (3 << 6)
    #define WM8960_DCLK_DIV_6       (4 << 6)
    #define WM8960_DCLK_DIV_8       (5 << 6)
    #define WM8960_DCLK_DIV_12      (6 << 6)
    #define WM8960_DCLK_DIV_16      (7 << 6)

    #define WM8960_TOCLK_F19        (0 << 1)
    #define WM8960_TOCLK_F21        (1 << 1)

    #define WM8960_OPCLK_DIV_1      (0 << 0)
    #define WM8960_OPCLK_DIV_2      (1 << 0)
    #define WM8960_OPCLK_DIV_3      (2 << 0)
    #define WM8960_OPCLK_DIV_4      (3 << 0)
    #define WM8960_OPCLK_DIV_5_5        (4 << 0)
    #define WM8960_OPCLK_DIV_6      (5 << 0)
#endif

#if 0//from kernel/.../sound/wm8960.h
    #define WM8960_DRES_400R 0
    #define WM8960_DRES_200R 1
    #define WM8960_DRES_600R 2
    #define WM8960_DRES_150R 3
    #define WM8960_DRES_MAX  3

    struct wm8960_data {
        bool capless;  /* Headphone outputs configured in capless mode */

        bool shared_lrclk;  /* DAC and ADC LRCLKs are wired together */
    };
#endif 

struct reg_default {
    unsigned int reg;
    unsigned int def;
};

#endif
