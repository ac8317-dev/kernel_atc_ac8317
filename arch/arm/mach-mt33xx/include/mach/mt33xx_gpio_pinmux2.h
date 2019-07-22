/*
 * arch/arm/mach-mt85xx/include/mach/mt8555_gpio_pinmux.h
 *
 * Copyright (C) 2010 MediaTek Inc
 *
 * This program is not free software; you can not redistribute it
 * or modify it without license from MediaTek Inc.
 *
 */

#ifndef __MT8555_GPIO_PINMUX_H
#define __MT8555_GPIO_PINMUX_H

// GPIO name definitions
#define PIN_GPIO0           0
#define PIN_GPIO1           1
#define PIN_GPIO2           2
#define PIN_GPIO3           3
#define PIN_GPIO4           4
#define PIN_GPIO5           5
#define PIN_GPIO6           6
#define PIN_GPIO7           7
#define PIN_AMUTE           8
#define PIN_AOBCK           9
#define PIN_AOLRCK          10
#define PIN_AOMCLK          11
#define PIN_AOSDATA0        12
#define PIN_AOSDATA1        13
#define PIN_AOSDATA2        14
#define PIN_AOSDATA3        15
#define PIN_AOSDATA4        16
#define PIN_FECFREQ         17
#define PIN_FECMOD          18
#define PIN_FEDMO           19
#define PIN_FEEJECT         20
#define PIN_FEFG            21
#define PIN_FEFMO           22
#define PIN_FEFMO2          23     
#define PIN_FEFMO3          24     
#define PIN_FEFMO4          25
#define PIN_FEGAINSW1       26
#define PIN_FEGAINSW2       27
#define PIN_FEGAINSW3       28
#define PIN_FEGIO0          29       
#define PIN_FEGIO1          30 
#define PIN_FEGIO10         31

#define PIN_FEGIO11         32
#define PIN_FEGIO12         33
#define PIN_FEGIO13         34
#define PIN_FEGIO2          35
#define PIN_FEGIO3          36
#define PIN_FEGIO4          37
#define PIN_FEGIO5          38
#define PIN_FEGIO6          39   
#define PIN_FEGIO7          40
#define PIN_FEGIO8          41
#define PIN_FEGIO9          42
#define PIN_FELIMIT         43
#define PIN_FEMPXOUT1       44
#define PIN_FEMPXOUT2       45
#define PIN_FEMPXOUT3       46
#define PIN_FEOSCEN         47      
#define PIN_FEPLAY          48
#define PIN_FETRAYIN        49
#define PIN_FETRAYOUT       50
#define PIN_FETRAYPWM       51  
#define PIN_MCIN            52   
#define PIN_NFALE           53
#define PIN_NFCEN           54
#define PIN_NFCEN2          55
#define PIN_NFCLE           56
#define PIN_NFD0            57
#define PIN_NFD1            58      
#define PIN_NFD2            59
#define PIN_NFD3            60      
#define PIN_NFD4            61
#define PIN_NFD5            62     
#define PIN_NFD6            63

#define PIN_NFD7            64      
#define PIN_NFRBN           65
#define PIN_NFRBN2          66
#define PIN_NFREN           67
#define PIN_NFWEN           68
#define PIN_SCL             69
#define PIN_SDA             70
#define PIN_SFCK            71
#define PIN_SFCS            72
#define PIN_SFDI            73
#define PIN_SFDO            74
#define PIN_SPBCK           75
#define PIN_SPDATA          76
#define PIN_SPDIF           77
#define PIN_SPLRCK          78
#define PIN_SPMCLK          79
#define PIN_VINCLK          80
#define PIN_VIND0           81
#define PIN_VIND1           82  
#define PIN_VIND10          83   
#define PIN_VIND11          84
#define PIN_VIND12          85
#define PIN_VIND13          86
#define PIN_VIND14          87
#define PIN_VIND15          88
#define PIN_VIND16          89      
#define PIN_VIND17          90
#define PIN_VIND18          91      
#define PIN_VIND19          92
#define PIN_VIND2           93     
#define PIN_VIND20          94     
#define PIN_VIND21          95     
                        
#define PIN_VIND22          96
#define PIN_VIND23          97
#define PIN_VIND24          98
#define PIN_VIND25          99
#define PIN_VIND26          100
#define PIN_VIND27          101
#define PIN_VIND28          102
#define PIN_VIND29          103
#define PIN_VIND3           104
#define PIN_VIND30          105
#define PIN_VIND31          106
#define PIN_VIND32          107
#define PIN_VIND33          108
#define PIN_VIND34          109
#define PIN_VIND35          110
#define PIN_VIND4           111    
#define PIN_VIND5           112
#define PIN_VIND6           113
#define PIN_VIND7           114
#define PIN_VIND8           115
#define PIN_VIND9           116
#define PIN_VINHSYNC        117
#define PIN_VINVSYNC        118
#define PIN_VOUTCLK1        119
#define PIN_VOUTD0          120
#define PIN_VOUTD1          121
#define PIN_VOUTD10         122
#define PIN_VOUTD11         123
#define PIN_VOUTD12         124
#define PIN_VOUTD13         125
#define PIN_VOUTD14         126
#define PIN_VOUTD15         127

#define PIN_VOUTD2          128
#define PIN_VOUTD3          129
#define PIN_VOUTD4          130
#define PIN_VOUTD5          131
#define PIN_VOUTD6          132
#define PIN_VOUTD7          133             
#define PIN_VOUTD8          134
#define PIN_VOUTD9          135
#define PIN_VOUTHSYNC       136
#define PIN_VOUTVSYNC       137
#define PIN_TDO             138
#define PIN_TRST            139
#define PIN_TMS             140
#define PIN_TDI             141
#define PIN_TCK             142

#define PIN_VFD_STB         143
#define PIN_VFD_CLK         144 
#define PIN_VFD_DATA        145
#define PIN_LCDRD           146
//                                             147
#define PIN_ETMDIO          148
#define PIN_ETMDC           149
#define PIN_ETRXER          150
#define PIN_ETTXER          151
#define PIN_ETRXDV          152
#define PIN_ETTXEN          153 
#define PIN_ETCRS           154 
#define PIN_ETCOL           155
#define PIN_ETRXD3          156
#define PIN_ETRXD2          157
#define PIN_ETRXD1          158
#define PIN_ETRXD0          159
#define PIN_ETTXD3          160
#define PIN_ETTXD2          161
#define PIN_ETTXD1          162  
#define PIN_ETTXD0          163
#define PIN_ETRXCLK         164 
#define PIN_ETTXCLK         165
#define PIN_HDMI_SCK        166
#define PIN_HDMI_SD         167 
#define PIN_HDMI_HTPLG      168
#define PIN_HDMI_CEC        169 
#define PIN_UATXD           170
#define PIN_UARXD           171 
#define PIN_IR              172 
#define PIN_GPIO8           173 
   
#define PIN_UNKNOWN         255

// Total GPIO is 173.
#define TOTAL_GPIO_NUM      173


//8555 Pin mux function definitions
#define AUD_MCLK_SEL            0
#define SEL_AOMCLK              0
#define SEL_AOBCK               0
#define SEL_AOLRCK              0

#define LIN_SPDATA_SEL          2
#define SEL_LINE_SPDATA         2

#define AUD_SDATA0_SEL          6
#define SEL_AOSDATA0            6

#define AUD_SDATA1_SEL          7
#define SEL_AOSDATA1            7

#define AUD_SDATA2_SEL          8
#define SEL_AOSDATA2            8

#define AUD_SDATA3_SEL          9
#define SEL_AOSDATA3            9

#define AUD_SDATA4_SEL          10
#define SEL_AOSDATA4            10

#define AUD_SDATA5_SEL          11
#define SEL_AOSDATA5            11

#define AUD_MUTE_SEL            12
#define SEL_AMUTE               12

#define AUD_SPDIF_SEL           14 
#define SEL_SPDIF               14

#define AUD_SPDATA_SEL          15 
#define SEL_SPDATA0             15

#define AUD_SPLRCK_SEL          18
#define SEL_SPLRCK              18

#define AUD_SPBCK_SEL           19
#define SEL_SPBCK               19

#define AUD_SPMCLK_SEL          20
#define SEL_SPMCLK              20

#define FESF_SEL                21
#define SEL_FESF                21

#define I2C_SEL                 22
#define SEL_SCL                 22 
#define SEL_SDA                 22

#define EXT_INTR4_SEL           23
#define SEL_EXT_INTR4           23

#define FEJTAG_SEL              27
#define SEL_FE_JTDO             27
#define SEL_FE_JTCLK            27
#define SEL_FE_JTMS             27
#define SEL_FE_JTDI             27
#define SEL_FE_JTRST_B          27
#define SEL_FE_JRTCLK           27

#define FELVDS_SEL              29
#define SEL_LVDS1P              29
#define SEL_LVDS1N              29
#define SEL_LVDS2P              29
#define SEL_LVDS2N              29
#define SEL_LVDS3P              29
#define SEL_LVDS3N              29
#define SEL_LVDS4P              29
#define SEL_LVDS4N              29
#define SEL_LVDS5P              29
#define SEL_LVDS5N              29

#define NAND_SEL                30
#define SEL_NFREN               30
#define SEL_NFCLE               30 
#define SEL_NFCEN               30 
#define SEL_NFWEN               30
#define SEL_NFALE               30
#define SEL_NFRBN               30
#define SEL_NFD0                30
#define SEL_NFD1                30
#define SEL_NFD2                30
#define SEL_NFD3                30
#define SEL_NFD4                30
#define SEL_NFD5                30
#define SEL_NFD6                30
#define SEL_NFD7                30

#define NAND_RDY2_SEL           31
#define SEL_NFRBN2              31 

#define AUD2_SACD_SEL           43
#define SEL_AO2SDATA4           43
#define SEL_AO2SDATA5           43 

#define EXT_SF_CS_SEL1          44
#define SEL_EXT_SF_CS1          44
          
#define EXT_SF_SCK_SEL          45
#define SEL_EXT_SF_SCK          45

#define M27_OUT_SEL             52
#define SEL_MJC                 52

#define SF_CS2_SEL              56  
#define SEL_SFCS2               56

#define PD_RS232_SEL            60 
#define SEL_UATXD1              60
#define SEL_UARXD1              60

#define NAND_CEN2_SEL           64
#define SEL_NFCEN2              64 

#define NAND_CEN3_SEL           68
#define SEL_NFCEN3              68 

#define NAND_CEN4_SEL           72
#define SEL_NFCEN4              72 

#define EXT_INTR1_SEL           76
#define SEL_EXT_INTR1           76

#define EXT_INTR2_SEL           80
#define SEL_EXT_INTR2           80

#define RS232_D3_SEL            84
#define SEL_UATXD3              84
#define SEL_UARXD3              84

#define HDMI_MSCK_EN_SEL        87
#define SEL_HDMI_MSCK_EN        87

#define SI2C_SEL                88 
#define SEL_SCLS                88 
#define SEL_SDAS                88

#define HDMI_I2C_SEL            92 
#define SEL_MSD                 92
#define SEL_MSCK                92 

#define TEST_IN_SEL             99 
#define SEL_TIND31              99
#define SEL_TIND30              99 
#define SEL_TIND29              99
#define SEL_TIND28              99
#define SEL_TIND27              99
#define SEL_TIND26              99
#define SEL_TIND25              99
#define SEL_TIND24              99
#define SEL_TIND23              99
#define SEL_TIND22              99
#define SEL_TIND21              99
#define SEL_TIND20              99
#define SEL_TIND19              99
#define SEL_TIND18              99
#define SEL_TIND17              99
#define SEL_TIND16              99
#define SEL_TIND15              99
#define SEL_TIND14              99
#define SEL_TIND13              99
#define SEL_TIND12              99
#define SEL_TIND11              99
#define SEL_TIND10              99
#define SEL_TIND9               99
#define SEL_TIND8               99
#define SEL_TIND7               99
#define SEL_TIND6               99
#define SEL_TIND5               99
#define SEL_TIND4               99
#define SEL_TIND3               99
#define SEL_TIND2               99
#define SEL_TIND1               99
#define SEL_TIND0               99

#define MONITOR_SEL             100 
#define SEL_MOUTB31             100
#define SEL_MOUTB30             100
#define SEL_MOUTB29             100
#define SEL_MOUTB28             100
#define SEL_MOUTB27             100
#define SEL_MOUTB26             100
#define SEL_MOUTB25             100
#define SEL_MOUTB24             100
#define SEL_MOUTB23             100
#define SEL_MOUTB22             100
#define SEL_MOUTB21             100
#define SEL_MOUTB20             100
#define SEL_MOUTB19             100
#define SEL_MOUTB18             100
#define SEL_MOUTB17             100
#define SEL_MOUTB16             100
#define SEL_MOUTB15             100
#define SEL_MOUTB14             100
#define SEL_MOUTB13             100
#define SEL_MOUTB12             100
#define SEL_MOUTB11             100
#define SEL_MOUTB10             100
#define SEL_MOUTB9              100
#define SEL_MOUTB8              100
#define SEL_MOUTB7              100
#define SEL_MOUTB6              100
#define SEL_MOUTB5              100
#define SEL_MOUTB4              100
#define SEL_MOUTB3              100
#define SEL_MOUTB2              100
#define SEL_MOUTB1              100
#define SEL_MOUTB0              100

#define AOMCLK_PWM_SEL          104      
#define SEL_AOMCLK_PWM          104

#define AOBCK_PWM_SEL           105  
#define SEL_AOBCK_PWM           105

#define AOLRCK_PWM_SEL          106  
#define SEL_AOLRCK_PWM          106

#define HDMI_RX_SEL             108
#define SEL_HDMI_RX             108

#define HDMI_RX_SPDF_SEL        109 
#define SEL_HDMI_RX_SPDF        109

#define HDMI_RX_ACK_SEL         110 
#define SEL_HDMI_RX_ACK         110

#define HDMI_EXT_DATA_SEL       114 
#define SEL_HDMI_EXT_DATA       114

#define HDMI_EXT_DSD_SEL        115 
#define SEL_HDMI_EXT_DSD        115

#define HDMI_EXT_SPDF_SEL       116 
#define SEL_HDMI_EXT_SPDF       116

#define HDMI_EXT_ACK_SEL        117 
#define SEL_HDMI_EXT_ACK        117

#define HDMI_EXT_BCK_SEL        118 
#define SEL_HDMI_EXT_BCK        118

#define HDMI_EXT_LRCK_SEL       119 
#define SEL_HDMI_EXT_LRCK       119

#define HDMI_RX_SDATA0_SEL      120 
#define SEL_HDMI_RX_SDATA0      120

#define HDMI_EXT_RX_SPDF_SEL    121 
#define SEL_HDMI_EXT_RX_SPDF    121

#define EFVDDQ_SEL              124
#define SEL_EFVDDQ              124

#define EFSRC_SEL               125 
#define SEL_EFSRC               125

#define MONITOR_MODE_SEL        128
#define SEL_MONITOR_MODE        128

#define MONITOR_DRAM_SEL        136        
#define SEL_MONITOR_DRAM        136

#define VDOUT0_5_SEL            144 
#define SEL_VOUTVSYNC_FIFO      144
#define SEL_VOUTHSYNC_FIFO      145
#define SEL_VOUTD0_1            146
#define SEL_VOUTD2_7            147
#define SEL_VOUTD8_9            148
#define SEL_VOUTD10_11          149

#define NAND_16BIT_SEL          150  
#define SEL_NFD8                150 
#define SEL_NFD9                150 
#define SEL_NFD10               150 
#define SEL_NFD11               150 
#define SEL_NFD12               150 
#define SEL_NFD13               150 
#define SEL_NFD14               150 
#define SEL_NFD15               150 

#define PWM_SEL                 152
#define SEL_PWM                 152

#define VDOUT6_8_SEL            157 
#define SEL_VDOUT12_15          157 
#define SEL_VOUTCLK2            158

#define USBPHY_CK_SEL           160 
#define SEL_USBPHY              160

#define HDMI_TST_SEL            161 
#define SEL_HDMI_TST            161

#define SF_SEL                  162
#define SEL_SFCS                162
#define SEL_SFCK                162
#define SEL_SFDI                162
#define SEL_SFDO                162

#define EXT_ACLK_SEL            168
#define SEL_APLL_CLK            168

#define SCNT_CK_SEL             172
#define SEL_SCNT_CK             172

#define MCIN_MPMCLK_SEL         175
#define SEL_MCIN_MPMCLK         175

#define MCIN_MPBCK_SEL          178 
#define SEL_MCIN_MPBCK          178

#define MCIN_MPLRCK_SEL         181 
#define SEL_MCIN_MPLRCK         181

#define VOUTCLK1_SEL            184 
#define SEL_VOUTCLK1            184

#define SPDATA1_SEL             186 
#define SEL_SPDATA1             186 

#define SPDATA2_SEL             188 
#define SEL_SPDATA2             188

#define SPDATA3_SEL             190 
#define SEL_SPDATA3             190

#define PROT_SEL                192 
#define SEL_PROT                192

#define FEDIGITAL_0_SEL         224 
#define SEL_FEDIGITAL_0         224

#define FEDIGITAL_1_SEL         225 
#define SEL_FEDIGITAL_1         225

#define FEDIGITAL_2_SEL         226 
#define SEL_FEDIGITAL_2         226

#define FEDIGITAL_3_SEL         227 
#define SEL_FEDIGITAL_3         227

#define FEDIGITAL_4_SEL         228 
#define SEL_FEDIGITAL_4         228

#define FEDIGITAL_5_SEL         229 
#define SEL_FEDIGITAL_5         229

#define FEDIGITAL_6_SEL         230 
#define SEL_FEDIGITAL_6         230

#define FEDIGITAL_7_SEL         231 
#define SEL_FEDIGITAL_7         231

#define FEDIGITAL_8_SEL         232 
#define SEL_FEDIGITAL_8         232

#define FEDIGITAL_9_SEL         233 
#define SEL_FEDIGITAL_9         233

#define FEDIGITAL_10_SEL         234 
#define SEL_FEDIGITAL_10         234

#define FEDIGITAL_11_SEL         235 
#define SEL_FEDIGITAL_11         235

#define FEDIGITAL_12_SEL         236 
#define SEL_FEDIGITAL_12         236

#define FEDIGITAL_13_SEL         237 
#define SEL_FEDIGITAL_13         237

#define FEDIGITAL_14_SEL         238 
#define SEL_FEDIGITAL_14         238

#define FEDIGITAL_15_SEL         239 
#define SEL_FEDIGITAL_15         239

#define FEDIGITAL_16_SEL         240 
#define SEL_FEDIGITAL_16         240

#define FEDIGITAL_17_SEL         241 
#define SEL_FEDIGITAL_17         241

#define FEDIGITAL_18_SEL         242 
#define SEL_FEDIGITAL_18         242

#define FEDIGITAL_19_SEL         243 
#define SEL_FEDIGITAL_19         243

#define FEDIGITAL_20_SEL         244 
#define SEL_FEDIGITAL_20         244

#define FEDIGITAL_21_SEL         245 
#define SEL_FEDIGITAL_21         245

#define FEDIGITAL_22_SEL         246 
#define SEL_FEDIGITAL_22         246

#define FEDIGITAL_23_SEL         247 
#define SEL_FEDIGITAL_23         247

#define FEDIGITAL_24_SEL         248 
#define SEL_FEDIGITAL_24         248

#define FEDIGITAL_25_SEL         249 
#define SEL_FEDIGITAL_25         249

#define FEDIGITAL_26_SEL         250 
#define SEL_FEDIGITAL_26         250

#define FEDIGITAL_27_SEL         251 
#define SEL_FEDIGITAL_27         251

#define FEDIGITAL_28_SEL         252 
#define SEL_FEDIGITAL_28         252

#define FEDIGITAL_29_SEL         253 
#define SEL_FEDIGITAL_29         253

#define FEDIGITAL_30_SEL         254 
#define SEL_FEDIGITAL_30         254

#define FEDIGITAL_31_SEL         255 
#define SEL_FEDIGITAL_31         255

#define FEDIGITAL_32_SEL         256 
#define SEL_FEDIGITAL_32         256

#define FEDIGITAL_33_SEL         257 
#define SEL_FEDIGITAL_33         257

#define FEDIGITAL_34_SEL         258 
#define SEL_FEDIGITAL_34         258

#define MSSER_CLK_BS_SEK         260
#define SEL_MSBS                 260
#define SEL_MSCLK                260

#define MS_BS_PRE_SEL            263
#define SEL_MS_BS_PRE            263

#define MSSER_SEL                264
#define SEL_MSD0                 264

#define MSPAL_SEL                268
#define SEL_MSD1                 268
#define SEL_MSD2                 268
#define SEL_MSD3                 268

#define SDSER_CLK_CMD_SEL        272
#define SEL_SDCLK                272
#define SEL_SDCMD                272

#define SDSER_SEL                276
#define SEL_SDD0                 276

#define SD1PAL_SEL               280
#define SEL_SDD1                 280

#define SD23PAL_SEL              284
#define SEL_SDD2                 284
#define SEL_SDD3                 284

#define VOUTHSYNC_SEL           288
#define SEL_VOUTHSYNC           288 

#define VOUTVSYNC_SEL           290
#define SEL_VOUTVSYNC           290

#define VOUTD0_1_SEL            292 
#define SEL_VOUTD0              292
#define SEL_VOUTD1              292

#define VOUTD2_5_SEL            294
#define SEL_VOUTD2              294
#define SEL_VOUTD3              294
#define SEL_VOUTD4              294
#define SEL_VOUTD5              294

#define VOUTD6_9_SEL            297 
#define SEL_VOUTD6              297
#define SEL_VOUTD7              297
#define SEL_VOUTD8              297
#define SEL_VOUTD9              297

#define VOUTD10_11_SEL          300  
#define SEL_VOUTD10             300 
#define SEL_VOUTD11             300 

#define VOUTD12_15_SEL          302
#define SEL_VOUTD12             302
#define SEL_VOUTD13             302
#define SEL_VOUTD14             302
#define SEL_VOUTD15             302

#define VOUTD16_19_SEL          305
#define SEL_VOUTD16             305
#define SEL_VOUTD17             305
#define SEL_VOUTD18             305
#define SEL_VOUTD19             305

#define VOUTD20_21_SEL          307  
#define SEL_VOUTD20             307 
#define SEL_VOUTD21             307 

#define VOUTD22_25_SEL          309
#define SEL_VOUTD22             309
#define SEL_VOUTD23             309
#define SEL_VOUTD24             309
#define SEL_VOUTD25             309

#define VOUTD26_29_SEL          311
#define SEL_VOUTD26             311
#define SEL_VOUTD27             311
#define SEL_VOUTD28             311
#define SEL_VOUTD29             311

#define VOUTDE_SEL              313
#define SEL_VOUTDE              313

#define EXT_SF_CS_SEL0          316
#define SEL_EXT_SF_CS0          316           

#define EXT_SE_SEL              317
#define SEL_EXT_SFDI            317
#define SEL_EXT_SFDO            317

#define SPI_SEL                 318
#define SEL_SPI_SDATA           318
#define SEL_SPI_CLE             318
#define SEL_SPI_CLK             318
#define SEL_SPI_COB             318

#define TMX_PWM_SEL             319
#define SEL_TMX_PWM             319

#define EXT_TS0_SEL             320
#define SEL_EXT_TS0_DATA0       320
#define SEL_EXT_TS0_SYNC        320
#define SEL_EXT_TS0_VALID       320
#define SEL_EXT_TS0_CLK         320

#define EXT_TS0_PARA_SEL        321
#define SEL_EXT_TS0_DATA1       321
#define SEL_EXT_TS0_DATA2       321
#define SEL_EXT_TS0_DATA3       321
#define SEL_EXT_TS0_DATA4       321
#define SEL_EXT_TS0_DATA5       321
#define SEL_EXT_TS0_DATA6       321
#define SEL_EXT_TS0_DATA7       321

#define EXT_TS1_SEL             322
#define SEL_EXT_TS1_DATA0       322
#define SEL_EXT_TS1_SYNC        322
#define SEL_EXT_TS1_VALID       322
#define SEL_EXT_TS1_CLK         322

#define EXT_TS1_PARA_SEL        323
#define SEL_EXT_TS1_DATA1       323
#define SEL_EXT_TS1_DATA2       323
#define SEL_EXT_TS1_DATA3       323
#define SEL_EXT_TS1_DATA4       323
#define SEL_EXT_TS1_DATA5       323
#define SEL_EXT_TS1_DATA6       323
#define SEL_EXT_TS1_DATA7       323

#define TSOUT_SEL               324
#define SEL_TSOUT_DATA0         324
#define SEL_TSOUT_SYNC          324
#define SEL_TSOUT_VALID         324
#define SEL_TSOUT_CLK           324

#define TSOUT_PARA_SEL          325
#define SEL_TSOUT_DATA1         325
#define SEL_TSOUT_DATA2         325
#define SEL_TSOUT_DATA3         325
#define SEL_TSOUT_DATA4         325
#define SEL_TSOUT_DATA5         325
#define SEL_TSOUT_DATA6         325
#define SEL_TSOUT_DATA7         325

#define VINCLK_SEL              326
#define SEL_VINCLK              326

#define VINHSYNC_SEL            328 
#define SEL_VINHSYNC            328

#define VINVSYNC_SEL            330 
#define SEL_VINVSYNC            330

#define VIND_0_3_SEL            332    
#define SEL_VIND0               332
#define SEL_VIND1               332
#define SEL_VIND2               332
#define SEL_VIND3               332

#define VIND_4_11_SEL           333 
#define SEL_VIND4               333 
#define SEL_VIND5               333
#define SEL_VIND6               333
#define SEL_VIND7               333
#define SEL_VIND8               333
#define SEL_VIND9               333
#define SEL_VIND10              333
#define SEL_VIND11              333

#define VIND_12_13_SEL          334     
#define SEL_VIND12              334
#define SEL_VIND13              334

#define VIND_14_15_SEL          335    
#define SEL_VIND14              335
#define SEL_VIND15              335

#define VIND_16_23_SEL          336    
#define SEL_VIND16              336
#define SEL_VIND17              336
#define SEL_VIND18              336
#define SEL_VIND19              336
#define SEL_VIND20              336
#define SEL_VIND21              336
#define SEL_VIND22              336
#define SEL_VIND23              336

#define VIND_24_25_SEL          337
#define SEL_VIND24              337
#define SEL_VIND25              337

#define VIND_26_35_SEL          338     
#define SEL_VIND26              338
#define SEL_VIND27              338
#define SEL_VIND28              338
#define SEL_VIND29              338
#define SEL_VIND30              338
#define SEL_VIND31              338
#define SEL_VIND32              338
#define SEL_VIND33              338
#define SEL_VIND34              338
#define SEL_VIND35              338

#define ETLED_0_SEL             340
#define SEL_ETLED_0             340

#define ETLED_1_SEL             344
#define SEL_ETLED_1             344

#define ETLED_2_SEL             348 
#define SEL_ETLED_2             348

#define SPDIF_IN_COAXIAL_SEL    352
#define SEL_SPDIF_IN_COAXIAL    352

#define SPDIF_IN_OPTICAL_SEL    354 
#define SEL_SPDIF_IN_OPTICAL    354

#define AUD2_SEL                356
#define SEL_AO2MCLK             356 
#define SEL_AO2BCK              356
#define SEL_AO2LRCK             356
#define SEL_AO2SDATA0           356
#define SEL_AO2SDATA1           356
#define SEL_AO2SDATA2           356

#define AUD_MCIN_SEL            358 
#define SEL_MCIN                358

#define RS232_D2_SEL            360 
#define SEL_UATXD2              360
#define SEL_UARXD2              360

#define JTAG2_SEL               363
#define SEL_TDO2                363
#define SEL_TCK2                363
#define SEL_TMS2                363
#define SEL_TDI2                363
#define SEL_TRST2               363

#define HDMI_RX_I2C_SEL         364
#define SEL_HDMI_RX_I2C_SCL     364
#define SEL_HDMI_RX_I2C_SDA     364

#define JTAG_HDMI_SEL           368
#define SEL_JTAG_TCK            368

#define JTAG_4PIN_SEL           369 
#define SEL_JTAG_TRST           369

#define HDMI_RX_SPDIF_SEL       370 
#define SEL_HDMI_RX_SPDIF       370

#define AUD2_DATA3_SEL          372
#define SEL_AO2SDATA3           372

#define AUD2_SPDIF2_SEL         373
#define SEL_SPDIF2              373

#define VOUT_PWDN_CLK_SEL       384
#define VOUT_PWDN_SYNC_SEL      385
#define VOUT_PWDN_DO_D2_SEL     386 
#define VOUT_PWDN_D3_SEL        387 
#define VOUT_PWDN_D4_D5_SEL     388 
#define VOUT_PWDN_D6_D7_SEL     389 
#define VOUT_PWDN_D8_D15_SEL    390 
#define VOUT_VIN_CLK_SEL        391 
#define VOUT_VIN_SYNC_SEL       392
#define VOUT_VIN_D1_D3_SEL      393 
#define VOUT_VIN_D4_D7_SEL      394 
#define VOUT_VIN_D8_D9_SEL      395 
#define VOUT_VIN_D10_D11_SEL    396  
#define VOUT_VIN_D12_D15_SEL    397
#define VOUT_VIN_D16_D19_SEL    398
#define VOUT_VIN_D20_D21_SEL    399
#define VOUT_VIN_D22_D23_SEL    400
#define VOUT_VIN_D24_SEL        401
#define VOUT_VIN_D25_D29_SEL    402
#define VOUT_VIN_D31_SEL        403
#define VOUT_GPIO0_SEL          404
#define VOUT_GPIO1_SEL          405
#define VOUT_GPIO2_SEL          406
#define VOUT_GPIO3_SEL          407
#define VOUT_VIN_D0_SEL         408
#define DGI_PAD_MUL_SEL         415

//PAD_FECTL @ 0X280~0X284
#define SEL_FECTL_0             416
#define SEL_FECTL_1             417
#define SEL_FECTL_2             418
#define SEL_FECTL_3             419
#define SEL_FECTL_4             420
#define SEL_FECTL_5             421
#define SEL_FECTL_6             422
#define SEL_FECTL_7             423
#define SEL_FECTL_8             424
#define SEL_FECTL_9             425
#define SEL_FECTL_10             426
#define SEL_FECTL_11             427
#define SEL_FECTL_12             428
#define SEL_FECTL_13             429
#define SEL_FECTL_14             430
#define SEL_FECTL_15             431
#define SEL_FECTL_16             432
#define SEL_FECTL_17             433
#define SEL_FECTL_18             434
#define SEL_FECTL_19             435
#define SEL_FECTL_20             436
#define SEL_FECTL_21             437
#define SEL_FECTL_22             438
#define SEL_FECTL_23             439
#define SEL_FECTL_24             440
#define SEL_FECTL_25             441
#define SEL_FECTL_26             442
#define SEL_FECTL_27             443
#define SEL_FECTL_28             444
#define SEL_FECTL_29             445
#define SEL_FECTL_30             446
#define SEL_FECTL_31             447
#define SEL_FECTL_32             448
#define SEL_FECTL_33             449
#define SEL_FECTL_34             450

//PAD_PWMCTL @ 0X288
#define SEL_PWM_RELATCH          480
#define SEL_APLL_CLK_INV         481
#define SEL_HIN_SPMCLK2          496
#define SEL_HIN_SPBCK2           498
#define SEL_HIN_SPLRCK2          500
#define SEL_HIN_SPSDATA2         502

#define MAX_PINMUX_SEL           504


#endif /* __MT8555_GPIO_PINMUX_H */

