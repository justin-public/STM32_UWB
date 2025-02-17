/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/main.c
  * @author  MCD Application Team
  * @version V3.5.0
  * @date    08-April-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */
/*! ----------------------------------------------------------------------------
 *  @file    main.c
 *  @brief   Double-sided two-way ranging (DS TWR) responder example code
 *
 *           This is a simple code example which acts as the responder in a DS TWR distance measurement exchange. This application waits for a "poll"
 *           message (recording the RX time-stamp of the poll) expected from the "DS TWR initiator" example code (companion to this application), and
 *           then sends a "response" message recording its TX time-stamp, after which it waits for a "final" message from the initiator to complete
 *           the exchange. The final message contains the remote initiator's time-stamps of poll TX, response RX and final TX. With this data and the
 *           local time-stamps, (of poll RX, response TX and final RX), this example application works out a value for the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices, which it writes to the LCD.
 *
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "lcd.h"
#include "port.h"
#include "lcd_oled.h"
#include "trilateration.h"
#include <math.h>
#include "kalman.h"
#include "AT24C02.h"
#include "stm32_eval.h"

/* Example application name and version to display on LCD screen. */
#define RNG_DELAY_MS 250

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
#if 1
static dwt_config_t config =
{
    2,               /* Channel number. */
    DWT_PRF_64M,     /* Pulse repetition frequency. */
    DWT_PLEN_1024,   /* Preamble length. */
    DWT_PAC32,       /* Preamble acquisition chunk size. Used in RX only. */
    9,               /* TX preamble code. Used in TX only. */
    9,               /* RX preamble code. Used in RX only. */
    1,               /* Use non-standard SFD (Boolean) */
    DWT_BR_110K,     /* Data rate. */
    DWT_PHRMODE_STD, /* PHY header mode. */
    (1025 + 64 - 32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
};
#endif

#if 0
static dwt_config_t config =
{
    5,               /* Channel number - 채널 5 사용 (더 짧은 거리에 적합) */
    DWT_PRF_16M,     /* Pulse repetition frequency - 16M으로 변경 (전력 감소) */
    DWT_PLEN_256,    /* Preamble length - 더 짧은 프리앰블 사용 */
    DWT_PAC8,        /* Preamble acquisition chunk - 짧은 거리에 맞게 조정 */
    9,               /* TX preamble code - 유지 */
    9,               /* RX preamble code - 유지 */
    1,               /* Use non-standard SFD - 유지 */
    DWT_BR_6M8,      /* Data rate - 더 높은 데이터 레이트 사용 (전력 효율) */
    DWT_PHRMODE_STD, /* PHY header mode - 유지 */
    (256 + 64 - 8)   /* SFD timeout - 새로운 프리앰블 길이에 맞게 조정 */
};
#endif
/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
//#define TX_ANT_DLY 16436
//#define RX_ANT_DLY 16436
#define TX_ANT_DLY 0
#define RX_ANT_DLY 32950


/* Frames used in the ranging process. See NOTE 2 below. */
static uint8 rx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 tx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 rx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 distance_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xAA, 0, 0,0, 0, 0};
static uint8 tx_poll_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x21, 0, 0};
static uint8 rx_resp_msg[] =  {0x41, 0x88, 0, 0x0, 0xDE, 'V', 'E', 'W', 'A', 0x10, 0x02, 0, 0, 0, 0};
static uint8 tx_final_msg[] = {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0x23, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 angle_msg[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xFE, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
static uint8 Semaphore_Release[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE0, 0, 0, 0};
static uint8 Tag_Statistics[] =                      {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE1, 0, 0, 0};
static uint8 Master_Release_Semaphore[] =            {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE2, 0, 0, 0};
static uint8 Tag_Statistics_response[] =             {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE3, 0, 0, 0};
static uint8 Master_Release_Semaphore_comfirm[] =    {0x41, 0x88, 0, 0x0, 0xDE, 'W', 'A', 'V', 'E', 0xE4, 0, 0, 0};

/* Length of the common part of the message (up to and including the function code, see NOTE 2 below). */
#define ALL_MSG_COMMON_LEN 10
/* Index to access some of the fields in the frames involved in the process. */
#define ALL_MSG_SN_IDX 2
#define ALL_MSG_TAG_IDX 3
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
#define ANGLE_MSG_IDX 10
#define LOCATION_FLAG_IDX 11
#define LOCATION_INFO_LEN_IDX 12
#define LOCATION_INFO_START_IDX 13
#define ANGLE_MSG_MAX_LEN 30

/* Frame sequence number, incremented after each transmission. */
static uint8 frame_seq_nb = 0;
static uint8 frame_seq_nb_semaphore = 0;

/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 24
static uint8 rx_buffer[RX_BUF_LEN];

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. */
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 ? and 1 ? = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500
/* Receive final timeout. See NOTE 5 below. */
#define FINAL_RX_TIMEOUT_UUS 3300


/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 2800 //2700 will fail
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700



/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;
typedef unsigned long long uint64;
static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Speed of light in air, in metres per second. */
#ifndef SPEED_OF_LIGHT
#define SPEED_OF_LIGHT 299702547
#endif

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;

/* String used to display measured distance on LCD screen (16 characters maximum). */
char dist_str[16] = {0};

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void compute_angle_send_to_anthor0(int distance1, int distance2, int distance3);
static void distance_mange(void);
void USART_puts(uint8_t *s,uint8_t len);

//#define TAG
#define TAG_ID 0x0F
#define MASTER_TAG 0x0F
#define MAX_SLAVE_TAG 0x02
#define SLAVE_TAG_START_INDEX 0x01

#define ANTHOR
#define ANCHOR_MAX_NUM 3
#define ANCHOR_IND 0  // 0 1 2
//#define ANCHOR_IND ANCHOR_NUM

uint8 Semaphore[MAX_SLAVE_TAG];

vec3d AnchorList[ANCHOR_MAX_NUM];
vec3d tag_best_solution;
int Anthordistance[ANCHOR_MAX_NUM];
int Anthordistance_count[ANCHOR_MAX_NUM];

#define ANCHOR_REFRESH_COUNT 5

/* Private macro ---------- ---------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
/* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */


void dwt_dumpregisters(char *str, size_t strSize)
{
    uint32 reg = 0;
    uint8 buff[5];
    int i;
    int cnt ;

#if (0)
    //first print all single registers
    for(i=0; i<0x3F; i++)
    {
        dwt_readfromdevice(i, 0, 5, buff) ;
        str += cnt = sprintf(str,"reg[%02X]=%02X%02X%02X%02X%02X",i,buff[4], buff[3], buff[2], buff[1], buff[0] ) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x20
    for(i=0; i<=32; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x20,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x20,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x21
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x21,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x21,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x23
    for(i=0; i<=0x20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x23,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x23,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#else
    //reg 0x24
    for(i=0; i<=12; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x24,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x24,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x27
    for(i=0; i<=44; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x27,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x27,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x28
    for(i=0; i<=64; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x28,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x28,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2A
    for(i=0; i<20; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2A,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2A,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2B
    for(i=0; i<24; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2B,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2B,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x2f
    for(i=0; i<40; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x2f,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x2f,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x31
    for(i=0; i<84; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x31,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x31,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }

    //reg 0x36 = PMSC_ID
    for(i=0; i<=48; i+=4)
    {
        reg = dwt_read32bitoffsetreg(0x36,i) ;
        str += cnt = sprintf(str,"reg[%02X:%02X]=%08X",0x36,i,reg) ;
        str += cnt = sprintf(str,"\n") ;
    }
#endif
}

void Anchor_Array_Init(void)
{
    int anchor_index = 0;
    for(anchor_index = 0; anchor_index < ANCHOR_MAX_NUM; anchor_index++)
    {
        Anthordistance[anchor_index] = 0;
        Anthordistance_count[anchor_index] = 0;
    }
}
void Semaphore_Init(void)
{
    int tag_index = 0 ;
    for(tag_index = 0; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        Semaphore[tag_index]  = 0;
    }
}

int Sum_Tag_Semaphore_request(void)
{
    int tag_index = 0 ;
    int sum_request = 0;
    for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
    {
        sum_request+=Semaphore[tag_index];
    }
    return sum_request;
}

#define TX_PGDELAY_CH5 0xC5

void configureTXPower(dwt_txconfig_t *config){
    config->PGdly = TX_PGDELAY_CH5;
    config->power = 0x1F1F1F1F;
    dwt_configuretxrf(config);
}


void Tag_Measure_Dis(void)
{
    uint8 dest_anthor = 0,frame_len = 0;
    float final_distance = 0;
    for(dest_anthor = 0 ;  dest_anthor<ANCHOR_MAX_NUM; dest_anthor++)
    {
        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
        /* Write frame data to DW1000 and prepare transmission. See NOTE 7 below. */
        tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
        tx_poll_msg[ALL_MSG_TAG_IDX] = TAG_ID;                  //��վ�յ���ǩ����Ϣ��������TAG_ID,�ڻ�վ�ظ���ǩ��ʱ��Ҳ��Ҫָ��TAG_ID,ֻ��TAG_IDһ�²�������

        dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
        dwt_writetxfctrl(sizeof(tx_poll_msg), 0);

        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
         * set by dwt_setrxaftertxdelay() has elapsed. */
        dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);

        //GPIO_SetBits(GPIOA,GPIO_Pin_2);
        //TODO
        dwt_rxenable(0);//�����ӵģ�Ĭ��tx��Ӧ���Զ��л�rx������Ŀǰdebug ���ֲ�û���Զ��򿪣�����ǿ�ƴ�rx

        /* We assume that the transmission is achieved correctly, poll for reception of a frame or error/timeout. See NOTE 8 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };
        GPIO_SetBits(GPIOA,GPIO_Pin_1);

        if (status_reg & SYS_STATUS_RXFCG)
        {
            //printf("T01\r\n");
					
					  /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
            if (frame_len <= RX_BUF_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }

            if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)//���TAG_ID
                continue;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;

            /* As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
            rx_buffer[ALL_MSG_SN_IDX] = 0;

            if (memcmp(rx_buffer, rx_resp_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                uint32 final_tx_time;

                /* Retrieve poll transmission and response reception timestamp. */
                poll_tx_ts = get_tx_timestamp_u64();
                resp_rx_ts = get_rx_timestamp_u64();

                /* Compute final message transmission time. See NOTE 9 below. */
                final_tx_time = (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
                dwt_setdelayedtrxtime(final_tx_time);

                /* Final TX timestamp is the transmission time we programmed plus the TX antenna delay. */
                final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

                /* Write all timestamps in the final message. See NOTE 10 below. */
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP_RX_TS_IDX], resp_rx_ts);
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);

                /* Write and send final message. See NOTE 7 below. */
                tx_final_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_final_msg[ALL_MSG_TAG_IDX] = TAG_ID;
                dwt_writetxdata(sizeof(tx_final_msg), tx_final_msg, 0);
                dwt_writetxfctrl(sizeof(tx_final_msg), 0);

                //TODO maybe need longer time
                //dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                //dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*2);
                dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED );

                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                /* Increment frame sequence number after transmission of the poll message (modulo 256). */
                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good/fail RX frame event in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    if(rx_buffer[ALL_MSG_TAG_IDX] != TAG_ID)
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;

                    /*As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */
                    rx_buffer[ALL_MSG_SN_IDX] = 0;

                    if (memcmp(rx_buffer, distance_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                       // final_distance = rx_buffer[10] + (float)rx_buffer[11]/100;
                        Anthordistance[rx_buffer[12]] +=(rx_buffer[10]*1000 + rx_buffer[11]*10);
                        Anthordistance_count[rx_buffer[12]] ++;
                        {
                            int Anchor_Index = 0;
                            while(Anchor_Index < ANCHOR_MAX_NUM)
                            {
                                if(Anthordistance_count[Anchor_Index] >=ANCHOR_REFRESH_COUNT )
                                {
                                    distance_mange();
                                    Anchor_Index = 0;
									//clear all
                                    while(Anchor_Index < ANCHOR_MAX_NUM)
                                    {
                                        Anthordistance_count[Anchor_Index] = 0;
                                        Anthordistance[Anchor_Index] = 0;
										Anchor_Index++;
                                    }
                                    break;
                                }
								Anchor_Index++;
                            }
                        }
                    }
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            // sprintf(dist_str, "%08x",status_reg);
            // OLED_ShowString(0, 2,"           ");
            // OLED_ShowString(0, 2,dist_str);
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
        /* Execute a delay between ranging exchanges. */
        // deca_sleep(RNG_DELAY_MS);
        frame_seq_nb++;
    }

}
#if 0
typedef struct {
    float x;
    float y;
} Point2D;

// 800 600 133


#define WINDOW_WIDTH  133   // 윈도우 너비 (픽셀)
#define WINDOW_HEIGHT 133   // 윈도우 높이 (픽셀)
#define SCALE_FACTOR 100    // 1m를 100픽셀로 표현

typedef struct {
    int x;
    int y;
} WindowPoint;
#endif

typedef struct {
    float x;
    float y;
} Point2D;

typedef struct {
    int x;
    int y;
} WindowPoint;

typedef struct {
    Point2D anchors[3];          
    float prev_pos[2];           
    float prev_cov[2][2];        
    uint8_t is_initialized;      
} IndoorPositioning;

/*---------------------------------------------------------------*/
void init_positioning(IndoorPositioning* sys, 
                     float x1, float y1,   // Anchor 1 position
                     float x2, float y2,   // Anchor 2 position
                     float x3, float y3);  // Anchor 3 position

// Calculate position from distances
uint8_t calculate_position(IndoorPositioning* sys,
                         const float distances[3],
                         Point2D* result);

static void apply_kalman_filter(IndoorPositioning* sys, const Point2D* measured, Point2D* result);

#define Q_NOISE 10.0f    
#define R_NOISE 20.0f    
/*---------------------------------------------------------------*/

#if 0
float calculate_distance(Point2D p1, Point2D p2) {
    return sqrtf((p1.x - p2.x) * (p1.x - p2.x) + 
                 (p1.y - p2.y) * (p1.y - p2.y));
}
#endif

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
//   /*!< At this stage the microcontroller clock setting is already configured,
//        this is done through SystemInit() function which is called from startup
//        file (startup_stm32f10x_xx.s) before to branch to application main.
//        To reconfigure the default setting of SystemInit() function, refer to
//        system_stm32f10x.c file
//      */

double final_distance =  0;

//=============================================================//
/**************************************************************/
/********More Information Please Visit Our Website*************/
/***********************bphero.com.cn**************************/
/**********************Version V1.1****************************/
/**************************************************************/
//=============================================================//

int main(void)
{
    uint8 anthor_index = 0;
    uint8 tag_index = 0;

    uint8 Semaphore_Enable = 0 ;
    uint8 Waiting_TAG_Release_Semaphore = 0;
    int8 frame_len = 0;

    /* Start with board specific hardware init. */
    peripherals_init();
	
    printf("epic dwm1000!\r\n");

    /* Reset and initialise DW1000.
     * 초기화 시, DW1000 클록은 일시적으로 크리스탈 속도로 설정해야 합니다. 
     * 초기화가 완료된 후에는 최적화를 위해 SPI 속도를 증가시킬 수 있습니다. 
     * performance. */
    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
   
    spi_set_rate_low();
    if(dwt_initialise(DWT_LOADUCODE) == -1)
    {
        printf("dwm1000 init fail!\r\n");
        OLED_ShowString(0,0,"INIT FAIL");
        while (1)
        {
            STM_EVAL_LEDOn(LED1);
            deca_sleep(100);
            STM_EVAL_LEDOff(LED1);
            deca_sleep(100);
        }
    }
    spi_set_rate_high();

    /* Configure DW1000. See NOTE 6 below. */
    dwt_configure(&config);
    
    dwt_txconfig_t txconfig;
    configureTXPower(&txconfig);
    
    dwt_setleds(1);
    /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);
    //OLED_ShowString(0,0,"INIT PASS");

    printf("init pass!\r\n");
		
    #if 1
    AnchorList[0].x =0.12;
    AnchorList[0].y =0.34;
    AnchorList[0].z =0;

    AnchorList[1].x =0.25;
    AnchorList[1].y =0;
    AnchorList[1].z =0;

    AnchorList[2].x =0;
    AnchorList[2].y =0;
    AnchorList[2].z =0;
    #endif
    
    #if 0
    AnchorList[0].x =0;
    AnchorList[0].y =0;
    AnchorList[0].z =0;

    AnchorList[1].x =0;
    AnchorList[1].y =0;
    AnchorList[1].z =0;

    AnchorList[2].x =0;
    AnchorList[2].y =0;
    AnchorList[2].z =0;
    #endif

    int rx_ant_delay =32880;
    int index = 0 ;
#ifdef ANTHOR
    Anchor_Array_Init();
    /* Loop forever initiating ranging exchanges. */
    //OLED_ShowString(0,0,"DS TWR ANTHOR");
    //OLED_ShowString(0,2,"Distance:");

    //KalMan_PramInit();

    while (1)
    {
        /* Clear reception timeout to start next ranging process. */
        dwt_setrxtimeout(0);
        /* Activate reception immediately. */
        dwt_rxenable(0);

        /* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
        { };

        if (status_reg & SYS_STATUS_RXFCG)
        {
            /* Clear good RX frame event in the DW1000 status register. */
             dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

            /* A frame has been received, read it into the local buffer. */
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
            if (frame_len <= RX_BUFFER_LEN)
            {
                dwt_readrxdata(rx_buffer, frame_len, 0);
            }
            /* Check that the frame is a poll sent by "DS TWR initiator" example.
             * As the sequence number field of the frame is not relevant, it is cleared to simplify the validation of the frame. */

            if(rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM != ANCHOR_IND)
                continue;

            anthor_index = rx_buffer[ALL_MSG_SN_IDX]%ANCHOR_MAX_NUM;
            tag_index = rx_buffer[ALL_MSG_TAG_IDX];

            rx_buffer[ALL_MSG_SN_IDX] = 0;
            rx_buffer[ALL_MSG_TAG_IDX] = 0;

            if (memcmp(rx_buffer, rx_poll_msg, ALL_MSG_COMMON_LEN) == 0)
            {
                /* Retrieve poll reception timestamp. */
                poll_rx_ts = get_rx_timestamp_u64();           

                /* Set expected delay and timeout for final message reception. */
                dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
                dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);

                /* Write and send the response message. See NOTE 9 below.*/
                tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                tx_resp_msg[ALL_MSG_TAG_IDX] = tag_index;
                dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
                dwt_writetxfctrl(sizeof(tx_resp_msg), 0);
                dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
			  
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                if (status_reg & SYS_STATUS_RXFCG)
                {
                    /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);

                    /* A frame has been received, read it into the local buffer. */
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                    if (frame_len <= RX_BUF_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    if(tag_index != rx_buffer[ALL_MSG_TAG_IDX])
                        continue;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    if (memcmp(rx_buffer, rx_final_msg, ALL_MSG_COMMON_LEN) == 0)
                    {
                        uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
                        uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                        double Ra, Rb, Da, Db;
                        int64 tof_dtu;

                        /* Retrieve response transmission and final reception timestamps. */
                        resp_tx_ts = get_tx_timestamp_u64();
                        final_rx_ts = get_rx_timestamp_u64();

                        /* Get timestamps embedded in the final message. */
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
                        final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                        /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
                        poll_rx_ts_32 = (uint32)poll_rx_ts;
                        resp_tx_ts_32 = (uint32)resp_tx_ts;
                        final_rx_ts_32 = (uint32)final_rx_ts;
                        Ra = (double)(resp_rx_ts - poll_tx_ts);
                        Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                        Da = (double)(final_tx_ts - resp_rx_ts);
                        Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                        tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                        tof = tof_dtu * DWT_TIME_UNITS;
                        distance = tof * SPEED_OF_LIGHT;
                        distance = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//�����ȥ����ϵ��
                        // sprintf(dist_str, "dis: %3.2f m", distance);
                        //printf("before kalman fliter Distance:%3.2f m\r\n",rx_buffer[12],final_distance);
                        //kalman filter
                        //distance =  KalMan_Update(&distance);
                        //  sprintf(dist_str, "dis: %3.2f m", distance);
                        //    printf("after kalman fliter Distance:%3.2f m\r\n",rx_buffer[12],final_distance);
                        //�����������͸�TAG
                        int temp = (int)(distance*100);
                        distance_msg[10] = temp/100;
                        // a=x;  //�Զ�����ת����ȡ��������
                        distance_msg[11] = temp%100;  //��100���100ȡ�࣬�õ�2λС���������
                        distance_msg[12] = anthor_index;

                        distance_msg[ALL_MSG_SN_IDX] = frame_seq_nb;
                        distance_msg[ALL_MSG_TAG_IDX] = tag_index;
                        dwt_writetxdata(sizeof(distance_msg), distance_msg, 0);
                        dwt_writetxfctrl(sizeof(distance_msg), 0);

                        /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
                         * set by dwt_setrxaftertxdelay() has elapsed. */
                        dwt_starttx(DWT_START_TX_IMMEDIATE );
                      //  while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                      //  { };
                        /* Display computed distance on LCD. */
                        // OLED_ShowString(0,6,"                ");
                        sprintf(dist_str, "DIST: %3.2f m", distance);
                        //lcd_display_str(dist_str);
                        // OLED_ShowString(0,6,dist_str);
												printf(dist_str);
                    }
                }
                else
                {
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }

            else if (memcmp(rx_buffer, angle_msg, ALL_MSG_COMMON_LEN) == 0 && ANCHOR_IND == 0)
            {
                if(rx_buffer[LOCATION_FLAG_IDX] == 1)//location infomartion
                {
                    rx_buffer[ALL_MSG_TAG_IDX] = tag_index;
                    USART_puts(&rx_buffer[LOCATION_INFO_START_IDX],rx_buffer[LOCATION_INFO_LEN_IDX]);
                }
                else //follow car
                {
                    putchar(rx_buffer[10]);
                }
            }
        }
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        }
    }
#endif

#ifdef TAG
    /* Set expected response's delay and timeout. See NOTE 4 and 5 below.
     * As this example only handles one incoming frame with always the same delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    if(TAG_ID == MASTER_TAG)
    {
        //OLED_ShowString(0,0,"DS MASTER TAG:");
    }
    else
    {
        //OLED_ShowString(0,0,"DS SLAVE TAG:");
    }

    //OLED_ShowString(0,2,"Distance:");

    if(TAG_ID ==  MASTER_TAG)
    {
        Semaphore_Enable = 1 ;
        Semaphore_Init();
        Waiting_TAG_Release_Semaphore = 0;
    }
    else
    {
        Semaphore_Enable = 0 ;
    }
    //Master TAG0
    while(1)
    {
        if(Semaphore_Enable == 1)
        {
            GPIO_ResetBits(GPIOA,GPIO_Pin_1);
            GPIO_ResetBits(GPIOA,GPIO_Pin_2);

            //send message to anthor,TAG<->ANTHOR
            Tag_Measure_Dis();//measuer distance between tag and all anthor
            Semaphore_Enable = 0 ;

            if(TAG_ID != MASTER_TAG)
            {
                //send release semaphore to master tag
                Semaphore_Release[ALL_MSG_SN_IDX] = frame_seq_nb;
                Semaphore_Release[ALL_MSG_TAG_IDX] = TAG_ID;
                dwt_writetxdata(sizeof(Semaphore_Release), Semaphore_Release, 0);
                dwt_writetxfctrl(sizeof(Semaphore_Release), 0);

                dwt_starttx(DWT_START_TX_IMMEDIATE );
                while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                { };
                GPIO_SetBits(GPIOA,GPIO_Pin_2);
            }
        }

        if(TAG_ID == MASTER_TAG)//master  tag
        {
            //statistics tag
            if(Sum_Tag_Semaphore_request() == 0)
            {
                for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
                {
                    Tag_Statistics[ALL_MSG_SN_IDX] = 0;
                    Tag_Statistics[ALL_MSG_TAG_IDX] = tag_index;
                    dwt_writetxdata(sizeof(Tag_Statistics), Tag_Statistics, 0);
                    dwt_writetxfctrl(sizeof(Tag_Statistics), 0);
                    dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                    while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                    { };

                    if (status_reg & SYS_STATUS_RXFCG)
                    {
                        /* Clear good RX frame event and TX frame sent in the DW1000 status register. */
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                        /* A frame has been received, read it into the local buffer. */
                        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                        if (frame_len <= RX_BUF_LEN)
                        {
                            dwt_readrxdata(rx_buffer, frame_len, 0);
                        }
                        rx_buffer[ALL_MSG_SN_IDX] = 0;

                        if(rx_buffer[ALL_MSG_TAG_IDX] == tag_index)
                        {
                            uint8 temp = rx_buffer[ALL_MSG_TAG_IDX] ;
                            rx_buffer[ALL_MSG_TAG_IDX] =0;
                            if (memcmp(rx_buffer, Tag_Statistics_response, ALL_MSG_COMMON_LEN) == 0)
                            {
                                Semaphore[temp] = 1;
                                GPIO_SetBits(GPIOA,GPIO_Pin_2);
                            }
                        }
                    }
                    else
                    {
                        /* Clear RX error events in the DW1000 status register. */
                        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                        //GPIO_SetBits(GPIOA,GPIO_Pin_1);
                    }
                }
                //print all the tags in network
                for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
                {
                    if(Semaphore[tag_index] == 1)
                    {
                        // printf("Tag%d In NetWork!\r\n",tag_index);
                    }
                }
            }

            //pick one tag ,send Semaphore message
            //release to specific tag(TAG ID)
            //master tag send release signal,and the specific tag send comfirm message
            if(Waiting_TAG_Release_Semaphore == 0 && Sum_Tag_Semaphore_request() != 0)
            {
                Semaphore[0] = 0;//slave tag must not use tag_id = 0x00!!
                for(tag_index = SLAVE_TAG_START_INDEX; tag_index <MAX_SLAVE_TAG; tag_index++)
                {
                    if(Semaphore[tag_index] == 1)
                    {
                        // printf("Release Semaphore to Tag%d!\r\n",tag_index);
                        // dwt_setrxtimeout(0);

                        dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
                        dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
                        Master_Release_Semaphore[ALL_MSG_SN_IDX] = 0;
                        Master_Release_Semaphore[ALL_MSG_TAG_IDX] = tag_index;
                        dwt_writetxdata(sizeof(Master_Release_Semaphore), Master_Release_Semaphore, 0);
                        dwt_writetxfctrl(sizeof(Master_Release_Semaphore), 0);
                        dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);

                        while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                        { };

                        if (status_reg & SYS_STATUS_RXFCG)
                        {
                            GPIO_SetBits(GPIOA,GPIO_Pin_1);
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);
                            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;
                            if (frame_len <= RX_BUF_LEN)
                            {
                                dwt_readrxdata(rx_buffer, frame_len, 0);
                            }
                            rx_buffer[ALL_MSG_SN_IDX] = 0;

                            if(rx_buffer[ALL_MSG_TAG_IDX] == tag_index)
                            {
                                rx_buffer[ALL_MSG_TAG_IDX] = 0;
                                GPIO_SetBits(GPIOA,GPIO_Pin_3);
                                // USART_puts(rx_buffer,frame_len);
                                if (memcmp(rx_buffer, Master_Release_Semaphore_comfirm, ALL_MSG_COMMON_LEN) == 0)
                                {
                                    //if the tag recive a semaphore, wait release remaphore
                                    Waiting_TAG_Release_Semaphore ++;
                                    break;//only release one semphore once
                                }
                            }
                        }
                        else//the tag may leave net,clear semaphore
                        {
                            Semaphore[tag_index] = 0 ;
                            //printf("tag may leave net!\r\n");
                            //GPIO_SetBits(GPIOA,GPIO_Pin_1);
                            /* Clear RX error events in the DW1000 status register. */
                            //sprintf(dist_str, "%08x",status_reg);
                            //OLED_ShowString(0, 2,"           "); OLED_ShowString(0, 2,dist_str);
                            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                        }
                    }
                }
            }

            if(Waiting_TAG_Release_Semaphore == 0 )
            {
                // GPIO_SetBits(GPIOA,GPIO_Pin_2);GPIO_SetBits(GPIOA,GPIO_Pin_1);
            }
            //Master tag waitting for specific tag Semaphore Release message
            if( Waiting_TAG_Release_Semaphore >0)
            {
                //  printf("Waiting for Release Semaphore!\r\n");
                dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS*5);//about 10ms,need adjust!!
                dwt_rxenable(0);
                while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
                { };

                if (status_reg & SYS_STATUS_RXFCG)
                {
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);
                    frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                    if (frame_len <= RX_BUFFER_LEN)
                    {
                        dwt_readrxdata(rx_buffer, frame_len, 0);
                    }

                    rx_buffer[ALL_MSG_SN_IDX] = 0;
                    uint8 temp=rx_buffer[ALL_MSG_TAG_IDX] ;
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    if (memcmp(rx_buffer, Semaphore_Release, ALL_MSG_COMMON_LEN) == 0)
                    {
                        if(Semaphore[temp] == 1)
                        {
                            Semaphore[temp] = 0 ;
                            if(Waiting_TAG_Release_Semaphore > 0 )
                            {
                                Waiting_TAG_Release_Semaphore --;
                            }
                        }
                    }
                }
                else
                {
                    //maybe the tag leave network
                    if(Waiting_TAG_Release_Semaphore > 0)
                    {
                        Waiting_TAG_Release_Semaphore--;
                        Semaphore[tag_index] = 0 ;
                    }
                    /* Clear RX error events in the DW1000 status register. */
                    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
                }
            }
            //���ܴ��ڵ����⣬TAG�յ�Semaphore û���ͷž��뿪���磬����Master TAG�޷��ջ�Semaphore�������Ҫ��ʱ��ʵ�֣���ʱһ��ʱ�䣬����Ȼû���յ�TAG �ͷ�Semaphore����Ҫǿ��ȡ��
            //if all tag have serviced by  master tag
            //master tag can measure the distance
            if(Sum_Tag_Semaphore_request() == 0)
            {
                Semaphore_Enable = 1 ;
                Waiting_TAG_Release_Semaphore= 0;
            }
        }
        else  //slave tags
        {
            //SLAVE TAG ����Ĭ�ϵȴ�MASTER TAG����ͳ����Ϣ�Լ��ͷ��ź���
            dwt_setrxtimeout(0);
            dwt_rxenable(0);

            /* Poll for reception of a frame or error/timeout. See NOTE 7 below. */
            while (!((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR)))
            { };

            if (status_reg & SYS_STATUS_RXFCG)
            {
                /* Clear good RX frame event in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG|SYS_STATUS_TXFRS);//clear rx & tx flag at the same time

                /* A frame has been received, read it into the local buffer. */
                frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;
                if (frame_len <= RX_BUFFER_LEN)
                {
                    dwt_readrxdata(rx_buffer, frame_len, 0);
                }
                rx_buffer[ALL_MSG_SN_IDX] = 0;
                if(rx_buffer[ALL_MSG_TAG_IDX] == TAG_ID)
                {
                    rx_buffer[ALL_MSG_TAG_IDX] = 0;
                    if (memcmp(rx_buffer, Tag_Statistics, ALL_MSG_COMMON_LEN) == 0)
                    {
                        //GPIO_SetBits(GPIOA,GPIO_Pin_3);
                        Tag_Statistics_response[ALL_MSG_SN_IDX] = frame_seq_nb;
                        Tag_Statistics_response[ALL_MSG_TAG_IDX] = TAG_ID;
                        dwt_writetxdata(sizeof(Tag_Statistics_response), Tag_Statistics_response, 0);
                        dwt_writetxfctrl(sizeof(Tag_Statistics_response), 0);
                        dwt_starttx(DWT_START_TX_IMMEDIATE );

                        GPIO_SetBits(GPIOA,GPIO_Pin_2);
                    }

                    if (memcmp(rx_buffer, Master_Release_Semaphore, ALL_MSG_COMMON_LEN) == 0)
                    {
                        Master_Release_Semaphore_comfirm[ALL_MSG_SN_IDX] = frame_seq_nb;
                        Master_Release_Semaphore_comfirm[ALL_MSG_TAG_IDX] = TAG_ID;
                        dwt_writetxdata(sizeof(Master_Release_Semaphore_comfirm), Master_Release_Semaphore_comfirm, 0);
                        dwt_writetxfctrl(sizeof(Master_Release_Semaphore_comfirm), 0);

                        dwt_starttx(DWT_START_TX_IMMEDIATE);
                        while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
                        { };

                        Semaphore_Enable = 1;
                        GPIO_SetBits(GPIOA,GPIO_Pin_1);
                    }
                }
            }
            else
            {
                /* Clear RX error events in the DW1000 status register. */
                dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
            }
        }
    }
#endif
}

#define Filter_N 5  //max filter use in this system
#define Filter_D 5  //each filter contain "Filter_D" data
int Value_Buf[Filter_N][Filter_D]= {0};
int filter_index[Filter_N] = {0};
int filter(int input, int fliter_idx )
{
    char count = 0;
    int sum = 0;
    if(input > 0)
    {
        Value_Buf[fliter_idx][filter_index[fliter_idx]++]=input;
        if(filter_index[fliter_idx] == Filter_D) filter_index[fliter_idx] = 0;

        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }
    else
    {
        for(count = 0; count<Filter_D; count++)
        {
            sum += Value_Buf[fliter_idx][count];
        }
        return (int)(sum/Filter_D);
    }

}

void init_positioning(IndoorPositioning* sys, 
                     float x1, float y1,
                     float x2, float y2,
                     float x3, float y3) {
    sys->anchors[0].x = x1;
    sys->anchors[0].y = y1;
    sys->anchors[1].x = x2;
    sys->anchors[1].y = y2;
    sys->anchors[2].x = x3;
    sys->anchors[2].y = y3;
    
    sys->is_initialized = 0;
    
    // Initialize covariance matrix
    sys->prev_cov[0][0] = 100.0f;
    sys->prev_cov[0][1] = 0.0f;
    sys->prev_cov[1][0] = 0.0f;
    sys->prev_cov[1][1] = 100.0f;
}

static void matrix_2x2_inverse(const float input[2][2], float output[2][2]){
    float det = input[0][0] * input[1][1] - input[0][1] * input[1][0];
    if (fabsf(det) < 1e-6f){
        output[0][0] = output[1][1] = 1.0f;
        output[0][1] = output[1][0] = 0.0f;
        return;
    }
    float inv_det = 1.0f / det;
    output[0][0] = input[1][1] * inv_det;
    output[0][1] = -input[0][1] * inv_det;
    output[1][0] = -input[1][0] * inv_det;
    output[1][1] = input[0][0] * inv_det;
}

uint8_t calculate_position(IndoorPositioning* sys, const float distances[3], Point2D* result)
{
    float x1 = sys->anchors[0].x;
    float y1 = sys->anchors[0].y;
    float x2 = sys->anchors[1].x;
    float y2 = sys->anchors[1].y;
    float x3 = sys->anchors[2].x;
    float y3 = sys->anchors[2].y;

    float r1 = distances[0];
    float r2 = distances[1];
    float r3 = distances[2];

    float A = 2.0f * (x2 - x1);
    float B = 2.0f * (y2 - y1);
    float C = r1*r1 - r2*r2 - x1*x1 + x2*x2 - y1*y1 + y2*y2;
    float D = 2.0f * (x3 - x1);
    float E = 2.0f * (y3 - y1);
    float F = r1*r1 - r3*r3 - x1*x1 + x3*x3 - y1*y1 + y3*y3;

    // Solve system of equations
    float det = A*E - B*D;
    if (fabsf(det) < 1e-6f) {
        return 0; // Cannot solve
    }
    
    Point2D raw_pos;
    raw_pos.x = (C*E - B*F) / det;
    raw_pos.y = (A*F - C*D) / det;

    apply_kalman_filter(sys, &raw_pos, result);

    return 1;
}

static void apply_kalman_filter(IndoorPositioning* sys, const Point2D* measured, Point2D* result)
{
    if (!sys->is_initialized) {
        result->x = measured->x;
        result->y = measured->y;
        sys->prev_pos[0] = measured->x;
        sys->prev_pos[1] = measured->y;
        sys->is_initialized = 1;
        return;
    }

    float predicted_pos[2] = {sys->prev_pos[0], sys->prev_pos[1]};
    float predicted_cov[2][2];
    predicted_cov[0][0] = sys->prev_cov[0][0] + Q_NOISE;
    predicted_cov[0][1] = sys->prev_cov[0][1];
    predicted_cov[1][0] = sys->prev_cov[1][0];
    predicted_cov[1][1] = sys->prev_cov[1][1] + Q_NOISE;

    float S[2][2];
    S[0][0] = predicted_cov[0][0] + R_NOISE;
    S[0][1] = predicted_cov[0][1];
    S[1][0] = predicted_cov[1][0];
    S[1][1] = predicted_cov[1][1] + R_NOISE;

    float S_inv[2][2];
    matrix_2x2_inverse(S, S_inv);
    
    float K[2][2];
    K[0][0] = predicted_cov[0][0] * S_inv[0][0] + predicted_cov[0][1] * S_inv[1][0];
    K[0][1] = predicted_cov[0][0] * S_inv[0][1] + predicted_cov[0][1] * S_inv[1][1];
    K[1][0] = predicted_cov[1][0] * S_inv[0][0] + predicted_cov[1][1] * S_inv[1][0];
    K[1][1] = predicted_cov[1][0] * S_inv[0][1] + predicted_cov[1][1] * S_inv[1][1];

    float innovation[2] = {
        measured->x - predicted_pos[0],
        measured->y - predicted_pos[1]
    };

    result->x = predicted_pos[0] + K[0][0] * innovation[0] + K[0][1] * innovation[1];
    result->y = predicted_pos[1] + K[1][0] * innovation[0] + K[1][1] * innovation[1];

    sys->prev_cov[0][0] = (1.0f - K[0][0]) * predicted_cov[0][0];
    sys->prev_cov[0][1] = (1.0f - K[0][1]) * predicted_cov[0][1];
    sys->prev_cov[1][0] = (1.0f - K[1][0]) * predicted_cov[1][0];
    sys->prev_cov[1][1] = (1.0f - K[1][1]) * predicted_cov[1][1];

    sys->prev_pos[0] = result->x;
    sys->prev_pos[1] = result->y;
}



static Point2D prev_pos = {0.0f, 0.0f};  // 이전 좌표 저장
float MAX_CHANGE = 5.0f;  // 최대 이동 허용 범위 (cm)
Point2D smooth_position(Point2D new_pos) {
    if (fabs(new_pos.x - prev_pos.x) > MAX_CHANGE ||
        fabs(new_pos.y - prev_pos.y) > MAX_CHANGE) {
        // 변동이 너무 크면 이전 좌표 반환
        return prev_pos;
    }
    // 변동이 적당하면 좌표 업데이트
    prev_pos = new_pos;
    return new_pos;
}

#if 0
WindowPoint convert_to_window_coordinates(float x, float y)
{
    WindowPoint window_pos;
    
    // cm 단위로 스케일링
    const float SCALE_X = 1920.0f / 170.0f;  // pixels/cm (너비 기준)             // 119 --> 170
    const float SCALE_Y = 1080.0f / 170.0f;   // pixels/cm (높이 기준)             // 60  --> 90
    
    window_pos.x = (int)(x * SCALE_X);  // cm 단위 스케일링
    window_pos.y = (int)(y * SCALE_Y);  // cm 단위 스케일링
    
    // 좌표 범위 제한
    window_pos.x = fmin(fmax(window_pos.x, 0), 1919);
    window_pos.y = fmin(fmax(window_pos.y, 0), 1079);
    
    return window_pos;
}
#endif

WindowPoint convert_to_window_coordinates(float x, float y)
{
    WindowPoint window_pos;
    
    // 1. 입력 좌표의 실제 범위 정의
    const float MIN_X = -50.0f;     // 실제 좌표의 최소 X값 (여유분 포함)    // -5
    const float MAX_X = 170.0f;     // 실제 좌표의 최대 X값
    const float MIN_Y = -50.0f;       // 실제 좌표의 최소 Y값                 // 0.0
    const float MAX_Y = 170.0f;     // 실제 좌표의 최대 Y값 (90을 초과하는 경우 대비)
    
    // 2. 입력 좌표를 0~1 범위로 정규화
    float normalized_x = (x - MIN_X) / (MAX_X - MIN_X);
    float normalized_y = (y - MIN_Y) / (MAX_Y - MIN_Y);
    
    // 3. 정규화된 값을 화면 좌표로 변환
    window_pos.x = (int)(normalized_x * 1920.0f);
    window_pos.y = (int)(normalized_y * 1080.0f);
    
    // 4. 좌표가 화면 범위를 벗어나지 않도록 제한
    // 음수 좌표나 범위 초과 좌표도 적절히 표시됨
    window_pos.x = (window_pos.x < 0) ? 0 : (window_pos.x > 1919 ? 1919 : window_pos.x);
    window_pos.y = (window_pos.y < 0) ? 0 : (window_pos.y > 1079 ? 1079 : window_pos.y);
    
    return window_pos;
}

#define FILTER_WINDOW_SIZE 5  // 이동 평균 창 크기

float moving_average_filter(float new_value, float *buffer, int *count){
    float sum = 0.0f;
    buffer[*count % FILTER_WINDOW_SIZE] = new_value; 
    (*count)++;
    int size = (*count < FILTER_WINDOW_SIZE) ? *count : FILTER_WINDOW_SIZE;

    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;  // 평균값 반환
}

#if 0
static void distance_mange(void)
{
    char dist_str0[16], dist_str1[16], dist_str2[16];
    char coord_str[32];                                   // 32
    float distance0 = 0, distance1 = 0, distance2 = 0;
    float raw_distance0 = 0, raw_distance1 = 0, raw_distance2 = 0; 
    
    static float buffer0[FILTER_WINDOW_SIZE] = {0};
    static float buffer1[FILTER_WINDOW_SIZE] = {0};
    static float buffer2[FILTER_WINDOW_SIZE] = {0};
    static int count0 = 0, count1 = 0, count2 = 0;
    
    #if 0
    Point2D anchor1 = {0.0, 0.0};        // 원점 좌상단
    Point2D anchor2 = {1.60, 0.0};       // 우상단 (119cm = 1.19m)        
    Point2D anchor3 = {0.0, 0.75};      // 우하단 (60cm = 0.60m)
    #endif
    #if 0
    Point2D anchor1 = {0.0, 0.0};        // 원점 좌상단
    Point2D anchor2 = {1.19, 0.0};       // 우상단 (119cm = 1.19m)        
    Point2D anchor3 = {1.19, 0.60};      // 우하단 (60cm = 0.60m)
    #endif
    Point2D anchor1 = {0.0f, 0.5f};        // 원점 좌상단
    Point2D anchor2 = {119.0f, 0.5f};       // 우상단 (119cm = 1.19m)             
    Point2D anchor3 = {119.0f, 60.5f};      // 우하단 (60cm = 0.60m)
    {
        int Anchor_Index = 0;
        while(Anchor_Index < ANCHOR_MAX_NUM)
        {
            if(Anthordistance_count[Anchor_Index] > 0 )
            {
                Anthordistance[Anchor_Index] =filter((int)(Anthordistance[Anchor_Index]/Anthordistance_count[Anchor_Index]),Anchor_Index);
            }
			      Anchor_Index++;
        }    
    }
    if(Anthordistance_count[0]>0)
    {
        raw_distance0 = (float)Anthordistance[0]/10.0f;  // mm to m
        distance0 = moving_average_filter(raw_distance0, buffer0, &count0);  // 필터 적용
        sprintf(dist_str0, "a0:%3.2fm", distance0);
        //USART_puts((char*)dist_str0, 16);
    }
    if(Anthordistance_count[1]>0)
    {
        raw_distance1 = (float)Anthordistance[1]/10.0f;
        distance1 = moving_average_filter(raw_distance1, buffer1, &count1);
        sprintf(dist_str1, "a1:%3.2fm", distance1);
        //USART_puts((char*)dist_str1, 16);
    }
    if(Anthordistance_count[2]>0)
    {
        raw_distance2 = (float)Anthordistance[2]/10.0f;
        distance2 = moving_average_filter(raw_distance2, buffer2, &count2);
        sprintf(dist_str2, "a2:%3.2fm", distance2);
        //USART_puts((char*)dist_str2, 16);    
    }
    if(distance0 > 0.0f && distance1 > 0.0f && distance2 > 0.0f) {
        //Point2D real_position = calculate_position(distance0, distance1, distance2, anchor1, anchor2, anchor3);
        //sprintf(coord_str, "Real(%.2f,%.2f)\n", real_position.x, real_position.y);
        //int len = strlen(coord_str);
        //USART_puts((char*)coord_str, len);
        
        //real_position = smooth_position(real_position);    
        //WindowPoint window_pos = convert_to_window_coordinates(real_position.x, real_position.y);
        
        // 디버깅을 위해 실제 위치도 출력
        //sprintf(coord_str, "Real(%.2f,%.2f) Win(%d,%d)\n", 
                //real_position.x, real_position.y, window_pos.x, window_pos.y);
        //int len = strlen(coord_str);
        //USART_puts((char*)coord_str, len);
    } else {
        USART_puts("Invalid distances", 16);
    }
}
#endif

static void distance_mange(void){
    char dist_str0[16], dist_str1[16], dist_str2[16];
    char coord_str[32];
    float distance0 = 0, distance1 = 0, distance2 = 0;
    float raw_distance0 = 0, raw_distance1 = 0, raw_distance2 = 0;

    static float buffer0[FILTER_WINDOW_SIZE] = {0};
    static float buffer1[FILTER_WINDOW_SIZE] = {0};
    static float buffer2[FILTER_WINDOW_SIZE] = {0};
    static int count0 = 0, count1 = 0, count2 = 0;

    static IndoorPositioning positioning_system;
    static uint8_t is_system_initialized = 0;

    if (!is_system_initialized) {
        init_positioning(&positioning_system,
                        0.0f, 0.5f,      
                        119.0f, 0.5f,    
                        119.0f, 60.5f);  
        is_system_initialized = 1;
    }
    
    int Anchor_Index = 0;
    while(Anchor_Index < ANCHOR_MAX_NUM){
        if(Anthordistance_count[Anchor_Index] > 0 )
        {
            Anthordistance[Anchor_Index] =filter((int)(Anthordistance[Anchor_Index]/Anthordistance_count[Anchor_Index]),Anchor_Index);
        }
		Anchor_Index++;
    }
    if(Anthordistance_count[0] > 0)
    {
        raw_distance0 = (float)Anthordistance[0]/10.0f;
        distance0 = moving_average_filter(raw_distance0, buffer0, &count0);
        //sprintf(dist_str0, "a0:%3.2fm", distance0);
        //USART_puts((char*)dist_str0, 16);
    }
    if(Anthordistance_count[1] > 0)
    {
        raw_distance1 = (float)Anthordistance[1]/10.0f;
        distance1 = moving_average_filter(raw_distance1, buffer1, &count1);
        //sprintf(dist_str1, "a1:%3.2fm", distance1);
        //USART_puts((char*)dist_str1, 16);
    }
    if(Anthordistance_count[2] > 0)
    {
        raw_distance2 = (float)Anthordistance[2]/10.0f;
        distance2 = moving_average_filter(raw_distance2, buffer2, &count2);
        //sprintf(dist_str2, "a2:%3.2fm", distance2);
        //USART_puts((char*)dist_str2, 16);
    }
    sprintf(coord_str, "d0:%3.2f,d1:%3.2f,d2:%3.2f\n", distance0, distance1, distance2);
    int len = strlen(coord_str);
    USART_puts((char*)coord_str, len);
    deca_sleep(RNG_DELAY_MS);
    #if 0
    if(distance0 > 0.0f && distance1 > 0.0f && distance2 > 0.0f){
        Point2D calculated_position;
        float distances[3] = {distance0, distance1, distance2};

        if (calculate_position(&positioning_system, distances, &calculated_position)){
            WindowPoint window_pos = convert_to_window_coordinates(calculated_position.x, calculated_position.y);
            //sprintf(coord_str, "Pos(%.2f,%.2f)\n", calculated_position.x, calculated_position.y);
            sprintf(coord_str, "Real(%.2f,%.2f) Win(%d,%d)\n", calculated_position.x, calculated_position.y, window_pos.x, window_pos.y);
            //sprintf(coord_str, "Pos(%d,%d)\n", window_pos.x, window_pos.y);
            int len = strlen(coord_str);
            USART_puts((char*)coord_str, len);
            deca_sleep(RNG_DELAY_MS);

        }
        else{
            USART_puts("Position calculation failed", 24);
        }
    }
    else{
        USART_puts("Invalid distances", 16);
    }
    #endif
}

 
#define DISTANCE3 0.27

//**************************************************************//
//distance1 anthor0 <--> TAG  mm
//distance2 anthor1 <--> TAG  mm
//distance3 anthor2 <--> TAG  mm
//**************************************************************//
static void compute_angle_send_to_anthor0(int distance1, int distance2,int distance3)
{
    static int framenum = 0 ;

#if 0 //compute angle for smartcar
    float dis3_constans = DISTANCE3;
    float cos = 0;
    float angle = 0 ;
    float dis1 = (float)distance1/1000; //m
    float dis2 = (float)distance2/1000;  //m

    if(dis1 + dis3_constans < dis2 || dis2+dis3_constans < dis1)
    {
        //printf("ERROR!\r\n");
        //return;
    }
    cos = (dis1*dis1 + dis3_constans* dis3_constans - dis2*dis2)/(2*dis1*dis3_constans);
    angle  = acos(cos)*180/3.1415926;
    printf("cos = %f, arccos = %f\r\n",cos,angle);
    sprintf(dist_str, "angle: %3.2f m", angle);
    printf(dist_str);
	//OLED_ShowString(0, 6,"            ");
    //OLED_ShowString(0, 6,dist_str);

    if(dis1 > 1)
    {
        if(angle > 110)
        {
            printf("turn right\r\n");
            angle_msg[10] = 'R';
        }
        else if(angle < 75)
        {
            printf("turn left\r\n");
            angle_msg[10] = 'L';
        }
        else
        {
            printf("forward\r\n");
            angle_msg[10] = 'F';
        }
    }
    else
    {
        printf("stay here\r\n");
        angle_msg[10] = 'S';
    }
    angle_msg[LOCATION_FLAG_IDX] = 0;

#else
    //location
    {
        uint8 len = 0;
        angle_msg[LOCATION_FLAG_IDX] = 1;

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'm';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 'r';

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = 0x02;
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = TAG_ID;//TAG ID

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)(framenum&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((framenum>>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);

        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance2/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] =  (uint8)((distance3/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = (uint8)((distance1/10 >>8)&0xFF);
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\n';
        angle_msg[LOCATION_INFO_START_IDX + (len++)] = '\r';


        angle_msg[LOCATION_INFO_LEN_IDX] = len;
        //MAX LEN
        if(LOCATION_INFO_START_IDX + len -2 >ANGLE_MSG_MAX_LEN)
        {
            while(1);//toggle LED
        }
        //USART_puts((char*)dist_str,16);

    }
#endif
    //only anthor0 recive angle message
    angle_msg[ALL_MSG_SN_IDX] = framenum;
    angle_msg[ALL_MSG_TAG_IDX] = TAG_ID;

    dwt_writetxdata(sizeof(angle_msg), angle_msg, 0);
    dwt_writetxfctrl(sizeof(angle_msg), 0);

    /* Start transmission, indicating that a response is expected so that reception is enabled automatically after the frame is sent and the delay
     * set by dwt_setrxaftertxdelay() has elapsed. */
    dwt_starttx(DWT_START_TX_IMMEDIATE );
    while (!(dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS))
    { };

    framenum++;

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        *ts += ts_field[i] << (i * 8);
    }
}
/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

    /* Infinite loop */
    while (1)
    {
    }
}
#endif

PUTCHAR_PROTOTYPE
{
    /* Place your implementation of fputc here */
    /* ��SR�Ĵ����е�TC��־ */

    USART_ClearFlag(EVAL_COM1,USART_FLAG_TC);
    /* e.g. write a character to the USART */
    USART_SendData(EVAL_COM1, (uint8_t) ch);
    /* Loop until the end of transmission */
    while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_TC) == RESET)
    {}
    return ch;
}


/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 8. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()). It is also to be noted that, when using
 *    delayed send, the time set for transmission must be far enough in the future so that the DW1000 IC has the time to process and start the
 *    transmission of the frame at the wanted time. If the transmission command is issued too late compared to when the frame is supposed to be sent,
 *    this is indicated by an error code returned by dwt_starttx() API call. Here it is not tested, as the values of the delays between frames have
 *    been carefully defined to avoid this situation.
 * 10. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/

/**
  * @}
  */
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
