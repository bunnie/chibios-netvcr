/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"

#include "printf.h"

#include "orchard.h"

#include "analog.h"
#include "demod.h"
#include "mac.h"
#include "updater.h"

#include "murmur3.h"

#include <string.h>

void *stream;

#define DEBUG_STREAMING  0
#define OSCOPE_PROFILING 0
uint8_t screenpos = 0;

volatile uint8_t dataReadyFlag = 0; // global flag, careful of sync issues when multi-threaded...
volatile adcsample_t *bufloc;
size_t buf_n;

static const SerialConfig serialConfig = {
  115200,
};

static const ADCConfig adccfg1 = {
  /* Perform initial calibration */
  true
};

static void phy_demodulate(void) {
  int frames;

#if OSCOPE_PROFILING // pulse a gpio to easily measure CPU load of demodulation
  GPIOB->PSOR |= (1 << 6);   // sets to high
  GPIOB->PCOR |= (1 << 6);   // clears to low
  
  // this is happening once every 1.748ms with NB_FRAMES = 16, NB_SAMPLES = 8
  // computed about 0.0413ms -> 41.3us per call overhead for OS required ~2.5% overhead
#endif
  // demodulation handler based on microphone data coming in
  for( frames = 0; frames < NB_FRAMES; frames++ ) {
    FSKdemod(dm_buf + (frames * NB_SAMPLES), NB_SAMPLES, putBitMac); // putBitMac is callback to MAC layer
  }
  dataReadyFlag = 0;
}

__attribute__((noreturn))
void demod_loop(void) {
  uint32_t i;

  // stop systick interrupts
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
  nvicDisableVector(HANDLER_SYSTICK);
  
  // infinite loops, prevents other system items from starting
  while(TRUE) {
    pktPtr = 0;
    while( !pktReady ) {
      if( dataReadyFlag ) {
        // copy from the double-buffer into a demodulation buffer
        for( i = 0 ; i < buf_n; i++ ) { 
          dm_buf[i] = (int16_t) (((int16_t) bufloc[i]) - 2048);
        }
        // call handler, which includes the demodulation routine
        phy_demodulate();
      }
    }

    // unstripe the transition xor's used to keep baud sync
    if( (pktBuf[0] & PKTTYPE_MASK) == PKTTYPE_DATA ) {
      for( i = 0; i < PKT_LEN - 4; i++ ) {
        if( (i % 16) == 7 )
          pktBuf[i] ^= 0x55;
        else if( (i % 16) == 15)
          pktBuf[i] ^= 0xAA;
      }
    }
    
#define RAWDATA_CHECK 0
#if RAWDATA_CHECK
    uint32_t hash;
    uint32_t txhash;
    uint16_t pkt_len;
    // replace the code in this #if bracket with the storage flashing code
    if( (pktBuf[0] & PKTTYPE_MASK) == PKTTYPE_DATA ) {
      pkt_len = PKT_LEN;
      tfp_printf( "\n\r data packet:" );
    } else {
      tfp_printf( "\n\r control packet:" );
      pkt_len = CTRL_LEN;
    }
    
    for( i = 0; i < 16; i++ ) { // abridged dump
      if( i % 32 == 0 ) {
	tfp_printf( "\n\r" );
      }
      tfp_printf( "%02x", pktBuf[i] /* isprint(pktBuf[i]) ? pktBuf[i] : '.'*/ );
    }
    // check hash
    MurmurHash3_x86_32(pktBuf, pkt_len - 4 /* packet minus hash */, MURMUR_SEED_BLOCK, &hash);

    txhash = (pktBuf[pkt_len-4] & 0xFF) | (pktBuf[pkt_len-3] & 0xff) << 8 |
      (pktBuf[pkt_len-2] & 0xFF) << 16 | (pktBuf[pkt_len-1] & 0xff) << 24;
      
    tfp_printf( " tx: %08x rx: %08x\n\r", txhash, hash);
    if( txhash != hash ) {
      tfp_printf( " fail\n\r" );
    } else {
      tfp_printf( " pass\n\r" );
    }

    pktReady = 0; // we've extracted packet data, so clear the buffer flag
#else
    
    updaterPacketProcess(pktBuf);
#endif
  }
}

/**
 * @name    Alignment support macros
 */
/**
 * @brief   Alignment size constant.
 */
#define MEM_ALIGN_SIZE      sizeof(stkalign_t)

/**
 * @brief   Alignment mask constant.
 */
#define MEM_ALIGN_MASK      (MEM_ALIGN_SIZE - 1U)

/**
 * @brief   Alignment helper macro.
 */
#define MEM_ALIGN_PREV(p)   ((size_t)(p) & ~MEM_ALIGN_MASK)

/**
 * @brief   Alignment helper macro.
 */
#define MEM_ALIGN_NEXT(p)   MEM_ALIGN_PREV((size_t)(p) + MEM_ALIGN_MASK)

/**
 * @brief   Core memory status.
 *
 * @return              The size, in bytes, of the free core memory.
 *
 * @xclass
 */
static size_t chCoreGetStatusX(void) {
  uint8_t *nextmem;
  uint8_t *endmem;
  extern uint8_t __heap_base__[];
  extern uint8_t __heap_end__[];

  /*lint -save -e9033 [10.8] Required cast operations.*/
  nextmem = (uint8_t *)MEM_ALIGN_NEXT(__heap_base__);
  endmem = (uint8_t *)MEM_ALIGN_PREV(__heap_end__);
  /*lint restore*/

  /*lint -save -e9033 [10.8] The cast is safe.*/
  return (size_t)(endmem - nextmem);
  /*lint -restore*/
}

static void putc_x(void *storage, char c) {
  (void) storage;

  chnWrite(&SD1, (const uint8_t *) &c, 1);
}
  

/*
 * "main" thread, separate from idle thread
 */
static THD_WORKING_AREA(waThread1, 512);
static THD_FUNCTION(Thread1, arg) {
  (void)arg;

  GPIOB->PSOR |= (1 << 6);   // red off
  GPIOB->PCOR |= (1 << 7);   // green on
  GPIOB->PSOR |= (1 << 10);  // blue off

  // init the serial interface
  sdStart(&SD1, &serialConfig);
  //  sd_lld_init();
  //  sd_lld_start((&SD1), &serialConfig);
  init_printf(NULL,putc_x);
  stream = stream_driver;

  //chnWrite( &SD1, (const uint8_t *) "\r\n\r\nOrchard audio wtf loader.\r\n", 32);
  //chThdSleepMilliseconds(1000);
  tfp_printf( "\r\n\r\nOrchard audio bootloader.  Based on build %s\r\n", gitversion);
  tfp_printf( "core free memory : %d bytes\r\n", chCoreGetStatusX());
  chThdSleepMilliseconds(100); // give a little time for the status message to appear
  
  //i2cStart(i2cDriver, &i2c_config);
  adcStart(&ADCD1, &adccfg1);
  analogStart();
  
  demodInit();

  flashStart();

  /*
    clock rate: 0.020833us/clock, 13.3us/sample @ 75kHz
    jitter notes: 6.8us jitter on 1st cycle; out to 11.7us on last cycle
    each frame of 8 samples (call to FSKdemod() takes ~56.3us to process, with a ~2.5us gap between calls to FSKdemod()
    a total of 32 frames is taking:
       1.867-1.878ms (1.872ms mean) to process (random noise), 
       1.859-1.866ms (1.863ms mean) to process (0 tone),
       1.862-1.868ms (1.865ms mean) to process (1 tone),
       ** jitter seems to be data-dependent differences in code path length 
    every 3.480ms +/- 2us -> 261 samples. 
      +/-2us jitter due to ~when we catch 13.3us/sample edge vs system state (within synchronizing tolerance)
    **should be 3.413ms -> 256 samples, 67 microseconds are "extra" -> 5.025 -> 5 samples per 256 samples

    hypotheses:
      - actual effective sample rate is not 75kHz, it's 76.464kHz
        * measured rate = 13.34us(13.25-13.43us jitter spread) => 74.962kHz (within 500ppm correct)
        ** however! every 3.481ms (287Hz) we have an extra-wide gap at 87.76us (4.21k cycles), with a fast 
           second sample time of ~5.329us - 5.607us later (e.g., the natural next point to grab a sample). 
        ** this happens in the middle of the IRQ handler. the gap is actually from 87.07us-87.56us long.
        ** fwiw the actual ADC handler completes in 752ns fairly deterministically
      - we're deterministically missing 5 interrupts every cycle
      - there's a coding bug causing us to mis-count # samples
     
    other notes:
      - adding print's during runtime adds jitter to the processing time, 
        but the processing start is deterministic to within 1.8us
      - processing start determinism is improved by putting constant data in
      - we've counted 32 frames being processed during the processing times
   */
  //(gdb) x 0xe000e180   // shows the interrupts that are enabled
  //0xe000e180:0x00009000
  // x/32x 0xe000e400
  NVIC_SetPriority(ADC0_IRQn, 0);
  NVIC_SetPriority(UART0_IRQn, 3);

  while( !(((volatile SysTick_Type *)SysTick)->CTRL & SysTick_CTRL_COUNTFLAG_Msk) )
    ;
  SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
  
  NVIC_DisableIRQ(PendSV_IRQn);
  NVIC_DisableIRQ(SysTick_IRQn);
  //x/2x 0xe000ed1c
  NVIC_SetPriority(SVCall_IRQn, 3);
  NVIC_SetPriority(PendSV_IRQn, 3);
  NVIC_SetPriority(SysTick_IRQn, 3);
  NVIC_DisableIRQ(PendSV_IRQn);
  NVIC_DisableIRQ(SysTick_IRQn);
  
  analogUpdateMic();  // starts mic sampling loop (interrupt-driven and automatic)
  demod_loop();
}


/*
 * Threads static table, one entry per thread. The number of entries must
 * match NIL_CFG_NUM_THREADS.
 */
THD_TABLE_BEGIN
  THD_TABLE_ENTRY(waThread1, "demod", Thread1, NULL)
THD_TABLE_END
  

/*
 * Application entry point.
 */
int main(void)
{
  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  while(1)  /// this is now the "idle" thread
    ;

}

