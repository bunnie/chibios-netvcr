/*
    ChibiOS - Copyright (C) 2013-2015 Fabio Utzig

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

/**
 * @file    KL2x/serial_lld.c
 * @brief   Kinetis KL2x Serial Driver subsystem low level driver source.
 *
 * @addtogroup SERIAL
 * @{
 */

#include "osal.h"
#include "hal.h"

#include "kl02x.h"

/*===========================================================================*/
/* Driver local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Driver exported variables.                                                */
/*===========================================================================*/

/**
 * @brief   SD1 driver identifier.
 */
SerialDriver SD1;

/*===========================================================================*/
/* Driver local variables and types.                                         */
/*===========================================================================*/

/**
 * @brief   Driver default configuration.
 */
static const SerialConfig default_config = {
  115200
};

typedef struct ser_buf_ {
  unsigned char buf[BUF_SIZE];
  uint8_t rd_ptr;
  uint8_t wr_ptr;
} ser_buf;

#define BUF_SIZE 16
static ser_buf rx_buf;
static ser_buf tx_buf;

static unsigned char rxbuf[BUF_SIZE];
static uint8_t rxbuf_rd_ptr = 0;
static uint8_t rxbuf_wr_ptr = 0;

static unsigned char txbuf[BUF_SIZE];
static uint8_t txbuf_rd_ptr = 0;
static unit8_t txbuf_wr_ptr = 0;

/*===========================================================================*/
/* Driver local functions.                                                   */
/*===========================================================================*/

static uint32_t read_buf(ser_buf *sb) {
  unsigned char retc;
  if( sb->wr_ptr == sb->rd_ptr )
    return -1;

  retc = sb->buf[sb->rd_ptr];
  sb->rd_ptr = (sb->rd_ptr + 1) % BUF_SIZE;

  return ((uint32_t) retc) & 0xFF;
}

static uint32_t write_buf(unsigned char c, ser_buf *sb) {
  uint8_t wrnext = (sb->wr_ptr + 1) % BUF_SIZE;

  if( wrnext == sb->rd_ptr )
    return -1;

  sb->buf[sb->wr_ptr] = c;
  sb->wr_ptr = wrnext;

  return 0;
}

/**
 * @brief   Common IRQ handler.
 * @note    Tries hard to clear all the pending interrupt sources, we don't
 *          want to go through the whole ISR and have another interrupt soon
 *          after.
 *
 * @param[in] u         pointer to an UART I/O block
 * @param[in] sdp       communication channel associated to the UART
 */
static void serve_interrupt(SerialDriver *sdp) {
  UARTLP_TypeDef *u = sdp->uart;

  if (u->S1 & UARTx_S1_RDRF) {
    write_buf(u->D, &rx_buf);
  }

  if (u->S1 & UARTx_S1_TDRE) {
    msg_t b;

    b = read_buf(&tx_buf);
    if( b >= 0 ) {
       u->D = b;
    }
  }

  if (u->S1 & UARTx_S1_IDLE)
    u->S1 = UARTx_S1_IDLE;  // Clear IDLE (S1 bits are write-1-to-clear).

  if (u->S1 & (UARTx_S1_OR | UARTx_S1_NF | UARTx_S1_FE | UARTx_S1_PF)) {
    // FIXME: need to add set_error()
    // Clear flags (S1 bits are write-1-to-clear).
    u->S1 = UARTx_S1_OR | UARTx_S1_NF | UARTx_S1_FE | UARTx_S1_PF;
  }
}

// returns character if available, otherwise -1
uint32_t getc_poll(void) {
  return read_buf(&rx_buf);
}

// returns -1 if buffer is overflow
uint32_t putc_x(void *p, char c) {
  (void) p;
  
  UARTLP_TypeDef *u = sdp->uart;

  if ( (u->S1 & UARTx_S1_TDRE) ) {
    if( tx_buf.wr_ptr == tx_buf.rd_ptr ) {
      // if txbuf empty, send the character
      u->D = c;
      u->C2 |= UARTx_C2_TIE;
      return 0;
    } else {
      // otherwise, send the oldest txbuf entry, and enqueue this character to send
      u->D = (unsigned char) read_buf(&tx_buf);
      return write_buf(c, &tx_buf);
    }
  } else {
    // at the moment, if we overflow, just lose characters
    // this is mostly for debug & status output anyways
    return write_buf(c, &tx_buf);
  }
}

/**
 * @brief   Common UART configuration.
 *
 */
static void configure_uart(UARTLP_TypeDef *uart, const SerialConfig *config)
{
  uint32_t uart_clock;

  uart->C1 = 0;
  uart->C3 = UARTx_C3_ORIE | UARTx_C3_NEIE | UARTx_C3_FEIE | UARTx_C3_PEIE;
  uart->S1 = UARTx_S1_IDLE | UARTx_S1_OR | UARTx_S1_NF | UARTx_S1_FE | UARTx_S1_PF;
  while (uart->S1 & UARTx_S1_RDRF) {
    (void)uart->D;
  }

    if (uart == UART0) {
        /* UART0 can be clocked from several sources. */
        uart_clock = KINETIS_UART0_CLOCK_FREQ;
    }

  /* FIXME: change fixed OSR = 16 to dynamic value based on baud */
  uint16_t divisor = (uart_clock / 16) / config->sc_speed;
  uart->C4 = UARTx_C4_OSR & (16 - 1);
  uart->BDH = (divisor >> 8) & UARTx_BDH_SBR;
  uart->BDL = (divisor & UARTx_BDL_SBR);

  uart->C2 = UARTx_C2_RE | UARTx_C2_RIE | UARTx_C2_TE;
}

/*===========================================================================*/
/* Driver interrupt handlers.                                                */
/*===========================================================================*/

OSAL_IRQ_HANDLER(Vector70) {

  OSAL_IRQ_PROLOGUE();
  serve_interrupt(&SD1);
  OSAL_IRQ_EPILOGUE();
}

/*===========================================================================*/
/* Driver exported functions.                                                */
/*===========================================================================*/

/**
 * @brief   Low level serial driver initialization.
 *
 * @notapi
 */
void sd_lld_init(void) {

  /* Driver initialization.*/
  sdObjectInit(&SD1, NULL, notify1);
  SD1.uart = UART0;
  
  init_printf(NULL,putc_x);

  rx_buf.rd_ptr = 0; rx_buf.wr_ptr = 0;
  tx_buf.rd_ptr = 0; tx_buf.wr_ptr = 0;
}

/**
 * @brief   Low level serial driver configuration and (re)start.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 * @param[in] config    the architecture-dependent serial driver configuration.
 *                      If this parameter is set to @p NULL then a default
 *                      configuration is used.
 *
 * @notapi
 */
void sd_lld_start(SerialDriver *sdp, const SerialConfig *config) {

  if (config == NULL)
    config = &default_config;

  if (sdp->state == SD_STOP) {
    /* Enables the peripheral.*/

    if (sdp == &SD1) {
      SIM->SCGC4 |= SIM_SCGC4_UART0;
      SIM->SOPT2 =
              (SIM->SOPT2 & ~SIM_SOPT2_UART0SRC_MASK) |
              SIM_SOPT2_UART0SRC(KINETIS_UART0_CLOCK_SRC);
      configure_uart(sdp->uart, config);
      nvicEnableVector(UART0_IRQn, KINETIS_SERIAL_UART0_PRIORITY);
    }
  }
  /* Configures the peripheral.*/

}

/**
 * @brief   Low level serial driver stop.
 * @details De-initializes the USART, stops the associated clock, resets the
 *          interrupt vector.
 *
 * @param[in] sdp       pointer to a @p SerialDriver object
 *
 * @notapi
 */
void sd_lld_stop(SerialDriver *sdp) {

  if (sdp->state == SD_READY) {
    /* TODO: Resets the peripheral.*/

    if (sdp == &SD1) {
      nvicDisableVector(UART0_IRQn);
      SIM->SCGC4 &= ~SIM_SCGC4_UART0;
    }

}


/** @} */
