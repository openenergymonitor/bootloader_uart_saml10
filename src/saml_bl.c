/**
 *
 * \brief UART bootloader for SAM L10
 *
 * Copyright (c) 2018 Microchip Technology Inc.
 *               2025 Angus Logan (awjlogan@gmail.com)
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the Licence at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

//-----------------------------------------------------------------------------
#include "saml10.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//-----------------------------------------------------------------------------
#define BL_REQ_PORT 0 // PA
#define BL_REQ_PIN  PIN_PA11

#define BL_LED_PORT 0
#define BL_LED_PIN  PIN_PA14
#define BL_LED_EN   1

#define BL_REG_PORT 0
#define BL_REG_PIN  PIN_PA00
#define BL_REG_HIGH 1
#define BL_REG_EN   1

#define UART_TX_PORT 0 // PA
#define UART_TX_PIN  PIN_PA08D_SERCOM2_PAD0
#define UART_TX_MUX  MUX_PA08D_SERCOM2_PAD0

#define UART_RX_PORT 0 // PA
#define UART_RX_PIN  PIN_PA09D_SERCOM2_PAD1
#define UART_RX_MUX  MUX_PA09D_SERCOM2_PAD1

#define UART_SERCOM          SERCOM2
#define UART_SERCOM_GCLK_ID  SERCOM2_GCLK_ID_CORE
#define UART_SERCOM_MASK_REG APBCMASK
#define UART_SERCOM_MASK_BIT MCLK_APBCMASK_SERCOM2
#define UART_SERCOM_TXPO     SERCOM_USART_CTRLA_TXPO(0 /*PAD2*/)
#define UART_SERCOM_RXPO     SERCOM_USART_CTRLA_RXPO(1 /*PAD3*/)

#define UART_BAUDRATE 115200

#define BOOTLOADER_SIZE   1024
#define APPLICATION_START (FLASH_ADDR + BOOTLOADER_SIZE)
#define BOOTPROT          0x2 /* Fuse value to use to protect 1K */

#define PAGE_SIZE            NVMCTRL_PAGE_SIZE
#define ERASE_BLOCK_SIZE     NVMCTRL_ROW_SIZE
#define PAGES_IN_ERASE_BLOCK NVMCTRL_ROW_PAGES

#define TIMER_INTERVAL 100 // ms

#define GUARD_SIZE    4
#define OFFSET_SIZE   4
#define SIZE_SIZE     4
#define CRC_SIZE      4
#define ARB_WORD_SIZE 4
#define DATA_SIZE     ERASE_BLOCK_SIZE

#define WORDS(x) ((int)((x) / sizeof(uint32_t)))

#define ALIGN_MASK 0xffffff00 // 256 bytes

#define DATA_FLASH_ADDR NVMCTRL_DATAFLASH
#define DATA_FLASH_SIZE 2048

#define USER_ROW_ADDR NVMCTRL_USER
#define USER_ROW_SIZE 256

#define BOCOR_ROW_ADDR NVMCTRL_BOCOR
#define BOCOR_ROW_SIZE 256

#define BL_REQUEST 0x2b620bc3

enum {
  BL_CMD_UNLOCK = 0xa0,
  BL_CMD_DATA   = 0xa1,
  BL_CMD_VERIFY = 0xa2,
  BL_CMD_RESET  = 0xa3,
};

enum {
  BL_RESP_OK       = 0x50,
  BL_RESP_ERROR    = 0x51,
  BL_RESP_INVALID  = 0x52,
  BL_RESP_CRC_OK   = 0x53,
  BL_RESP_CRC_FAIL = 0x54,
};

//-----------------------------------------------------------------------------
static bool bl_request(void);
static void command_task(void);
static void delay_cycles(uint32_t cycles);
static void flash_task(void);
static void led_task(void);
static void run_application(void);
static void send_response(int resp);
static void sys_init(void);
static void timer_reset(void);
static bool timer_expired(void);
static void uart_init(void);
static void uart_sync(void);
static void uart_task(void);

//-----------------------------------------------------------------------------
static uint32_t *ram = (uint32_t *)HSRAM_ADDR;
static uint32_t  uart_buffer[WORDS(GUARD_SIZE + OFFSET_SIZE + DATA_SIZE)];
static int       uart_command = 0;
static uint32_t  flash_data[WORDS(DATA_SIZE)];
static uint32_t  flash_addr        = 0;
static bool      flash_data_ready  = false;
static uint32_t  unlock_begin      = 0;
static uint32_t  unlock_end        = 0;
static bool      timer_expired_flg = false;

//-----------------------------------------------------------------------------
#define CONFIGURE_PMUX(dir)                                                    \
  do {                                                                         \
    PORT->Group[UART_##dir##_PORT].PINCFG[UART_##dir##_PIN].reg |=             \
        PORT_PINCFG_PMUXEN;                                                    \
    if (UART_##dir##_PIN & 1)                                                  \
      PORT->Group[UART_##dir##_PORT].PMUX[UART_##dir##_PIN >> 1].bit.PMUXO =   \
          UART_##dir##_MUX;                                                    \
    else                                                                       \
      PORT->Group[UART_##dir##_PORT].PMUX[UART_##dir##_PIN >> 1].bit.PMUXE =   \
          UART_##dir##_MUX;                                                    \
  } while (0)

//-----------------------------------------------------------------------------
static void sys_init(void) {
  PAC->WRCTRL.reg    = PAC_WRCTRL_PERID(ID_DSU) | PAC_WRCTRL_KEY_CLR;
  NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CACHEDIS | NVMCTRL_CTRLB_RWS(2);
  NVMCTRL->CTRLC.reg = NVMCTRL_CTRLC_MANW;

// Enable external regulator before going to high speed
#if BL_REG_EN == 1
  PORT->Group[BL_REG_PORT].DIRSET.reg = (1 << BL_REG_PIN);
#if BL_REG_HIGH == 1
  PORT->Group[BL_REG_PORT].OUTSET.reg = (1 << BL_REG_PIN);
#else
  PORT->GROUP[BL_REG_PORT].OUTCLR.reg = (1 << BL_REG_PIN);
#endif // BL_REG_HIGH
#endif // BL_REG_EN

  // Switch to the highest performance level
  PM->INTFLAG.reg = PM_INTFLAG_PLRDY;
  PM->PLCFG.reg   = PM_PLCFG_PLSEL_PL2;
  while (0 == PM->INTFLAG.bit.PLRDY)
    ;

  // Switch to 16MHz clock (disable prescaler)
  OSCCTRL->OSC16MCTRL.reg =
      OSCCTRL_OSC16MCTRL_ENABLE | OSCCTRL_OSC16MCTRL_FSEL_16;

// Switch on the LED
#if BL_LED_EN == 1
  PORT->Group[BL_LED_PORT].DIRSET.reg = (1 << BL_LED_PIN);
  PORT->Group[BL_LED_PORT].OUTSET.reg = (1 << BL_LED_PIN);
#endif // BL_LED_EN == 1
}

//-----------------------------------------------------------------------------
static void run_application(void) {
  uint32_t msp          = *(uint32_t *)(APPLICATION_START);
  uint32_t reset_vector = *(uint32_t *)(APPLICATION_START + 4);

  if (0xffffffff == msp)
    return;

  __set_MSP(msp);

  __asm("bx %0" ::"r"(reset_vector));
}

//-----------------------------------------------------------------------------
static void delay_cycles(uint32_t cycles) {
  cycles /= 4;

  __asm volatile("1: sub %[cycles], %[cycles], #1 \n"
                 "   nop \n"
                 "   bne 1b \n"
                 : [cycles] "+l"(cycles));
}

//-----------------------------------------------------------------------------
static void led_task(void) {
  if (timer_expired_flg) {
    PORT->Group[BL_LED_PORT].OUTTGL.reg = (1 << BL_LED_PIN);
  }
}

//-----------------------------------------------------------------------------
static bool bl_request(void) {
  PORT->Group[BL_REQ_PORT].DIRCLR.reg = (1 << BL_REQ_PIN);
  PORT->Group[BL_REQ_PORT].OUTSET.reg = (1 << BL_REQ_PIN);
  PORT->Group[BL_REQ_PORT].PINCFG[BL_REQ_PIN].reg =
      PORT_PINCFG_INEN | PORT_PINCFG_PULLEN;

  delay_cycles(1 *
               (4000000 / 1000)); // 1 ms, CPU frequency is 4 MHz at this point

  if (0 == (PORT->Group[BL_REQ_PORT].IN.reg & (1 << BL_REQ_PIN)))
    return true;

  return false;
}

//-----------------------------------------------------------------------------
static void timer_reset(void) {
  SysTick->CTRL = 0;
  SysTick->LOAD = (F_CPU / 1000u) * TIMER_INTERVAL;
  SysTick->VAL  = SysTick->LOAD;
  SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;
}

//-----------------------------------------------------------------------------
static bool timer_expired(void) {
  return (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) > 0;
}

//-----------------------------------------------------------------------------
static void uart_init(void) {
  CONFIGURE_PMUX(RX);
  CONFIGURE_PMUX(TX);

  MCLK->UART_SERCOM_MASK_REG.reg |= UART_SERCOM_MASK_BIT;

  GCLK->PCHCTRL[UART_SERCOM_GCLK_ID].reg =
      GCLK_PCHCTRL_GEN(0) | GCLK_PCHCTRL_CHEN;
  while (0 == (GCLK->PCHCTRL[UART_SERCOM_GCLK_ID].reg & GCLK_PCHCTRL_CHEN))
    ;

  UART_SERCOM->USART.CTRLA.reg =
      SERCOM_USART_CTRLA_DORD | SERCOM_USART_CTRLA_MODE(1 /*USART_INT_CLK*/) |
      SERCOM_USART_CTRLA_FORM(0 /*USART*/) | SERCOM_USART_CTRLA_SAMPR(1) |
      UART_SERCOM_TXPO | UART_SERCOM_RXPO;

  UART_SERCOM->USART.CTRLB.reg = SERCOM_USART_CTRLB_RXEN |
                                 SERCOM_USART_CTRLB_TXEN |
                                 SERCOM_USART_CTRLB_CHSIZE(0 /*8 bits*/);

#define BAUD_VAL (F_CPU / (16 * UART_BAUDRATE))
#define FP_VAL   ((F_CPU / UART_BAUDRATE - 16 * BAUD_VAL) / 2)

  UART_SERCOM->USART.BAUD.reg = SERCOM_USART_BAUD_FRACFP_BAUD(BAUD_VAL) |
                                SERCOM_USART_BAUD_FRACFP_FP(FP_VAL);

  UART_SERCOM->USART.CTRLA.reg |= SERCOM_USART_CTRLA_ENABLE;
}

//-----------------------------------------------------------------------------
static void send_response(int resp) {
  while (0 == UART_SERCOM->USART.INTFLAG.bit.DRE)
    ;
  UART_SERCOM->USART.DATA.reg = resp;
}

//-----------------------------------------------------------------------------
static void uart_sync(void) {
  while (0 == UART_SERCOM->USART.INTFLAG.bit.TXC)
    ;
}

//-----------------------------------------------------------------------------
static void uart_task(void) {
  static int ptr      = 0;
  static int command  = 0;
  static int size     = 0;
  uint8_t   *byte_buf = (uint8_t *)uart_buffer;
  int        data;

  if (uart_command)
    return;

  if (0 == UART_SERCOM->USART.INTFLAG.bit.RXC)
    return;

  data = UART_SERCOM->USART.DATA.reg;

  if (timer_expired_flg)
    command = 0;

  if (0 == command) {
    ptr            = 0;
    command        = data;
    uart_buffer[0] = 0;

    if (BL_CMD_UNLOCK == command)
      size = GUARD_SIZE + OFFSET_SIZE + SIZE_SIZE;
    else if (BL_CMD_DATA == command)
      size = GUARD_SIZE + OFFSET_SIZE + DATA_SIZE;
    else if (BL_CMD_VERIFY == command)
      size = GUARD_SIZE + CRC_SIZE;
    else if (BL_CMD_RESET == command)
      size = GUARD_SIZE + ARB_WORD_SIZE * 4;
    else
      size = 0;
  } else if (ptr < size) {
    byte_buf[ptr++] = data;
  }

  if (ptr == size) {
    uart_command = command;
    command      = 0;
  }

  timer_reset();
}

//-----------------------------------------------------------------------------
static void command_task(void) {
  if (BL_REQUEST != uart_buffer[0]) {
    send_response(BL_RESP_INVALID);
  } else if (BL_CMD_UNLOCK == uart_command) {
    uint32_t begin = uart_buffer[1];
    uint32_t size  = uart_buffer[2];
    uint32_t end   = begin + size;
    bool     pass =
        (begin == (begin & ALIGN_MASK)) && (size == (size & ALIGN_MASK)) &&
        (end >= begin) && (end > begin) &&
        ((end <= FLASH_SIZE) ||
         (begin >= DATA_FLASH_ADDR &&
          end <= (DATA_FLASH_ADDR + DATA_FLASH_SIZE)) ||
         (begin >= USER_ROW_ADDR && end <= (USER_ROW_ADDR + USER_ROW_SIZE)) ||
         (begin >= BOCOR_ROW_ADDR && end <= (BOCOR_ROW_ADDR + BOCOR_ROW_SIZE)));

    if (pass) {
      unlock_begin = begin;
      unlock_end   = end;
      send_response(BL_RESP_OK);
    } else {
      unlock_begin = 0;
      unlock_end   = 0;
      send_response(BL_RESP_ERROR);
    }
  } else if (BL_CMD_DATA == uart_command) {
    flash_addr = uart_buffer[1];

    if ((unlock_begin <= flash_addr) && ((flash_addr + DATA_SIZE) <= unlock_end) &&
        ((flash_addr + DATA_SIZE) >= flash_addr)) {
      for (int i = 0; i < WORDS(DATA_SIZE); i++)
        flash_data[i] = uart_buffer[i + 2];

      flash_data_ready = true;

      send_response(BL_RESP_OK);
    } else {
      send_response(BL_RESP_ERROR);
    }
  } else if (BL_CMD_VERIFY == uart_command) {
    uint32_t addr = unlock_begin;
    uint32_t size = unlock_end - unlock_begin;
    uint32_t crc  = uart_buffer[1];

    if (unlock_end > unlock_begin) {
      DSU->ADDR.reg    = addr;
      DSU->LENGTH.reg  = size;
      DSU->DATA.reg    = 0xffffffff;
      DSU->STATUSA.reg = DSU->STATUSA.reg;
      DSU->CTRL.reg    = DSU_CTRL_CRC;

      while (0 == DSU->STATUSA.bit.DONE)
        ;

      if ((0 == DSU->STATUSA.bit.BERR) && (crc == DSU->DATA.reg))
        send_response(BL_RESP_CRC_OK);
      else
        send_response(BL_RESP_CRC_FAIL);
    } else {
      send_response(BL_RESP_INVALID);
    }
  } else if (BL_CMD_RESET == uart_command) {
    // Unrolling the loop here saves significant amount of Flash
    ram[0] = uart_buffer[1];
    ram[1] = uart_buffer[2];
    ram[2] = uart_buffer[3];
    ram[3] = uart_buffer[4];

    send_response(BL_RESP_OK);
    uart_sync();

    NVIC_SystemReset();
  } else {
    send_response(BL_RESP_INVALID);
  }

  uart_command = 0;
}

//-----------------------------------------------------------------------------
static void flash_task(void) {
  uint32_t *src = flash_data;
  uint32_t *dst = (uint32_t *)flash_addr;

  NVMCTRL->ADDR.reg = flash_addr;

  if (0 == (flash_addr % ERASE_BLOCK_SIZE)) {
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;

    while (0 == NVMCTRL->STATUS.bit.READY)
      uart_task();
  }

  for (int page = 0; page < PAGES_IN_ERASE_BLOCK; page++) {
    for (int i = 0; i < WORDS(PAGE_SIZE); i++)
      dst[i] = src[i];

    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_WP;

    while (0 == NVMCTRL->STATUS.bit.READY)
      uart_task();

    src += WORDS(PAGE_SIZE);
    dst += WORDS(PAGE_SIZE);
  }

  flash_data_ready = false;
}

//-----------------------------------------------------------------------------
__attribute__((noinline)) // Prevent LTO from inlining main() into the reset
                          // handler
                          int
                          main(void) {
  if (!bl_request())
    run_application();

  sys_init();
  uart_init();

  timer_reset();
  while (1) {
    timer_expired_flg = timer_expired();
    uart_task();

#if BL_LED_EN == 1
    led_task();
#endif // BL_LED_EN == 1

    timer_expired_flg = false;

    if (flash_data_ready)
      flash_task();
    else if (uart_command)
      command_task();
  }

  return 0;
}
