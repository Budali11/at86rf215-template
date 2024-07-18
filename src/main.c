/*
 * sam4lc8c4 template.c
 *
 * Created: 7/12/2024 5:15:15 PM
 * Author : budali11
 */ 


#include "sam4lc8c.h"
#include "sysclk.h"
#include "usart.h"
#include "ioport.h"
#include "conf_clock.h"
#include "interrupt.h"
#include "spi.h"
#include "at86rf215.h"
#include <stdio.h>
#include "gpio.h"

#define SPI_DLYBS 0x40U
#define SPI_DLYBCT 0x10U

ioport_pin_t REB215_int_pin = IOPORT_CREATE_PIN(IOPORT_GPIOC, 25);
ioport_pin_t REB215_rst_pin = IOPORT_CREATE_PIN(IOPORT_GPIOC, 0);
Usart *REB215_usart = USART0;
Spi *REB215_spi = SPI;

void delay(uint32_t n)
{
  for (uint32_t i = 0; i < n; i++) {
    for (uint32_t j = 0; j < 1000; j++) {
      nop();
    }
  }
}

static void get_clk_infomation(void);
static void spi_init(Spi *hspi);
static void usart_init(Usart *husart, uint32_t baudrate);
extern void REB215_IRQ_Callback(void);

int main(void)
{
  /* Initialize the SAM system */
  SystemInit();

  /* Initialize system */
  sysclk_init();

  /* led */
  sysclk_enable_peripheral_clock(GPIO);
  ioport_set_pin_dir(LED_0_PIN, IOPORT_DIR_OUTPUT);

  /* usart init */
  usart_init(REB215_usart, 115200);

  /* spi init */
  spi_init(REB215_spi);

  /* at86rf215 interrupt io init */
  ioport_set_pin_dir(REB215_int_pin, IOPORT_DIR_INPUT);
  ioport_set_pin_mode(REB215_int_pin, IOPORT_MODE_PULLDOWN);
  // interrupt at rising edge
  GPIO->GPIO_PORT[2].GPIO_IMR0 |= 1 << 25;
  GPIO->GPIO_PORT[2].GPIO_IMR1 &= ~(1 << 25);

  // filter glitch
  GPIO->GPIO_PORT[2].GPIO_GFER |= 1 << 25;

  gpio_enable_pin_interrupt(REB215_int_pin);
  gpio_set_pin_callback(REB215_int_pin, REB215_IRQ_Callback, 0);
  ioport_enable_pin(REB215_int_pin);

  /* at86rf215 rst io init */
  ioport_set_pin_dir(REB215_rst_pin, IOPORT_DIR_OUTPUT);
  ioport_set_pin_mode(REB215_rst_pin, IOPORT_MODE_PULLUP);

  /* test */
  get_clk_infomation();

  /* Replace with your application code */
  rf215_reset();
  rf215_init_09();
  delay(100);

  uint16_t id = rf215_read_id();
  char msg[24] = {0};
  sprintf(msg, "at86rf215 id: 0x%x\r\n", id);
  usart_write_line(REB215_usart, msg);
  usart_write_line(REB215_usart, "now testing.\r\n");

  IQ_receive_test();

  while (1) {
    delay(1000);
    ioport_set_pin_level(LED_0_PIN, true);
    delay(1000);
    ioport_set_pin_level(LED_0_PIN, false);
  }
}

static void get_clk_infomation(void)
{
  uint32_t cpu_clk = sysclk_get_cpu_hz();
  uint32_t hsb_clk = sysclk_get_hsb_hz();
  uint32_t pba_clk = sysclk_get_pba_hz();
  uint32_t pbb_clk = sysclk_get_pbb_hz();
  uint32_t pbc_clk = sysclk_get_pbc_hz();

  (void)cpu_clk;
  (void)hsb_clk;
  (void)pba_clk;
  (void)pbb_clk;
  (void)pbc_clk;
}

static void spi_init(Spi *hspi)
{
  /* enable clock, and clock source selection */
  sysclk_enable_peripheral_clock(hspi);

  /* enable ioports */
  uint32_t io_init_table[][2] = {
    {IOPORT_CREATE_PIN(IOPORT_GPIOA, 22), MUX_PA22A_SPI_MOSI},
    {IOPORT_CREATE_PIN(IOPORT_GPIOA, 21), MUX_PA21A_SPI_MISO},
    {IOPORT_CREATE_PIN(IOPORT_GPIOC, 30), MUX_PC30B_SPI_SCK},
    {IOPORT_CREATE_PIN(IOPORT_GPIOC, 3), MUX_PC03A_SPI_NPCS0},
  };
  for (int i = 0; i < 4; i++) {
    ioport_set_pin_mode(io_init_table[i][0], io_init_table[i][1]);
    ioport_disable_pin(io_init_table[i][0]);
  }
  
  /* spi register configuration */
  /* reset */
  spi_disable(hspi);
  spi_reset(hspi);

  /* master mode */
  spi_set_master_mode(hspi);

  spi_disable_mode_fault_detect(hspi);

  /* phase and parity_type */
  spi_set_clock_phase(hspi, 0, 1); // 0 stands for channel 0, 1 stands for NCPHA = 1
  spi_set_clock_polarity(hspi, 0, 0); // the first zero selects channel 1, the second sets CPOL = 0

  /* select bits per transfer */
  spi_set_bits_per_transfer(hspi, 0, SPI_CSR_BITS_8_BPT);

  /* set delays */
  spi_set_transfer_delay(hspi, 0, SPI_DLYBS, SPI_DLYBCT);

  /* set clk_spi division */
  hspi->SPI_CSR[0] |= (48u << SPI_CSR_SCBR_Pos);

  /* enable receive fifo */
  hspi->SPI_MR |= SPI_MR_RXFIFOEN;

  /* fixed peripheral select */
  spi_set_fixed_peripheral_select(hspi);
  spi_disable_peripheral_select_decode(hspi);
  spi_set_peripheral_chip_select_value(hspi, spi_get_pcs(0));

  /* enable spi */
  spi_enable(hspi);
}

static void usart_init(Usart *husart, uint32_t baudrate)
{
  /* enable interrupt */
  NVIC_EnableIRQ(USART0_IRQn);

  uint32_t husart_io_rx = IOPORT_CREATE_PIN(IOPORT_GPIOB, 0);
  uint32_t husart_io_tx = IOPORT_CREATE_PIN(IOPORT_GPIOB, 1);
  /* init io ports */
  /* usart0 */
  ioport_set_pin_mode(husart_io_rx, MUX_PB00B_USART0_RXD);
  ioport_set_pin_mode(husart_io_tx, MUX_PB01B_USART0_TXD);

  /* disable the GPIO to control the pin */
  ioport_disable_pin(husart_io_rx);
  ioport_disable_pin(husart_io_tx);

  /* enable usart0 clock */
  sysclk_enable_peripheral_clock(husart);

  /* init usart to normal mode */
  sam_usart_opt_t usart_init_opt = {
    .baudrate = baudrate,
    .stop_bits = US_MR_NBSTOP_1_BIT,
    .char_length = US_MR_CHRL_8_BIT,
    .parity_type = US_MR_PAR_NO,
    .channel_mode = US_MR_CHMODE_NORMAL,
  };
  usart_init_rs232(husart, &usart_init_opt, sysclk_get_peripheral_bus_hz(husart));

  /* enable usart transmitter and receiver */
  usart_enable_tx(husart);
  usart_enable_rx(husart);

  /* config for usart receive interrupt */
  usart_enable_interrupt(husart, US_IER_RXRDY);
}

void USART0_Handler(void)
{
  static char buffer[16] = {0};
  static char* pb = buffer;
  if ((usart_get_status(REB215_usart) & US_CSR_RXRDY) == 1){
    // receive interrupt
    usart_getchar(REB215_usart, pb);
    pb = (pb == buffer+15)? buffer: pb+1;
    if (pb == buffer) {
      for (int i = 0; i < 16; i++) 
        usart_putchar(REB215_usart, buffer[i]);
    }
  }
}


