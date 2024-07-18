#include "at86rf215.h"
#include "spi.h"
#include "usart.h"
#include "ioport.h"

#define DONTCARE 0

#define CONFIG_RAF_IF_CONVERSION    (0U << 5)
#define CONFIG_RAF_IF_SHIFT         (0U << 4)
#define CONFIG_RAF_RECEIVER_BW      0x02U 
#define CONFIG_RDF_RCUT_FREQ        (0x04U << 5)
#define CONFIG_RDF_SAMPLE_RATE      (0x0aU << 0)
#define CONFIG_RDF_AGC              0 /* disable AGC */
#define CONFIG_CHANNEL_SPACE        30U
#define CONFIG_CCF                  (915.0)
#define CONFIG_CCF_                 (CONFIG_CCF - 754.0)
#define CONFIG_CCF__                ((CONFIG_CCF_*1e6)/198.364)
#define CONFIG_CCF___               ((uint32_t)(CONFIG_CCF__))
#define CONFIG_CHANNEL_NUM          0
#define CONFIG_CHANNEL_MODE         (2U << 6)

extern Usart *REB215_usart;
extern Spi *REB215_spi;
bool trxprep2trxrdy = false;
extern void delay(uint32_t n);

static uint16_t rf215_read_register(Spi *hspi, uint32_t address, uint16_t *data)
{
  uint8_t cmds[2] = {0};
  uint16_t ret = 0;
  cmds[0] = ((address >> 8u) & 0x3f); // read register
  cmds[1] = address & 0xff;

  // keep nss line negative when waitting for reply
  spi_configure_cs_behavior(hspi, 0, SPI_CS_KEEP_LOW);

  for (int i = 0; i < 2; i++) {
    spi_write(hspi, cmds[i], DONTCARE, DONTCARE);
    spi_read(hspi, &ret, DONTCARE); // read out useless data
  }
  spi_write(hspi, 0, DONTCARE, DONTCARE); // write something generate clock
  /* read out useful data */
  spi_read(hspi, &ret, DONTCARE);

  // release nss line
  spi_configure_cs_behavior(hspi, 0, SPI_CS_RISE_NO_TX);

  if (data != NULL) 
    *data = ret;
  return ret;
}

static void rf215_write_register(Spi *hspi, uint16_t address, uint8_t data)
{
  uint8_t cmds[3] = {0, 0, data};
  uint16_t trash = 0;
  cmds[0] = ((address >> 8u) & 0x3f) | (1U << 7) & ~(1U << 6); // write register
  cmds[1] = address & 0xff;

  spi_configure_cs_behavior(hspi, 0, SPI_CS_KEEP_LOW);
  for (int i = 0; i < 3; i++) {
    spi_write(hspi, cmds[i], DONTCARE, DONTCARE);
    spi_read(hspi, &trash, DONTCARE); // read out useless data
  }
  spi_configure_cs_behavior(hspi, 0, SPI_CS_RISE_NO_TX);
}

void rf215_init_09(void)
{
  /* disable TX power */
  rf215_write_register(REB215_spi, RF09_PAC, 0x00);
  delay(100);
  /* target AGC = -42dB */
  rf215_write_register(REB215_spi, RF09_AGCS, 0xf7);
  delay(100);
  /* disable CLKO */
  rf215_write_register(REB215_spi, RF_CLKO, 0x00);
  delay(100);
  /* enable interrupt */
  rf215_write_register(REB215_spi, RF09_IRQM, 0x3f); // enable all interrupt
  delay(100);
  
  /* clear interrupt status */
  // rf215_read_register(REB215_spi, RF09_IRQS, NULL);
  if (rf215_read_register(REB215_spi, RF09_IRQM, NULL) != 0x3f) {
    usart_write_line(REB215_usart, "interrupt not enable.");
  }
}

uint16_t rf215_read_id(void)
{
  return rf215_read_register(REB215_spi, RF_PN, NULL);
}

void rf215_reset(void)
{
  // rf215_write_register(REB215_spi, RF_RST, CMD_RF_RESET);
  // // wait until TRXOFF state
  // while (rf215_read_register(REB215_spi, RF09_STATE, NULL) != STATE_TRXOFF);
  extern ioport_pin_t REB215_rst_pin;
  extern void delay(uint32_t n);
  ioport_set_pin_level(REB215_rst_pin, true);
  delay(300);
  ioport_set_pin_level(REB215_rst_pin, false);
  delay(300);
  ioport_set_pin_level(REB215_rst_pin, true);
}

void rf215_set_route_IQ_directly(void)
{
  /* set Transceiver I/Q Data Interface Configuration Register */
  rf215_write_register(REB215_spi, RF_IQIFC1, RF_MODE_RF);
}

void IQ_receive_test(void)
{
  // after reset
  // now state: TRXOFF
  // send CMD=RX to enter TRANSITION state 
  // and eventually enter RX state
  rf215_write_register(REB215_spi, RF09_CMD, CMD_RF_TRXOFF);
  delay(100);

  /* 1. enable I\Q radio mode */
  rf215_set_route_IQ_directly();
  delay(100);

  /* 2. configure the receiver frontend */
  /* set receiver analog frontend */
  rf215_write_register(REB215_spi, RF09_RXBWC, CONFIG_RAF_IF_CONVERSION | CONFIG_RAF_IF_SHIFT | CONFIG_RAF_RECEIVER_BW);
  delay(100);

  /* set receiver digital frontend */
  rf215_write_register(REB215_spi, RF09_RXDFE, CONFIG_RDF_RCUT_FREQ | CONFIG_RDF_SAMPLE_RATE);
  delay(100);

  /* set Automatic Gain Controller (disable) */
  rf215_write_register(REB215_spi, RF09_AGCC, CONFIG_RDF_AGC);
  delay(100);

  /* 3. configure the channel */
  rf215_write_register(REB215_spi, RF09_CS, CONFIG_CHANNEL_SPACE);
  delay(100);
  rf215_write_register(REB215_spi, RF09_CCF0L, (CONFIG_CCF___ >> 8) & 0xff);
  delay(100);
  rf215_write_register(REB215_spi, RF09_CCF0H, (CONFIG_CCF___ >> 16) & 0xff);
  delay(100);
  rf215_write_register(REB215_spi, RF09_CNL, CONFIG_CCF___ & 0xff);
  delay(100);
  //rf215_write_register(REB215_spi, RF09_CNM, CONFIG_CHANNEL_MODE | (CONFIG_CHANNEL_NUM >> 8) & 0x01);
  rf215_write_register(REB215_spi, RF09_CNM, CONFIG_CHANNEL_MODE);
  delay(100);


  /* 4. switch to state TXPREP */
  rf215_write_register(REB215_spi, RF09_CMD, CMD_RF_TXPREP);
  delay(100);

  uint8_t state = 0;
  state = rf215_read_register(REB215_spi, RF09_STATE, NULL);
  if (state == STATE_TRXOFF) {
    usart_write_line(REB215_usart, "trxoff.\r\n");
  }
  else if (state == STATE_TXPREP) {
    usart_write_line(REB215_usart, "txprep.\r\n");
  }
  else if (state == STATE_RX) {
    usart_write_line(REB215_usart, "rx.\r\n");
  }
  else {
    usart_write_line(REB215_usart, "else.\r\n");
  }
  /* 5. wait for TRXRDY interrupt */ 
  // while (trxprep2trxrdy != true);
  // trxprep2trxrdy = false;

  /* 6. switch to RX */
  rf215_write_register(REB215_spi, RF09_CMD, CMD_RF_RX);
  delay(100);

  while (rf215_read_register(REB215_spi, RF09_STATE, NULL) != STATE_RX)
    delay(100);
  usart_write_line(REB215_usart, "receiving.\r\n");
  /* 7. set up a timer (TODO)*/
}

void TRXRDY_callback(void)
{
  trxprep2trxrdy = true;
}

typedef struct int_processing__ {
  uint8_t IRQM;
  const char *msg;
  void (*callback)(void);
}int_processing;

int_processing REB215_int_proc_table[] = {
  { IRQM_IQIFSF, "I\\Q data interface synchronization fails.", NULL},
  { IRQM_TRXERR, "Transceiver errer occur.", NULL},
  { IRQM_BATLOW, "Battery Low.", NULL},
  { IRQM_EDC   , "Energy measurement completed.", NULL},
  { IRQM_TRXRDY, "State changed to TXPREP.", TRXRDY_callback},
  { IRQM_WAKEUP, "Wake up.", NULL},
};

void REB215_IRQ_Callback(void)
{
  /* read interrupt flag register */
  uint16_t int_flag = 0;
  rf215_read_register(REB215_spi, RF09_IRQS, &int_flag);
  for (int i = 0; i < RADIO_IRQM_NUM; i++) {
    if (int_flag == REB215_int_proc_table[i].IRQM) {
      usart_write_line(REB215_usart, REB215_int_proc_table[i].msg);
      if (REB215_int_proc_table[i].callback != NULL)
        REB215_int_proc_table[i].callback();
      break;
    }
  }
}
