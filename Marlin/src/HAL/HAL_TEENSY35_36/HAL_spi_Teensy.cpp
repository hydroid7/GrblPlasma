#if defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)

#include "HAL.h"
#include <SPI.h>
#include <pins_arduino.h>
#include "spi_pins.h"
#include "../../core/macros.h"

static SPISettings spiConfig;

#define USE_TEENSY3_SPI

// Teensy 3.0 functions  (copied from sdfatlib20130629)
#include <kinetis.h>
// Limit initial fifo to three entries to avoid fifo overrun
#define SPI_INITIAL_FIFO_DEPTH 3
// define some symbols that are not in mk20dx128.h
#ifndef SPI_SR_RXCTR
#define SPI_SR_RXCTR 0XF0
#endif  // SPI_SR_RXCTR
#ifndef SPI_PUSHR_CONT
#define SPI_PUSHR_CONT 0X80000000
#endif   // SPI_PUSHR_CONT
#ifndef SPI_PUSHR_CTAS
#define SPI_PUSHR_CTAS(n) (((n) & 7) << 28)
#endif  // SPI_PUSHR_CTAS
// Standard SPI functions
/** Initialize SPI bus */

void spiBegin() {
  SIM_SCGC6 |= SIM_SCGC6_SPI0;
}

void spiInit(uint8_t spiRate) {
  switch (spiRate) {
    // the top 2 speeds are set to 24 MHz, for the SD library defaults
    case 0:  spiConfig = SPISettings(24000000, MSBFIRST, SPI_MODE0); break;
    case 1:  spiConfig = SPISettings(24000000, MSBFIRST, SPI_MODE0); break;
    case 2:  spiConfig = SPISettings(8000000, MSBFIRST, SPI_MODE0); break;
    case 3:  spiConfig = SPISettings(4000000, MSBFIRST, SPI_MODE0); break;
    case 4:  spiConfig = SPISettings(3000000, MSBFIRST, SPI_MODE0); break;
    case 5:  spiConfig = SPISettings(2000000, MSBFIRST, SPI_MODE0); break;
    default: spiConfig = SPISettings(400000, MSBFIRST, SPI_MODE0);
  }
  SPI.begin();
}

/** SPI receive a byte */
uint8_t spiRec() {
  SPI0_MCR |= SPI_MCR_CLR_RXF;
  SPI0_SR = SPI_SR_TCF;
  SPI0_PUSHR = 0xFF;
  while (!(SPI0_SR & SPI_SR_TCF)) {}
  return SPI0_POPR;
}
/** SPI receive multiple bytes */
uint8_t spiRec(uint8_t* buf, size_t len) {
  // clear any data in RX FIFO
  SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_CLR_RXF | SPI_MCR_PCSIS(0x1F);
  // use 16 bit frame to avoid TD delay between frames
  // get one byte if len is odd
  if (len & 1) {
    *buf++ = spiRec();
    len--;
  }
  // initial number of words to push into TX FIFO
  int nf = len/2 < SPI_INITIAL_FIFO_DEPTH ? len/2 : SPI_INITIAL_FIFO_DEPTH;
  for (int i = 0; i < nf; i++) {
    SPI0_PUSHR = SPI_PUSHR_CONT | SPI_PUSHR_CTAS(1) | 0XFFFF;
  }
  uint8_t* limit = buf + len - 2*nf;
  while (buf < limit) {
    while (!(SPI0_SR & SPI_SR_RXCTR)) {}
    SPI0_PUSHR = SPI_PUSHR_CONT | SPI_PUSHR_CTAS(1) | 0XFFFF;
    uint16_t w = SPI0_POPR;
    *buf++ = w >> 8;
    *buf++ = w & 0XFF;
  }
  // limit for rest of RX data
  limit += 2*nf;
  while (buf < limit) {
    while (!(SPI0_SR & SPI_SR_RXCTR)) {}
    uint16_t w = SPI0_POPR;
    *buf++ = w >> 8;
    *buf++ = w & 0XFF;
  }
  return 0;
}
void spiRecIgnore(size_t len) {
  // clear any data in RX FIFO
  SPI0_MCR = SPI_MCR_MSTR | SPI_MCR_CLR_RXF | SPI_MCR_PCSIS(0x1F);
  // use 16 bit frame to avoid TD delay between frames
  // get one byte if len is odd
  if (len & 1) {
    spiRec();
    len--;
  }
  // initial number of words to push into TX FIFO
  int nf = len/2 < SPI_INITIAL_FIFO_DEPTH ? len/2 : SPI_INITIAL_FIFO_DEPTH;
  for (int i = 0; i < nf; i++) {
    SPI0_PUSHR = SPI_PUSHR_CONT | SPI_PUSHR_CTAS(1) | 0XFFFF;
    len -= 2;
  }
  //uint8_t* limit = buf + len - 2*nf;
  //while (buf < limit) {
  while (len > 0) {
    while (!(SPI0_SR & SPI_SR_RXCTR)) {}
    SPI0_PUSHR = SPI_PUSHR_CONT | SPI_PUSHR_CTAS(1) | 0XFFFF;
    SPI0_POPR;
    len -= 2;
  }
  // limit for rest of RX data
  while (nf > 0) {
    while (!(SPI0_SR & SPI_SR_RXCTR)) {}
    SPI0_POPR;
    nf--;
  }
}
/** SPI send a byte */
void spiSend(uint8_t b) {
  SPI0_MCR |= SPI_MCR_CLR_RXF;
  SPI0_SR = SPI_SR_TCF;
  SPI0_PUSHR = b;
  while (!(SPI0_SR & SPI_SR_TCF)) {}
}
/** SPI send multiple bytes */

#elif defined(__IMXRT1052__)  || defined(__IMXRT1062__)
 #define USE_TEENSY4_SPI

 void spiInit(uint8_t spiRate) {
  switch (spiRate) {
    // the top 2 speeds are set to 24 MHz, for the SD library defaults
    case 0:  spiConfig = SPISettings(25200000, MSBFIRST, SPI_MODE0); break;
    case 1:  spiConfig = SPISettings(24000000, MSBFIRST, SPI_MODE0); break;
    case 2:  spiConfig = SPISettings(8000000, MSBFIRST, SPI_MODE0); break;
    case 3:  spiConfig = SPISettings(4000000, MSBFIRST, SPI_MODE0); break;
    case 4:  spiConfig = SPISettings(3000000, MSBFIRST, SPI_MODE0); break;
    case 5:  spiConfig = SPISettings(2000000, MSBFIRST, SPI_MODE0); break;
    default: spiConfig = SPISettings(400000, MSBFIRST, SPI_MODE0);
  }
  SPI.begin();
}

 void spiSend(uint8_t b) {
	SPI.transfer(b);
 }

 uint8_t spiRec(void) {
	return SPI.transfer(0xff);
 }

 void spiRec(uint8_t* buf, size_t len) {
	memset(buf, 0xFF, len);
	SPI.transfer(buf, len);
 }

 void spiRecIgnore(size_t len) {
	for (size_t i=0; i < len; i++)
		SPI.transfer(0xff);
 }

//------------------------------------------------------------------------------
#else
// functions for hardware SPI
/** Send a byte to the card */
void spiSend(uint8_t b) {
  SPDR = b;
  while (!(SPSR & (1 << SPIF)));
}
/** Receive a byte from the card */
uint8_t spiRec(void) {
  spiSend(0XFF);
  return SPDR;
}

#endif
