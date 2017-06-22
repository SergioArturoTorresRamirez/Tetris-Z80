#ifndef SPI_H_
#define SPI_H_

#include "smz80.h"

#define  PPI_SPI_PORT  PPI_PORTA
#define  PPI_SPI_MOSI  0
#define  PPI_SPI_SS    1
#define  PPI_SPI_SCK   2

void spiInit();
void spiWrite8(uint8_t data);
void spiWrite16(uint16_t data);
void spiWriteBuffer8(uint8_t* buffer, uint8_t bufferLen);
void spiWriteBuffer16(uint16_t* buffer, uint8_t bufferLen);

void spiInit() {

    PPI_CTRL = 0x81;
    PPI_SPI_PORT = (1 << PPI_SPI_SS);
}

void spiWrite8(uint8_t data) {
    unsigned char i;

    PPI_SPI_PORT = 0;  //Selecciona el chip y pone a cero la linea de datos y reloj
    
    for (i = 0; i < 8; ++i)
    {
        if( (data << i) & 0x80 )                    // si el bit MSB es 1
            PPI_SPI_PORT |= (1 << PPI_SPI_MOSI);    // Pone en alto el pin de datos
        else
            PPI_SPI_PORT &= ~(1 << PPI_SPI_MOSI);    // Pone en alto el pin de datos
        
        PPI_SPI_PORT |= (1 << PPI_SPI_SCK);         // Pone a 1 la linea del reloj
        PPI_SPI_PORT &= ~(1 << PPI_SPI_SCK);        // pone a 0 la linea del reloj.
    }

    PPI_SPI_PORT = (1 << PPI_SPI_SS);               // Deshabilita el chip
}

void spiWrite16(uint16_t data) {
    unsigned char i;

    PPI_SPI_PORT = 0;  //Selecciona el chip y pone a cero la linea de datos y reloj
    
    for (i = 0; i < 16; ++i)
    {
        if( (data << i) & 0x8000 )                  // si el bit MSB es 1
            PPI_SPI_PORT |= (1 << PPI_SPI_MOSI);    // Pone en alto el pin de datos
        else
            PPI_SPI_PORT &= ~(1 << PPI_SPI_MOSI);    // Pone en alto el pin de datos
        
        PPI_SPI_PORT |= (1 << PPI_SPI_SCK);         // Pone a 1 la linea del reloj.
       // delay_ms(300);
        PPI_SPI_PORT &= ~(1 << PPI_SPI_SCK);        // pone a 0 la linea del reloj.

    }

    PPI_SPI_PORT = (1 << PPI_SPI_SS);               // Deshabilita el chip
}

void spiWriteBuffer8(uint8_t* buffer, uint8_t bufferLen) {
    unsigned char i;

    for (i = 0; i < bufferLen; ++i)
        spiWrite8(buffer[i]);

}

void spiWriteBuffer16(uint16_t* buffer, uint8_t bufferLen) {
    unsigned char i;
    
    for (i = 0; i < bufferLen; ++i)
        spiWrite16(buffer[i]);

}

#endif