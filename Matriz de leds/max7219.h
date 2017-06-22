#ifndef MAX7219_H_
#define MAX7219_H_

#include "spi.h"

#define MAX_ROWS    8
#define MAX_COLS    8
#define BLACK       0
#define WHITE       1
#define OFF         0
#define ON          1

#define ROW1_REG    0x01
#define ROW2_REG    0x02
#define ROW3_REG    0x03
#define ROW4_REG    0x04
#define ROW5_REG    0x05
#define ROW6_REG    0x06
#define ROW7_REG    0x07
#define ROW8_REG    0x08
#define MODE_REG    0x09
#define BRIGHT_REG  0x0A
#define SCANL_REG   0x0B
#define TEST_REG    0x0F
#define SHUTDOWN_REG    0x0C

uint8_t _displayBuffer[8] = {OFF, OFF, OFF, OFF, OFF, OFF, OFF, OFF};

void max7219WriteRegister(unsigned char reg, unsigned char data);
void max7219Init();
void max7219Test();
void max7219SetBright(uint8_t bright);
void max7219WriteDisplayBuffer(unsigned char *buffer, unsigned char len);
void max7219ReadDisplayBuffer(unsigned char *buffer, unsigned char len);
void max7219DrawRow(unsigned char row, unsigned char data);
void max7219DrawColumn(unsigned char column, unsigned char data);
uint8_t max7219ReadPixel(unsigned char x, unsigned char y);
uint8_t max7219ReadRow(unsigned char row);
uint8_t max7219ReadColumn(unsigned char column);


void max7219WriteRegister(unsigned char reg, unsigned char data) {
    uint16_t max_data;

    max_data = (((uint16_t)(reg)) & (0x0F)) << 8;
    max_data |= data;

    spiWrite16(max_data);

}

void max7219Init() {
    spiInit();
    //spiWrite16(0x0C01);
    //spiWrite16(0x0B07);
    delay_ms(10);
    max7219WriteRegister(SHUTDOWN_REG, 0x01);
    delay_ms(10);
    max7219WriteRegister(MODE_REG, 0x00);
    delay_ms(10);
    max7219WriteRegister(SCANL_REG, 0x07);
    delay_ms(10);
    max7219WriteRegister(BRIGHT_REG, 0x01);
    delay_ms(10);
    max7219Clear();
    delay_ms(10);
}

void max7219Clear() {

    max7219WriteRegister(ROW1_REG, 0x00);
    max7219WriteRegister(ROW2_REG, 0x00);
    max7219WriteRegister(ROW3_REG, 0x00);
    max7219WriteRegister(ROW4_REG, 0x00);
    max7219WriteRegister(ROW5_REG, 0x00);
    max7219WriteRegister(ROW6_REG, 0x00);
    max7219WriteRegister(ROW7_REG, 0x00);
    max7219WriteRegister(ROW8_REG, 0x00);
}
void max7219DrawPixel(unsigned char x, unsigned char y, unsigned char color) {
    
    uint8_t ren, col;

    if(x<1 || x > 8 || y < 1 || y > 8)
        return ;

    ren = _displayBuffer[y];
    col = ( 1 << (x-1) );

    if(color)
        ren |= col;
    else
        ren &= ~col;

    _displayBuffer[y] = ren;
    max7219WriteRegister(y, _displayBuffer[y]);

}
void max7219Test() {
    
    unsigned char i;
    unsigned char j;
    
    for( i=1;i<9;i++ )
    {
           for( j=1;j<9;j++ )
        {
             max7219DrawPixel(i, j, (unsigned char)1);
             delay_ms(300);
             max7219DrawPixel(i, j,(unsigned char)0);
        } 
    }

}

void max7219SetBright(uint8_t bright) {
    bright = bright & 0x0F;
    max7219WriteRegister(BRIGHT_REG, bright);
}

void max7219WriteDisplayBuffer(unsigned char *buffer, unsigned char len) {
    unsigned char i;

    if(len >8)
        len = 8;

    for (i = 0; i < len; ++i)
        max7219WriteRegister(ROW1_REG + i, buffer[i]);
        
}

void max7219ReadDisplayBuffer(unsigned char *buffer, unsigned char len) {
    unsigned char i;

    if(len >8)
        len = 8;

    for (i = 0; i < len; ++i)
        buffer[i] = _displayBuffer[i];
}



void max7219DrawRow(unsigned char row, unsigned char data) {
    if(row < 1 || row > 8)
        return ;
    _displayBuffer[row] = data;
    max7219WriteRegister(row, data);
}

void max7219DrawColumn(unsigned char column, unsigned char data) {

    unsigned i, color;
    for (i = 0; i < 8; ++i)
    {
        color = (data>>i) & 0x01 ;
        max7219DrawPixel(column, i, color);
    }
}

uint8_t max7219ReadPixel(unsigned char x, unsigned char y) {
    uint8_t ren, col;

    if(x<1 || x > 8 || y < 1 || y > 8)
        return ;

    return _displayBuffer[y] & ( 1 << (x-1) );
}

uint8_t max7219ReadRow(unsigned char row) {
    if(row < 1 || row > 8)
        return 0;

    return _displayBuffer[row];
}

uint8_t max7219ReadColumn(unsigned char column) {

}


#endif