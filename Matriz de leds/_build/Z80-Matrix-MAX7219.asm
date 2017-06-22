;--------------------------------------------------------
; File Created by SDCC : free open source ANSI-C Compiler
; Version 3.5.0 #9253 (Jun 20 2015) (Linux)
; This file was generated Thu Jun 22 16:57:47 2017
;--------------------------------------------------------
	.module main
	.optsdcc -mz80
	
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _revizaRenglon
	.globl _mueveAbajo
	.globl _revizaPerdio
	.globl _iniciaJuego
	.globl _mueveIzquierda
	.globl _mueveDerecha
	.globl _giraIzquierda
	.globl _giraDerecha
	.globl _dibujaCara
	.globl _llenaFigura
	.globl _borraFigura
	.globl _dibujaFigura
	.globl _system_init
	.globl _isr_vector38
	.globl _isr_vector66
	.globl _max7219DrawPixel
	.globl _max7219Clear
	.globl _isprint
	.globl __displayBuffer
	.globl _cont
	.globl _io_write
	.globl _io_read
	.globl _io_write_buffer
	.globl _io_read_buffer
	.globl _uart_init
	.globl _uart_set_baudrate
	.globl _uart_write
	.globl _uart_read
	.globl _uart_available
	.globl _uart_print
	.globl _uart_read_line
	.globl _uart_disable_interrupts
	.globl _uart_enable_interrupts
	.globl _ppi_init
	.globl _ppi_set_portc_bit
	.globl _ppi_clear_portc_bit
	.globl _delay_10us
	.globl _delay_100us
	.globl _delay_ms
	.globl _putchar
	.globl _getchar
	.globl _spiInit
	.globl _spiWrite8
	.globl _spiWrite16
	.globl _spiWriteBuffer8
	.globl _spiWriteBuffer16
	.globl _max7219WriteRegister
	.globl _max7219Init
	.globl _max7219Test
	.globl _max7219SetBright
	.globl _max7219WriteDisplayBuffer
	.globl _max7219ReadDisplayBuffer
	.globl _max7219DrawRow
	.globl _max7219DrawColumn
	.globl _max7219ReadPixel
	.globl _max7219ReadRow
	.globl _max7219ReadColumn
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
_PPI_PORTA	=	0x0000
_PPI_PORTB	=	0x0001
_PPI_PORTC	=	0x0002
_PPI_CTRL	=	0x0003
_URRBR	=	0x0070
_URTHR	=	0x0070
_URIER	=	0x0071
_URIIR	=	0x0072
_URLCR	=	0x0073
_URLSR	=	0x0075
_URMCR	=	0x0074
_URMSR	=	0x0076
_URDLL	=	0x0070
_URDLM	=	0x0071
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _DATA
___ret_aux:
	.ds 1
_cont::
	.ds 2
;--------------------------------------------------------
; ram data
;--------------------------------------------------------
	.area _INITIALIZED
__displayBuffer::
	.ds 8
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area _DABS (ABS)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area _HOME
	.area _GSINIT
	.area _GSFINAL
	.area _GSINIT
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area _HOME
	.area _HOME
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area _CODE
;../SMZ80_SDK/V1/include/smz80.h:238: void io_write(char port_addr, char data) {
;	---------------------------------
; Function io_write
; ---------------------------------
_io_write::
;../SMZ80_SDK/V1/include/smz80.h:248: __endasm;
	ld ix, #2
	add ix,sp
	ld c, (ix)
	inc ix
	ld a,(ix)
	out (c), a
	ret
;../SMZ80_SDK/V1/include/smz80.h:251: char io_read(char port_addr) {
;	---------------------------------
; Function io_read
; ---------------------------------
_io_read::
;../SMZ80_SDK/V1/include/smz80.h:259: __endasm;
	LD IX, #2
	ADD IX,SP
	LD C, (IX)
	IN A,(C)
	LD (___ret_aux),A
;../SMZ80_SDK/V1/include/smz80.h:260: return __ret_aux;
	ld	iy,#___ret_aux
	ld	l,0 (iy)
	ret
;../SMZ80_SDK/V1/include/smz80.h:262: void io_write_buffer(char port_addr, char* buffer_out, char count) {
;	---------------------------------
; Function io_write_buffer
; ---------------------------------
_io_write_buffer::
;../SMZ80_SDK/V1/include/smz80.h:277: __endasm;
	LD IX, #2
	ADD IX,SP
	LD C, (IX)
	INC IX
	LD L,(IX)
	INC IX
	LD H,(IX)
	INC IX
	LD B,(IX)
	OTIR
	ret
;../SMZ80_SDK/V1/include/smz80.h:279: void io_read_buffer(char port_addr, char* buffer_in, char count) {
;	---------------------------------
; Function io_read_buffer
; ---------------------------------
_io_read_buffer::
;../SMZ80_SDK/V1/include/smz80.h:294: __endasm;
	LD IX, #2
	ADD IX,SP
	LD C, (IX)
	INC IX
	LD L,(IX)
	INC IX
	LD H,(IX)
	INC IX
	LD B,(IX)
	INIR
	ret
;../SMZ80_SDK/V1/include/smz80.h:297: void uart_init(const uart_cfg_t *uart_config) {
;	---------------------------------
; Function uart_init
; ---------------------------------
_uart_init::
	push	ix
	ld	ix,#0
	add	ix,sp
;../SMZ80_SDK/V1/include/smz80.h:299: uart_set_baudrate(uart_config->baudrate);
	ld	e,4 (ix)
	ld	d,5 (ix)
	ld	a,(de)
	push	de
	push	af
	inc	sp
	call	_uart_set_baudrate
	inc	sp
	pop	de
;../SMZ80_SDK/V1/include/smz80.h:300: URIER = uart_config->interrupt;
	ld	l, e
	ld	h, d
	ld	bc, #0x0004
	add	hl, bc
	ld	a,(hl)
	out	(_URIER),a
;../SMZ80_SDK/V1/include/smz80.h:301: URLCR = (uart_config->stop_bits) | (uart_config->parity) | (uart_config->word_length);
	ld	l, e
	ld	h, d
	inc	hl
	ld	b,(hl)
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	ld	a,(hl)
	or	a, b
	ld	h,d
	ld	l, e
	inc	hl
	inc	hl
	inc	hl
	ld	d,(hl)
	or	a, d
	out	(_URLCR),a
	pop	ix
	ret
;../SMZ80_SDK/V1/include/smz80.h:303: void uart_set_baudrate(const uart_baudrate_t baudrate) {
;	---------------------------------
; Function uart_set_baudrate
; ---------------------------------
_uart_set_baudrate::
;../SMZ80_SDK/V1/include/smz80.h:304: URLCR |= BV(UDLAB);
	in	a,(_URLCR)
	set	7, a
	out	(_URLCR),a
;../SMZ80_SDK/V1/include/smz80.h:305: URDLL = baudrate;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URDLL),a
;../SMZ80_SDK/V1/include/smz80.h:306: URDLM = ((uint16_t)baudrate)>>8;
	ld	a, #0x00
	out	(_URDLM),a
;../SMZ80_SDK/V1/include/smz80.h:307: URLCR &= ~BV(UDLAB);
	in	a,(_URLCR)
	and	a, #0x7F
	out	(_URLCR),a
	ret
;../SMZ80_SDK/V1/include/smz80.h:309: void uart_write(char c) {
;	---------------------------------
; Function uart_write
; ---------------------------------
_uart_write::
;../SMZ80_SDK/V1/include/smz80.h:310: while( !(URLSR & BV(UTHRE)))
00101$:
	in	a,(_URLSR)
	and	a, #0x20
	jr	NZ,00103$
;../SMZ80_SDK/V1/include/smz80.h:311: NOP();    
	NOP
	jr	00101$
00103$:
;../SMZ80_SDK/V1/include/smz80.h:312: URTHR = c;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URTHR),a
	ret
;../SMZ80_SDK/V1/include/smz80.h:314: char uart_read() {
;	---------------------------------
; Function uart_read
; ---------------------------------
_uart_read::
;../SMZ80_SDK/V1/include/smz80.h:315: while(!(URLSR & BV(UDR))) 
00101$:
	in	a,(_URLSR)
	rrca
	jr	C,00103$
;../SMZ80_SDK/V1/include/smz80.h:316: NOP();
	NOP
	jr	00101$
00103$:
;../SMZ80_SDK/V1/include/smz80.h:317: return URRBR;
	in	a,(_URRBR)
	ld	l,a
	ret
;../SMZ80_SDK/V1/include/smz80.h:319: int uart_available(){
;	---------------------------------
; Function uart_available
; ---------------------------------
_uart_available::
;../SMZ80_SDK/V1/include/smz80.h:320: return (URLSR & BV(UDR));
	in	a,(_URLSR)
	and	a, #0x01
	ld	l,a
	ld	h,#0x00
	ret
;../SMZ80_SDK/V1/include/smz80.h:322: void uart_print(const char* str) {
;	---------------------------------
; Function uart_print
; ---------------------------------
_uart_print::
;../SMZ80_SDK/V1/include/smz80.h:323: while(*str)       
	pop	bc
	pop	hl
	push	hl
	push	bc
00101$:
	ld	a,(hl)
	or	a, a
	ret	Z
;../SMZ80_SDK/V1/include/smz80.h:324: putchar(*str++); // envía el siguiente caracter. 
	inc	hl
	ld	e,a
	rla
	sbc	a, a
	ld	d,a
	push	hl
	push	de
	call	_putchar
	pop	af
	pop	hl
	jr	00101$
;../SMZ80_SDK/V1/include/smz80.h:326: int uart_read_line(char* str) {
;	---------------------------------
; Function uart_read_line
; ---------------------------------
_uart_read_line::
	push	ix
	ld	ix,#0
	add	ix,sp
	push	af
;../SMZ80_SDK/V1/include/smz80.h:327: int n=0;
	ld	bc,#0x0000
;../SMZ80_SDK/V1/include/smz80.h:329: while(n<MAXLINE-1 && (c=getchar()) != '\n' && c !='\r') {
00111$:
	ld	a,c
	sub	a, #0xFF
	ld	a,b
	rla
	ccf
	rra
	sbc	a, #0x83
	jp	NC,00113$
	push	bc
	call	_getchar
	ld	a,l
	pop	bc
	ld	h,a
	sub	a, #0x0A
	jp	Z,00113$
	ld	a,h
	sub	a, #0x0D
	jr	Z,00113$
;../SMZ80_SDK/V1/include/smz80.h:333: putchar(c);
	ld	-2 (ix),h
	ld	a,h
	rla
	sbc	a, a
	ld	-1 (ix),a
;../SMZ80_SDK/V1/include/smz80.h:330: if(c == 0x7F || c==0x08) {
	ld	a,h
	cp	a,#0x7F
	jr	Z,00105$
	sub	a, #0x08
	jr	NZ,00106$
00105$:
;../SMZ80_SDK/V1/include/smz80.h:331: if(n>0){
	xor	a, a
	cp	a, c
	sbc	a, b
	jp	PO, 00149$
	xor	a, #0x80
00149$:
	jp	P,00111$
;../SMZ80_SDK/V1/include/smz80.h:332: str[--n]='\0';
	dec	bc
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	(hl),#0x00
;../SMZ80_SDK/V1/include/smz80.h:333: putchar(c);
	push	bc
	pop	de
	pop	hl
	push	hl
	push	de
	push	hl
	call	_putchar
	ld	hl, #0x0020
	ex	(sp),hl
	call	_putchar
	pop	af
	pop	de
	pop	hl
	push	hl
	push	de
	push	hl
	call	_putchar
	pop	af
	pop	bc
	jr	00111$
00106$:
;../SMZ80_SDK/V1/include/smz80.h:339: if(isprint(c)) {
	push	hl
	push	bc
	ld	e,-2 (ix)
	ld	d,-1 (ix)
	push	de
	call	_isprint
	pop	af
	ld	d,h
	pop	bc
	pop	af
	ld	h,a
	ld	a,d
	or	a,l
	jr	Z,00111$
;../SMZ80_SDK/V1/include/smz80.h:340: str[n++]=c;
	push	bc
	pop	iy
	inc	bc
	ld	e,4 (ix)
	ld	d,5 (ix)
	add	iy, de
	ld	0 (iy), h
;../SMZ80_SDK/V1/include/smz80.h:341: putchar(c);
	push	bc
	pop	de
	pop	hl
	push	hl
	push	de
	push	hl
	call	_putchar
	pop	af
	pop	bc
	jp	00111$
00113$:
;../SMZ80_SDK/V1/include/smz80.h:344: str[n]='\0';     
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	(hl),#0x00
;../SMZ80_SDK/V1/include/smz80.h:345: putchar('\n');
	push	bc
	ld	hl,#0x000A
	push	hl
	call	_putchar
	pop	af
;../SMZ80_SDK/V1/include/smz80.h:346: return n;
	pop	hl
	ld	sp, ix
	pop	ix
	ret
;../SMZ80_SDK/V1/include/smz80.h:348: void uart_disable_interrupts() {
;	---------------------------------
; Function uart_disable_interrupts
; ---------------------------------
_uart_disable_interrupts::
;../SMZ80_SDK/V1/include/smz80.h:349: URIER = 0;
	ld	a,#0x00
	out	(_URIER),a
	ret
;../SMZ80_SDK/V1/include/smz80.h:351: void uart_enable_interrupts(uart_interrupt_t int_cfg) {
;	---------------------------------
; Function uart_enable_interrupts
; ---------------------------------
_uart_enable_interrupts::
;../SMZ80_SDK/V1/include/smz80.h:352: URIER = int_cfg;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	out	(_URIER),a
	ret
;../SMZ80_SDK/V1/include/smz80.h:356: void ppi_init(const ppi_cfg_t *ppi_config) {
;	---------------------------------
; Function ppi_init
; ---------------------------------
_ppi_init::
	push	ix
	ld	ix,#0
	add	ix,sp
;../SMZ80_SDK/V1/include/smz80.h:357: PPI_CTRL = 0x80 | ppi_config->mode | (ppi_config->pcl_dir << PCPCL) | (ppi_config->pch_dir << PCPCH) | (ppi_config->pa_dir << PCPA) | (ppi_config->pb_dir << PCPB);
	ld	c,4 (ix)
	ld	b,5 (ix)
	ld	a,(bc)
	set	7, a
	ld	e,a
	push	bc
	pop	iy
	ld	a,3 (iy)
	or	a, e
	ld	e,a
	push	bc
	pop	iy
	ld	a,4 (iy)
	rlca
	rlca
	rlca
	and	a,#0xF8
	or	a, e
	ld	e,a
	ld	l, c
	ld	h, b
	inc	hl
	ld	a,(hl)
	rlca
	rlca
	rlca
	rlca
	and	a,#0xF0
	or	a, e
	ld	d,a
	ld	l, c
	ld	h, b
	inc	hl
	inc	hl
	ld	a,(hl)
	add	a, a
	or	a, d
	out	(_PPI_CTRL),a
	pop	ix
	ret
;../SMZ80_SDK/V1/include/smz80.h:359: void ppi_set_portc_bit(const char bit) {
;	---------------------------------
; Function ppi_set_portc_bit
; ---------------------------------
_ppi_set_portc_bit::
;../SMZ80_SDK/V1/include/smz80.h:360: PPI_CTRL = 1 | bit << 1;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	add	a, a
	set	0, a
	out	(_PPI_CTRL),a
	ret
;../SMZ80_SDK/V1/include/smz80.h:362: void ppi_clear_portc_bit(const char bit) {
;	---------------------------------
; Function ppi_clear_portc_bit
; ---------------------------------
_ppi_clear_portc_bit::
;../SMZ80_SDK/V1/include/smz80.h:363: PPI_CTRL = bit << 1;
	ld	hl, #2+0
	add	hl, sp
	ld	a, (hl)
	add	a, a
	out	(_PPI_CTRL),a
	ret
;../SMZ80_SDK/V1/include/smz80.h:366: void delay_10us(){
;	---------------------------------
; Function delay_10us
; ---------------------------------
_delay_10us::
;../SMZ80_SDK/V1/include/smz80.h:375: __endasm;
	EXX
	EX AF,AF'
	LD B,#0x2
	    LOOP_10:
	DJNZ LOOP_10
	EX AF,AF'
	EXX
	ret
;../SMZ80_SDK/V1/include/smz80.h:377: void delay_100us(){
;	---------------------------------
; Function delay_100us
; ---------------------------------
_delay_100us::
;../SMZ80_SDK/V1/include/smz80.h:386: __endasm;
	EXX
	EX AF,AF'
	LD B,#0x3A
	    LOOP_100:
	DJNZ LOOP_100
	EX AF,AF'
	EXX
	ret
;../SMZ80_SDK/V1/include/smz80.h:388: void delay_ms(int ms) {
;	---------------------------------
; Function delay_ms
; ---------------------------------
_delay_ms::
	push	ix
	ld	ix,#0
	add	ix,sp
;../SMZ80_SDK/V1/include/smz80.h:391: while(ms--)
	ld	c,4 (ix)
	ld	b,5 (ix)
00102$:
	ld	e, c
	ld	d, b
	dec	bc
	ld	a,d
	or	a,e
	jr	Z,00108$
;../SMZ80_SDK/V1/include/smz80.h:392: for(i=0;i<0x106;i++)
	ld	hl,#0x0106
00107$:
;../SMZ80_SDK/V1/include/smz80.h:393: __asm__("nop");
	nop
	ex	de,hl
	dec	de
	ld	l, e
;../SMZ80_SDK/V1/include/smz80.h:392: for(i=0;i<0x106;i++)
	ld	a,d
	ld	h,a
	or	a,e
	jr	NZ,00107$
	jr	00102$
00108$:
	pop	ix
	ret
;../SMZ80_SDK/V1/include/smz80.h:395: int putchar(int c) {
;	---------------------------------
; Function putchar
; ---------------------------------
_putchar::
;../SMZ80_SDK/V1/include/smz80.h:397: if(c=='\n')
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	sub	a, #0x0A
	jr	NZ,00102$
	ld	a,1 (iy)
	or	a, a
	jr	NZ,00102$
;../SMZ80_SDK/V1/include/smz80.h:398: uart_write('\r');
	ld	a,#0x0D
	push	af
	inc	sp
	call	_uart_write
	inc	sp
00102$:
;../SMZ80_SDK/V1/include/smz80.h:399: uart_write(c);
	ld	iy,#2
	add	iy,sp
	ld	h,0 (iy)
	push	hl
	inc	sp
	call	_uart_write
	inc	sp
;../SMZ80_SDK/V1/include/smz80.h:401: return c;
	pop	bc
	pop	hl
	push	hl
	push	bc
	ret
;../SMZ80_SDK/V1/include/smz80.h:403: char getchar() {
;	---------------------------------
; Function getchar
; ---------------------------------
_getchar::
;../SMZ80_SDK/V1/include/smz80.h:406: return uart_read();
	jp	_uart_read
;spi.h:17: void spiInit() {
;	---------------------------------
; Function spiInit
; ---------------------------------
_spiInit::
;spi.h:19: PPI_CTRL = 0x81;
	ld	a,#0x81
	out	(_PPI_CTRL),a
;spi.h:20: PPI_SPI_PORT = (1 << PPI_SPI_SS);
	ld	a,#0x02
	out	(_PPI_PORTA),a
	ret
;spi.h:23: void spiWrite8(uint8_t data) {
;	---------------------------------
; Function spiWrite8
; ---------------------------------
_spiWrite8::
;spi.h:26: PPI_SPI_PORT = 0;  //Selecciona el chip y pone a cero la linea de datos y reloj
	ld	a,#0x00
	out	(_PPI_PORTA),a
;spi.h:28: for (i = 0; i < 8; ++i)
	ld	d,#0x00
00105$:
;spi.h:30: if( (data << i) & 0x80 )                    // si el bit MSB es 1
	ld	iy,#2
	add	iy,sp
	ld	l,0 (iy)
	ld	h,#0x00
	ld	b,d
	inc	b
	jr	00123$
00122$:
	add	hl, hl
00123$:
	djnz	00122$
	bit	7, l
	jr	Z,00102$
;spi.h:31: PPI_SPI_PORT |= (1 << PPI_SPI_MOSI);    // Pone en alto el pin de datos
	in	a,(_PPI_PORTA)
	set	0, a
	out	(_PPI_PORTA),a
	jr	00103$
00102$:
;spi.h:33: PPI_SPI_PORT &= ~(1 << PPI_SPI_MOSI);    // Pone en alto el pin de datos
	in	a,(_PPI_PORTA)
	and	a, #0xFE
	out	(_PPI_PORTA),a
00103$:
;spi.h:35: PPI_SPI_PORT |= (1 << PPI_SPI_SCK);         // Pone a 1 la linea del reloj
	in	a,(_PPI_PORTA)
	set	2, a
	out	(_PPI_PORTA),a
;spi.h:36: PPI_SPI_PORT &= ~(1 << PPI_SPI_SCK);        // pone a 0 la linea del reloj.
	in	a,(_PPI_PORTA)
	and	a, #0xFB
	out	(_PPI_PORTA),a
;spi.h:28: for (i = 0; i < 8; ++i)
	inc	d
	ld	a,d
	sub	a, #0x08
	jr	C,00105$
;spi.h:39: PPI_SPI_PORT = (1 << PPI_SPI_SS);               // Deshabilita el chip
	ld	a,#0x02
	out	(_PPI_PORTA),a
	ret
;spi.h:42: void spiWrite16(uint16_t data) {
;	---------------------------------
; Function spiWrite16
; ---------------------------------
_spiWrite16::
;spi.h:45: PPI_SPI_PORT = 0;  //Selecciona el chip y pone a cero la linea de datos y reloj
	ld	a,#0x00
	out	(_PPI_PORTA),a
;spi.h:47: for (i = 0; i < 16; ++i)
	ld	d,#0x00
00105$:
;spi.h:49: if( (data << i) & 0x8000 )                  // si el bit MSB es 1
	ld	b,d
	push	af
	ld	hl, #4
	add	hl, sp
	ld	a, (hl)
	inc	hl
	ld	h, (hl)
	ld	l, a
	pop	af
	inc	b
	jr	00123$
00122$:
	add	hl, hl
00123$:
	djnz	00122$
	add	hl, hl
	jr	NC,00102$
;spi.h:50: PPI_SPI_PORT |= (1 << PPI_SPI_MOSI);    // Pone en alto el pin de datos
	in	a,(_PPI_PORTA)
	set	0, a
	out	(_PPI_PORTA),a
	jr	00103$
00102$:
;spi.h:52: PPI_SPI_PORT &= ~(1 << PPI_SPI_MOSI);    // Pone en alto el pin de datos
	in	a,(_PPI_PORTA)
	and	a, #0xFE
	out	(_PPI_PORTA),a
00103$:
;spi.h:54: PPI_SPI_PORT |= (1 << PPI_SPI_SCK);         // Pone a 1 la linea del reloj.
	in	a,(_PPI_PORTA)
	set	2, a
	out	(_PPI_PORTA),a
;spi.h:56: PPI_SPI_PORT &= ~(1 << PPI_SPI_SCK);        // pone a 0 la linea del reloj.
	in	a,(_PPI_PORTA)
	and	a, #0xFB
	out	(_PPI_PORTA),a
;spi.h:47: for (i = 0; i < 16; ++i)
	inc	d
	ld	a,d
	sub	a, #0x10
	jr	C,00105$
;spi.h:60: PPI_SPI_PORT = (1 << PPI_SPI_SS);               // Deshabilita el chip
	ld	a,#0x02
	out	(_PPI_PORTA),a
	ret
;spi.h:63: void spiWriteBuffer8(uint8_t* buffer, uint8_t bufferLen) {
;	---------------------------------
; Function spiWriteBuffer8
; ---------------------------------
_spiWriteBuffer8::
;spi.h:66: for (i = 0; i < bufferLen; ++i)
	ld	e,#0x00
00103$:
	ld	hl,#4
	add	hl,sp
	ld	a,e
	sub	a, (hl)
	ret	NC
;spi.h:67: spiWrite8(buffer[i]);
	ld	hl, #2
	add	hl, sp
	ld	a, (hl)
	inc	hl
	ld	h, (hl)
	ld	l, a
	ld	d,#0x00
	add	hl, de
	ld	h,(hl)
	push	de
	push	hl
	inc	sp
	call	_spiWrite8
	inc	sp
	pop	de
;spi.h:66: for (i = 0; i < bufferLen; ++i)
	inc	e
	jr	00103$
;spi.h:71: void spiWriteBuffer16(uint16_t* buffer, uint8_t bufferLen) {
;	---------------------------------
; Function spiWriteBuffer16
; ---------------------------------
_spiWriteBuffer16::
;spi.h:74: for (i = 0; i < bufferLen; ++i)
	ld	d,#0x00
00103$:
	ld	hl,#4
	add	hl,sp
	ld	a,d
	sub	a, (hl)
	ret	NC
;spi.h:75: spiWrite16(buffer[i]);
	ld	l,d
	ld	h,#0x00
	add	hl, hl
	ld	e,l
	ld	b,h
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	add	a, e
	ld	c,a
	ld	a,1 (iy)
	adc	a, b
	ld	b,a
	ld	l, c
	ld	h, b
	ld	c,(hl)
	inc	hl
	ld	b,(hl)
	push	de
	push	bc
	call	_spiWrite16
	pop	af
	pop	de
;spi.h:74: for (i = 0; i < bufferLen; ++i)
	inc	d
	jr	00103$
;max7219.h:42: void max7219WriteRegister(unsigned char reg, unsigned char data) {
;	---------------------------------
; Function max7219WriteRegister
; ---------------------------------
_max7219WriteRegister::
	push	ix
	ld	ix,#0
	add	ix,sp
;max7219.h:45: max_data = (((uint16_t)(reg)) & (0x0F)) << 8;
	ld	a, 4 (ix)
	and	a, #0x0F
	ld	h,a
	ld	l,#0x00
;max7219.h:46: max_data |= data;
	ld	e,5 (ix)
	ld	d,#0x00
	ld	a,l
	or	a, e
	ld	l,a
	ld	a,h
	or	a, d
	ld	h,a
;max7219.h:48: spiWrite16(max_data);
	push	hl
	call	_spiWrite16
	pop	af
	pop	ix
	ret
;max7219.h:52: void max7219Init() {
;	---------------------------------
; Function max7219Init
; ---------------------------------
_max7219Init::
;max7219.h:53: spiInit();
	call	_spiInit
;max7219.h:56: delay_ms(10);
	ld	hl,#0x000A
	push	hl
	call	_delay_ms
;max7219.h:57: max7219WriteRegister(SHUTDOWN_REG, 0x01);
	ld	hl, #0x010C
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:58: delay_ms(10);
	ld	hl, #0x000A
	ex	(sp),hl
	call	_delay_ms
;max7219.h:59: max7219WriteRegister(MODE_REG, 0x00);
	ld	hl, #0x0009
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:60: delay_ms(10);
	ld	hl, #0x000A
	ex	(sp),hl
	call	_delay_ms
;max7219.h:61: max7219WriteRegister(SCANL_REG, 0x07);
	ld	hl, #0x070B
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:62: delay_ms(10);
	ld	hl, #0x000A
	ex	(sp),hl
	call	_delay_ms
;max7219.h:63: max7219WriteRegister(BRIGHT_REG, 0x01);
	ld	hl, #0x010A
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:64: delay_ms(10);
	ld	hl, #0x000A
	ex	(sp),hl
	call	_delay_ms
	pop	af
;max7219.h:65: max7219Clear();
	call	_max7219Clear
;max7219.h:66: delay_ms(10);
	ld	hl,#0x000A
	push	hl
	call	_delay_ms
	pop	af
	ret
;max7219.h:69: void max7219Clear() {
;	---------------------------------
; Function max7219Clear
; ---------------------------------
_max7219Clear::
;max7219.h:71: max7219WriteRegister(ROW1_REG, 0x00);
	ld	hl,#0x0001
	push	hl
	call	_max7219WriteRegister
;max7219.h:72: max7219WriteRegister(ROW2_REG, 0x00);
	ld	hl, #0x0002
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:73: max7219WriteRegister(ROW3_REG, 0x00);
	ld	hl, #0x0003
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:74: max7219WriteRegister(ROW4_REG, 0x00);
	ld	hl, #0x0004
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:75: max7219WriteRegister(ROW5_REG, 0x00);
	ld	hl, #0x0005
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:76: max7219WriteRegister(ROW6_REG, 0x00);
	ld	hl, #0x0006
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:77: max7219WriteRegister(ROW7_REG, 0x00);
	ld	hl, #0x0007
	ex	(sp),hl
	call	_max7219WriteRegister
;max7219.h:78: max7219WriteRegister(ROW8_REG, 0x00);
	ld	hl, #0x0008
	ex	(sp),hl
	call	_max7219WriteRegister
	pop	af
	ret
;max7219.h:80: void max7219DrawPixel(unsigned char x, unsigned char y, unsigned char color) {
;	---------------------------------
; Function max7219DrawPixel
; ---------------------------------
_max7219DrawPixel::
	push	ix
	ld	ix,#0
	add	ix,sp
;max7219.h:84: if(x<1 || x > 8 || y < 1 || y > 8)
	ld	a,4 (ix)
	sub	a, #0x01
	jr	C,00109$
	ld	a,#0x08
	sub	a, 4 (ix)
	jr	C,00109$
	ld	a,5 (ix)
	sub	a, #0x01
	jr	C,00109$
	ld	a,#0x08
	sub	a, 5 (ix)
;max7219.h:85: return ;
	jr	C,00109$
;max7219.h:87: ren = _displayBuffer[y];
	ld	a,5 (ix)
	add	a, #<(__displayBuffer)
	ld	c,a
	ld	a,#0x00
	adc	a, #>(__displayBuffer)
	ld	b,a
	ld	a,(bc)
	ld	d,a
;max7219.h:88: col = ( 1 << (x-1) );
	ld	h,4 (ix)
	dec	h
	push	af
	ld	e,#0x01
	pop	af
	inc	h
	jr	00128$
00127$:
	sla	e
00128$:
	dec	h
	jr	NZ,00127$
;max7219.h:90: if(color)
	ld	a,6 (ix)
	or	a, a
	jr	Z,00107$
;max7219.h:91: ren |= col;
	ld	a,d
	or	a, e
	jr	00108$
00107$:
;max7219.h:93: ren &= ~col;
	ld	a,e
	cpl
	ld	e,a
	ld	a,d
	and	a, e
00108$:
;max7219.h:95: _displayBuffer[y] = ren;
	ld	(bc),a
;max7219.h:96: max7219WriteRegister(y, _displayBuffer[y]);
	push	af
	inc	sp
	ld	a,5 (ix)
	push	af
	inc	sp
	call	_max7219WriteRegister
	pop	af
00109$:
	pop	ix
	ret
;max7219.h:99: void max7219Test() {
;	---------------------------------
; Function max7219Test
; ---------------------------------
_max7219Test::
;max7219.h:104: for( i=1;i<9;i++ )
	ld	d,#0x01
;max7219.h:106: for( j=1;j<9;j++ )
00109$:
	ld	e,#0x01
00103$:
;max7219.h:108: max7219DrawPixel(i, j, (unsigned char)1);
	push	de
	ld	a,#0x01
	push	af
	inc	sp
	ld	a,e
	push	af
	inc	sp
	push	de
	inc	sp
	call	_max7219DrawPixel
	inc	sp
	ld	hl,#0x012C
	ex	(sp),hl
	call	_delay_ms
	pop	af
	pop	de
;max7219.h:110: max7219DrawPixel(i, j,(unsigned char)0);
	push	de
	xor	a, a
	push	af
	inc	sp
	ld	a,e
	push	af
	inc	sp
	push	de
	inc	sp
	call	_max7219DrawPixel
	pop	af
	inc	sp
	pop	de
;max7219.h:106: for( j=1;j<9;j++ )
	inc	e
	ld	a,e
	sub	a, #0x09
	jr	C,00103$
;max7219.h:104: for( i=1;i<9;i++ )
	inc	d
	ld	a,d
	sub	a, #0x09
	jr	C,00109$
	ret
;max7219.h:116: void max7219SetBright(uint8_t bright) {
;	---------------------------------
; Function max7219SetBright
; ---------------------------------
_max7219SetBright::
;max7219.h:117: bright = bright & 0x0F;
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	and	a, #0x0F
	ld	0 (iy),a
;max7219.h:118: max7219WriteRegister(BRIGHT_REG, bright);
	ld	d, 0 (iy)
	ld	e,#0x0A
	push	de
	call	_max7219WriteRegister
	pop	af
	ret
;max7219.h:121: void max7219WriteDisplayBuffer(unsigned char *buffer, unsigned char len) {
;	---------------------------------
; Function max7219WriteDisplayBuffer
; ---------------------------------
_max7219WriteDisplayBuffer::
;max7219.h:124: if(len >8)
	ld	a,#0x08
	ld	iy,#4
	add	iy,sp
	sub	a, 0 (iy)
	jr	NC,00102$
;max7219.h:125: len = 8;
	ld	0 (iy),#0x08
00102$:
;max7219.h:127: for (i = 0; i < len; ++i)
	ld	e,#0x00
00105$:
	ld	hl,#4
	add	hl,sp
	ld	a,e
	sub	a, (hl)
	ret	NC
;max7219.h:128: max7219WriteRegister(ROW1_REG + i, buffer[i]);
	ld	hl, #2
	add	hl, sp
	ld	a, (hl)
	inc	hl
	ld	h, (hl)
	ld	l, a
	ld	d,#0x00
	add	hl, de
	ld	h,(hl)
	ld	d,e
	inc	d
	push	de
	push	hl
	inc	sp
	push	de
	inc	sp
	call	_max7219WriteRegister
	pop	af
	pop	de
;max7219.h:127: for (i = 0; i < len; ++i)
	ld	e,d
	jr	00105$
;max7219.h:132: void max7219ReadDisplayBuffer(unsigned char *buffer, unsigned char len) {
;	---------------------------------
; Function max7219ReadDisplayBuffer
; ---------------------------------
_max7219ReadDisplayBuffer::
	push	ix
	ld	ix,#0
	add	ix,sp
;max7219.h:135: if(len >8)
	ld	a,#0x08
	sub	a, 6 (ix)
	jr	NC,00111$
;max7219.h:136: len = 8;
	ld	6 (ix),#0x08
;max7219.h:138: for (i = 0; i < len; ++i)
00111$:
	ld	e,#0x00
00105$:
	ld	a,e
	sub	a, 6 (ix)
	jr	NC,00107$
;max7219.h:139: buffer[i] = _displayBuffer[i];
	ld	a,4 (ix)
	add	a, e
	ld	c,a
	ld	a,5 (ix)
	adc	a, #0x00
	ld	b,a
	ld	hl,#__displayBuffer
	ld	d,#0x00
	add	hl, de
	ld	a,(hl)
	ld	(bc),a
;max7219.h:138: for (i = 0; i < len; ++i)
	inc	e
	jr	00105$
00107$:
	pop	ix
	ret
;max7219.h:144: void max7219DrawRow(unsigned char row, unsigned char data) {
;	---------------------------------
; Function max7219DrawRow
; ---------------------------------
_max7219DrawRow::
	push	ix
	ld	ix,#0
	add	ix,sp
;max7219.h:145: if(row < 1 || row > 8)
	ld	a,4 (ix)
	sub	a, #0x01
	jr	C,00104$
	ld	a,#0x08
	sub	a, 4 (ix)
;max7219.h:146: return ;
	jr	C,00104$
;max7219.h:147: _displayBuffer[row] = data;
	ld	de,#__displayBuffer+0
	ld	l,4 (ix)
	ld	h,#0x00
	add	hl,de
	ld	a,5 (ix)
	ld	(hl),a
;max7219.h:148: max7219WriteRegister(row, data);
	ld	h,5 (ix)
	ld	l,4 (ix)
	push	hl
	call	_max7219WriteRegister
	pop	af
00104$:
	pop	ix
	ret
;max7219.h:151: void max7219DrawColumn(unsigned char column, unsigned char data) {
;	---------------------------------
; Function max7219DrawColumn
; ---------------------------------
_max7219DrawColumn::
;max7219.h:154: for (i = 0; i < 8; ++i)
	ld	de,#0x0000
00102$:
;max7219.h:156: color = (data>>i) & 0x01 ;
	ld	b,e
	push	af
	ld	iy,#5
	add	iy,sp
	ld	l,0 (iy)
	pop	af
	inc	b
	jr	00112$
00111$:
	srl	l
00112$:
	djnz	00111$
	ld	a,l
	and	a, #0x01
	ld	h,a
	ld	l,#0x00
;max7219.h:157: max7219DrawPixel(column, i, color);
	ld	b,e
	push	de
	push	hl
	inc	sp
	push	bc
	inc	sp
	ld	hl, #6+0
	add	hl, sp
	ld	a, (hl)
	push	af
	inc	sp
	call	_max7219DrawPixel
	pop	af
	inc	sp
	pop	de
;max7219.h:154: for (i = 0; i < 8; ++i)
	inc	de
	ld	a,e
	sub	a, #0x08
	ld	a,d
	sbc	a, #0x00
	jr	C,00102$
	ret
;max7219.h:161: uint8_t max7219ReadPixel(unsigned char x, unsigned char y) {
;	---------------------------------
; Function max7219ReadPixel
; ---------------------------------
_max7219ReadPixel::
	push	ix
	ld	ix,#0
	add	ix,sp
;max7219.h:164: if(x<1 || x > 8 || y < 1 || y > 8)
	ld	a,4 (ix)
	sub	a, #0x01
	jr	C,00106$
	ld	a,#0x08
	sub	a, 4 (ix)
	jr	C,00106$
	ld	a,5 (ix)
	sub	a, #0x01
	jr	C,00106$
	ld	a,#0x08
	sub	a, 5 (ix)
;max7219.h:165: return ;
	jr	C,00106$
;max7219.h:167: return _displayBuffer[y] & ( 1 << (x-1) );
	ld	de,#__displayBuffer+0
	ld	l,5 (ix)
	ld	h,#0x00
	add	hl,de
	ld	h,(hl)
	ld	b,4 (ix)
	dec	b
	push	af
	ld	l,#0x01
	pop	af
	inc	b
	jr	00121$
00120$:
	sla	l
00121$:
	djnz	00120$
	ld	a,h
	and	a, l
	ld	l,a
00106$:
	pop	ix
	ret
;max7219.h:170: uint8_t max7219ReadRow(unsigned char row) {
;	---------------------------------
; Function max7219ReadRow
; ---------------------------------
_max7219ReadRow::
;max7219.h:171: if(row < 1 || row > 8)
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	sub	a, #0x01
	jr	C,00101$
	ld	a,#0x08
	sub	a, 0 (iy)
	jr	NC,00102$
00101$:
;max7219.h:172: return 0;
	ld	l,#0x00
	ret
00102$:
;max7219.h:174: return _displayBuffer[row];
	ld	a,#<(__displayBuffer)
	ld	hl,#2
	add	hl,sp
	add	a, (hl)
	ld	l, a
	ld	a, #>(__displayBuffer)
	adc	a, #0x00
	ld	h, a
	ld	l,(hl)
	ret
;max7219.h:177: uint8_t max7219ReadColumn(unsigned char column) {
;	---------------------------------
; Function max7219ReadColumn
; ---------------------------------
_max7219ReadColumn::
;max7219.h:179: }
	ret
;main.c:35: ISR_NMI(){
;	---------------------------------
; Function isr_vector66
; ---------------------------------
_isr_vector66::
	push	af
	push	bc
	push	de
	push	hl
	push	iy
;main.c:38: }
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	retn
;main.c:40: ISR_INT_38(){
;	---------------------------------
; Function isr_vector38
; ---------------------------------
_isr_vector38::
	push	af
	push	bc
	push	de
	push	hl
	push	iy
;main.c:41: }
	pop	iy
	pop	hl
	pop	de
	pop	bc
	pop	af
	reti
;main.c:51: void system_init(){
;	---------------------------------
; Function system_init
; ---------------------------------
_system_init::
;main.c:53: max7219Init();
	call	_max7219Init
;main.c:54: PPI_CTRL = 0x89;
	ld	a,#0x89
	out	(_PPI_CTRL),a
;main.c:55: cont =1;
	ld	hl,#0x0001
	ld	(_cont),hl
	ret
;main.c:63: void dibujaFigura(pixel f[3][3])
;	---------------------------------
; Function dibujaFigura
; ---------------------------------
_dibujaFigura::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-9
	add	hl,sp
	ld	sp,hl
;main.c:68: for( i=0;i<3;i++ )
	ld	-7 (ix),#0x00
	ld	-6 (ix),#0x00
	ld	de,#0x0000
;main.c:70: for( j=0;j<3;j++ )
00112$:
	ld	a,4 (ix)
	add	a, e
	ld	-5 (ix),a
	ld	a,5 (ix)
	adc	a, d
	ld	-4 (ix),a
	ld	hl,#0x0000
	ex	(sp), hl
	ld	bc,#0x0000
00105$:
;main.c:73: if(f[i][j].val==1)
	push	hl
	ld	l,-5 (ix)
	ld	h,-4 (ix)
	push	hl
	pop	iy
	pop	hl
	add	iy, bc
	push	iy
	pop	hl
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	-2 (ix), a
	dec	a
	jr	NZ,00106$
;main.c:74: max7219DrawPixel(f[i][j].x,f[i][j].y,f[i][j].val);
	push	iy
	pop	hl
	inc	hl
	ld	a,(hl)
	ld	-1 (ix),a
	ld	a, 0 (iy)
	ld	-3 (ix),a
	push	bc
	push	de
	ld	h,-2 (ix)
	ld	l,-1 (ix)
	push	hl
	ld	a,-3 (ix)
	push	af
	inc	sp
	call	_max7219DrawPixel
	pop	af
	inc	sp
	pop	de
	pop	bc
00106$:
;main.c:70: for( j=0;j<3;j++ )
	inc	bc
	inc	bc
	inc	bc
	inc	-9 (ix)
	jr	NZ,00131$
	inc	-8 (ix)
00131$:
	ld	a,-9 (ix)
	sub	a, #0x03
	ld	a,-8 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00105$
;main.c:68: for( i=0;i<3;i++ )
	ld	hl,#0x0009
	add	hl,de
	ex	de,hl
	inc	-7 (ix)
	jr	NZ,00132$
	inc	-6 (ix)
00132$:
	ld	a,-7 (ix)
	sub	a, #0x03
	ld	a,-6 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00112$
	ld	sp, ix
	pop	ix
	ret
;main.c:81: void borraFigura(pixel f[3][3])
;	---------------------------------
; Function borraFigura
; ---------------------------------
_borraFigura::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-8
	add	hl,sp
	ld	sp,hl
;main.c:86: for( i=0;i<3;i++ )
	ld	-6 (ix),#0x00
	ld	-5 (ix),#0x00
	ld	de,#0x0000
;main.c:88: for( j=0;j<3;j++ )
00112$:
	ld	a,4 (ix)
	add	a, e
	ld	-2 (ix),a
	ld	a,5 (ix)
	adc	a, d
	ld	-1 (ix),a
	ld	hl,#0x0000
	ex	(sp), hl
	ld	bc,#0x0000
00105$:
;main.c:90: if(f[i][j].val==1)
	push	hl
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	pop	iy
	pop	hl
	add	iy, bc
	push	iy
	pop	hl
	inc	hl
	inc	hl
	ld	a,(hl)
	dec	a
	jr	NZ,00106$
;main.c:91: max7219DrawPixel(f[i][j].x,f[i][j].y,0);
	push	iy
	pop	hl
	inc	hl
	ld	a,(hl)
	ld	-3 (ix),a
	ld	a, 0 (iy)
	ld	-4 (ix),a
	push	bc
	push	de
	xor	a, a
	push	af
	inc	sp
	ld	h,-3 (ix)
	ld	l,-4 (ix)
	push	hl
	call	_max7219DrawPixel
	pop	af
	inc	sp
	pop	de
	pop	bc
00106$:
;main.c:88: for( j=0;j<3;j++ )
	inc	bc
	inc	bc
	inc	bc
	inc	-8 (ix)
	jr	NZ,00131$
	inc	-7 (ix)
00131$:
	ld	a,-8 (ix)
	sub	a, #0x03
	ld	a,-7 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00105$
;main.c:86: for( i=0;i<3;i++ )
	ld	hl,#0x0009
	add	hl,de
	ex	de,hl
	inc	-6 (ix)
	jr	NZ,00132$
	inc	-5 (ix)
00132$:
	ld	a,-6 (ix)
	sub	a, #0x03
	ld	a,-5 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00112$
	ld	sp, ix
	pop	ix
	ret
;main.c:106: void llenaFigura(pixel f[3][3] , int tipo)
;	---------------------------------
; Function llenaFigura
; ---------------------------------
_llenaFigura::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-15
	add	hl,sp
	ld	sp,hl
;main.c:108: f[0][0].x=1;
	ld	e,4 (ix)
	ld	d,5 (ix)
	ld	a,#0x01
	ld	(de),a
;main.c:109: f[0][1].x=1;
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	inc	hl
	ld	(hl),#0x01
;main.c:110: f[0][2].x=1;
	ld	hl,#0x0006
	add	hl,de
	ld	(hl),#0x01
;main.c:112: f[1][0].x=2;
	ld	hl,#0x0009
	add	hl,de
	ld	(hl),#0x02
;main.c:113: f[1][1].x=2;
	ld	hl,#0x000C
	add	hl,de
	ld	(hl),#0x02
;main.c:114: f[1][2].x=2;
	ld	hl,#0x000F
	add	hl,de
	ld	(hl),#0x02
;main.c:116: f[2][0].x=3;
	ld	hl,#0x0012
	add	hl,de
	ld	(hl),#0x03
;main.c:117: f[2][1].x=3;
	ld	hl,#0x0015
	add	hl,de
	ld	(hl),#0x03
;main.c:118: f[2][2].x=3;
	ld	hl,#0x0018
	add	hl,de
	ld	(hl),#0x03
;main.c:120: f[0][0].y=1;
	ld	l, e
	ld	h, d
	inc	hl
	ld	(hl),#0x01
;main.c:121: f[0][1].y=2;
	ld	hl,#0x0004
	add	hl,de
	ld	(hl),#0x02
;main.c:122: f[0][2].y=3;
	ld	hl,#0x0007
	add	hl,de
	ld	(hl),#0x03
;main.c:124: f[1][0].y=1;
	ld	hl,#0x000A
	add	hl,de
	ld	(hl),#0x01
;main.c:125: f[1][1].y=2;
	ld	hl,#0x000D
	add	hl,de
	ld	(hl),#0x02
;main.c:126: f[1][2].y=3;
	ld	hl,#0x0010
	add	hl,de
	ld	(hl),#0x03
;main.c:128: f[2][0].y=1;
	ld	hl,#0x0013
	add	hl,de
	ld	(hl),#0x01
;main.c:129: f[2][1].y=2;
	ld	hl,#0x0016
	add	hl,de
	ld	(hl),#0x02
;main.c:130: f[2][2].y=3;
	ld	hl,#0x0019
	add	hl,de
	ld	(hl),#0x03
;main.c:131: switch(tipo)
	ld	a,6 (ix)
	sub	a, #0x01
	ld	a,7 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00106$
	ld	a,#0x05
	cp	a, 6 (ix)
	ld	a,#0x00
	sbc	a, 7 (ix)
	jp	PO, 00123$
	xor	a, #0x80
00123$:
	jp	M,00106$
	ld	a,6 (ix)
	add	a,#0xFF
	ld	-11 (ix),a
;main.c:135: f[0][0].val=0;    // [0][0][0]
	ld	c, e
	ld	b, d
	inc	bc
	inc	bc
;main.c:136: f[0][1].val=1;
	ld	hl,#0x0005
	add	hl,de
	ld	-13 (ix),l
	ld	-12 (ix),h
;main.c:137: f[0][2].val=1;
	ld	hl,#0x0008
	add	hl,de
	ex	(sp), hl
;main.c:138: f[1][0].val=1;
	ld	hl,#0x000B
	add	hl,de
	ld	-2 (ix),l
	ld	-1 (ix),h
;main.c:139: f[1][1].val=1;
	ld	hl,#0x000E
	add	hl,de
	ld	-4 (ix),l
	ld	-3 (ix),h
;main.c:140: f[1][2].val=0;
	ld	hl,#0x0011
	add	hl,de
	ld	-6 (ix),l
	ld	-5 (ix),h
;main.c:141: f[2][0].val=0;
	ld	hl,#0x0014
	add	hl,de
	ld	-8 (ix),l
	ld	-7 (ix),h
;main.c:142: f[2][1].val=0;
	ld	hl,#0x0017
	add	hl,de
	ld	-10 (ix),l
	ld	-9 (ix),h
;main.c:143: f[2][2].val=0;
	ld	hl,#0x001A
	add	hl,de
;main.c:131: switch(tipo)
	push	hl
	ld	e,-11 (ix)
	ld	d,#0x00
	ld	hl,#00124$
	add	hl,de
	add	hl,de
	add	hl,de
	pop	de
	jp	(hl)
00124$:
	jp	00101$
	jp	00102$
	jp	00103$
	jp	00104$
	jp	00105$
;main.c:133: case 1://Z            // [0][1][1]
00101$:
;main.c:135: f[0][0].val=0;    // [0][0][0]
	xor	a, a
	ld	(bc),a
;main.c:136: f[0][1].val=1;
	ld	l,-13 (ix)
	ld	h,-12 (ix)
	ld	(hl),#0x01
;main.c:137: f[0][2].val=1;
	pop	hl
	push	hl
	ld	(hl),#0x01
;main.c:138: f[1][0].val=1;
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	(hl),#0x01
;main.c:139: f[1][1].val=1;
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),#0x01
;main.c:140: f[1][2].val=0;
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	(hl),#0x00
;main.c:141: f[2][0].val=0;
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),#0x00
;main.c:142: f[2][1].val=0;
	ld	l,-10 (ix)
	ld	h,-9 (ix)
	ld	(hl),#0x00
;main.c:143: f[2][2].val=0;
	xor	a, a
	ld	(de),a
;main.c:144: break;
	jp	00106$
;main.c:145: case 2://T
00102$:
;main.c:147: f[0][0].val=1;   //[1][1][1]    
	ld	a,#0x01
	ld	(bc),a
;main.c:148: f[0][1].val=1;   //[0][1][0]
	ld	l,-13 (ix)
	ld	h,-12 (ix)
	ld	(hl),#0x01
;main.c:149: f[0][2].val=1;   //[0][0][0]
	pop	hl
	push	hl
	ld	(hl),#0x01
;main.c:150: f[1][0].val=0;
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	(hl),#0x00
;main.c:151: f[1][1].val=1;
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),#0x01
;main.c:152: f[1][2].val=0;
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	(hl),#0x00
;main.c:153: f[2][0].val=0;
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),#0x00
;main.c:154: f[2][1].val=0;
	ld	l,-10 (ix)
	ld	h,-9 (ix)
	ld	(hl),#0x00
;main.c:155: f[2][2].val=0;
	xor	a, a
	ld	(de),a
;main.c:156: break;
	jp	00106$
;main.c:157: case 3://L
00103$:
;main.c:159: f[0][0].val=0;   //[0][0][1]
	xor	a, a
	ld	(bc),a
;main.c:160: f[0][1].val=0;   //[1][1][1]
	ld	l,-13 (ix)
	ld	h,-12 (ix)
	ld	(hl),#0x00
;main.c:161: f[0][2].val=1;   //[0][0][0]
	pop	hl
	push	hl
	ld	(hl),#0x01
;main.c:162: f[1][0].val=1;
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	(hl),#0x01
;main.c:163: f[1][1].val=1;
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),#0x01
;main.c:164: f[1][2].val=1;
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	(hl),#0x01
;main.c:165: f[2][0].val=0;
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),#0x00
;main.c:166: f[2][1].val=0;
	ld	l,-10 (ix)
	ld	h,-9 (ix)
	ld	(hl),#0x00
;main.c:167: f[2][2].val=0;
	xor	a, a
	ld	(de),a
;main.c:168: break;
	jr	00106$
;main.c:169: case 4://Cubo
00104$:
;main.c:170: f[0][0].val=1;   //[1][1][0]
	ld	a,#0x01
	ld	(bc),a
;main.c:171: f[0][1].val=1;   //[]1[1][0]
	ld	l,-13 (ix)
	ld	h,-12 (ix)
	ld	(hl),#0x01
;main.c:172: f[0][2].val=0;   //[0][0][0]
	pop	hl
	push	hl
	ld	(hl),#0x00
;main.c:173: f[1][0].val=1;
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	(hl),#0x01
;main.c:174: f[1][1].val=1;
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),#0x01
;main.c:175: f[1][2].val=0;
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	(hl),#0x00
;main.c:176: f[2][0].val=0;
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),#0x00
;main.c:177: f[2][1].val=0;
	ld	l,-10 (ix)
	ld	h,-9 (ix)
	ld	(hl),#0x00
;main.c:178: f[2][2].val=0;
	xor	a, a
	ld	(de),a
;main.c:179: break;
	jr	00106$
;main.c:180: case 5:// linea
00105$:
;main.c:181: f[0][0].val=0;   //[0][1][0]
	xor	a, a
	ld	(bc),a
;main.c:182: f[0][1].val=1;   //[0][1][0]
	ld	l,-13 (ix)
	ld	h,-12 (ix)
	ld	(hl),#0x01
;main.c:183: f[0][2].val=0;   //[0][1][0]
	pop	hl
	push	hl
	ld	(hl),#0x00
;main.c:184: f[1][0].val=0;
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	(hl),#0x00
;main.c:185: f[1][1].val=1;
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),#0x01
;main.c:186: f[1][2].val=0;
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	(hl),#0x00
;main.c:187: f[2][0].val=0;
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	ld	(hl),#0x00
;main.c:188: f[2][1].val=1;
	ld	l,-10 (ix)
	ld	h,-9 (ix)
	ld	(hl),#0x01
;main.c:189: f[2][2].val=0;
	xor	a, a
	ld	(de),a
;main.c:191: }
00106$:
;main.c:192: cont++;
	ld	iy,#_cont
	inc	0 (iy)
	jr	NZ,00125$
	ld	iy,#_cont
	inc	1 (iy)
00125$:
;main.c:193: if(cont>5)
	ld	a,#0x05
	ld	iy,#_cont
	cp	a, 0 (iy)
	ld	a,#0x00
	ld	iy,#_cont
	sbc	a, 1 (iy)
	jp	PO, 00126$
	xor	a, #0x80
00126$:
	jp	P,00109$
;main.c:194: cont=1;
	ld	hl,#0x0001
	ld	(_cont),hl
00109$:
	ld	sp, ix
	pop	ix
	ret
;main.c:197: void dibujaCara(int index)
;	---------------------------------
; Function dibujaCara
; ---------------------------------
_dibujaCara::
;main.c:201: for(i=1;i<9;i++)
	ld	d,#0x01
;main.c:203: for(j=1;j<9;j++)
00113$:
	ld	e,#0x01
00107$:
;main.c:205: max7219DrawPixel(i,j,0);
	push	de
	xor	a, a
	push	af
	inc	sp
	ld	a,e
	push	af
	inc	sp
	push	de
	inc	sp
	call	_max7219DrawPixel
	pop	af
	inc	sp
	pop	de
;main.c:203: for(j=1;j<9;j++)
	inc	e
	ld	a,e
	sub	a, #0x09
	jr	C,00107$
;main.c:201: for(i=1;i<9;i++)
	inc	d
	ld	a,d
	sub	a, #0x09
	jr	C,00113$
;main.c:209: switch(index)
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	sub	a, #0x01
	ld	a,1 (iy)
	rla
	ccf
	rra
	sbc	a, #0x80
	ret	C
	ld	a,#0x03
	cp	a, 0 (iy)
	ld	a,#0x00
	sbc	a, 1 (iy)
	jp	PO, 00139$
	xor	a, #0x80
00139$:
	ret	M
	ld	iy,#2
	add	iy,sp
	ld	a,0 (iy)
	add	a,#0xFF
	ld	e,a
	ld	d,#0x00
	ld	hl,#00140$
	add	hl,de
	add	hl,de
	add	hl,de
	jp	(hl)
00140$:
	jp	00103$
	jp	00104$
	jp	00105$
;main.c:211: case 1: //:)
00103$:
;main.c:212: max7219DrawPixel(3,1,1);
	ld	hl,#0x0101
	push	hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:213: max7219DrawPixel(2,2,1);
	inc	sp
	ld	hl,#0x0102
	ex	(sp),hl
	ld	a,#0x02
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:214: max7219DrawPixel(3,3,1);
	inc	sp
	ld	hl,#0x0103
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:215: max7219DrawPixel(3,6,1);
	inc	sp
	ld	hl,#0x0106
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:216: max7219DrawPixel(2,7,1);
	inc	sp
	ld	hl,#0x0107
	ex	(sp),hl
	ld	a,#0x02
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:217: max7219DrawPixel(3,8,1);
	inc	sp
	ld	hl,#0x0108
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:218: max7219DrawPixel(6,3,1);
	inc	sp
	ld	hl,#0x0103
	ex	(sp),hl
	ld	a,#0x06
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:219: max7219DrawPixel(7,4,1);
	inc	sp
	ld	hl,#0x0104
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:220: max7219DrawPixel(7,5,1);
	inc	sp
	ld	hl,#0x0105
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:221: max7219DrawPixel(6,6,1);
	inc	sp
	ld	hl,#0x0106
	ex	(sp),hl
	ld	a,#0x06
	push	af
	inc	sp
	call	_max7219DrawPixel
	pop	af
	inc	sp
;main.c:222: break;
	ret
;main.c:223: case 2: // O_O
00104$:
;main.c:224: max7219DrawPixel(2,2,1);
	ld	hl,#0x0102
	push	hl
	ld	a,#0x02
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:225: max7219DrawPixel(3,1,1);
	inc	sp
	ld	hl,#0x0101
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:226: max7219DrawPixel(3,3,1);
	inc	sp
	ld	hl,#0x0103
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:227: max7219DrawPixel(4,2,1);
	inc	sp
	ld	hl,#0x0102
	ex	(sp),hl
	ld	a,#0x04
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:229: max7219DrawPixel(2,7,1);
	inc	sp
	ld	hl,#0x0107
	ex	(sp),hl
	ld	a,#0x02
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:230: max7219DrawPixel(3,6,1);
	inc	sp
	ld	hl,#0x0106
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:231: max7219DrawPixel(3,8,1);
	inc	sp
	ld	hl,#0x0108
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:232: max7219DrawPixel(4,7,1);
	inc	sp
	ld	hl,#0x0107
	ex	(sp),hl
	ld	a,#0x04
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:234: max7219DrawPixel(7,3,1);
	inc	sp
	ld	hl,#0x0103
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:235: max7219DrawPixel(7,4,1);
	inc	sp
	ld	hl,#0x0104
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:236: max7219DrawPixel(7,5,1);
	inc	sp
	ld	hl,#0x0105
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:237: max7219DrawPixel(7,6,1);
	inc	sp
	ld	hl,#0x0106
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
	pop	af
	inc	sp
;main.c:238: break;
	ret
;main.c:239: case 3: // -_-
00105$:
;main.c:240: max7219DrawPixel(3,1,1);
	ld	hl,#0x0101
	push	hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:241: max7219DrawPixel(3,2,1);
	inc	sp
	ld	hl,#0x0102
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:242: max7219DrawPixel(3,3,1);
	inc	sp
	ld	hl,#0x0103
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:244: max7219DrawPixel(3,6,1);
	inc	sp
	ld	hl,#0x0106
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:245: max7219DrawPixel(3,7,1);
	inc	sp
	ld	hl,#0x0107
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:246: max7219DrawPixel(3,8,1);
	inc	sp
	ld	hl,#0x0108
	ex	(sp),hl
	ld	a,#0x03
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:248: max7219DrawPixel(7,3,1);
	inc	sp
	ld	hl,#0x0103
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:249: max7219DrawPixel(7,4,1);
	inc	sp
	ld	hl,#0x0104
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:250: max7219DrawPixel(7,5,1);
	inc	sp
	ld	hl,#0x0105
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
;main.c:251: max7219DrawPixel(7,6,1);
	inc	sp
	ld	hl,#0x0106
	ex	(sp),hl
	ld	a,#0x07
	push	af
	inc	sp
	call	_max7219DrawPixel
	pop	af
	inc	sp
;main.c:253: }
	ret
;main.c:265: void giraDerecha(pixel f[3][3])
;	---------------------------------
; Function giraDerecha
; ---------------------------------
_giraDerecha::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-45
	add	hl,sp
	ld	sp,hl
;main.c:269: borraFigura(f);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_borraFigura
	pop	af
;main.c:270: for(i=0;i<3;i++)
	ld	hl,#0x0002
	add	hl,sp
	ld	-6 (ix),l
	ld	-5 (ix),h
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-2 (ix),#0x00
	ld	-1 (ix),#0x00
	ld	-4 (ix),#0x00
	ld	-3 (ix),#0x00
;main.c:272: for(j=0;j<3;j++)
00123$:
	ld	a,-6 (ix)
	add	a, -2 (ix)
	ld	-8 (ix),a
	ld	a,-5 (ix)
	adc	a, -1 (ix)
	ld	-7 (ix),a
	ld	a,4 (ix)
	add	a, -2 (ix)
	ld	-10 (ix),a
	ld	a,5 (ix)
	adc	a, -1 (ix)
	ld	-9 (ix),a
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	-12 (ix),#0x00
	ld	-11 (ix),#0x00
00109$:
;main.c:274: _nueva[i][j].x=f[i][j].x; // Pasa las coordenadas acutuales de la figura en ese pixel
	ld	a,-8 (ix)
	add	a, -12 (ix)
	ld	e,a
	ld	a,-7 (ix)
	adc	a, -11 (ix)
	ld	d,a
	ld	a,-10 (ix)
	add	a, -12 (ix)
	ld	c,a
	ld	a,-9 (ix)
	adc	a, -11 (ix)
	ld	b,a
	ld	a,(bc)
	ld	(de),a
;main.c:275: _nueva[i][j].y=f[i][j].y; // Pasa las coordenadas acutuales de la figura en ese pixel
	push	de
	pop	iy
	inc	iy
	ld	l, c
	ld	h, b
	inc	hl
	ld	a,(hl)
	ld	0 (iy), a
;main.c:276: _nueva[i][j].val=f[2-j][i].val;// Asigna el valor que corresponde al rotar la matriz
	inc	de
	inc	de
	ld	a,#0x02
	sub	a, -16 (ix)
	ld	l,a
	ld	a,#0x00
	sbc	a, -15 (ix)
	ld	h,a
	ld	c, l
	ld	b, h
	add	hl, hl
	add	hl, hl
	add	hl, hl
	add	hl, bc
	ld	c,l
	ld	b,h
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	ld	c,-4 (ix)
	ld	b,-3 (ix)
	add	hl,bc
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	(de),a
;main.c:272: for(j=0;j<3;j++)
	ld	a,-12 (ix)
	add	a, #0x03
	ld	-12 (ix),a
	ld	a,-11 (ix)
	adc	a, #0x00
	ld	-11 (ix),a
	inc	-16 (ix)
	jr	NZ,00175$
	inc	-15 (ix)
00175$:
	ld	a,-16 (ix)
	sub	a, #0x03
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00109$
;main.c:270: for(i=0;i<3;i++)
	ld	a,-2 (ix)
	add	a, #0x09
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a, #0x00
	ld	-1 (ix),a
	ld	a,-4 (ix)
	add	a, #0x03
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	inc	-45 (ix)
	jr	NZ,00176$
	inc	-44 (ix)
00176$:
	ld	a,-45 (ix)
	sub	a, #0x03
	ld	a,-44 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00123$
;main.c:280: for(i=0;i<3;i++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-12 (ix),#0x00
	ld	-11 (ix),#0x00
;main.c:282: for(j=0;j<3;j++)
00128$:
	ld	a,-6 (ix)
	add	a, -12 (ix)
	ld	-10 (ix),a
	ld	a,-5 (ix)
	adc	a, -11 (ix)
	ld	-9 (ix),a
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	-8 (ix),#0x00
	ld	-7 (ix),#0x00
00113$:
;main.c:284: if(max7219ReadPixel(_nueva[i][j].x,_nueva[i][j].y) != 0) // el bit esta ocupado en la malla
	ld	a,-10 (ix)
	add	a, -8 (ix)
	ld	e,a
	ld	a,-9 (ix)
	adc	a, -7 (ix)
	ld	d,a
	ld	l, e
	ld	h, d
	inc	hl
	ld	b,(hl)
	ld	a,(de)
	push	bc
	inc	sp
	push	af
	inc	sp
	call	_max7219ReadPixel
	pop	af
	ld	c,l
	ld	a,c
	or	a, a
;main.c:286: return;
	jp	NZ,00121$
;main.c:282: for(j=0;j<3;j++)
	ld	a,-8 (ix)
	add	a, #0x03
	ld	-8 (ix),a
	ld	a,-7 (ix)
	adc	a, #0x00
	ld	-7 (ix),a
	inc	-16 (ix)
	jr	NZ,00177$
	inc	-15 (ix)
00177$:
	ld	a,-16 (ix)
	sub	a, #0x03
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00113$
;main.c:280: for(i=0;i<3;i++)
	ld	a,-12 (ix)
	add	a, #0x09
	ld	-12 (ix),a
	ld	a,-11 (ix)
	adc	a, #0x00
	ld	-11 (ix),a
	inc	-45 (ix)
	jr	NZ,00178$
	inc	-44 (ix)
00178$:
	ld	a,-45 (ix)
	sub	a, #0x03
	ld	a,-44 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00128$
;main.c:291: for(i=0;i<3;i++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-12 (ix),#0x00
	ld	-11 (ix),#0x00
;main.c:293: for(j=0;j<3;j++)
00132$:
	ld	a,4 (ix)
	add	a, -12 (ix)
	ld	-10 (ix),a
	ld	a,5 (ix)
	adc	a, -11 (ix)
	ld	-9 (ix),a
	ld	a,-6 (ix)
	add	a, -12 (ix)
	ld	-8 (ix),a
	ld	a,-5 (ix)
	adc	a, -11 (ix)
	ld	-7 (ix),a
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	-4 (ix),#0x00
	ld	-3 (ix),#0x00
00117$:
;main.c:295: f[i][j].x = _nueva[i][j].x;
	ld	a,-10 (ix)
	add	a, -4 (ix)
	ld	-2 (ix),a
	ld	a,-9 (ix)
	adc	a, -3 (ix)
	ld	-1 (ix),a
	ld	a,-8 (ix)
	add	a, -4 (ix)
	ld	-14 (ix),a
	ld	a,-7 (ix)
	adc	a, -3 (ix)
	ld	-13 (ix),a
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	a,(hl)
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	(hl),a
;main.c:296: f[i][j].y = _nueva[i][j].y;
	ld	e,-2 (ix)
	ld	d,-1 (ix)
	inc	de
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	inc	hl
	ld	a,(hl)
	ld	(de),a
;main.c:297: f[i][j].val = _nueva[i][j].val;
	ld	e,-2 (ix)
	ld	d,-1 (ix)
	inc	de
	inc	de
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	(de),a
;main.c:293: for(j=0;j<3;j++)
	ld	a,-4 (ix)
	add	a, #0x03
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	inc	-16 (ix)
	jr	NZ,00179$
	inc	-15 (ix)
00179$:
	ld	a,-16 (ix)
	sub	a, #0x03
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00117$
;main.c:291: for(i=0;i<3;i++)
	ld	a,-12 (ix)
	add	a, #0x09
	ld	-12 (ix),a
	ld	a,-11 (ix)
	adc	a, #0x00
	ld	-11 (ix),a
	inc	-45 (ix)
	jr	NZ,00180$
	inc	-44 (ix)
00180$:
	ld	a,-45 (ix)
	sub	a, #0x03
	ld	a,-44 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00132$
00121$:
	ld	sp, ix
	pop	ix
	ret
;main.c:312: void giraIzquierda(pixel f[3][3])
;	---------------------------------
; Function giraIzquierda
; ---------------------------------
_giraIzquierda::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-45
	add	hl,sp
	ld	sp,hl
;main.c:316: borraFigura(f);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_borraFigura
	pop	af
;main.c:317: for(i=0;i<3;i++)
	ld	hl,#0x0002
	add	hl,sp
	ld	-12 (ix),l
	ld	-11 (ix),h
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-14 (ix),#0x00
	ld	-13 (ix),#0x00
;main.c:319: for(j=0;j<3;j++)
00123$:
	ld	a,-12 (ix)
	add	a, -14 (ix)
	ld	-8 (ix),a
	ld	a,-11 (ix)
	adc	a, -13 (ix)
	ld	-7 (ix),a
	ld	a,4 (ix)
	add	a, -14 (ix)
	ld	-10 (ix),a
	ld	a,5 (ix)
	adc	a, -13 (ix)
	ld	-9 (ix),a
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	-2 (ix),#0x00
	ld	-1 (ix),#0x00
	ld	-4 (ix),#0x00
	ld	-3 (ix),#0x00
00109$:
;main.c:321: _nueva[i][j].x=f[i][j].x;
	ld	a,-8 (ix)
	add	a, -2 (ix)
	ld	e,a
	ld	a,-7 (ix)
	adc	a, -1 (ix)
	ld	d,a
	ld	a,-10 (ix)
	add	a, -2 (ix)
	ld	c,a
	ld	a,-9 (ix)
	adc	a, -1 (ix)
	ld	b,a
	ld	a,(bc)
	ld	(de),a
;main.c:322: _nueva[i][j].y=f[i][j].y;
	push	de
	pop	iy
	inc	iy
	ld	l, c
	ld	h, b
	inc	hl
	ld	a,(hl)
	ld	0 (iy), a
;main.c:323: _nueva[i][j].val=f[j][2-i].val;
	inc	de
	inc	de
	ld	a,4 (ix)
	add	a, -4 (ix)
	ld	c,a
	ld	a,5 (ix)
	adc	a, -3 (ix)
	ld	b,a
	ld	h,-45 (ix)
	ld	a,#0x02
	sub	a, h
	push	de
	ld	e,a
	add	a, a
	add	a, e
	pop	de
	ld	l, a
	ld	h,#0x00
	add	hl,bc
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	(de),a
;main.c:319: for(j=0;j<3;j++)
	ld	a,-2 (ix)
	add	a, #0x03
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a, #0x00
	ld	-1 (ix),a
	ld	a,-4 (ix)
	add	a, #0x09
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	inc	-16 (ix)
	jr	NZ,00175$
	inc	-15 (ix)
00175$:
	ld	a,-16 (ix)
	sub	a, #0x03
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00109$
;main.c:317: for(i=0;i<3;i++)
	ld	a,-14 (ix)
	add	a, #0x09
	ld	-14 (ix),a
	ld	a,-13 (ix)
	adc	a, #0x00
	ld	-13 (ix),a
	inc	-45 (ix)
	jr	NZ,00176$
	inc	-44 (ix)
00176$:
	ld	a,-45 (ix)
	sub	a, #0x03
	ld	a,-44 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00123$
;main.c:326: for(i=0;i<3;i++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-4 (ix),#0x00
	ld	-3 (ix),#0x00
;main.c:328: for(j=0;j<3;j++)
00128$:
	ld	a,-12 (ix)
	add	a, -4 (ix)
	ld	-2 (ix),a
	ld	a,-11 (ix)
	adc	a, -3 (ix)
	ld	-1 (ix),a
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	-10 (ix),#0x00
	ld	-9 (ix),#0x00
00113$:
;main.c:330: if(max7219ReadPixel(_nueva[i][j].x,_nueva[i][j].y) != 0) // el bit esta ocupado en la malla
	ld	a,-2 (ix)
	add	a, -10 (ix)
	ld	e,a
	ld	a,-1 (ix)
	adc	a, -9 (ix)
	ld	d,a
	ld	l, e
	ld	h, d
	inc	hl
	ld	b,(hl)
	ld	a,(de)
	push	bc
	inc	sp
	push	af
	inc	sp
	call	_max7219ReadPixel
	pop	af
	ld	c,l
	ld	a,c
	or	a, a
;main.c:332: return;
	jp	NZ,00121$
;main.c:328: for(j=0;j<3;j++)
	ld	a,-10 (ix)
	add	a, #0x03
	ld	-10 (ix),a
	ld	a,-9 (ix)
	adc	a, #0x00
	ld	-9 (ix),a
	inc	-16 (ix)
	jr	NZ,00177$
	inc	-15 (ix)
00177$:
	ld	a,-16 (ix)
	sub	a, #0x03
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00113$
;main.c:326: for(i=0;i<3;i++)
	ld	a,-4 (ix)
	add	a, #0x09
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	inc	-45 (ix)
	jr	NZ,00178$
	inc	-44 (ix)
00178$:
	ld	a,-45 (ix)
	sub	a, #0x03
	ld	a,-44 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00128$
;main.c:336: for(i=0;i<3;i++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-4 (ix),#0x00
	ld	-3 (ix),#0x00
;main.c:338: for(j=0;j<3;j++)
00132$:
	ld	a,4 (ix)
	add	a, -4 (ix)
	ld	-2 (ix),a
	ld	a,5 (ix)
	adc	a, -3 (ix)
	ld	-1 (ix),a
	ld	a,-12 (ix)
	add	a, -4 (ix)
	ld	-10 (ix),a
	ld	a,-11 (ix)
	adc	a, -3 (ix)
	ld	-9 (ix),a
	ld	-16 (ix),#0x00
	ld	-15 (ix),#0x00
	ld	-8 (ix),#0x00
	ld	-7 (ix),#0x00
00117$:
;main.c:340: f[i][j].x = _nueva[i][j].x;
	ld	a,-2 (ix)
	add	a, -8 (ix)
	ld	-14 (ix),a
	ld	a,-1 (ix)
	adc	a, -7 (ix)
	ld	-13 (ix),a
	ld	a,-10 (ix)
	add	a, -8 (ix)
	ld	-6 (ix),a
	ld	a,-9 (ix)
	adc	a, -7 (ix)
	ld	-5 (ix),a
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	a,(hl)
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	ld	(hl),a
;main.c:341: f[i][j].y = _nueva[i][j].y;
	ld	e,-14 (ix)
	ld	d,-13 (ix)
	inc	de
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	inc	hl
	ld	a,(hl)
	ld	(de),a
;main.c:342: f[i][j].val = _nueva[i][j].val;
	ld	e,-14 (ix)
	ld	d,-13 (ix)
	inc	de
	inc	de
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	(de),a
;main.c:338: for(j=0;j<3;j++)
	ld	a,-8 (ix)
	add	a, #0x03
	ld	-8 (ix),a
	ld	a,-7 (ix)
	adc	a, #0x00
	ld	-7 (ix),a
	inc	-16 (ix)
	jr	NZ,00179$
	inc	-15 (ix)
00179$:
	ld	a,-16 (ix)
	sub	a, #0x03
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00117$
;main.c:336: for(i=0;i<3;i++)
	ld	a,-4 (ix)
	add	a, #0x09
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	inc	-45 (ix)
	jr	NZ,00180$
	inc	-44 (ix)
00180$:
	ld	a,-45 (ix)
	sub	a, #0x03
	ld	a,-44 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00132$
00121$:
	ld	sp, ix
	pop	ix
	ret
;main.c:403: void mueveDerecha(pixel f[3][3])
;	---------------------------------
; Function mueveDerecha
; ---------------------------------
_mueveDerecha::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-21
	add	hl,sp
	ld	sp,hl
;main.c:406: borraFigura(f);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_borraFigura
	pop	af
;main.c:409: for(i=0;i<3;i++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-7 (ix),#0x00
	ld	-6 (ix),#0x00
;main.c:411: for(j=0;j<3;j++)
00143$:
	ld	a,4 (ix)
	add	a, -7 (ix)
	ld	-9 (ix),a
	ld	a,5 (ix)
	adc	a, -6 (ix)
	ld	-8 (ix),a
	ld	-19 (ix),#0x00
	ld	-18 (ix),#0x00
	ld	-11 (ix),#0x00
	ld	-10 (ix),#0x00
00123$:
;main.c:413: if(f[i][j].val==1)
	ld	a,-9 (ix)
	add	a, -11 (ix)
	ld	-13 (ix),a
	ld	a,-8 (ix)
	adc	a, -10 (ix)
	ld	-12 (ix),a
	ld	l,-13 (ix)
	ld	h,-12 (ix)
	inc	hl
	inc	hl
	ld	a,(hl)
	dec	a
	jr	NZ,00124$
;main.c:415: if(f[i][j].y+1 >= 9) // se sale de la malla
	ld	l,-13 (ix)
	ld	h,-12 (ix)
	inc	hl
	ld	e,(hl)
	ld	d,#0x00
	inc	de
	ld	a, e
	sub	a, #0x09
	ld	a,d
	rla
	ccf
	rra
	sbc	a, #0x80
;main.c:417: return; // no puede moverse la figura a la derecha
	jp	NC,00139$
00124$:
;main.c:411: for(j=0;j<3;j++)
	ld	a,-11 (ix)
	add	a, #0x03
	ld	-11 (ix),a
	ld	a,-10 (ix)
	adc	a, #0x00
	ld	-10 (ix),a
	inc	-19 (ix)
	jr	NZ,00227$
	inc	-18 (ix)
00227$:
	ld	a,-19 (ix)
	sub	a, #0x03
	ld	a,-18 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00123$
;main.c:409: for(i=0;i<3;i++)
	ld	a,-7 (ix)
	add	a, #0x09
	ld	-7 (ix),a
	ld	a,-6 (ix)
	adc	a, #0x00
	ld	-6 (ix),a
	inc	-21 (ix)
	jr	NZ,00228$
	inc	-20 (ix)
00228$:
	ld	a,-21 (ix)
	sub	a, #0x03
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00143$
;main.c:422: for(i=0;i<3;i++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	de,#0x0000
;main.c:424: for(j=2;j>=0;j--)
00149$:
	ld	a,4 (ix)
	add	a, e
	ld	-13 (ix),a
	ld	a,5 (ix)
	adc	a, d
	ld	-12 (ix),a
	ld	bc,#0x0002
	ld	-11 (ix),#0x06
	ld	-10 (ix),#0x00
00127$:
;main.c:426: if(f[i][j].val==1)
	ld	a,-13 (ix)
	add	a, -11 (ix)
	ld	-9 (ix),a
	ld	a,-12 (ix)
	adc	a, -10 (ix)
	ld	-8 (ix),a
	ld	l,-9 (ix)
	ld	h,-8 (ix)
	inc	hl
	inc	hl
	ld	a,(hl)
	dec	a
	jr	NZ,00128$
;main.c:428: if(max7219ReadPixel(f[i][j].x,f[i][j].y+1) != 0) // el bit esta ocupado en la malla
	ld	l,-9 (ix)
	ld	h,-8 (ix)
	inc	hl
	ld	a,(hl)
	inc	a
	ld	l,-9 (ix)
	ld	h,-8 (ix)
	ld	b,(hl)
	push	de
	push	af
	inc	sp
	push	bc
	inc	sp
	call	_max7219ReadPixel
	pop	af
	ld	a,l
	pop	de
	or	a, a
	jr	Z,00130$
;main.c:430: return;
	jp	00139$
;main.c:433: break;
00128$:
;main.c:424: for(j=2;j>=0;j--)
	ld	a,-11 (ix)
	add	a,#0xFD
	ld	-11 (ix),a
	ld	a,-10 (ix)
	adc	a,#0xFF
	ld	-10 (ix),a
	dec	bc
	bit	7, b
	jr	Z,00127$
00130$:
;main.c:422: for(i=0;i<3;i++)
	ld	hl,#0x0009
	add	hl,de
	ex	de,hl
	inc	-21 (ix)
	jr	NZ,00231$
	inc	-20 (ix)
00231$:
	ld	a,-21 (ix)
	sub	a, #0x03
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00149$
;main.c:441: for(i=0;i<3;i++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-13 (ix),#0x00
	ld	-12 (ix),#0x00
;main.c:443: for(j=0;j<3;j++)
00152$:
	ld	a,4 (ix)
	add	a, -13 (ix)
	ld	c,a
	ld	a,5 (ix)
	adc	a, -12 (ix)
	ld	b,a
	ld	de,#0x0000
	ld	-11 (ix),#0x00
	ld	-10 (ix),#0x00
00131$:
;main.c:445: f[i][j].y=f[i][j].y+1;
	ld	a,c
	add	a, -11 (ix)
	ld	-9 (ix),a
	ld	a,b
	adc	a, -10 (ix)
	ld	-8 (ix),a
	inc	-9 (ix)
	jr	NZ,00232$
	inc	-8 (ix)
00232$:
	ld	l,-9 (ix)
	ld	h,-8 (ix)
	inc	(hl)
;main.c:443: for(j=0;j<3;j++)
	ld	a,-11 (ix)
	add	a, #0x03
	ld	-11 (ix),a
	ld	a,-10 (ix)
	adc	a, #0x00
	ld	-10 (ix),a
	inc	de
	ld	a,e
	sub	a, #0x03
	ld	a,d
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00131$
;main.c:441: for(i=0;i<3;i++)
	ld	a,-13 (ix)
	add	a, #0x09
	ld	-13 (ix),a
	ld	a,-12 (ix)
	adc	a, #0x00
	ld	-12 (ix),a
	inc	-21 (ix)
	jr	NZ,00233$
	inc	-20 (ix)
00233$:
	ld	a,-21 (ix)
	sub	a, #0x03
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00152$
;main.c:449: if(f[0][2].y > 8)
	ld	a,4 (ix)
	ld	-13 (ix),a
	ld	a,5 (ix)
	ld	-12 (ix),a
	ld	l,-13 (ix)
	ld	h,-12 (ix)
	ld	de, #0x0007
	add	hl, de
	ld	h,(hl)
	ld	a,#0x08
	sub	a, h
	jp	NC,00139$
;main.c:452: for(i=0;i<3;i++)
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-11 (ix),#0x00
	ld	-10 (ix),#0x00
	ld	-9 (ix),#0x00
	ld	-8 (ix),#0x00
;main.c:454: for(j=2;j>=0;j--)
00158$:
	ld	a,-11 (ix)
	add	a, -13 (ix)
	ld	-7 (ix),a
	ld	a,-10 (ix)
	adc	a, -12 (ix)
	ld	-6 (ix),a
	ld	a,-9 (ix)
	add	a, -13 (ix)
	ld	-15 (ix),a
	ld	a,-8 (ix)
	adc	a, -12 (ix)
	ld	-14 (ix),a
	ld	-19 (ix),#0x02
	ld	-18 (ix),#0x00
	ld	-17 (ix),#0x06
	ld	-16 (ix),#0x00
	ld	-2 (ix),#0x06
	ld	-1 (ix),#0x00
00135$:
;main.c:456: if(j==0)
	ld	a,-18 (ix)
	or	a,-19 (ix)
	jr	NZ,00117$
;main.c:458: f[i][j].x=f[i][j].x;
	ld	a,-15 (ix)
	add	a, -2 (ix)
	ld	e,a
	ld	a,-14 (ix)
	adc	a, -1 (ix)
	ld	d,a
	ld	a,(de)
	ld	(de),a
;main.c:459: f[i][j].y=f[i][j].y-1;
	ld	l, e
	ld	h, d
	inc	hl
	ld	b,(hl)
	dec	b
	ld	(hl),b
;main.c:460: f[i][j].val=0;  
	inc	de
	inc	de
	xor	a, a
	ld	(de),a
	jr	00136$
00117$:
;main.c:463: f[i][j].x=f[i][j-1].x;
	ld	a,-7 (ix)
	add	a, -17 (ix)
	ld	-4 (ix),a
	ld	a,-6 (ix)
	adc	a, -16 (ix)
	ld	-3 (ix),a
	ld	a,-19 (ix)
	ld	-5 (ix),a
	dec	-5 (ix)
	ld	l,-5 (ix)
	ld	c,l
	add	hl, hl
	add	hl, bc
	ld	a,-7 (ix)
	add	a, l
	ld	e,a
	ld	a,-6 (ix)
	adc	a, #0x00
	ld	d,a
	ld	a,(de)
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),a
;main.c:464: f[i][j].y=f[i][j-1].y;
	ld	c,-4 (ix)
	ld	b,-3 (ix)
	inc	bc
	ld	l, e
	ld	h, d
	inc	hl
	ld	a,(hl)
	ld	(bc),a
;main.c:465: f[i][j].val=f[i][j-1].val;
	ld	a,-4 (ix)
	add	a, #0x02
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	(hl),a
00136$:
;main.c:454: for(j=2;j>=0;j--)
	ld	a,-17 (ix)
	add	a,#0xFD
	ld	-17 (ix),a
	ld	a,-16 (ix)
	adc	a,#0xFF
	ld	-16 (ix),a
	ld	a,-2 (ix)
	add	a,#0xFD
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a,#0xFF
	ld	-1 (ix),a
	ld	l,-19 (ix)
	ld	h,-18 (ix)
	dec	hl
	ld	-19 (ix),l
	ld	-18 (ix),h
	bit	7, -18 (ix)
	jp	Z,00135$
;main.c:452: for(i=0;i<3;i++)
	ld	a,-11 (ix)
	add	a, #0x09
	ld	-11 (ix),a
	ld	a,-10 (ix)
	adc	a, #0x00
	ld	-10 (ix),a
	ld	a,-9 (ix)
	add	a, #0x09
	ld	-9 (ix),a
	ld	a,-8 (ix)
	adc	a, #0x00
	ld	-8 (ix),a
	inc	-21 (ix)
	jr	NZ,00234$
	inc	-20 (ix)
00234$:
	ld	a,-21 (ix)
	sub	a, #0x03
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00158$
00139$:
	ld	sp, ix
	pop	ix
	ret
;main.c:528: void mueveIzquierda(pixel f[3][3])
;	---------------------------------
; Function mueveIzquierda
; ---------------------------------
_mueveIzquierda::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-21
	add	hl,sp
	ld	sp,hl
;main.c:531: borraFigura(f);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_borraFigura
	pop	af
;main.c:534: for(i=0;i<3;i++)
	ld	-19 (ix),#0x00
	ld	-18 (ix),#0x00
	ld	-4 (ix),#0x00
	ld	-3 (ix),#0x00
;main.c:536: for(j=0;j<3;j++)
00143$:
	ld	a,4 (ix)
	add	a, -4 (ix)
	ld	-2 (ix),a
	ld	a,5 (ix)
	adc	a, -3 (ix)
	ld	-1 (ix),a
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-6 (ix),#0x00
	ld	-5 (ix),#0x00
00123$:
;main.c:538: if(f[i][j].val==1)
	ld	a,-2 (ix)
	add	a, -6 (ix)
	ld	-8 (ix),a
	ld	a,-1 (ix)
	adc	a, -5 (ix)
	ld	-7 (ix),a
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	inc	hl
	inc	hl
	ld	a,(hl)
	dec	a
	jr	NZ,00124$
;main.c:540: if(f[i][j].y-1 <= 0) // se sale de la malla
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	inc	hl
	ld	l,(hl)
	ld	h,#0x00
	dec	hl
	xor	a, a
	cp	a, l
	sbc	a, h
	jp	PO, 00227$
	xor	a, #0x80
00227$:
	jp	M,00124$
;main.c:542: return; // no puede moverse la figura a la Izquierda
	jp	00139$
00124$:
;main.c:536: for(j=0;j<3;j++)
	ld	a,-6 (ix)
	add	a, #0x03
	ld	-6 (ix),a
	ld	a,-5 (ix)
	adc	a, #0x00
	ld	-5 (ix),a
	inc	-21 (ix)
	jr	NZ,00228$
	inc	-20 (ix)
00228$:
	ld	a,-21 (ix)
	sub	a, #0x03
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00123$
;main.c:534: for(i=0;i<3;i++)
	ld	a,-4 (ix)
	add	a, #0x09
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	inc	-19 (ix)
	jr	NZ,00229$
	inc	-18 (ix)
00229$:
	ld	a,-19 (ix)
	sub	a, #0x03
	ld	a,-18 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00143$
;main.c:548: for(i=0;i<3;i++)
	ld	-19 (ix),#0x00
	ld	-18 (ix),#0x00
	ld	de,#0x0000
;main.c:550: for(j=0;j<3;j++)
00149$:
	ld	a,4 (ix)
	add	a, e
	ld	-8 (ix),a
	ld	a,5 (ix)
	adc	a, d
	ld	-7 (ix),a
	ld	hl,#0x0000
	ex	(sp), hl
	ld	bc,#0x0000
00127$:
;main.c:552: if(f[i][j].val==1)
	ld	a,-8 (ix)
	add	a, c
	ld	-6 (ix),a
	ld	a,-7 (ix)
	adc	a, b
	ld	-5 (ix),a
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	inc	hl
	inc	hl
	ld	a,(hl)
	dec	a
	jr	NZ,00128$
;main.c:554: if(max7219ReadPixel(f[i][j].x,f[i][j].y-1) != 0)
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	inc	hl
	ld	c,(hl)
	dec	c
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	h,(hl)
	push	de
	ld	a,c
	push	af
	inc	sp
	push	hl
	inc	sp
	call	_max7219ReadPixel
	pop	af
	ld	a,l
	pop	de
	or	a, a
	jr	Z,00130$
;main.c:556: return;
	jp	00139$
;main.c:559: break;
00128$:
;main.c:550: for(j=0;j<3;j++)
	inc	bc
	inc	bc
	inc	bc
	inc	-21 (ix)
	jr	NZ,00232$
	inc	-20 (ix)
00232$:
	ld	a,-21 (ix)
	sub	a, #0x03
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00127$
00130$:
;main.c:548: for(i=0;i<3;i++)
	ld	hl,#0x0009
	add	hl,de
	ex	de,hl
	inc	-19 (ix)
	jr	NZ,00233$
	inc	-18 (ix)
00233$:
	ld	a,-19 (ix)
	sub	a, #0x03
	ld	a,-18 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00149$
;main.c:566: for(i=0;i<3;i++)
	ld	-19 (ix),#0x00
	ld	-18 (ix),#0x00
	ld	-8 (ix),#0x00
	ld	-7 (ix),#0x00
;main.c:568: for(j=0;j<3;j++)
00152$:
	ld	a,4 (ix)
	add	a, -8 (ix)
	ld	c,a
	ld	a,5 (ix)
	adc	a, -7 (ix)
	ld	b,a
	ld	de,#0x0000
	ld	-6 (ix),#0x00
	ld	-5 (ix),#0x00
00131$:
;main.c:570: f[i][j].y=f[i][j].y-1;
	ld	a,c
	add	a, -6 (ix)
	ld	-2 (ix),a
	ld	a,b
	adc	a, -5 (ix)
	ld	-1 (ix),a
	inc	-2 (ix)
	jr	NZ,00234$
	inc	-1 (ix)
00234$:
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	a,(hl)
	add	a,#0xFF
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	ld	(hl),a
;main.c:568: for(j=0;j<3;j++)
	ld	a,-6 (ix)
	add	a, #0x03
	ld	-6 (ix),a
	ld	a,-5 (ix)
	adc	a, #0x00
	ld	-5 (ix),a
	inc	de
	ld	a,e
	sub	a, #0x03
	ld	a,d
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00131$
;main.c:566: for(i=0;i<3;i++)
	ld	a,-8 (ix)
	add	a, #0x09
	ld	-8 (ix),a
	ld	a,-7 (ix)
	adc	a, #0x00
	ld	-7 (ix),a
	inc	-19 (ix)
	jr	NZ,00235$
	inc	-18 (ix)
00235$:
	ld	a,-19 (ix)
	sub	a, #0x03
	ld	a,-18 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00152$
;main.c:574: if(f[0][0].y < 1)
	ld	a,4 (ix)
	ld	-8 (ix),a
	ld	a,5 (ix)
	ld	-7 (ix),a
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	inc	hl
	ld	a, (hl)
	sub	a, #0x01
	jp	NC,00139$
;main.c:577: for(i=0;i<3;i++)
	ld	-19 (ix),#0x00
	ld	-18 (ix),#0x00
	ld	-6 (ix),#0x00
	ld	-5 (ix),#0x00
	ld	-2 (ix),#0x00
	ld	-1 (ix),#0x00
;main.c:579: for(j=0;j<3;j++)
00158$:
	ld	a,-6 (ix)
	add	a, -8 (ix)
	ld	-4 (ix),a
	ld	a,-5 (ix)
	adc	a, -7 (ix)
	ld	-3 (ix),a
	ld	a,-2 (ix)
	add	a, -8 (ix)
	ld	-10 (ix),a
	ld	a,-1 (ix)
	adc	a, -7 (ix)
	ld	-9 (ix),a
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-12 (ix),#0x00
	ld	-11 (ix),#0x00
	ld	-14 (ix),#0x00
	ld	-13 (ix),#0x00
00135$:
;main.c:581: if(j==2)
	ld	a,-21 (ix)
	sub	a, #0x02
	jr	NZ,00117$
	ld	a,-20 (ix)
	or	a, a
	jr	NZ,00117$
;main.c:583: f[i][j].x=f[i][j].x;
	ld	a,-10 (ix)
	add	a, -14 (ix)
	ld	e,a
	ld	a,-9 (ix)
	adc	a, -13 (ix)
	ld	d,a
	ld	a,(de)
	ld	(de),a
;main.c:584: f[i][j].y=f[i][j].y+1;
	ld	l, e
	ld	h, d
	inc	hl
	inc	(hl)
;main.c:585: f[i][j].val=0;  
	inc	de
	inc	de
	xor	a, a
	ld	(de),a
	jr	00136$
00117$:
;main.c:588: f[i][j].x=f[i][j+1].x;
	ld	a,-4 (ix)
	add	a, -12 (ix)
	ld	-16 (ix),a
	ld	a,-3 (ix)
	adc	a, -11 (ix)
	ld	-15 (ix),a
	ld	a,-21 (ix)
	ld	-17 (ix),a
	inc	-17 (ix)
	ld	l,-17 (ix)
	ld	c,l
	add	hl, hl
	add	hl, bc
	ld	a,-4 (ix)
	add	a, l
	ld	e,a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	d,a
	ld	a,(de)
	ld	l,-16 (ix)
	ld	h,-15 (ix)
	ld	(hl),a
;main.c:589: f[i][j].y=f[i][j+1].y;
	ld	c,-16 (ix)
	ld	b,-15 (ix)
	inc	bc
	ld	l, e
	ld	h, d
	inc	hl
	ld	a,(hl)
	ld	(bc),a
;main.c:590: f[i][j].val=f[i][j+1].val;
	ld	a,-16 (ix)
	add	a, #0x02
	ld	-16 (ix),a
	ld	a,-15 (ix)
	adc	a, #0x00
	ld	-15 (ix),a
	ld	l, e
	ld	h, d
	inc	hl
	inc	hl
	ld	a,(hl)
	ld	l,-16 (ix)
	ld	h,-15 (ix)
	ld	(hl),a
00136$:
;main.c:579: for(j=0;j<3;j++)
	ld	a,-12 (ix)
	add	a, #0x03
	ld	-12 (ix),a
	ld	a,-11 (ix)
	adc	a, #0x00
	ld	-11 (ix),a
	ld	a,-14 (ix)
	add	a, #0x03
	ld	-14 (ix),a
	ld	a,-13 (ix)
	adc	a, #0x00
	ld	-13 (ix),a
	inc	-21 (ix)
	jr	NZ,00238$
	inc	-20 (ix)
00238$:
	ld	a,-21 (ix)
	sub	a, #0x03
	ld	a,-20 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00135$
;main.c:577: for(i=0;i<3;i++)
	ld	a,-6 (ix)
	add	a, #0x09
	ld	-6 (ix),a
	ld	a,-5 (ix)
	adc	a, #0x00
	ld	-5 (ix),a
	ld	a,-2 (ix)
	add	a, #0x09
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a, #0x00
	ld	-1 (ix),a
	inc	-19 (ix)
	jr	NZ,00239$
	inc	-18 (ix)
00239$:
	ld	a,-19 (ix)
	sub	a, #0x03
	ld	a,-18 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00158$
00139$:
	ld	sp, ix
	pop	ix
	ret
;main.c:597: void iniciaJuego(pixel f[3][3])
;	---------------------------------
; Function iniciaJuego
; ---------------------------------
_iniciaJuego::
	push	ix
	ld	ix,#0
	add	ix,sp
	dec	sp
;main.c:602: dibujaCara(2);
	ld	hl,#0x0002
	push	hl
	call	_dibujaCara
;main.c:603: delay_ms(400);
	ld	hl, #0x0190
	ex	(sp),hl
	call	_delay_ms
;main.c:604: dibujaCara(3);
	ld	hl, #0x0003
	ex	(sp),hl
	call	_dibujaCara
;main.c:605: delay_ms(400);
	ld	hl, #0x0190
	ex	(sp),hl
	call	_delay_ms
;main.c:606: dibujaCara(2);
	ld	hl, #0x0002
	ex	(sp),hl
	call	_dibujaCara
;main.c:607: delay_ms(400);
	ld	hl, #0x0190
	ex	(sp),hl
	call	_delay_ms
;main.c:608: dibujaCara(3);
	ld	hl, #0x0003
	ex	(sp),hl
	call	_dibujaCara
;main.c:609: delay_ms(400);
	ld	hl, #0x0190
	ex	(sp),hl
	call	_delay_ms
;main.c:610: dibujaCara(1);
	ld	hl, #0x0001
	ex	(sp),hl
	call	_dibujaCara
;main.c:611: delay_ms(400);
	ld	hl, #0x0190
	ex	(sp),hl
	call	_delay_ms
	pop	af
;main.c:612: for(i=1;i<9;i++)
	ld	d,#0x01
;main.c:614: for(j=1;j<9;j++)
00121$:
	ld	e,#0x01
00107$:
;main.c:616: max7219DrawPixel(i,j,1);
	push	de
	ld	a,#0x01
	push	af
	inc	sp
	ld	a,e
	push	af
	inc	sp
	push	de
	inc	sp
	call	_max7219DrawPixel
	pop	af
	inc	sp
	pop	de
;main.c:614: for(j=1;j<9;j++)
	inc	e
	ld	a,e
	sub	a, #0x09
	jr	C,00107$
;main.c:612: for(i=1;i<9;i++)
	inc	d
	ld	a,d
	sub	a, #0x09
	jr	C,00121$
;main.c:619: for(i=1;i<9;i++)
	ld	d,#0x01
;main.c:621: for(j=1;j<9;j++)
00125$:
	ld	e,#0x01
00111$:
;main.c:623: max7219DrawPixel(i,j,0);
	push	de
	xor	a, a
	push	af
	inc	sp
	ld	a,e
	push	af
	inc	sp
	push	de
	inc	sp
	call	_max7219DrawPixel
	pop	af
	inc	sp
	pop	de
;main.c:621: for(j=1;j<9;j++)
	inc	e
	ld	a,e
	sub	a, #0x09
	jr	C,00111$
;main.c:619: for(i=1;i<9;i++)
	inc	d
	ld	a,d
	sub	a, #0x09
	jr	C,00125$
;main.c:627: for(i=0;i<3;i++)
	ld	-1 (ix),#0x00
	ld	de,#0x0000
;main.c:629: for(j=0;j<3;j++)
00129$:
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,de
	ld	bc,#0x0000
00115$:
;main.c:631: f[i][j].val=0;
	push	hl
	pop	iy
	push	bc
	ld	b,#0x00
	add	iy, bc
	pop	bc
	inc	iy
	inc	iy
	ld	0 (iy), #0x00
;main.c:629: for(j=0;j<3;j++)
	inc	c
	inc	c
	inc	c
	inc	b
	ld	a,b
	sub	a, #0x03
	jr	C,00115$
;main.c:627: for(i=0;i<3;i++)
	ld	hl,#0x0009
	add	hl,de
	ex	de,hl
	inc	-1 (ix)
	ld	a,-1 (ix)
	sub	a, #0x03
	jr	C,00129$
;main.c:634: max7219Clear();
	call	_max7219Clear
;main.c:635: llenaFigura(f,cont);
	ld	hl,(_cont)
	push	hl
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_llenaFigura
	pop	af
	pop	af
;main.c:636: dibujaFigura(f);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_dibujaFigura
	pop	af
	inc	sp
	pop	ix
	ret
;main.c:641: void revizaPerdio(pixel f[3][3])
;	---------------------------------
; Function revizaPerdio
; ---------------------------------
_revizaPerdio::
;main.c:644: for(i=1;i<9;i++)
	ld	h,#0x01
00104$:
;main.c:646: if(max7219ReadPixel(1,i) != 0)
	push	hl
	push	hl
	inc	sp
	ld	a,#0x01
	push	af
	inc	sp
	call	_max7219ReadPixel
	pop	af
	ld	a,l
	pop	hl
	or	a, a
	jr	Z,00105$
;main.c:648: iniciaJuego(f);
	pop	bc
	pop	hl
	push	hl
	push	bc
	push	hl
	call	_iniciaJuego
	pop	af
;main.c:649: break;
	ret
00105$:
;main.c:644: for(i=1;i<9;i++)
	inc	h
	ld	a,h
	sub	a, #0x09
	jr	C,00104$
	ret
;main.c:707: void mueveAbajo(pixel f[3][3])
;	---------------------------------
; Function mueveAbajo
; ---------------------------------
_mueveAbajo::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-16
	add	hl,sp
	ld	sp,hl
;main.c:710: borraFigura(f);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_borraFigura
	pop	af
;main.c:713: for(i=0;i<3;i++)
	ld	-14 (ix),#0x00
	ld	-13 (ix),#0x00
	ld	-2 (ix),#0x00
	ld	-1 (ix),#0x00
;main.c:715: for(j=0;j<3;j++)
00143$:
	ld	a,4 (ix)
	add	a, -2 (ix)
	ld	-8 (ix),a
	ld	a,5 (ix)
	adc	a, -1 (ix)
	ld	-7 (ix),a
	ld	hl,#0x0000
	ex	(sp), hl
	ld	-10 (ix),#0x00
	ld	-9 (ix),#0x00
00123$:
;main.c:717: if(f[i][j].val==1)
	ld	a,-8 (ix)
	add	a, -10 (ix)
	ld	-4 (ix),a
	ld	a,-7 (ix)
	adc	a, -9 (ix)
	ld	-3 (ix),a
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	inc	hl
	inc	hl
	ld	a,(hl)
	dec	a
	jr	NZ,00124$
;main.c:719: if(f[i][j].x+1 >= 9) // se sale de la malla
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	ld	e,(hl)
	ld	d,#0x00
	inc	de
	ld	a, e
	sub	a, #0x09
	ld	a,d
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00124$
;main.c:721: dibujaFigura(f);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_dibujaFigura
	pop	af
;main.c:722: revizaRenglon();
	call	_revizaRenglon
;main.c:724: llenaFigura(f,cont);
	ld	hl,(_cont)
	push	hl
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_llenaFigura
	pop	af
	pop	af
;main.c:725: return; // no puede moverse la figura a abajo
	jp	00139$
00124$:
;main.c:715: for(j=0;j<3;j++)
	ld	a,-10 (ix)
	add	a, #0x03
	ld	-10 (ix),a
	ld	a,-9 (ix)
	adc	a, #0x00
	ld	-9 (ix),a
	inc	-16 (ix)
	jr	NZ,00231$
	inc	-15 (ix)
00231$:
	ld	a,-16 (ix)
	sub	a, #0x03
	ld	a,-15 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00123$
;main.c:713: for(i=0;i<3;i++)
	ld	a,-2 (ix)
	add	a, #0x09
	ld	-2 (ix),a
	ld	a,-1 (ix)
	adc	a, #0x00
	ld	-1 (ix),a
	inc	-14 (ix)
	jr	NZ,00232$
	inc	-13 (ix)
00232$:
	ld	a,-14 (ix)
	sub	a, #0x03
	ld	a,-13 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00143$
;main.c:731: for(i=0;i<3;i++)
	ld	-14 (ix),#0x00
	ld	-13 (ix),#0x00
	ld	de,#0x0000
;main.c:733: for(j=2;j>0;j--)
00149$:
	ld	hl,#0x0002
	ex	(sp), hl
	ld	bc,#0x0012
00127$:
;main.c:735: if(f[j][i].val==1)
	ld	l,4 (ix)
	ld	h,5 (ix)
	add	hl,bc
	add	hl, de
	push	hl
	pop	iy
	push	iy
	pop	hl
	inc	hl
	inc	hl
	ld	a,(hl)
	dec	a
	jr	NZ,00128$
;main.c:738: if(max7219ReadPixel(f[j][i].x+1,f[j][i].y) != 0)
	push	iy
	pop	hl
	inc	hl
	ld	b,(hl)
	ld	a, 0 (iy)
	inc	a
	push	de
	push	bc
	inc	sp
	push	af
	inc	sp
	call	_max7219ReadPixel
	pop	af
	ld	a,l
	pop	de
	or	a, a
	jr	Z,00130$
;main.c:740: dibujaFigura(f);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_dibujaFigura
	pop	af
;main.c:741: revizaRenglon();
	call	_revizaRenglon
;main.c:742: revizaPerdio(f); 
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_revizaPerdio
	pop	af
;main.c:743: llenaFigura(f,cont);
	ld	hl,(_cont)
	push	hl
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_llenaFigura
	pop	af
	pop	af
;main.c:744: return;
	jp	00139$
;main.c:747: break;
00128$:
;main.c:733: for(j=2;j>0;j--)
	ld	a,c
	add	a,#0xF7
	ld	c,a
	ld	a,b
	adc	a,#0xFF
	ld	b,a
	pop	hl
	push	hl
	dec	hl
	ex	(sp), hl
	xor	a, a
	cp	a, -16 (ix)
	sbc	a, -15 (ix)
	jp	PO, 00235$
	xor	a, #0x80
00235$:
	jp	M,00127$
00130$:
;main.c:731: for(i=0;i<3;i++)
	inc	de
	inc	de
	inc	de
	inc	-14 (ix)
	jr	NZ,00236$
	inc	-13 (ix)
00236$:
	ld	a,-14 (ix)
	sub	a, #0x03
	ld	a,-13 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00149$
;main.c:754: for(i=0;i<3;i++)
	ld	-14 (ix),#0x00
	ld	-13 (ix),#0x00
	ld	-4 (ix),#0x00
	ld	-3 (ix),#0x00
;main.c:756: for(j=0;j<3;j++)
00152$:
	ld	a,4 (ix)
	add	a, -4 (ix)
	ld	-10 (ix),a
	ld	a,5 (ix)
	adc	a, -3 (ix)
	ld	-9 (ix),a
	ld	de,#0x0000
	ld	bc,#0x0000
00131$:
;main.c:758: f[i][j].x=f[i][j].x+1;
	ld	l,-10 (ix)
	ld	h,-9 (ix)
	add	hl,bc
	inc	(hl)
;main.c:756: for(j=0;j<3;j++)
	inc	bc
	inc	bc
	inc	bc
	inc	de
	ld	a,e
	sub	a, #0x03
	ld	a,d
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00131$
;main.c:754: for(i=0;i<3;i++)
	ld	a,-4 (ix)
	add	a, #0x09
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	inc	-14 (ix)
	jr	NZ,00237$
	inc	-13 (ix)
00237$:
	ld	a,-14 (ix)
	sub	a, #0x03
	ld	a,-13 (ix)
	rla
	ccf
	rra
	sbc	a, #0x80
	jr	C,00152$
;main.c:762: if(f[2][0].x > 8)
	ld	l,4 (ix)
	ld	h,5 (ix)
	ld	de, #0x0012
	add	hl, de
	ld	h,(hl)
	ld	a,#0x08
	sub	a, h
	jp	NC,00139$
;main.c:765: for(i=0;i<3;i++)
	ld	bc,#0x0000
	ld	de,#0x0000
	ld	-4 (ix),#0x00
	ld	-3 (ix),#0x00
;main.c:767: for(j=2;j>=0;j--)
00158$:
	ld	hl,#0x0002
	ex	(sp), hl
	ld	-10 (ix),#0x12
	ld	-9 (ix),#0x00
	ld	-8 (ix),#0x12
	ld	-7 (ix),#0x00
00135$:
;main.c:776: f[j][i].x=f[j-1][i].x;
	ld	a,-16 (ix)
	add	a,#0xFF
	ld	-2 (ix),a
	ld	a,-15 (ix)
	adc	a,#0xFF
	ld	-1 (ix),a
;main.c:769: if(j==0)
	ld	a,-15 (ix)
	or	a,-16 (ix)
	jr	NZ,00117$
;main.c:771: f[j][i].x=f[j][i].x-1;
	ld	a,4 (ix)
	add	a, -8 (ix)
	ld	h,a
	ld	a,5 (ix)
	adc	a, -7 (ix)
	ld	l,a
	ld	a,h
	add	a, -4 (ix)
	ld	-6 (ix),a
	ld	a,l
	adc	a, -3 (ix)
	ld	-5 (ix),a
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	a,(hl)
	add	a,#0xFF
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	(hl),a
;main.c:772: f[j][i].y=f[j][i].y;
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	inc	hl
	ld	a,(hl)
	ld	(hl),a
;main.c:773: f[j][i].val=0;  
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	inc	hl
	inc	hl
	ld	(hl),#0x00
	jr	00136$
00117$:
;main.c:776: f[j][i].x=f[j-1][i].x;
	ld	a,4 (ix)
	add	a, -10 (ix)
	ld	l,a
	ld	a,5 (ix)
	adc	a, -9 (ix)
	ld	h,a
	add	hl,de
	ld	-6 (ix),l
	ld	-5 (ix),h
	push	de
	ld	e,-2 (ix)
	ld	d,-1 (ix)
	ld	l, e
	ld	h, d
	add	hl, hl
	add	hl, hl
	add	hl, hl
	add	hl, de
	pop	de
	ld	-12 (ix),l
	ld	-11 (ix),h
	ld	a,4 (ix)
	add	a, -12 (ix)
	ld	l,a
	ld	a,5 (ix)
	adc	a, -11 (ix)
	ld	h,a
	add	hl, de
	push	hl
	pop	iy
	ld	a, 0 (iy)
	ld	l,-6 (ix)
	ld	h,-5 (ix)
	ld	(hl),a
;main.c:777: f[j][i].y=f[j-1][i].y;
	ld	a,-6 (ix)
	add	a, #0x01
	ld	-12 (ix),a
	ld	a,-5 (ix)
	adc	a, #0x00
	ld	-11 (ix),a
	push	iy
	pop	hl
	inc	hl
	ld	a,(hl)
	ld	l,-12 (ix)
	ld	h,-11 (ix)
	ld	(hl),a
;main.c:778: f[j][i].val=f[j-1][i].val;
	ld	a,-6 (ix)
	add	a, #0x02
	ld	-12 (ix),a
	ld	a,-5 (ix)
	adc	a, #0x00
	ld	-11 (ix),a
	ld	a,2 (iy)
	ld	l,-12 (ix)
	ld	h,-11 (ix)
	ld	(hl),a
00136$:
;main.c:767: for(j=2;j>=0;j--)
	ld	a,-10 (ix)
	add	a,#0xF7
	ld	-10 (ix),a
	ld	a,-9 (ix)
	adc	a,#0xFF
	ld	-9 (ix),a
	ld	a,-8 (ix)
	add	a,#0xF7
	ld	-8 (ix),a
	ld	a,-7 (ix)
	adc	a,#0xFF
	ld	-7 (ix),a
	ld	a,-2 (ix)
	ld	-16 (ix),a
	ld	a,-1 (ix)
	ld	-15 (ix),a
	bit	7, -1 (ix)
	jp	Z,00135$
;main.c:765: for(i=0;i<3;i++)
	inc	de
	inc	de
	inc	de
	ld	a,-4 (ix)
	add	a, #0x03
	ld	-4 (ix),a
	ld	a,-3 (ix)
	adc	a, #0x00
	ld	-3 (ix),a
	inc	bc
	ld	a,c
	sub	a, #0x03
	ld	a,b
	rla
	ccf
	rra
	sbc	a, #0x80
	jp	C,00158$
;main.c:782: dibujaFigura(f);
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_dibujaFigura
	pop	af
;main.c:783: revizaRenglon();
	call	_revizaRenglon
;main.c:784: llenaFigura(f,cont);
	ld	hl,(_cont)
	push	hl
	ld	l,4 (ix)
	ld	h,5 (ix)
	push	hl
	call	_llenaFigura
	pop	af
	pop	af
00139$:
	ld	sp, ix
	pop	ix
	ret
;main.c:791: void revizaRenglon()
;	---------------------------------
; Function revizaRenglon
; ---------------------------------
_revizaRenglon::
	push	ix
	ld	ix,#0
	add	ix,sp
	dec	sp
;main.c:799: for(i=1;i<9;i++)
	ld	d,#0x01
00119$:
;main.c:801: comp=1;
;main.c:802: for(j=1;j<9;j++)
	ld	hl,#0x0101
00112$:
;main.c:804: if(max7219ReadPixel(i,j) == 0)
	push	hl
	push	de
	push	hl
	inc	sp
	push	de
	inc	sp
	call	_max7219ReadPixel
	pop	af
	ld	a,l
	pop	de
	pop	hl
;main.c:806: comp=0;
	or	a,a
	jr	NZ,00113$
	ld	l,a
;main.c:807: break;
	jr	00103$
00113$:
;main.c:802: for(j=1;j<9;j++)
	inc	h
	ld	a,h
	sub	a, #0x09
	jr	C,00112$
00103$:
;main.c:810: if(comp==1)
	dec	l
	jr	NZ,00120$
;main.c:812: max7219DrawColumn(i,0x00);
	push	de
	xor	a, a
	push	af
	inc	sp
	push	de
	inc	sp
	call	_max7219DrawColumn
	pop	af
	pop	de
;main.c:813: for(k=i;k>1;k--)
	ld	c,d
00117$:
	ld	a,#0x01
	sub	a, c
	jr	NC,00120$
;main.c:815: for(l=1;l<9;l++)
	ld	e,c
	dec	e
	ld	-1 (ix),e
	ld	b,#0x01
00114$:
;main.c:817: if(max7219ReadPixel(k-1,l) != 0)
	push	bc
	push	de
	push	bc
	inc	sp
	ld	a,-1 (ix)
	push	af
	inc	sp
	call	_max7219ReadPixel
	pop	af
	ld	a,l
	pop	de
	pop	bc
	or	a, a
	jr	Z,00105$
;main.c:819: max7219DrawPixel(k,l,1);
	push	bc
	push	de
	ld	a,#0x01
	push	af
	inc	sp
	push	bc
	call	_max7219DrawPixel
	pop	af
	inc	sp
	pop	de
	pop	bc
	jr	00115$
00105$:
;main.c:823: max7219DrawPixel(k,l,0);
	push	bc
	push	de
	xor	a, a
	push	af
	inc	sp
	push	bc
	call	_max7219DrawPixel
	pop	af
	inc	sp
	pop	de
	pop	bc
00115$:
;main.c:815: for(l=1;l<9;l++)
	inc	b
	ld	a,b
	sub	a, #0x09
	jr	C,00114$
;main.c:813: for(k=i;k>1;k--)
	ld	c,e
	jr	00117$
00120$:
;main.c:799: for(i=1;i<9;i++)
	inc	d
	ld	a,d
	sub	a, #0x09
	jr	C,00119$
	inc	sp
	pop	ix
	ret
;main.c:834: int main(){
;	---------------------------------
; Function main
; ---------------------------------
_main::
	push	ix
	ld	ix,#0
	add	ix,sp
	ld	hl,#-45
	add	hl,sp
	ld	sp,hl
;main.c:840: system_init(); 
	call	_system_init
;main.c:841: iniciaJuego(_figura);
	ld	hl,#0x0000
	add	hl,sp
	ld	-14 (ix),l
	ld	-13 (ix),h
	ld	l,-14 (ix)
	ld	h,-13 (ix)
	push	hl
	call	_iniciaJuego
	pop	af
;main.c:846: for(i=0;i<10;i++)
00124$:
	ld	a,-14 (ix)
	ld	-16 (ix),a
	ld	a,-13 (ix)
	ld	-15 (ix),a
	ld	a,-14 (ix)
	ld	-12 (ix),a
	ld	a,-13 (ix)
	ld	-11 (ix),a
	ld	a,-14 (ix)
	ld	-10 (ix),a
	ld	a,-13 (ix)
	ld	-9 (ix),a
	ld	a,-14 (ix)
	ld	-8 (ix),a
	ld	a,-13 (ix)
	ld	-7 (ix),a
	ld	a,-14 (ix)
	ld	-6 (ix),a
	ld	a,-13 (ix)
	ld	-5 (ix),a
	ld	a,-14 (ix)
	ld	-4 (ix),a
	ld	a,-13 (ix)
	ld	-3 (ix),a
	ld	-18 (ix),#0x00
00115$:
;main.c:848: botones=PPI_PORTC ;
	in	a,(_PPI_PORTC)
	ld	-17 (ix),a
;main.c:851: if(botones & 0x01 > 0)
	bit	0, -17 (ix)
	jr	Z,00102$
;main.c:853: mueveIzquierda(_figura);
	ld	a,-16 (ix)
	ld	-2 (ix),a
	ld	a,-15 (ix)
	ld	-1 (ix),a
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	call	_mueveIzquierda
	pop	af
00102$:
;main.c:855: if(botones & 0x02 > 0)
	bit	1, -17 (ix)
	jr	Z,00104$
;main.c:857: giraIzquierda(_figura);
	ld	a,-12 (ix)
	ld	-2 (ix),a
	ld	a,-11 (ix)
	ld	-1 (ix),a
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	call	_giraIzquierda
	pop	af
00104$:
;main.c:859: if(botones & 0x04 > 0)
	bit	2, -17 (ix)
	jr	Z,00106$
;main.c:861: giraDerecha(_figura);
	ld	a,-10 (ix)
	ld	-2 (ix),a
	ld	a,-9 (ix)
	ld	-1 (ix),a
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	call	_giraDerecha
	pop	af
00106$:
;main.c:863: if(botones & 0x08 > 0)
	bit	3, -17 (ix)
	jr	Z,00108$
;main.c:865: mueveDerecha(_figura);
	ld	l,-8 (ix)
	ld	h,-7 (ix)
	push	hl
	call	_mueveDerecha
	pop	af
00108$:
;main.c:867: if(botones & 0x10 > 0)
	bit	4, -17 (ix)
	jr	Z,00110$
;main.c:869: mueveAbajo(_figura);
	ld	a,-6 (ix)
	ld	-2 (ix),a
	ld	a,-5 (ix)
	ld	-1 (ix),a
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	call	_mueveAbajo
	pop	af
00110$:
;main.c:872: dibujaFigura(_figura);
	ld	l,-4 (ix)
	ld	h,-3 (ix)
	push	hl
	call	_dibujaFigura
;main.c:873: delay_ms(100);
	ld	hl, #0x0064
	ex	(sp),hl
	call	_delay_ms
	pop	af
;main.c:846: for(i=0;i<10;i++)
	inc	-18 (ix)
	ld	a,-18 (ix)
	sub	a, #0x0A
	jp	C,00115$
;main.c:875: delay_ms(200);
	ld	hl,#0x00C8
	push	hl
	call	_delay_ms
	pop	af
;main.c:876: mueveAbajo(_figura);
	ld	a,-14 (ix)
	ld	-2 (ix),a
	ld	a,-13 (ix)
	ld	-1 (ix),a
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	call	_mueveAbajo
	pop	af
;main.c:877: dibujaFigura(_figura);
	ld	a,-14 (ix)
	ld	-2 (ix),a
	ld	a,-13 (ix)
	ld	-1 (ix),a
	ld	l,-2 (ix)
	ld	h,-1 (ix)
	push	hl
	call	_dibujaFigura
;main.c:878: delay_ms(400);
	ld	hl, #0x0190
	ex	(sp),hl
	call	_delay_ms
	pop	af
	jp	00124$
	.area _CODE
	.area _INITIALIZER
__xinit___displayBuffer:
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.db #0x00	; 0
	.area _CABS (ABS)
