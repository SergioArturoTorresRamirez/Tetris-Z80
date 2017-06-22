# Tetris-Z80
Implementación del juego tetris en el SMZ80 con una matriz de leds

# Conexiones

Se conectan 5 push-buttons al PPI del SMZ80 por el puerto C y la matríz de leds se controla
mediante el chip MAX7219, que va conectado al puerto A del PPI.

Los push-buttos tiene una configuración PULL-DOWN y se conectan al puerto C de la siguiente manera:

PC0 -> Push button para Flecha Izquierda
PC1 -> Push button para Girar Derecha
PC2 -> Push button para Girar Izquierda
PC3 -> Push button para Flecha derecha
PC4 -> Push button para Bajar figura

El chip MAX7219 se controla mediante una implementación del protocolo SPI por software, ya que
el SMZ80 no incorpora esta comunicación por Hardware. El chip se controla mediante las señales
"Data", "Clk" y "CS" que son generadas por el SMZ80 con el PPI en el puerto A.

La conexión del chip al puerto A es la siguiente:

PA0 -> Pin DIN del MAX7219
PA1 -> Pin CS del MAX7219
PA2 -> Pin CLK del MAX7219



#Imagenes del Hardware

		#Pantalla y botonera

		#SMZ80

		#Conexión