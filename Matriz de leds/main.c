/***************************************************************************
 *  Ejemplo UART 8250.
 *  
 *  Programa de ejemplo para configuración y uso de la UART 8250 con la
 *  API smz80.
 *  
 *  El programa configura la UART como 8N1 a 9600 baudios con la interrupcion
 *  por dato recibido habilitada por la interrupción INT del Z80 en modo 1.
 *  
 *  Cada que se recibe un caracter, se envia de regreso (ECO).
 *
 ***************************************************************************
 *
 *      Autor:  Alfredo Orozco de la Paz
 *      e-mail: alfredoopa@gmail.com 
 *                                                                  *FRED*
 ***************************************************************************
 */

/**
 * Definicion de las direcciones para el PPI y la UART
 * Si no se definen, no se pueden usar las funciones de la librería smz80.h.
 * 
 * se deben definir antes de incluir la librería.
 */

#define UART_BASE_ADDR 0x70
#define PPI_BASE_ADDR 0x00

#include "smz80.h"
#include "max7219.h"

int cont;

ISR_NMI(){

    /* Código de servicio de la interrupción NMI.*/
}

ISR_INT_38(){
}


typedef struct
{
    unsigned char x;
    unsigned char y;
    unsigned char val;
}pixel;

void system_init(){

    max7219Init();
    PPI_CTRL = 0x89;
    cont =1;
}


/*******************************************************************************
/ dibujaFigura: Muestra la figura en la matriz de lets.
/ - pixel[3][3]: Matriz bidimensional de pixeles que van a mostrarse.
********************************************************************************/
void dibujaFigura(pixel f[3][3])
{
    int i;
    int j;
    
    for( i=0;i<3;i++ )
    {
           for( j=0;j<3;j++ )
        {

            if(f[i][j].val==1)
                max7219DrawPixel(f[i][j].x,f[i][j].y,f[i][j].val);
           
        } 
    }  
   
}

void borraFigura(pixel f[3][3])
{
    int i;
    int j;
    
    for( i=0;i<3;i++ )
    {
           for( j=0;j<3;j++ )
        {
            if(f[i][j].val==1)
             max7219DrawPixel(f[i][j].x,f[i][j].y,0);
        } 
    }
}
/*******************************************************************************
/ llenaFigura 
/ -Funcion que llena la matriz de pixeles segun la figura que se quiera dibujar-
/ - tipo: Indica el tipo de figura con que se va a llenar el arreglo de pixeles.
        tipo = 1 -> Z
        tipo = 2 -> T
        tipo = 3 -> L
        tipo = 4 -> Cubo
        tipo = 5 -> Linea
/- pixel[3][3]: Matriz bidimencional de nueve pixeles que contendra la figura. 
********************************************************************************/
void llenaFigura(pixel f[3][3] , int tipo)
{
    f[0][0].x=1;
    f[0][1].x=1;
    f[0][2].x=1;
    
    f[1][0].x=2;
    f[1][1].x=2;
    f[1][2].x=2;
    
    f[2][0].x=3;
    f[2][1].x=3;
    f[2][2].x=3;
    
    f[0][0].y=1;
    f[0][1].y=2;
    f[0][2].y=3;
    
    f[1][0].y=1;
    f[1][1].y=2;
    f[1][2].y=3;

    f[2][0].y=1;
    f[2][1].y=2;
    f[2][2].y=3;
    switch(tipo)
    {
        case 1://Z            // [0][1][1]
                              // [1][1][0]
            f[0][0].val=0;    // [0][0][0]
            f[0][1].val=1;
            f[0][2].val=1;
            f[1][0].val=1;
            f[1][1].val=1;
            f[1][2].val=0;
            f[2][0].val=0;
            f[2][1].val=0;
            f[2][2].val=0;
        break;
         case 2://T

            f[0][0].val=1;   //[1][1][1]    
            f[0][1].val=1;   //[0][1][0]
            f[0][2].val=1;   //[0][0][0]
            f[1][0].val=0;
            f[1][1].val=1;
            f[1][2].val=0;
            f[2][0].val=0;
            f[2][1].val=0;
            f[2][2].val=0;
        break;
         case 3://L

            f[0][0].val=0;   //[0][0][1]
            f[0][1].val=0;   //[1][1][1]
            f[0][2].val=1;   //[0][0][0]
            f[1][0].val=1;
            f[1][1].val=1;
            f[1][2].val=1;
            f[2][0].val=0;
            f[2][1].val=0;
            f[2][2].val=0;
        break;
         case 4://Cubo
            f[0][0].val=1;   //[1][1][0]
            f[0][1].val=1;   //[]1[1][0]
            f[0][2].val=0;   //[0][0][0]
            f[1][0].val=1;
            f[1][1].val=1;
            f[1][2].val=0;
            f[2][0].val=0;
            f[2][1].val=0;
            f[2][2].val=0;
        break;
        case 5:// linea
            f[0][0].val=0;   //[0][1][0]
            f[0][1].val=1;   //[0][1][0]
            f[0][2].val=0;   //[0][1][0]
            f[1][0].val=0;
            f[1][1].val=1;
            f[1][2].val=0;
            f[2][0].val=0;
            f[2][1].val=1;
            f[2][2].val=0;
        break;
    }
    cont++;
    if(cont>5)
        cont=1;
    
}
void dibujaCara(int index)
{
    unsigned char i;
    unsigned char j;
      for(i=1;i<9;i++)
    {
       for(j=1;j<9;j++)
       {
           max7219DrawPixel(i,j,0);
       }
    }
    
    switch(index)
    {
        case 1: //:)
            max7219DrawPixel(3,1,1);
            max7219DrawPixel(2,2,1);
            max7219DrawPixel(3,3,1);
            max7219DrawPixel(3,6,1);
            max7219DrawPixel(2,7,1);
            max7219DrawPixel(3,8,1);
            max7219DrawPixel(6,3,1);
            max7219DrawPixel(7,4,1);
            max7219DrawPixel(7,5,1);
            max7219DrawPixel(6,6,1);
        break;
        case 2: // O_O
            max7219DrawPixel(2,2,1);
            max7219DrawPixel(3,1,1);
            max7219DrawPixel(3,3,1);
            max7219DrawPixel(4,2,1);
            
            max7219DrawPixel(2,7,1);
            max7219DrawPixel(3,6,1);
            max7219DrawPixel(3,8,1);
            max7219DrawPixel(4,7,1);
            
            max7219DrawPixel(7,3,1);
            max7219DrawPixel(7,4,1);
            max7219DrawPixel(7,5,1);
            max7219DrawPixel(7,6,1);
        break;
        case 3: // -_-
            max7219DrawPixel(3,1,1);
            max7219DrawPixel(3,2,1);
            max7219DrawPixel(3,3,1);
            
            max7219DrawPixel(3,6,1);
            max7219DrawPixel(3,7,1);
            max7219DrawPixel(3,8,1);
            
            max7219DrawPixel(7,3,1);
            max7219DrawPixel(7,4,1);
            max7219DrawPixel(7,5,1);
            max7219DrawPixel(7,6,1);
        break;
    }
}
/*******************************************************************************
/ giraDerecha: Rota la matriz que contiene la figura a la derecha
/ - pixel[3][3]: Matriz bidimensional de pixeles que se va a rotar.
/
/   Matriz normal   Matriz rotada                Correspondencia [ij]
/   [00][01][02]       [20][10][00]   [00] -> [20] | [01] -> [10] | [02] -> [00]
/   [10][11][12]   ->  [21][11][01]   [10] -> [21] | [11] -> [11] | [12] -> [01]
/   [20][21][22]       [22][12][02]   [20] -> [22] | [21] -> [12] | [22] -> [02]
/
********************************************************************************/
void giraDerecha(pixel f[3][3])
{
  int i,j;
  pixel _nueva[3][3];
  borraFigura(f);
  for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++)
        { 
            _nueva[i][j].x=f[i][j].x; // Pasa las coordenadas acutuales de la figura en ese pixel
            _nueva[i][j].y=f[i][j].y; // Pasa las coordenadas acutuales de la figura en ese pixel
            _nueva[i][j].val=f[2-j][i].val;// Asigna el valor que corresponde al rotar la matriz
        }
  }
  
for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++)
    {
        if(max7219ReadPixel(_nueva[i][j].x,_nueva[i][j].y) != 0) // el bit esta ocupado en la malla
        {
            return;
        }
    }
  }
  
for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++)
        {
            f[i][j].x = _nueva[i][j].x;
            f[i][j].y = _nueva[i][j].y;
            f[i][j].val = _nueva[i][j].val;
        }
  }
}

/*******************************************************************************
/ giraIzquierda: Rota la matriz que contiene la figura a la izquierda
/ - pixel[3][3]: Matriz bidimensional de pixeles que se va a rotar.
/
/   Matriz normal   Matriz rotada                Correspondencia [ij]
/   [00][01][02]       [02][12][22]   [00] -> [02] | [01] -> [12] | [02] -> [22]
/   [10][11][12]   ->  [01][11][21]   [10] -> [01] | [11] -> [11] | [12] -> [21]
/   [20][21][22]       [00][10][20]   [20] -> [00] | [21] -> [10] | [22] -> [20]
/
********************************************************************************/
void giraIzquierda(pixel f[3][3])
{
  int i,j;
  pixel _nueva[3][3];
  borraFigura(f);
  for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++)
        { 
            _nueva[i][j].x=f[i][j].x;
            _nueva[i][j].y=f[i][j].y;
            _nueva[i][j].val=f[j][2-i].val;
        }
  }
for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++)
    {
        if(max7219ReadPixel(_nueva[i][j].x,_nueva[i][j].y) != 0) // el bit esta ocupado en la malla
        {
            return;
        }
    }
 }
  for(i=0;i<3;i++)
  {
    for(j=0;j<3;j++)
        {
            f[i][j].x = _nueva[i][j].x;
            f[i][j].y = _nueva[i][j].y;
            f[i][j].val = _nueva[i][j].val;
        }
  }
  

  
}

/*******************************************************************************
/ mueveDerecha: Desplaza la figura a la derecha en la malla principal 
/ - pixel[3][3]: Matriz bidimensional de pixeles que contiene la figura que se 
/                va a desplazar.
/
/Se debe considerar que ningun pixel de la figura choque con un pixel ocupado 
/de la malla.
/
/    Ejemplo de mapa de pixeles de la malla y la ubicacion actual de la figura
/    en  la malla
/
/   Pixeles de la malla(LEDS)           Pixeles en los que esta
/                                       actualmente la figura T
/   [01][02][03][04][05][06][07][08]    [00][01][02]   
/   [09][10][11][12][13][14][15][16]    [09][10][11]
/   [17][18][19][20][21][22][23][24]    [17][18][19]
/   [25][26][27][28][29][30][31][32]
/   [33][34][35][36][37][38][39][40]
/   [41][42][43][44][45][46][47][48]
/   [49][50][51][52][53][54][55][56]
/   [57][58][59][60][61][61][63][64]
/
/       Representacion 
/   [1][1][1][0][0][0][0][0]  [1][1][1]   
/   [0][1][0][0][0][0][0][0]  [0][1][0]
/   [0][0][0][0][0][0][0][0]  [0][1][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/ 
/   Para mover la figura a la derecha se debe:
/        1.0- Revizar que ningun pixel de la figura colicione 
/             con un pixel activo de la malla.
/        2.0- Revizar que ningun pixel de la figura se salga de la malla.
/        3.0- Sumar un 1 a la coordenada x de cada pixel en la figura
/
/   [00][01][02] -> [01][02][03]
/   [09][10][11] -> [10][11][12]
/   [17][18][19] -> [18][19][20]
/
/              Representacion 
/       [0][1][1][1][0][0][0][0]
/       [0][0][1][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/
********************************************************************************/
void mueveDerecha(pixel f[3][3])
{
      int i,j;
      borraFigura(f);
    //Reviza que la figura no se salga de la malla o que un pixel de la figura
    //no choque con alguno de la malla
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        { 
            if(f[i][j].val==1)
            {
                if(f[i][j].y+1 >= 9) // se sale de la malla
                {
                    return; // no puede moverse la figura a la derecha
                }
            }
        }
    }
    for(i=0;i<3;i++)
    {
        for(j=2;j>=0;j--)
            { 
                if(f[i][j].val==1)
                {
                    if(max7219ReadPixel(f[i][j].x,f[i][j].y+1) != 0) // el bit esta ocupado en la malla
                    {
                        return;
                    }else
                    {
                        break;
                    }
                }
            }
    }
    

    //La figura puede moverce a la derecha
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
            { 
                f[i][j].y=f[i][j].y+1;
            }
    }
        
    if(f[0][2].y > 8)
    {
        //Se recorren columnas a la derecha
        for(i=0;i<3;i++)
        {
            for(j=2;j>=0;j--)
                { 
                    if(j==0)
                    {
                        f[i][j].x=f[i][j].x;
                        f[i][j].y=f[i][j].y-1;
                        f[i][j].val=0;  
                    }else
                    {
                        f[i][j].x=f[i][j-1].x;
                        f[i][j].y=f[i][j-1].y;
                        f[i][j].val=f[i][j-1].val;
                    }
                }
        }

    }

}

/*******************************************************************************
/ mueveIzquierda: Desplaza la figura a la izquierda en la malla principal 
/ - pixel[3][3]: Matriz bidimensional de pixeles que contiene la figura que se 
/                va a desplazar.
/
/Se debe considerar que ningun pixel de la figura choque con un pixel ocupado 
/de la malla o se salga de la malla.
/
/    Ejemplo de mapa de pixeles de la malla y la ubicacion actual de la figura
/    en  la malla
/
/   Pixeles de la malla(LEDS)           Pixeles en los que esta
/                                       actualmente la figura T
/   [01][02][03][04][05][06][07][08]    [06][07][08]   
/   [09][10][11][12][13][14][15][16]    [14][15][16]
/   [17][18][19][20][21][22][23][24]    [22][23][24]
/   [25][26][27][28][29][30][31][32]
/   [33][34][35][36][37][38][39][40]
/   [41][42][43][44][45][46][47][48]
/   [49][50][51][52][53][54][55][56]
/   [57][58][59][60][61][61][63][64]
/
/       Representacion 
/   [1][1][1][0][0][0][0][0]  [1][1][1]   
/   [0][1][0][0][0][0][0][0]  [0][1][0]
/   [0][0][0][0][0][0][0][0]  [0][1][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/ 
/   Para mover la figura a la derecha se debe:
/        1.0- Revizar que ningun pixel de la figura colicione 
/             con un pixel activo de la malla.
/        2.0- Revizar que ningun pixel de la figura se salga de la malla.
/        3.0- Sumar un 1 a la coordenada x de cada pixel en la figura
/
/   [06][07][08] -> [05][06][07]
/   [14][15][16] -> [13][14][15]
/   [22][23][24] -> [21][22][23]
/
/
/              Representacion 
/       [0][0][0][0][1][1][1][0]
/       [0][0][0][0][0][1][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/
********************************************************************************/
void mueveIzquierda(pixel f[3][3])
{
    int i,j;
    borraFigura(f);
    //Reviza que la figura no se salga de la malla o que un pixel de la figura
    //no choque con alguno de la malla
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        { 
            if(f[i][j].val==1)
            {
                if(f[i][j].y-1 <= 0) // se sale de la malla
                {
                    return; // no puede moverse la figura a la Izquierda
                }
            }
        }
    }
    
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
            { 
                if(f[i][j].val==1)
                {
                    if(max7219ReadPixel(f[i][j].x,f[i][j].y-1) != 0)
                    {
                        return;
                    }else
                    {
                        break;
                    }
                }
            }
    }
    
    //La figura puede moverce a la izquierda
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
            { 
                f[i][j].y=f[i][j].y-1;
            }
    }
    
    if(f[0][0].y < 1)
    {
        //Se recorren columnas a la derecha
        for(i=0;i<3;i++)
        {
            for(j=0;j<3;j++)
                { 
                    if(j==2)
                    {
                        f[i][j].x=f[i][j].x;
                        f[i][j].y=f[i][j].y+1;
                        f[i][j].val=0;  
                    }else
                    {
                        f[i][j].x=f[i][j+1].x;
                        f[i][j].y=f[i][j+1].y;
                        f[i][j].val=f[i][j+1].val;
                    }
                }
        }

    }
}
void iniciaJuego(pixel f[3][3])
{
    unsigned char i;
    unsigned char j;
    
        dibujaCara(2);
    delay_ms(400);
    dibujaCara(3);
    delay_ms(400);
    dibujaCara(2);
    delay_ms(400);
    dibujaCara(3);
    delay_ms(400);
    dibujaCara(1);
    delay_ms(400);
      for(i=1;i<9;i++)
    {
       for(j=1;j<9;j++)
       {
           max7219DrawPixel(i,j,1);
       }
    }
    for(i=1;i<9;i++)
    {
       for(j=1;j<9;j++)
       {
           max7219DrawPixel(i,j,0);
       }
    }

    for(i=0;i<3;i++)
    {
       for(j=0;j<3;j++)
       {
           f[i][j].val=0;
       }
    }
    max7219Clear();
    llenaFigura(f,cont);
    dibujaFigura(f);
}



void revizaPerdio(pixel f[3][3])
{
    unsigned char i;
    for(i=1;i<9;i++)
    {
        if(max7219ReadPixel(1,i) != 0)
        {
            iniciaJuego(f);
            break;
        }
    }
}
/*******************************************************************************
/ mueveAbajo: Desplaza la figura hacia abajo en la malla principal. 
/ - pixel[3][3]: Matriz bidimensional de pixeles que contiene la figura que se 
/                va a desplazar.
/
/Se debe considerar que ningun pixel de la figura choque con un pixel ocupado 
/de la malla o se salga de la malla.
/
/    Ejemplo de mapa de pixeles de la malla y la ubicacion actual de la figura
/    en  la malla
/
/   Pixeles de la malla(LEDS)           Pixeles en los que esta
/                                       actualmente la figura T
/   [01][02][03][04][05][06][07][08]    [06][07][08]   
/   [09][10][11][12][13][14][15][16]    [14][15][16]
/   [17][18][19][20][21][22][23][24]    [22][23][24]
/   [25][26][27][28][29][30][31][32]
/   [33][34][35][36][37][38][39][40]
/   [41][42][43][44][45][46][47][48]
/   [49][50][51][52][53][54][55][56]
/   [57][58][59][60][61][61][63][64]
/
/       Representacion 
/   [1][1][1][0][0][0][0][0]  [1][1][1]   
/   [0][1][0][0][0][0][0][0]  [0][1][0]
/   [0][0][0][0][0][0][0][0]  [0][1][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/   [0][0][0][0][0][0][0][0]
/ 
/   Para mover la figura hacia abajo se debe:
/        1.0- Revizar que ningun pixel de la figura colicione 
/             con un pixel activo de la malla.
/        2.0- Revizar que ningun pixel de la figura se salga de la malla.
/        3.0- Sumar un 1 a la coordenada x de cada pixel en la figura
/
/   [06][07][08] -> [05][06][07]
/   [14][15][16] -> [13][14][15]
/   [22][23][24] -> [21][22][23]
/
/
/              Representacion 
/       [0][0][0][0][1][1][1][0]
/       [0][0][0][0][0][1][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/       [0][0][0][0][0][0][0][0]
/
********************************************************************************/
void mueveAbajo(pixel f[3][3])
{
    int i,j;
    borraFigura(f);
    //Reviza que la figura no se salga de la malla o que un pixel de la figura
    //no choque con alguno de la malla
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
        { 
            if(f[i][j].val==1)
            {
                if(f[i][j].x+1 >= 9) // se sale de la malla
                {
                    dibujaFigura(f);
                    revizaRenglon();
                    //revizaPerdio(f); 
                    llenaFigura(f,cont);
                    return; // no puede moverse la figura a abajo
                }
            }
        }
    }
    
    for(i=0;i<3;i++)
    {
        for(j=2;j>0;j--)
            { 
                if(f[j][i].val==1)
                {
                    
                    if(max7219ReadPixel(f[j][i].x+1,f[j][i].y) != 0)
                    {
                        dibujaFigura(f);
                        revizaRenglon();
                        revizaPerdio(f); 
                        llenaFigura(f,cont);
                        return;
                    }else
                    {
                        break;
                    }
                }
            }
    }
    
    //La figura puede moverce a abajo 
    for(i=0;i<3;i++)
    {
        for(j=0;j<3;j++)
            { 
                f[i][j].x=f[i][j].x+1;
            }
    }
    
     if(f[2][0].x > 8)
    {
        //Se recorren columnas a la derecha
        for(i=0;i<3;i++)
        {
            for(j=2;j>=0;j--)
                { 
                    if(j==0)
                    {
                        f[j][i].x=f[j][i].x-1;
                        f[j][i].y=f[j][i].y;
                        f[j][i].val=0;  
                    }else
                    {
                        f[j][i].x=f[j-1][i].x;
                        f[j][i].y=f[j-1][i].y;
                        f[j][i].val=f[j-1][i].val;
                    }
                }
        }
        dibujaFigura(f);
        revizaRenglon();
        llenaFigura(f,cont);
        //revizaPerdio(f);

    }
   
}

void revizaRenglon()
{
    unsigned char i;
    unsigned char j;
    unsigned char k;
    unsigned char l;
    unsigned char comp;
    uint8_t ren;
     for(i=1;i<9;i++)
    {
        comp=1;
        for(j=1;j<9;j++)
        {
            if(max7219ReadPixel(i,j) == 0)
            {
                comp=0;
                break;
            }
        }
        if(comp==1)
        {
             max7219DrawColumn(i,0x00);
            for(k=i;k>1;k--)
            {
               for(l=1;l<9;l++)
               {
                   if(max7219ReadPixel(k-1,l) != 0)
                   {
                        max7219DrawPixel(k,l,1);
                   }
                   else
                   {
                       max7219DrawPixel(k,l,0);
                   }
               }
            }
        }
    }
    
}



int main(){
    
    pixel _figura[3][3];
    unsigned char i;
    unsigned char botones;
    
    system_init(); 
    iniciaJuego(_figura);
    botones=0;
    i=1;
    
    while(TRUE) {
    for(i=0;i<10;i++)
    {
        botones=PPI_PORTC ;
    
        
        if(botones & 0x01 > 0)
        {
            mueveIzquierda(_figura);
        }
         if(botones & 0x02 > 0)
        {
            giraIzquierda(_figura);
        }
         if(botones & 0x04 > 0)
        {
            giraDerecha(_figura);
        }
         if(botones & 0x08 > 0)
        {
            mueveDerecha(_figura);
        }
             if(botones & 0x10 > 0)
        {
            mueveAbajo(_figura);
        }
        
        dibujaFigura(_figura);
        delay_ms(100);
    }
    delay_ms(200);
    mueveAbajo(_figura);
    dibujaFigura(_figura);
    delay_ms(400);

    }
}


