//--Proyecto final para Digitales II.--
//Se busca recibir la información del sensor de campo magnético HMC5883 con la blue pill, y mediante leds, mostrar la dirección y sentido del campo magnético resultante.

//--Includes--
#include "main.h"
#include "math.h"
#include "stdlib.h"

//--Defines:--
#define GPIO_CRL 	0
#define GPIO_CRH 	1
#define GPIO_IDR 	2
#define GPIO_ODR 	3
#define APB2     	6			//0x18
#define APB1     	7			//0x1C
#define I2C_CR1  	0
#define I2C_CR2  	1
#define I2C_CCR  	7
#define I2C_TRISE  	8		//0x20
#define I2C_SR1  	5
#define I2C_SR2  	6
#define I2C_DR  	4
#define ISER0  		0
#define CTRL  		0
#define LOAD  		1
#define VAL  		2
#define PI 			3.141592

//--Direcciones y calibración del sensor HMC5883--

#define PARTE_BAJA_X  4
#define PARTE_ALTA_X  3
#define PARTE_BAJA_Z  6
#define PARTE_ALTA_Z  5
#define PARTE_BAJA_Y  8
#define PARTE_ALTA_Y  7
#define xcal 0.876932174
#define ycal 0.930176407
#define zcal 0.88133496

volatile unsigned char ADDRESS_HMC5883_WRITE=0x3C;
volatile unsigned char ADDRESS_HMC5883_READ=0x3D;

//--Variables globales utilizadas--

unsigned int lectura=0;		//usada para leer registros cuando es necesario leerlos para avanzar o limpiar un bit nada más.
int timer_done=0;			//Usada para rutina de tiempo por interrupción.
signed short int x;			//Varibles globales de 16 bits para las 3 mediciones. Por ser globales se inicializan en 0 automaticamente.
signed short int y;
signed short int z;

//--Carga de direcciones de periféricos--

volatile unsigned int *portC= (unsigned int*)(0x40011000);
volatile unsigned int *portB= (unsigned int*)(0x40010C00);
volatile unsigned int *portA= (unsigned int*)(0x40010800);
volatile unsigned int *Rcc=(unsigned int*)(0x40021000);
volatile unsigned int *I2C1A=(unsigned int*)(0x40005400);
volatile unsigned int *NVICA=(unsigned int*)(0xE000E100);
volatile unsigned int *SYSTICK=(unsigned int*)(0xE000E010);

//--Prototipos de funciones basicas I2C--

void enviar_start(void);
void enviar_direccion(unsigned char);
void escribir_dato(unsigned char);
unsigned char leer_dato(void);
void enviar_stop(void);
unsigned char leer_registro_sensor(unsigned char);

//--Prototipos de funciones de configuración/calibración del sensor de campo magético--

void configurar_sensor(void);
void calibrar_sensor(void);

//--Prototipos de funciones de configuracion del microcontrolador--

void config_clocks_puertos(void);
void configurar_i2c(void);
void configurar_systick(void);
void HAL_IncTick(void);
void delay_20ms(void);

//--Main:--

int main(void)
{
	double anguloXY=0;
	double anguloPolar=0;
	double hipotenusa=0;

	config_clocks_puertos();						//Configura los clocks del sistema y los de los puertos a utilizar.

	configurar_i2c();								//Configura el módulo I2C, enciende su clock, y también enciende el periférico en sí.

	configurar_systick();							//Deja preparado al systick para generar una rutina de tiempo de 20ms en el futuro.

//	calibrar_sensor();								//Sólo se necesita ejecutar esta función una vez. Los resultados fueron: xcal=0.876932174, ycal= 0.930176407, zcal= 0.88133496

	configurar_sensor();							//Configura el sensor con ganancia de (1/820) Gauss/cuenta, actualización de datos a 15Hz y modo de medición continua.


while (1)
{
	//Lectura de registros

	x=leer_registro_sensor(PARTE_BAJA_X );			//Conformo los 16 bits de X de forma completa, y multiplico por factor de calibración
	x|=(leer_registro_sensor(PARTE_ALTA_X )<<8);
	x*=xcal;

	y=leer_registro_sensor(PARTE_BAJA_Y);			//Conformo los 16 bits de Y de forma completa, y multiplico por factor de calibración
	y|=(leer_registro_sensor(PARTE_ALTA_Y )<<8);
	y*=ycal;

	z=leer_registro_sensor(PARTE_BAJA_Z );			//Conformo los 16 bits de Z de forma completa, y multiplico por factor de calibración
	z|=(leer_registro_sensor(PARTE_ALTA_Z )<<8);
	z*=zcal;

	//Cálculo de vectores y ángulos

	anguloXY= (atan2((double)y, (double)x)) *(180/PI);				//Calculo ángulo respecto a X de las coordenadas X e Y (ángulo azimutal)
	hipotenusa=sqrt((x*x)+(y*y));									//Calculo hipotenusa en el plano XY
	anguloPolar=(atan2((double)z, (double)hipotenusa))*(180/PI);	//Calculo ángulo polar (del vector resultante de campo magnético contra el plano XY)

	if(anguloXY<0)
		anguloXY+=360;												//Llevo el rango -180 a 180, a 0 a 360.

	//Apagar todos los leds
	portA[GPIO_ODR] &= 0xFFFFFF00;									//Puertos A [7:0] reseteados por las dudas.

	portC[GPIO_CRH] = ((portC[GPIO_CRH] & 0xff0fffff) | 0x00400000);  //Apago led de prueba

	if(!(abs(anguloPolar)<70))										//Si el campo está demasiado contenido en el eje Z únicamente, avisamos con led rojo que la medición no es buena.
	portC[GPIO_CRH] = ((portC[GPIO_CRH] & 0xff0fffff) | 0x00200000);

	//Prender el led correcto según ángulo XY del campo
	if(anguloXY>=22.5 && anguloXY<67.5)
	portA[GPIO_ODR] |= (1<<5);

	if(anguloXY>=67.5 && anguloXY<112.5)
	portA[GPIO_ODR] |= (1<<6);

	if(anguloXY>=112.5 && anguloXY<157.5)
	portA[GPIO_ODR] |= (1<<7);

	if(anguloXY>=157.5 && anguloXY<202.5)
	portA[GPIO_ODR] |= (1<<0);

	if(anguloXY>=202.5 && anguloXY<247.5)
	portA[GPIO_ODR] |= (1<<1);

	if(anguloXY>=247.5 && anguloXY<292.5)
	portA[GPIO_ODR] |= (1<<2);

	if(anguloXY>=292.5 && anguloXY<337.5)
	portA[GPIO_ODR] |= (1<<3);

	if(anguloXY>=337.5 || anguloXY<22.5)
	portA[GPIO_ODR] |= (1<<4);

	delay_20ms();//Meter delay correspondiente a 20hz para volver a pedir datos. Esto es requerido por el sensor y su configuración.


}
	return (0);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------

void config_clocks_puertos(void)
{
	//--Configuraciones de clock del sistema:-- No son necesarias, por defecto viene el RC de 8 MHz encendido.
	//--Activación de clock del puerto--
	//--Configuraciones de clock y puerto para el led a usar--: Activamos el clock del puerto a utilizar. El led es PC13.Bit 4 del APB2, del RCC
	Rcc[APB2]|= (1 << 4);
	//*portC[GPIO_CRH] = ((portC[GPIO_CRH] & 0xff0fffff) | 0x00200000);   Salida push pull con velocidad 2MHz. Comento la linea porque, por como es el HW, con esto ya prende el led.

	//Activamos el clock del puerto B, ya que se usan para I2C. Esto se configura en el APB2ENR (peripheral clock enable register).
	//que pertenece al RCC(reset and clock control). La base es offset: 0x18, con base en 0x4002 1000
	Rcc[APB2]|= (1 << 3);

	//Activamos el clock del puerto A: es el puerto donde está la mayoría de los leds a usar. Usamos puerto A del 0 al 7.
	Rcc[APB2]|= (1 << 2);

	//Configuramos puerto A, 0 al 7,  como salida, push pull max velocidad 2MHz.
	portA[GPIO_CRL] = 0x22222222;
	portA[GPIO_ODR] &= 0xFFFFFF00;		//Puertos A [7:0] reseteados por las dudas.

}
void configurar_i2c(void)
{
	//--Configuración de puertos para I2C--

	//El siguiente paso es configurar los usos alternativos de los GPIO para que puedan actuar como SDA y SCL del I2C.
	//Esto se saca de la página 181 del manual de referencia, que te dice que pines del puerto B serán los indicados, según el valor del bit
	//I2C1 remap, que pertenece a un registro que permite que más de un pin pueda actual como I2C1. En este caso, con ese bit en cero, los pines
	//son PB6 y PB7.
	//Entendido eso, ahora vamos a configurar a los registros del puerto B, más concretamente al GPIOB_CRH o L
	//Hay que poner los bits de modo en "Output mode, max speed 2MHz"(0b10), y los bits de Config en "Alternate function Open Drain"(11)
	//Necesitamos si o sí el Open Drain, porque I2C lo necesita.
	//Se supone entonces que el estado natural de la salida tiene que ser 1, por eso están las pull ups del I2C que lleva por norma.
	//Procedemos a poner las direcciones. La dirección del puerto B base es 0x4001 0C00, y el offset del Port configuration register low (llega a los primeros 8 pines,
	//tiene 16 máximo). Como queremos los pines 6 y 7, necesitamos el low. Tiene offset 0x00 así que no hay que sumar nada.
	//Como resumen, tenemos que poner 0b11 en los bits 31 y 30 (open drain), y  0b11 en los bits29 y 28 (output with max speed 50MHz), para el pin PB7
	//Luego, hay que repetir el procedimiento para los bits 27 al 24 para el pin PB6. Usamos max speed 50 MHz por un ejemplo que se da en el manual, para aprovecharlo.
	portB[GPIO_CRL]=((portB[GPIO_CRL] & 0x00FFFFFF) | 0xFF000000);

	//--Activación de clock para el módulo I2C--:Se debe setear el bit 21 del registro APB1

	Rcc[APB1]|= (1 << 21);

	//----Comienzo de la configuración del periférico I2C y sus registros----

	//Lo primero a hacer es asegurarse que el periférico esté apagado mientras le hacemos las configuraciones necesarias. Corroboramos esto (bit 0, I2C control register 1)
	//La base del I2C1 module es 0x4000 5400 y el offset es 0x00.
	I2C1A[I2C_CR1]&=(0xFFFFFFFE);

	//--Configuración del master mode del I2C1--
	//La secuencia de configuración del módulo está en la página 758 del reference manual.
	//1) Nos dice que primero vayamos a I2C_CR2 (control register 2) a programar bien  input clock para generar los timings correctos.
	//El módulo I2C está conectado al bus APB, que corre a la misma frecuencia que el core (8MHz).Tenemos que poner 0b001000 en los primeros 6 bits del registro I2C_CR2.
	//Puede aceptar como entrada entre 2MHz y 50MHz, para generar los timings correctos.
	I2C1A[I2C_CR2]= ((I2C1A[I2C_CR2] & 0xFFFFFFC0) | 0b001000);
	//2) Configuramos ahora el clock register, con el clock control register (CCR), I2C_CCR. Tiene 16 bits. 1ros 11 bits son para configurar la velocidad de clock del pin SCL.
	//El bit 15 es para configurar el Simple Mode (0) o Fast Mode (1), y el bit 14 es para configurar unas cosas para el Fast Mode solamente.
	//Según el manual, con 8MHz de clock input, tenemos que poner en los 11 bits de clock el número 0x28 para generar 100KHz de clock SCL. El motivo es el siguiente:
	//El período de 8MHz es 125ns, y si a esto lo multiplicamos por 40 (0x28) llegamos a 5000ns, que es Thigh. Thigh + Tlow = Tclk, y Thigh=Tlow. Con 10.000 ns de T se tiene 100KHz.
	//La cuenta que se hace en el registro es Thigh= (Valor del CCR)* Tclkinput
	I2C1A[I2C_CCR]&=~(1 << 15); //Seteo modo Simple Mode
	I2C1A[I2C_CCR] = ((I2C1A[I2C_CCR] & 0xFFFFF000) | 0x28); //Seteamos el CCR en 0x28 como es indicado
	//3)Ahora el procedimiento pide setear el Rise Time. Tiene 6 bits, y hay que cargarle 09h, que resulta de un ejemplo para puertos con 50MHz max speed.
	I2C1A[I2C_TRISE]=((I2C1A[I2C_TRISE] & 0xFFFFFFC0) | 0x9); 			//pag 783 manual para ver el ejemplo del 0x9.
	//4)Encender el periférico
	I2C1A[I2C_CR1]|=(0b1);
	//5) Envar el start bit, pero esto se realiza en la función "enviar_start()"
}

void enviar_start(void)
{
	I2C1A[I2C_CR1]|=(1<<8);						//Solicito enviar el bit de start.
	while(  !((I2C1A[I2C_SR1])&0b1) );			//Espero hasta que el bit "SB" se setee, indicando que el start fue enviado (el micro va más rapido que la comunicacion I2C)
	lectura=I2C1A[I2C_SR1];						//Para limpiar el bit SB, es necesario leer el registro de status SR1, seguido de una escritura en el data register. Esto último lo hace
												//la función "enviar_direccion()".
}

void enviar_direccion(unsigned char direccion )
{
	//1) Tenemos que escribir la dirección de a qué periférico le estamos hablando. La dirección del slave la tenemos que escribir en el data register, ya que se comparte.
	I2C1A[I2C_DR]=((I2C1A[I2C_DR] & 0xFF00) | direccion);
	while(!((I2C1A[I2C_SR1]&0b10)||(I2C1A[I2C_SR1]&(1<<10))) ); //Esperamos a que se reciba el ACK (ADDR=1) o bien haya ACK failure (AF=1)
	//2) Tenemos que leer el registro de status SR1 y SR2 para limpiar el bit ADDR, y dejar todo listo para la siguiente función
	lectura=I2C1A[I2C_SR1];
	lectura=I2C1A[I2C_SR2];
}

void enviar_stop(void)
{
	I2C1A[I2C_CR1]|=(1<<9);						//Solicito enviar el bit de stop, activando el stop bit.
	while(  ((I2C1A[I2C_CR1])&(1<<9)) );		//Espero hasta que el bit "STOP" se limpie por hardware, indicando que el stop fue enviado (el micro va más rapido que la comunicacion I2C)
}

void escribir_dato(unsigned char dato)
{
												//1) Ya limpio el ADDR, podemos poner un byte de datos para enviar. No debe usarse esta función para enviar el address.
	I2C1A[I2C_DR]=((I2C1A[I2C_DR] & 0xFF00) | dato);
	while (! ( (I2C1A[I2C_SR1]) & (1<<2) ) ); 	//Esperamos por el bit BTF (Byte transfer finished) que nos indica que la transmisión del byte ha terminado
}

unsigned char leer_dato(void)
{
	unsigned char datos=0;


	I2C1A[I2C_CR1]&=~(1<<10);					//Como leemos de a un solo byte de datos, no hay que enviar ACK en el momento en que se recibe la info (RxNE=1 en ese momento)
												//Por ello, deshabilitamos el ACK para enviar NACK

	while (! ( (I2C1A[I2C_SR1]) & (1<<6) ) );	//Esperar por el bit RxNE que se ponga en 1 (se pone en uno cuando el registro de datos tiene algo, ie. no está vacio (Not Empty))

	datos=I2C1A[I2C_DR];						//Leemos el DR y lo guardamos en "datos". Esto también, de paso, limpia el bit RxNE.
	I2C1A[I2C_CR1]|=(1<<10);					//Volvemos a activar ACK en cualquier caso.
	return (datos);
}

unsigned char leer_registro_sensor(unsigned char registro)
{
	unsigned char datos=0;

	enviar_start();								//Setear el start bit
	enviar_direccion(ADDRESS_HMC5883_WRITE);	//Enviamos direccion de escritura del sensor
	escribir_dato(registro);					//Byte de dirección de memoria a leer: parte baja de la medición de X (8 bits). Con esto el puntero interno del sensor ya queda bien ubicado
	enviar_stop();								//En lugar de enviar bit de re-start, enviamos bit de stop y luego volvemos a iniciar la comunicación.

	enviar_start();
	enviar_direccion(ADDRESS_HMC5883_READ);		//Enviamos dirección de lectura del sensor.
	datos=leer_dato();							//Leemos el byte y lo dejamos en la variable "datos" para retornarla.
	enviar_stop();
	return datos;
}

void configurar_sensor(void)
{
	enviar_start();								//Setear el start bit
	enviar_direccion(ADDRESS_HMC5883_WRITE);	//Enviamos direccion de escritura del sensor
	escribir_dato(00);							//Enviamos "byte de direccion de memoria", que corressponde al registro A de configuración del sensor.
	escribir_dato(0b01110000);					//Configuramos 8 muestras promediadas en cada output, 15 Hz de frecuencia de output,y modo "normal" de medición.
												//También se limpia bit 7, el cual por defecto está seteado y la hoja de datos indica que debe ser limpiado.
	enviar_stop();								//Cerramos comunicación, para luego poder escribir otro registro sin problemas.

	enviar_start();
	enviar_direccion(ADDRESS_HMC5883_WRITE);
	escribir_dato(01);							//Byte de dirección de memoria a escribir: seleccionamos registro de configuración B del sensor
	escribir_dato(0b11000000);					//Seteamos ganancia de (1/330) Gauss/cuenta.
	enviar_stop();

	enviar_start();
	enviar_direccion(ADDRESS_HMC5883_WRITE);
	escribir_dato(02);							//Byte de dirección de memoria a escribir: seleccionamos registro de configuración de modo de operación.
	escribir_dato(0b00000000);					//Modo de medición continua. De lo contrario, el sensor mide una sola vez.
	enviar_stop();
}

void calibrar_sensor(void)						//Esta función sólo se usó una vez, se incluye por si se necesita en el futuro para otro sensor.
{
	enviar_start();								//Setear el start bit
	enviar_direccion(ADDRESS_HMC5883_WRITE);	//Enviamos direccion de escritura del sensor
	escribir_dato(00);							//Enviamos "byte de direccion de memoria", que corressponde al registro A de configuración del sensor.
	escribir_dato(0b01110001);					//Configuramos modo calibración con bias positivo, además de 8 muestras promediadas en cada output, 15 Hz de frecuencia de output.
												//También se limpia bit 7, el cual por defecto está seteado y la hoja de datos indica que debe ser limpiado.
	enviar_stop();								//Cerramos comunicación, para luego poder escribir otro registro sin problemas.

	enviar_start();
	enviar_direccion(ADDRESS_HMC5883_WRITE);
	escribir_dato(01);							//Byte de dirección de memoria a escribir: seleccionamos registro de configuración B del sensor
	escribir_dato(0b01100000);					//Seteamos ganancia de (1/660) Gauss/cuenta, como indica el ejemplo de calbiración (pag 17 datasheet sensor)
	enviar_stop();

	enviar_start();
	enviar_direccion(ADDRESS_HMC5883_WRITE);
	escribir_dato(02);							//Byte de dirección de memoria a escribir: seleccionamos registro de configuración de modo de operación.
	escribir_dato(0b00000001);					//Modo de medición singular, requerido para el modo de calibración.
	enviar_stop();
	//El modo calibración está entonces activado. Esperamos 766 cuentas en X e Y, y 713 en Z (según datasheet).
	//Procedemos a tomar las mediciones de calibración

	x=leer_registro_sensor(PARTE_BAJA_X );			//Conformo los 16 bits de X de forma completa
	x|=(leer_registro_sensor(PARTE_ALTA_X )<<8);

	y=leer_registro_sensor(PARTE_BAJA_Y);			//Conformo los 16 bits de Y de forma completa
	y|=(leer_registro_sensor(PARTE_ALTA_Y )<<8);

	z=leer_registro_sensor(PARTE_BAJA_Z );			//Conformo los 16 bits de Z de forma completa
	z|=(leer_registro_sensor(PARTE_ALTA_Z )<<8);

//	xcal=766.0/x;
//	ycal=766.0/y;										//Calculamos factores de calibración de escala (pag 17 datasheet HMC5883)
//	zcal=713.0/z;
//	Los valores quedan comentados así no se pisan las definiciones con los defines. En caso de re-calcularlos, quitar los defines y descomentar.
}
void configurar_systick(void)
{
	//Configuramos el systick para demorar 20ms. Este depende de ARM así que sus cosas están en el programming manual del cortex M3.La dirección del systick es 0xE000E010.

	//Sus registros son: el control, el load value, el current value register, y el valor de calibración guardado. Todos con offset de 0x4 entre ellos.
	//Hay dos opciones de clock para el systick: el del sistema (8Mhz) y el de sistema (AHB, Advanced High Performance Bus) /8, para que de 1Mhz.

	SYSTICK[CTRL]|=(1<<1); 		//Configura input clock de 1Mhz y de paso activa la interrupción (o excepción según ARM) del systick.
	SYSTICK[LOAD]=0x4E20;		//Le cargo 20.000 decimal (4E20h) así tardará 20 mili segundos.
	SYSTICK[VAL]=0x00000000;	//El current value ya arranca en cero, pero igualmente lo fuerzo.
	NVICA[0] |= (1<<15);		//Activo interrupt del NVIC
}

void delay_20ms(void)
{
	SYSTICK[CTRL]|=(1<<0);//Activo el contador
	while(!timer_done);
	timer_done=0;
}

void HAL_IncTick(void)
{
	timer_done=1;
}
