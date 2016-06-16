#include <plib.h>
#include <GenericTypeDefs.h>

#pragma config FPLLIDIV = DIV_1, FPLLMUL = MUL_20, FPLLODIV = DIV_2, FWDTEN = OFF, FCKSM = CSECME, FPBDIV = DIV_8
#pragma config OSCIOFNC = ON, POSCMOD = HS, FSOSCEN = ON, FNOSC = PRIPLL
#pragma config CP = OFF, BWP = OFF, PWP = OFF

//  Macros del Sistema
#define	GetSystemClock() 			(80000000ul)
#define	GetPeripheralClock()		(GetSystemClock()/(1 << OSCCONbits.PBDIV))
#define	GetInstructionClock()		(GetSystemClock())
#define CRC16  0x8005
#define FLASHERASERESPONSE 0x01
#define LOGINRESPONSE  0X01
#define RESPONSETEST    0X06
#define ENDRESPONSETEST 0X15
#define GETRANDOM(min,max) ((rand()%(int)(((max)+1)-(min)))+(min))

UINT8   bandera = 0;
UINT8   caracter;   //captura un byte del buffer RX
UINT8   buf[1024];  //Almacena trama que llega a RX
UINT8   mensajeRespuesta[1024];     //Almacena trama de respuesta
UINT8   *ptrBuffer = &buf[0];

//  Prototipo de funciones
void    SendDataBuffer (const char *buffer, UINT32 tamano);
void    AnalizarData (UINT8 *respuesta);
void    ArmarTrama(UINT8 *ptrBufferRespuesta, UINT16 tamano, UINT8 mensaje_tipo, UINT8 sync);
void    ConfiguracionInicial();
UINT16 genCRC16(UINT8 *data, UINT16 tamano);

#pragma interrupt InterruptHandler single
void InterruptHandler(void)
{
    if(INTGetFlag(INT_SOURCE_UART_RX(UART1)))
    {
        OpenTimer1(T1_ON | T1_PS_1_8 | T1_SOURCE_INT, 10);
        ConfigIntTimer1(T1_INT_ON | T1_INT_PRIOR_2);
        caracter = UARTGetDataByte(UART1);
        *ptrBuffer = caracter;
        ptrBuffer++;
        INTClearFlag(INT_SOURCE_UART_RX(UART1));
    }
    if ( INTGetFlag(INT_SOURCE_UART_TX(UART1)) )
    {
      INTClearFlag(INT_SOURCE_UART_TX(UART1));
    }
    if(mT1GetIntFlag())
    {
        bandera++;
        if(bandera == 5)
        {
            ptrBuffer = &buf[0];
            bandera =0 ;
        }
        mT1ClearIntFlag();
    }
}
//ver = *data+(*(data+1)<<8);

//  Sección de Código

int main(void)
{   
    //Configuración del UART
    UINT8   *ptrMensajeRespuesta = &mensajeRespuesta[0];
    
    UINT32 tamano_rx=0;

    ConfiguracionInicial();

    AnalizarData(ptrMensajeRespuesta);



}
void ConfiguracionInicial()
{
    UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
    UARTSetFifoMode(UART1, UART_INTERRUPT_ON_TX_NOT_FULL | UART_INTERRUPT_ON_RX_NOT_EMPTY);
    UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
    UARTSetDataRate(UART1, GetPeripheralClock(), 57600);
    UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
    
    INTEnable(INT_SOURCE_UART_RX(UART1), INT_ENABLED);
    INTSetVectorPriority(INT_VECTOR_UART(UART1), INT_PRIORITY_LEVEL_2);
    mT1SetIntPriority(3);
    INTEnableSystemSingleVectoredInt();
}
void SendDataBuffer(const char *buffer, UINT32 tamano)
{
    while(tamano)
    {
        while(!UARTTransmitterIsReady(UART1))
            ;

        UARTSendDataByte(UART1, *buffer);

        buffer++;
        tamano--;
    }

    while(!UARTTransmissionHasCompleted(UART1))
        ;
}
/*
 void putByte(char caracter)
{
    
}
 
 */

void ArmarTrama(UINT8 *ptrBufferRespuesta, UINT16 tamano, UINT8 mensaje_tipo, UINT8 sync)
{
    
    *ptrBufferRespuesta = 0xEE;
    *(ptrBufferRespuesta+1) = (UINT8) (tamano >> 8);
    *(ptrBufferRespuesta+2) = (UINT8) tamano;
    *(ptrBufferRespuesta+3) = sync;
    *(ptrBufferRespuesta+4) = mensaje_tipo;
}
void AnalizarData(UINT8 *respuesta)
{
    UINT16   c;
    UINT8    i;
    UINT16   crcCalculado;
    UINT8    dato;
    UINT8   *ptrAnalizar;
    UINT8    *ptrCRC;
    ptrAnalizar = &buf[0];
    UINT8   sync;
    for(c=0;c<1024;c++)
    {
        if (*ptrAnalizar == 0xEE)
        {
            c= *(ptrAnalizar+2)+((*(ptrAnalizar+1))<<8);
            if(*(ptrAnalizar+3)== 0x20)
            {
                sync = 0x21;
            }
            else if(*(ptrAnalizar+3)==0x21)
            {
                sync = 0x20;
            }
            else
            {
                break;
            }
            ptrCRC = ptrAnalizar+4;
            crcCalculado = genCRC16(ptrCRC,c);
            if((*(ptrAnalizar+c+5)+((*(ptrAnalizar+c+4))<<8))==crcCalculado)
            {
                if((*(ptrAnalizar+4))==0x01)
                {
                    break;
                }
                else if((*(ptrAnalizar+4))==0x02)
                {
                    ArmarTrama(respuesta,c,0x03,sync);
                    #ifdef FLASHERASERESPONSE
                        *(respuesta+5)=0x02;
                    #else
                        *(respuesta+5) =0x01;
                    #endif
                        ptrCRC = respuesta+4;
                        crcCalculado = genCRC16(ptrCRC,2);        
                        *(respuesta+6)= (UINT8) (crcCalculado>>8);
                        *(respuesta+7)=(UINT8) crcCalculado;
                        SendDataBuffer(respuesta,7);
                        break;
                }
                else if((*(ptrAnalizar+4))==0x04)
                {
                    ArmarTrama(respuesta,c,0x05,sync);
                    *(respuesta+5)= GETRANDOM(0,4); 
                    if((*(respuesta+5))==0)
                    {
                        *(respuesta+6)=GETRANDOM(0,255);    //NextBlock
                        *(respuesta+7)=GETRANDOM(0,255);
                        ptrCRC = respuesta+4;
                        crcCalculado = genCRC16(ptrCRC,4);
                        *(respuesta+8)= (UINT8) (crcCalculado>>8);
                    *(respuesta+9)=(UINT8) crcCalculado;
                    }
                    else
                    {
                        ptrCRC = respuesta+4;
                        crcCalculado = genCRC16(ptrCRC,2);
                        *(respuesta+6)= (UINT8) (crcCalculado>>8);
                    *(respuesta+7)=(UINT8) crcCalculado;
                    }
                        SendDataBuffer(respuesta,7);
                     break; 
                }
                else if((*(ptrAnalizar+4))==0x06)
                {
                    ArmarTrama(respuesta,c,0x07,sync);
                    *(respuesta+5)=GETRANDOM(0,255);        //Battery HSB
                    *(respuesta+6)=GETRANDOM(0,255);        //Battery LSB
                    *(respuesta+7)=GETRANDOM(0,40); //Temperature
                    ptrCRC = respuesta+4;
                    crcCalculado = genCRC16(ptrCRC,4);
                    *(respuesta+8)= (UINT8) (crcCalculado>>8);
                    *(respuesta+9)=(UINT8) crcCalculado;
                    SendDataBuffer(respuesta,7);
                    break;
                }
                else if((*(ptrAnalizar+4))==0x12)
                {
                    ArmarTrama(respuesta,c,0x13,sync);
                    #ifdef LOGINRESPONSE
                        *(respuesta+5)=0x01;
                    #else
                        *(respuesta+5) =0x02;
                    #endif
                    ptrCRC = respuesta+4;
                    crcCalculado = genCRC16(ptrCRC,2);
                    *(respuesta+6)= (UINT8) (crcCalculado>>8);
                    *(respuesta+7)=(UINT8) crcCalculado;
                    SendDataBuffer(respuesta,2);
                    break;
                }
                else if((*(ptrAnalizar+4))==0x16)
                {
                    ArmarTrama(respuesta,c,0x17,sync);
                    *(respuesta+5)=0x06;        //Battery HSB
                    for(i=0;i<16;i++)
                        *(respuesta+6+i)=GETRANDOM(0,255);        //Battery LSB
                    ptrCRC = respuesta+4;
                    crcCalculado = genCRC16(ptrCRC,18);
                    *(respuesta+22)= (UINT8) (crcCalculado>>8);
                    *(respuesta+23)=(UINT8) crcCalculado;
                    SendDataBuffer(respuesta,18);
                    break;
                }
                else if((*(ptrAnalizar+4))==0x19)
                {
                    ArmarTrama(respuesta,c,0x20,sync);
                    *(respuesta+5)=0x06;        //Battery HSB
                    for(i=0;i<10;i++)
                        *(respuesta+6+i)=GETRANDOM(0,255);        //Battery LSB
                    ptrCRC = respuesta+4;
                    crcCalculado = genCRC16(ptrCRC,4);
                    *(respuesta+16)= (UINT8) (crcCalculado>>8);
                    *(respuesta+17)=(UINT8) crcCalculado;
                    SendDataBuffer(respuesta,12);
                    break;
                }
                else if((*(ptrAnalizar+4))==0x21)
                {
                    ArmarTrama(respuesta,c,0x22,sync);
                    #ifdef RESPONSETEST
                        *(respuesta+5)=0x06;
                    #else
                        *(respuesta+5) =0x15;
                    #endif
                    ptrCRC = respuesta+4;
                    crcCalculado = genCRC16(ptrCRC,2);
                    *(respuesta+6)= (UINT8) (crcCalculado>>8);
                    *(respuesta+7)=(UINT8) crcCalculado;
                    SendDataBuffer(respuesta,2);
                    break;
                }
                else if((*(ptrAnalizar+4))==0x23)
                {
                    ArmarTrama(respuesta,c,0x24,sync);
                    #ifdef ENDRESPONSETEST
                        *(respuesta+5)=0x06;
                    #else
                        *(respuesta+5) =0x15;
                    #endif
                    ptrCRC = respuesta+4;
                    crcCalculado = genCRC16(ptrCRC,2);
                    *(respuesta+6)= (UINT8) (crcCalculado>>8);
                    *(respuesta+7)=(UINT8) crcCalculado;
                    SendDataBuffer(respuesta,2);
                    break;
                }
            }
            else
                break;
            
        }
        else
        {
            ptrAnalizar++;
        }
    }
    ptrBuffer=&buf[0];
    
}

UINT16 genCRC16(UINT8 *data, UINT16 tamano)
{
    UINT16  salida = 0;
    int     bits_leidos = 0;
    int     bit_bandera;
    
    if(data == NULL)
        return 0;
    while(tamano > 0)
    {
        bit_bandera = salida >> 15;
        
        salida <<= 1;
        salida |=  (*data >> bits_leidos)&1;
        
        bits_leidos++;
        if(bits_leidos > 7)
        {
            bits_leidos = 0;
            data++;
            tamano--;
        }
        
        if(bit_bandera)
            salida ^= CRC16;
        
    }
    int i;
    for(i=0;i<16;++i)
    {
        bit_bandera = salida >> 15;
        salida <<= 1;
        if(bit_bandera)
            salida ^=  CRC16;
        
    }
    UINT16  crc = 0;
    i = 0x8000;
    int j= 0x0001;
    for(; i!=0; i>>=1, j<<=1)
    {
        if(i&salida)
            crc |= j;
    }
    return crc;
}