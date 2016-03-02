/*
 * File:   main.c
 * Author: Paul
 *
 * Created on 09 February 2016, 23:22
 */

// FBS
#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
#pragma config RBS = NO_RAM             // Boot Segment RAM Protection (No Boot RAM)

// FSS
#pragma config SWRP = WRPROTECT_OFF     // Secure Segment Program Write Protect (Secure segment may be written)
#pragma config SSS = NO_FLASH           // Secure Segment Program Flash Code Protection (No Secure Segment)
#pragma config RSS = NO_RAM             // Secure Segment Data RAM Protection (No Secure RAM)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Primary Oscillator (XT, HS, EC) w/ PLL)
//#pragma config FNOSC = FRC
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = HS              // Primary Oscillator Source (HS Oscillator Mode)
//#pragma config POSCMD = NONE
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are enabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF            // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = OFF // Watchdog Timer Enable (Watchdog timer enabled/disabled by user software)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC1/EMUC1 and PGD1/EMUD1)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)


#define FOSC 80000000UL
#define FCY (FOSC/2)
#define BAUDRATE 2000000
#define BRGVAL ((FCY/BAUDRATE)/4)-1
#define PIXEL_BUFFER_LEN 3694
#define DMA_BUFFER_LEN 16
#define RLE_BUFFER_LEN 4000
#define DEFAULT_EXPOSURE 20

typedef unsigned char byte;
typedef unsigned short ushort;

#include <xc.h>
#include <libpic30.h>


byte pixelBuffer[PIXEL_BUFFER_LEN];
volatile byte* pPixelBuffer = pixelBuffer;
const byte* pPixelBufferEnd = pixelBuffer + PIXEL_BUFFER_LEN;
unsigned dmaBuffer[DMA_BUFFER_LEN] __attribute__((space(dma)));
byte rleBuffer[RLE_BUFFER_LEN];
unsigned rleBufferPos = 0;

void __attribute__((__interrupt__,auto_psv)) _DMA5Interrupt(void)
{        
    if(pPixelBuffer != pPixelBufferEnd)
    {       
        unsigned *pDmaBuffer = dmaBuffer;
        // Transfer DMA buffer to pixel buffer.
        int i = 0;
        for(;i < DMA_BUFFER_LEN && pPixelBuffer != pPixelBufferEnd;++i)
        {            
            *pPixelBuffer = ((*pDmaBuffer) >> 2);  
            pDmaBuffer++;
            pPixelBuffer++;        
        }
    }
    IFS3bits.DMA5IF = 0; //Clear the DMA5 Interrupt Flag
}

inline void uartWrite(byte b)
{   
    while(U1STAbits.UTXBF);
    U1TXREG = b;         
}

inline byte uartAvailable()
{
    return U1STAbits.URXDA == 1;
}

inline char uartRead()
{    
    while(!uartAvailable());
    char temp = U1RXREG;
    IFS0bits.U1RXIF = 0;
    return temp;    
}

void rleFlush(byte val, byte len)
{            
    if(len > 3)
    {
        // Encode
        rleBuffer[rleBufferPos++] = 0xFF;                
        rleBuffer[rleBufferPos++] = len;        
        rleBuffer[rleBufferPos++] = val;        
    }         
    else if(val == 0xFF)
    {
        // Escape
        if(len > 1)
        {
            // Encode
            rleBuffer[rleBufferPos++] = 0xFF;                
            rleBuffer[rleBufferPos++] = len;        
            rleBuffer[rleBufferPos++] = 0xFF;                
        }
        else    
        {            
            rleBuffer[rleBufferPos++] = 0xFF;            
            rleBuffer[rleBufferPos++] = 0xFF;                        
        }
    }
    else
    {        
        for(;len > 0; --len)
        {            
            rleBuffer[rleBufferPos++] = val;            
        }
    }        
}



int main(void) 
{    
    // Clock setup - i am not certain i am doing this correctly.
    CLKDIVbits.PLLPRE = 1; // 5 N1
    CLKDIVbits.PLLPOST = 0; // 2 N2
    PLLFBD = 22; // 20 M 
    // Wait for PLL to lock
    while(OSCCONbits.LOCK != 1);   
    
    // Setup peripheral pins.
    RPOR2bits.RP5R = 0b10010; // RP5 = OC1 / M
    RPOR3bits.RP6R = 0b10011; // RP6 = OC2 / SH
    RPOR3bits.RP7R = 0b10100; // RP7 = OC3 / ICG          
    RPOR4bits.RP8R = 0b00011; // U1TX = RP8
    RPOR5bits.RP10R = 0b00100; // UT1RTS = RP10
    RPINR18bits.U1RXR = 9;  // U1RX = RP9    
    RPINR18bits.U1CTSR = 11; // U1CTS = RP11

    // Setup UART
    U1BRG  = BRGVAL;
    U1MODE = 0;
    U1MODEbits.UARTEN = 0;
    U1MODEbits.USIDL = 0;
    U1MODEbits.IREN = 0;
    U1MODEbits.RTSMD = 0;    
    U1MODEbits.UEN = 0;
    U1MODEbits.WAKE = 0;
    U1MODEbits.LPBACK = 0;
    U1MODEbits.ABAUD = 0;
    U1MODEbits.URXINV = 0;
    U1MODEbits.BRGH = 1;
    U1MODEbits.PDSEL = 1;
    U1MODEbits.STSEL = 0;         
    
    U1STAbits.URXISEL = 0;
    U1STAbits.UTXINV = 0;
    U1STAbits.UTXBRK = 0;        
    U1STAbits.URXISEL = 1;
    U1STAbits.ADDEN = 0; 	    

    IFS0bits.U1RXIF = 0;
    IFS0bits.U1TXIF = 0;
    
    U1MODEbits.UARTEN = 1;
    U1STAbits.UTXEN = 1;    
    
    // DMA
    DMA5CONbits.AMODE = 0;
    DMA5CONbits.MODE = 0; 
    DMA5PAD = (volatile unsigned int)&ADC1BUF0; // Point DMA to ADC1BUF0
    DMA5CNT = DMA_BUFFER_LEN-1; // 32 DMA request
    DMA5REQ = 13; // Select ADC1 as DMA Request source
    DMA5STA = __builtin_dmaoffset(dmaBuffer);    
    IFS3bits.DMA5IF = 0; //Clear the DMA interrupt flag bit
    IEC3bits.DMA5IE = 1; //Set the DMA interrupt enable bit
    DMA5CONbits.CHEN=1; // Enable DMA
    
    // Setup ADC    
    AD1PCFGLbits.PCFG12 = 0; // Analog input pins - AN12 as input.
    AD1CON1bits.ADSIDL = 0; // Stop in idle - OFF.
    AD1CON1bits.AD12B = 0; // 10/12bit - 10 bit.
    AD1CON1bits.FORM = 0; // Output format - 10bit Int.
    AD1CON1bits.SSRC = 4; // Sample Clock Source - Timer 5.
    AD1CON1bits.SIMSAM = 0;
    AD1CON1bits.ASAM = 1; // ADS Auto-Start - Enabled.    
    AD1CON2bits.VCFG = 3; // VRef - Vref+(Vrefh) Vref-(Vrefl).
    AD1CON2bits.CSCNA = 0; // Input Scan - Don't scan.
    AD1CON2bits.CHPS = 0; // Channel select - CH0.
    AD1CON2bits.SMPI = 0; // ADC interrupt - After 1 conversions.
    AD1CON2bits.BUFM = 0; // Buffer fill mode - fill one half then next.
    AD1CON2bits.ALTS = 0; // Always use channel sample A.
    AD1CON3bits.ADRC = 0; // ADC Conversion Clock - Use system clock.
    AD1CON3bits.ADCS = 2; // Tad = Tcy * (ADCS+1); Tad = (25ns * 3); Tad = 75ns.
    
    AD1CHS0bits.CH0NB = 0; // ADC SampleB Neg Ref - Vref-
    AD1CHS0bits.CH0SB = 0b01100; // ADC SampleB Pos Ref - AN12
    AD1CHS0bits.CH0NA = 0; // ADC SampleA Neg Ref - Vref-
    AD1CHS0bits.CH0SA = 0b01100; // ADC SampleA Pos Ref - AN12                   
    
    // ADC - DMA
    AD1CON1bits.ADDMABM = 1; // Don't Care: ADC address generation is ignored by DMA
    AD1CON2bits.SMPI = 0; // Don't Care
    AD1CON4bits.DMABL = 0b101; // Don't Care        
    
    AD1CON1bits.ADON = 1; 
    
    // Setup timer 2 / M
    T2CONbits.TON = 0; // Disable Timer
    T2CONbits.TCS = 0; // Select internal instruction cycle clock
    T2CONbits.TGATE = 0; // Disable Gated Timer mode
    T2CONbits.TCKPS = 0b00; // Select 1:1 Prescaler
    TMR2 = 0; // Clear timer register
    PR2 = 19; // Load the period value, period seems to be -1.
    
    // Setup timer 3 / SH
    T3CONbits.TON = 0;
    T3CONbits.TCS = 0;
    T3CONbits.TGATE = 0;
    T3CONbits.TCKPS = 0b00;
    TMR3 = 0;
    PR3 = 79;
    
    // Setup timer 5 / ADC
    T5CONbits.TON = 0;
    T5CONbits.TCS = 0;
    T5CONbits.TGATE = 0;
    T5CONbits.TCKPS = 0b00;
    TMR5 = 0;
    PR5 = 39;
    
    // Setup OC1/M
    OC1R = 0;  
    OC1RS = 10;
    OC1CONbits.OCTSEL = 0; // Timer 2
    OC1CONbits.OCM = 5; // Dual Compare + Enabled.
    
    // Setup OC2/SH    
    OC2R = 0;         
    OC2RS = 40;
    OC2CONbits.OCTSEL = 1; // Timer 3
    OC2CONbits.OCM = 5; // Dual Compare + Enabled.    
    
    // Setup OC3/ICG
    // Note: We use Timer 2 so when we flip the OCM (1/2) our rise/fall is in sync with M perfectly.
    OC3R = 0;         
    OC3CONbits.OCTSEL = 0; // Timer 2             
    OC3CONbits.OCM = 1; // Enable
    
    // Start timers.
    // If you want to get really perfect, adjust TMR* registers before starting.    
    T2CONbits.TON = 1; // M Timer    
    T3CONbits.TON = 1; // SH Timer
    T5CONbits.TON = 1; // ADC Timer.       

    byte exposure = DEFAULT_EXPOSURE;
    while(1)
    {   
        if(uartAvailable())
        {
            char uartRxByte = uartRead();            
            if(uartRxByte == 'X')
            {                
                exposure = (byte)uartRead();
                uartWrite(exposure);
            } 
            else if(uartRxByte == 'C')
            {       
                //int gap = 0;
                int i=1;
                byte rleValue = pixelBuffer[0];
                byte rleLength = 1;                    
                rleBufferPos = 0;
                for(; i < PIXEL_BUFFER_LEN;++i)   
                {                                  
                    // A very basic lossy RLE modification.
                    //gap = abs(buffer[i] - buffer[i-1]);                                                           
                    //if(buffer[i] != runValue)
                       // if(gap > 3)
                    if(pixelBuffer[i] != rleValue)
                    {                        
                        rleFlush(rleValue, rleLength);
                        rleValue = pixelBuffer[i];
                        rleLength = 1;
                        continue;
                    }
                    else
                    {
                        ++rleLength;                    
                        if(rleLength > 254)
                        {                                                
                            rleFlush(rleValue, 254);                        
                            rleLength -= 254;                       
                        }
                    }
                }
                // We always have at least 1 value in the run at the end.
                rleFlush(rleValue, rleLength);
               
                // Write the rle buffer length.
                uartWrite(rleBufferPos & 0xFF);
                uartWrite((rleBufferPos >> 8) & 0xFF);
                // Write the rle buffer.
                i=0; 
                for(; i < rleBufferPos;++i)   
                    uartWrite(rleBuffer[i]);
            }             
        }          
        
        // Wait for SH about to transition high.  
        // This is a bit dirty, and requires tuning if clock speeds change.
        while(TMR3 < 50);                        
        // Expose
        OC3CONbits.OCM = 2; // ICG LOW         
        int c = exposure;
        while(--c)        
            __delay_us(2);        
        // Begin data reading.
        OC3CONbits.OCM = 1; // ICG HIGH                   
        // Reset the pixel buffer pointer so DMA interrupt will save data.
        pPixelBuffer = pixelBuffer;         
        // Enable DMA
        DMA5CONbits.CHEN = 1;        
        // Wait until all pixels are read.
        while(pPixelBuffer != pPixelBufferEnd); 
        // Disable DMA, we're done, we have a frame.
        DMA5CONbits.CHEN = 0;                              
    }
    return 0;
}
