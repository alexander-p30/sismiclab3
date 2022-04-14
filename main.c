#include <msp430.h>
#include <stdint.h>

#define LED_RED_ON        (P1OUT |= BIT0)
#define LED_RED_OFF       (P1OUT &= ~BIT0)
#define LED_RED_TOGGLE    (P1OUT ^= BIT0)

#define LED_GREEN_ON      (P4OUT |= BIT7)
#define LED_GREEN_OFF     (P4OUT &= ~BIT7)
#define LED_GREEN_TOGGLE  (P4OUT ^= BIT7)

// UCB0SDA = P3.0; UCB0SCL = P3.1

// LCD STUFF

void initLCD();
void initADC();
void sendNibble(uint8_t isInstruction, unsigned char nibble);
void sendByte(uint8_t isInstruction, unsigned char byte);
void registerCharacter(unsigned char charMapping[8], uint8_t CGRamAddress);
void newChar();
void moveCursor(uint8_t line, uint8_t col);
void configButtons();
void buildBuffer(unsigned char buffer[2][16], char *line0, char *line1);
void flushBuffer(unsigned char buffer[2][16]);
void confRefreshRate();

// I2C STUFF

void initialize_I2C_UCB0_MasterTransmitter();
void master_TransmitOneByte(unsigned char address, unsigned char data);

// TransientBuffer
#define BUFFER_SIZE 4

typedef struct {
    unsigned long idx;
    int16_t data[BUFFER_SIZE];
    int16_t avg;
} TransientBuffer;

TransientBuffer xTBuf, yTBuf, ldr1TBuf, ldr2TBuf;
TransientBuffer *currentMeasurementBuffer = &xTBuf;

void initBuffer(TransientBuffer *buf);
void pushToBuffer(TransientBuffer *buf, int16_t val);
int16_t updateBufferAvg(TransientBuffer *buf);

// UTILS

const struct {
    uint8_t SET_CGRAM_ADDR;
    uint8_t SET_DDRAM_ADDR;
    uint8_t SET_ENTRY_MODE;
    uint8_t RETURN_HOME;
    uint8_t CLEAR;
} LCD_INSTR = { .SET_CGRAM_ADDR = 0b1000000, .SET_DDRAM_ADDR = 0b10000000,
                .SET_ENTRY_MODE = 0b100, .RETURN_HOME = 0b10, .CLEAR = 0b1 };

const struct {
    uint8_t PERIPHERAL;
    uint8_t LINE0;
    uint8_t LINE1;
    uint8_t CGRAM_LIMIT;
} LCD_ADDR =
        { .PERIPHERAL = 0x27, .LINE0 = 0, .LINE1 = 0x40, .CGRAM_LIMIT = 0x7 };

typedef enum {
    A1 = 1, A2 = 2, A3 = 3, A4 = 4, A5 = 5, A6 = 6
} measure_mode;

measure_mode currentMode = A1;

void updateCurrentMeasurementBuffer(TransientBuffer **msrBuf, measure_mode mod);

char number[17] = { 'A', A1 + '0', '=', '0', '.', '0', '0', '0', 'V', ' ', '0',
                    'x', '0', '0', '0', '0', '\0' };
char bar[17] = { ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
                 ' ', ' ', ' ', ' ', '\0' };

#define MAX_ADC_MEASURE 4096

void delay_us(unsigned int time_us, unsigned int loop_count);
void debounceS1();
void debounceS2();

#define TRUE 1
#define FALSE 0

int sw_pressed = 0, fraction = 0;

unsigned char buffer[2][16] = {
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0,
                                                              0, 0, 0, 0, 0, 0,
                                                              0, 0, 0, 0 } };

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    initBuffer(&xTBuf);
    initADC();
    initialize_I2C_UCB0_MasterTransmitter();
    configButtons();
    confRefreshRate();
    initLCD();
    sendByte(TRUE, LCD_INSTR.CLEAR);
    registerBlockChars();

    __enable_interrupt();

    //LED VERMELHO
    P1SEL &= ~BIT0;
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    //LED VERDE
    P4SEL &= ~BIT7;
    P4DIR |= BIT7;
    P4OUT &= ~BIT7;

    LED_RED_OFF;
    LED_GREEN_OFF;

    while (TRUE) {
        if (sw_pressed) {
            if (currentMode < A6) currentMode++;
            else currentMode = A1;
            updateCurrentMeasurementBuffer(&currentMeasurementBuffer,
                                           currentMode);
            number[1] = currentMode + '0';
            debounceS2();
            sw_pressed = 0;
        }
    }

    return 0;
}

void updateCurrentMeasurementBuffer(TransientBuffer **msrBuf, measure_mode mod) {
    switch (mod) {
    case A1:
        *msrBuf = &xTBuf;
        break;
    case A2:
        *msrBuf = &yTBuf;
        break;
    case A3:
        *msrBuf = &ldr1TBuf;
        break;
    case A4:
        *msrBuf = &ldr2TBuf;
        break;
    case A5:
        *msrBuf = &ldr1TBuf;
        break;
    case A6:
        *msrBuf = &ldr2TBuf;
        break;
    }

}

void registerBlockChars() {
    uint8_t i = 0;
    uint8_t line = 0b00000000;
    const uint8_t cursor = 0b10000;

    for (i = 0; i < 5; i++) {
        line = line | (cursor >> i);
        char blockCharacter[] =
                { line, line, line, line, line, line, line, line };
        registerCharacter(blockCharacter, i + 1);
    }
}

void debounceS2() {
    delay_cycles(10);
    do {
        P2IFG = 0;
    } while (P2IFG != 0);
    P2IE |= BIT1;
}

void buildBuffer(unsigned char buffer[2][16], char *line0, char *line1) {
    unsigned int line = 0, col;

    for (col = 0; col < 16; col++) {
        if (*line0) {
            buffer[line][col] = *line0;
            line0++;
        } else {
            buffer[line][col] = ' ';
        }
    }
    line = 1;
    for (col = 0; col < 16; col++) {
        if (*line1) {
            buffer[line][col] = *line1;
            line1++;
        } else {
            buffer[line][col] = ' ';
        }
    }
}

void initialize_I2C_UCB0_MasterTransmitter() {
    //Desliga o módulo
    UCB0CTL1 |= UCSWRST;

    //Configura os pinos
    P3SEL |= BIT0;
    P3SEL |= BIT1;

    UCB0CTL0 = UCMST |           //Master Mode
            UCMODE_3 |    //I2C Mode
            UCSYNC;         //Synchronous Mode

    UCB0CTL1 = UCSSEL__SMCLK |    //Clock Source: ACLK
            UCTR |                      //Transmitter
            UCSWRST;             //Mantém o módulo desligado

    //Divisor de clock para o BAUDRate
    UCB0BR0 = 2;
    UCB0BR1 = 0;

    //Liga o módulo.
    UCB0CTL1 &= ~UCSWRST;
}

void master_TransmitOneByte(unsigned char address, unsigned char data) {
    //Desligo todas as interrupções
    UCB0IE = 0;

    //Coloco o slave address
    UCB0I2CSA = address;

    //Espero a linha estar desocupada.
    if (UCB0STAT & UCBBUSY) return;

    //Peço um START
    UCB0CTL1 |= UCTXSTT;

    //Espero até o buffer de transmissão estar disponível
    while ((UCB0IFG & UCTXIFG) == 0);

    //Escrevo o dado
    UCB0TXBUF = data;

    //Aguardo o acknowledge
    while (UCB0CTL1 & UCTXSTT);

    //Verifico se é um ACK ou um NACK
    if ((UCB0IFG & UCNACKIFG) != 0) {
        //Peço uma condição de parada
        LED_RED_ON;
        UCB0CTL1 |= UCTXSTP;
    } else {
        //Peço uma condição de parada
        LED_GREEN_ON;
        UCB0CTL1 |= UCTXSTP;
    }

    return;
}

void sendNibble(uint8_t isInstruction, unsigned char nibble) {
    unsigned char byte = 0x00;
    byte = (nibble & 0x0F) << 4;
    byte |= BIT3;

    if (!isInstruction) {
        byte |= BIT0;
    }

    master_TransmitOneByte(LCD_ADDR.PERIPHERAL, byte);
    delay_us(300, 1);
    master_TransmitOneByte(LCD_ADDR.PERIPHERAL, byte | BIT2);
    delay_us(600, 1);
    master_TransmitOneByte(LCD_ADDR.PERIPHERAL, byte);
    delay_us(300, 1);
}

void sendByte(uint8_t isInstruction, unsigned char byte) {
    unsigned char nibble = ((byte >> 4) & 0x0F);
    sendNibble(isInstruction, nibble);
    nibble = (byte & 0x0F);
    sendNibble(isInstruction, nibble);
}

void registerCharacter(unsigned char characterMapping[8], uint8_t CGRamAddress) {
    // Little endian to big endian
    CGRamAddress = ((CGRamAddress & 0b000111) << 3) | (CGRamAddress >> 3);
    sendByte(TRUE, LCD_INSTR.SET_CGRAM_ADDR | CGRamAddress);

    uint8_t i = 0;
    for (i = 0; i < 8; i++)
        sendByte(FALSE, characterMapping[i]);

    sendByte(TRUE, LCD_INSTR.SET_DDRAM_ADDR | 0);
}

void initLCD() {
    sendNibble(TRUE, 0x03);
    sendNibble(TRUE, 0x03);
    sendNibble(TRUE, 0x03);
    sendNibble(TRUE, 0x02);

    sendByte(TRUE, 0x28);
    sendByte(TRUE, 0x08);
    sendByte(TRUE, 0x01);
    sendByte(TRUE, 0x06);
    sendByte(TRUE, 0x0F);
}

void flushBuffer(unsigned char buffer[2][16]) {
    sendByte(TRUE, LCD_INSTR.RETURN_HOME);
    uint8_t i, j;
    for (i = 0; i < 2; i++) {
        for (j = 0; j < 16; j++)
            sendByte(FALSE, buffer[i][j]);

        sendByte(TRUE, LCD_INSTR.SET_DDRAM_ADDR | LCD_ADDR.LINE1);
    }
}

void configButtons() {
    P2SEL &= ~BIT1;
    P2DIR &= ~BIT1;
    P2REN |= BIT1;
    P2OUT |= BIT1;

    P2IE |= BIT1;
    P2IES |= BIT1;

    do {
        P2IFG = 0;
    } while (P2IFG != 0);
}

#pragma vector = PORT2_VECTOR;
__interrupt void s1_isr(void) {
    switch (P2IV) {
    case P2IV_P2IFG1:
        P2IE &= ~BIT1;
        sw_pressed = 1;
        break;
    default:
        break;
    }
}

void confRefreshRate() {
    TB0CTL = TBSSEL__ACLK | ID__1 | MC__UP | TBCLR;
    TB0CCTL0 = CCIE;
    TB0CCTL1 = OUTMOD_2;

    int ticksToRefresh = 32768 >> 2;
    TB0CCR0 = ticksToRefresh;
    TB0CCR1 = ticksToRefresh >> 4;
}

const char hexDigits[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                             'A', 'B', 'C', 'D', 'E', 'F' };
unsigned char* numberToStr(char *numberStr, int voltFraction,
                           int measurementAvg) {
    int digit;
    uint8_t i = 0;
    for (i = 0; i < 4; i++) {
        numberStr[15 - i] = hexDigits[(measurementAvg & 0b1111)];
        measurementAvg >>= 4;
    }

    digit = voltFraction % 10;
    numberStr[7] = digit + '0';
    voltFraction = (voltFraction - digit) / 10;
    digit = voltFraction % 10;
    numberStr[6] = digit + '0';
    voltFraction = (voltFraction - digit) / 10;
    digit = voltFraction % 10;
    numberStr[5] = digit + '0';
    voltFraction = (voltFraction - digit) / 10;
    digit = voltFraction % 10;
    numberStr[3] = digit + '0';

    return numberStr;
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_interrupt(void) {
    switch (_even_in_range(ADC12IV, 0x24)) {
    case ADC12IV_ADC12IFG0:       //MEM0
        pushToBuffer(&xTBuf, ADC12MEM0);
        fraction = updateBufferAvg(currentMeasurementBuffer)
                * (33000 / (float) MAX_ADC_MEASURE);
        break;
    case ADC12IV_ADC12IFG1:       //MEM1
        pushToBuffer(&yTBuf, ADC12MEM1);
        break;
    case ADC12IV_ADC12IFG2:       //MEM2
        pushToBuffer(&ldr1TBuf, ADC12MEM2);
        break;
    case ADC12IV_ADC12IFG3:       //MEM3
        pushToBuffer(&ldr2TBuf, ADC12MEM3);
        break;
    default:
        break;
    }
}

void initADC() {
    P6SEL |= BIT1;
    P6SEL |= BIT2;
    P6SEL |= BIT3;
    P6SEL |= BIT4;

    // Desliga o ADC
    ADC12CTL0 &= ~ADC12ENC;

    ADC12CTL0 = ADC12SHT0_3 | ADC12ON;

    ADC12CTL1 = ADC12CSTARTADD_0 |
    ADC12SHS_3 |              // Trigger B0.1
            ADC12SHP |
            ADC12DIV_0 |
            ADC12SSEL_0 |             // MODCLK == 4.8 MHz
            ADC12CONSEQ_3;            // autoscan repeat

    ADC12CTL2 = ADC12TCOFF | ADC12RES_2;

    ADC12MCTL0 = ADC12SREF_0 | ADC12INCH_1; // A1 == P6.1
    ADC12MCTL1 = ADC12SREF_0 | ADC12INCH_2; // A2 == P6.2
    ADC12MCTL2 = ADC12SREF_0 | ADC12INCH_3; // A3 == P6.3
    ADC12MCTL3 = ADC12SREF_0 | ADC12EOS | ADC12INCH_4; // A4 == P6.4

    // Liga as interrupcoes
    ADC12IE = ADC12IE0 | ADC12IE1 | ADC12IE2 | ADC12IE3;

    // Liga o ADC
    ADC12CTL0 |= ADC12ENC;
}

char* computeBar(char *bar, int number) {
    uint8_t barFraction = (number / 3300.) * 16;
    uint8_t partialBar = barFraction % 10;

    uint8_t i = 0;
    for (i = 0; i < barFraction / 10; i++) {
        bar[i] = 5;
    }

    if (partialBar) bar[i++] = (partialBar / 2);

    for (i; i < 16; i++) {
        bar[i] = ' ';
    }

    return bar;
}

#pragma vector = TIMER0_B0_VECTOR
__interrupt void timer_a0_isr() {
    switch (TB0IV) {
    default:
        numberToStr(number, fraction / 10, currentMeasurementBuffer->avg);
        computeBar(bar, fraction);
        buildBuffer(&buffer, number, bar);
        flushBuffer(buffer);
        break;
    }
}

void delay_cycles(unsigned int loop_count) {
    while (loop_count--)
        __delay_cycles(50000);
}

void delay_us(unsigned int time_us, unsigned int loop_count) {
    while (loop_count--) {
        TA0CCR0 = time_us;
        TA0CTL = TASSEL__SMCLK | ID__1 | MC_1 | TACLR;

        while ((TA0CTL & TAIFG) == 0);

        TA0CTL = MC_0 | TACLR;
    }
}

void initBuffer(TransientBuffer *buf) {
    buf->idx = 0;
    buf->avg = 0;

    unsigned long i = 0;
    for (i = 0; i < BUFFER_SIZE; i++)
        buf->data[i] = 2048;
}

void pushToBuffer(TransientBuffer *buf, int16_t val) {
    unsigned long currentIndex = buf->idx;
    buf->data[currentIndex] = val;
    buf->idx = (currentIndex + 1) & 0b11;
}

int16_t updateBufferAvg(TransientBuffer *buf) {
    unsigned int i;
    int sum = 0;

    for (i = 0; i < BUFFER_SIZE; i++)
        sum += buf->data[i];

    buf->avg = sum >> 2;

    return buf->avg;
}