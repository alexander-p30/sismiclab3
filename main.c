#include <msp430.h>
#include <stdint.h>

#define LED_RED_ON        (P1OUT |= BIT0)
#define LED_RED_OFF       (P1OUT &= ~BIT0)
#define LED_RED_TOGGLE    (P1OUT ^= BIT0)

#define LED_GREEN_ON      (P4OUT |= BIT7)
#define LED_GREEN_OFF     (P4OUT &= ~BIT7)
#define LED_GREEN_TOGGLE  (P4OUT ^= BIT7)

// UCB0SDA = P3.0; UCB0SCL = P3.1

// I2C STUFF

void initI2C();
void I2CSendByte(unsigned char address, unsigned char data);

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

char number[17] = { 'A', A1 + '0', '=', '0', ',', '0', '0', '0', 'V', ' ', '0',
                    'x', '0', '0', '0', '0', '\0' };
char bar[17] = { ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
                 ' ', ' ', ' ', ' ', '\0' };

#define MAX_ADC_MEASURE 4096

void delay_us(unsigned int time_us, unsigned int loop_count);

#define TRUE 1
#define FALSE 0

int sw_pressed = 0, ready = 1, fraction = 0, flag = 0, debounce = 0;

unsigned char buffer[2][16] = {
        { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }, { 0, 0, 0, 0, 0, 0,
                                                              0, 0, 0, 0, 0, 0,
                                                              0, 0, 0, 0 } };

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
char* numberToStr(char *numberStr, int voltFraction, int measurementAvg,
                  measure_mode currentMode);
char* computeBar(char *bar, int number);
void updateRedLedPWM(measure_mode currentMode, int voltFraction);

int main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

    initBuffer(&xTBuf);
    initBuffer(&yTBuf);
    initBuffer(&ldr1TBuf);
    initBuffer(&ldr2TBuf);
    updateCurrentMeasurementBuffer(&currentMeasurementBuffer, currentMode);
    initADC();
    initI2C();
    configButtons();
    confRefreshRate();
    initLCD();
    sendByte(TRUE, LCD_INSTR.CLEAR);
    registerBlockChars();

    __enable_interrupt();

    // REFRESH RATE PIN
    P2DIR |= BIT0;
    P2SEL |= BIT0;
    P2REN |= BIT0;

    // PWM PIN
    P2DIR |= BIT4;
    P2SEL |= BIT4;

    // LED VERMELHO
    P1SEL &= ~BIT0;
    P1DIR |= BIT0;
    P1OUT &= ~BIT0;

    // LED VERDE
    P4SEL &= ~BIT7;
    P4DIR |= BIT7;
    P4OUT &= ~BIT7;

    LED_RED_OFF;
    LED_GREEN_OFF;
    while (TRUE) {
        if (flag) {
            flag = 0;
            numberToStr(number, fraction / 10, currentMeasurementBuffer->avg,
                        currentMode);
            computeBar(bar, fraction);
            buildBuffer(&buffer, number, bar);
            flushBuffer(buffer);
            updateRedLedPWM(currentMode, fraction);
        }

        if (sw_pressed && ready) {
            if (currentMode < A6) currentMode++;
            else currentMode = A1;
            updateCurrentMeasurementBuffer(&currentMeasurementBuffer,
                                           currentMode);
            sw_pressed = 0;
            ready = 0;
        }
    }

    return 0;
}

void updateRedLedPWM(measure_mode currMode, int voltFraction) {
    TA2CTL = TACLR;

    if (currMode < A5) {
        return;
    }

    float multiplier;

    if (voltFraction < 11000) {
        multiplier = 0.05;
    } else if (voltFraction < 22000) {
        multiplier = 0.5;
    } else {
        multiplier = 0.95;
    }

    TA2CTL = TASSEL__SMCLK | MC__UP | TACLR;
    TA2CCR0 = 10000;
    TA2CCR1 = 10000 * multiplier;
    TA2CCTL1 = OUTMOD_2;
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

void initI2C() {
    // Desliga o modulo
    UCB0CTL1 |= UCSWRST;

    //Configura os pinos
    P3SEL |= BIT0;
    P3SEL |= BIT1;

    UCB0CTL0 = UCMST | UCMODE_3 | UCSYNC;

    UCB0CTL1 = UCSSEL__SMCLK | UCTR | UCSWRST;

    // Configura BR
    UCB0BR0 = 2;
    UCB0BR1 = 0;

    // Liga o modulo
    UCB0CTL1 &= ~UCSWRST;
}

void I2CSendByte(unsigned char address, unsigned char data) {
    UCB0IE = 0;

    UCB0I2CSA = address;

    if (UCB0STAT & UCBBUSY) return;

    UCB0CTL1 |= UCTXSTT;

    while ((UCB0IFG & UCTXIFG) == 0);

    UCB0TXBUF = data;

    while (UCB0CTL1 & UCTXSTT);

    UCB0CTL1 |= UCTXSTP;

    return;
}

void sendNibble(uint8_t isInstruction, unsigned char nibble) {
    unsigned char byte = 0x00;
    byte = (nibble & 0x0F) << 4;
    byte |= BIT3;

    if (!isInstruction) {
        byte |= BIT0;
    }

    I2CSendByte(LCD_ADDR.PERIPHERAL, byte);
    delay_us(300, 1);
    I2CSendByte(LCD_ADDR.PERIPHERAL, byte | BIT2);
    delay_us(600, 1);
    I2CSendByte(LCD_ADDR.PERIPHERAL, byte);
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

void confRefreshRate() {
    TA1CTL = TASSEL__SMCLK | MC__UP;
    TA1CCTL0 = CCIE;
    TA1CCR0 = 32768 >> 2;

    TB0CTL = TBSSEL__ACLK | ID__1 | MC__UP | TBCLR;
    TB0CCTL1 = OUTMOD_2;

    const int ticksToRefresh = 32768 >> 4;
    TB0CCR0 = ticksToRefresh;
    TB0CCR1 = ticksToRefresh >> 1;
}

const char hexDigits[16] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
                             'A', 'B', 'C', 'D', 'E', 'F' };
const char numericalTemplate[17] = { 'A', A1 + '0', '=', '0', ',', '0', '0',
                                     '0', 'V', ' ', '0', 'x', '0', '0', '0',
                                     '0', '\0' };
const char qualitativeTemplate[17] = { 'A', A1 + '0', ':', ' ', ' ', ' ', ' ',
                                       ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
                                       ' ', '\0' };
const char level1[] = "ILUMINADO";
const char level2[] = "LUSCO-FUSCO";
const char level3[] = "ESCURO";
char* numberToStr(char *numberStr, int voltFraction, int measurementAvg,
                  measure_mode currentMode) {
    uint8_t i = 0;
    const char *currentLevelText;
    if (currentMode >= A5) {
        for (i = 0; i < 16; i++)
            numberStr[i] = qualitativeTemplate[i];

        if (voltFraction < 1100) {
            currentLevelText = level1;
        } else if (voltFraction < 2200) {
            currentLevelText = level2;
        } else {
            currentLevelText = level3;
        }

        for (i = 3; *currentLevelText; i++, currentLevelText++)
            numberStr[i] = *currentLevelText;

    } else {
        for (i = 0; i < 16; i++)
            numberStr[i] = numericalTemplate[i];

        int digit;
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
    }
    numberStr[1] = currentMode + '0';

    return numberStr;
}

#pragma vector = ADC12_VECTOR
__interrupt void ADC12_interrupt(void) {
    switch (_even_in_range(ADC12IV, 0x24)) {
    case ADC12IV_ADC12IFG4:       //MEM4
        pushToBuffer(&xTBuf, ADC12MEM0);
        updateBufferAvg(&xTBuf);

        pushToBuffer(&yTBuf, ADC12MEM1);
        updateBufferAvg(&yTBuf);

        pushToBuffer(&ldr1TBuf, ADC12MEM2);
        updateBufferAvg(&ldr1TBuf);

        pushToBuffer(&ldr2TBuf, ADC12MEM3);
        updateBufferAvg(&ldr2TBuf);

        fraction = currentMeasurementBuffer->avg
                * (33000 / (float) MAX_ADC_MEASURE);

        ready = ready || ADC12MEM4 > 500;
        sw_pressed = (sw_pressed || ADC12MEM4 < 500) && ready;
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
    P6SEL |= BIT5;
    P6REN |= BIT5;

    // Desliga o ADC
    ADC12CTL0 &= ~ADC12ENC;

    ADC12CTL0 = ADC12SHT0_3 | ADC12ON;

    ADC12CTL1 = ADC12CSTARTADD_0 | ADC12SHS_3 | ADC12SHP | ADC12DIV_0 | ADC12SSEL_0 | ADC12CONSEQ_3;

    ADC12CTL2 = ADC12TCOFF | ADC12RES_2;

    ADC12MCTL0 = ADC12SREF_0 | ADC12INCH_1; // A1 == P6.1
    ADC12MCTL1 = ADC12SREF_0 | ADC12INCH_2; // A2 == P6.2
    ADC12MCTL2 = ADC12SREF_0 | ADC12INCH_3; // A3 == P6.3
    ADC12MCTL3 = ADC12SREF_0 | ADC12INCH_4; // A4 == P6.4
    ADC12MCTL4 = ADC12SREF_0 | ADC12EOS | ADC12INCH_5;

    // Liga as interrupcoes
    ADC12IE = ADC12IE4;

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

#pragma vector = TIMER1_A0_VECTOR
__interrupt void timer_a1_isr() {
    switch (TA1IV) {
    default:
        flag = 1;
        P2OUT ^= BIT0;
        break;
    }
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
