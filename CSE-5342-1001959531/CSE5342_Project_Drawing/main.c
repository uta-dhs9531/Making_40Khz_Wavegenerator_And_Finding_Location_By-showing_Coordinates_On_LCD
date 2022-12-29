/*
 S0_X=readEeprom(0);
 S0_Y=readEeprom(1);
 S1_X=readEeprom(2);
 S1_Y=readEeprom(3);
 S2_X=readEeprom(4);
 S2_Y=readEeprom(5);

 irstatus on/off readEeprom(6);
 pwmFreq=readEeprom(7);   //Ir pwm freq   //ir=0, error=1,  beep 2 =2, beep 3 =3, beep fix = 4   // RUN POGRAM
 pwmPeriod=readEeprom(8);  //ir pwmperiod

 errorstatus on/off readEeprom(9);
 pwmFreq=readEeprom(10);   //error pwm freq
 pwmPeriod=readEeprom(11);  //error pwmperiod

 beep2status on/off readEeprom(12);
 pwmFreq=readEeprom(13);   //beep2 pwm freq
 pwmPeriod=readEeprom(14);  //beep2 pwmperiod

 beep3status on/off readEeprom(15);
 pwmFreq=readEeprom(16);   //beep3 pwm freq
 pwmPeriod=readEeprom(17);  //beep3 pwmperiod

 averageN=readEeprom(18);

 s0k0=readEeprom(19);
 s0k1=readEeprom(20);
 s1k0=readEeprom(21);
 s1k1=readEeprom(22);
 s2k0=readEeprom(23);
 s2k1=readEeprom(24);

 variance=readEeprom(25);

 fix status on/off readEeprom(26);
 pwmFreq=readEeprom(27);   //fix pwm freq
 pwmPeriod=readEeprom(28);  //fix pwmperiod


 */

#include <stdint.h>
#include <stdio.h> //itoa
#include <stdbool.h>
#include <stdlib.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "eeprom.h"
#include "gpio.h"
#include "nvic.h"
#include "wait.h"
#include "i2c0.h"
#include "i2c0_lcd.h"

//Macros
#define MAX_CHARS 80
#define MAX_FIELDS 10
#define CHAR_INTERFACE 50
#define BEEP_BUFFER_LENGTH 50
#define AVERAGE_BUFFER_MAX_LENGTH 50

#define ADDRESSI2C                   (0x20)

// PortB masks
#define SDA_MASK 8
#define SCL_MASK 4

// Pins
#define I2C0SCL PORTB,2
#define I2C0SDA PORTB,3

float S0_X;
float S0_Y;
float S1_X;
float S1_Y;
float S2_X;
float S2_Y;
float Ux1;
float Ux2;
float Uy1;
float Uy2;
float ka;
float kb;
float kc;
float k1;
float k2;
float x;
float y;
float s0k0;
float s0k1;
float s0c1;
float s1k0;
float s1k1;
float s1c1;
float s2k0;
float s2k1;
float s2c1;
float sensorN;
float count;
uint32_t ultrasonic0C;
uint32_t ultrasonic1C;
uint32_t ultrasonic2C;
uint32_t beepM;
uint32_t pwmfreq;
uint32_t pwmperiod;
uint32_t beepWriteIndex;
uint32_t beepReadIndex;
uint32_t v0;
uint32_t v1 = 0;
uint32_t v2;
//uint32_t y;
uint32_t v = 0;
uint16_t NodSensor;
uint8_t reseteverything=0;
uint32_t n;
uint32_t averageWriteIndex0;
uint32_t averageWriteIndex1;
uint32_t averageWriteIndex2;

uint32_t averageN;
uint32_t variance;
bool UL0;
bool UL1;
bool UL2;
bool primepump;
uint32_t avgValue0;
uint32_t avgValue1;
uint32_t avgValue2;
char strm[CHAR_INTERFACE];         //Char string to print on interface
uint32_t str[CHAR_INTERFACE];
uint32_t beepBuffer[BEEP_BUFFER_LENGTH];
uint32_t sensor0AveBuffer[AVERAGE_BUFFER_MAX_LENGTH];
uint32_t sensor1AveBuffer[AVERAGE_BUFFER_MAX_LENGTH];
uint32_t sensor2AveBuffer[AVERAGE_BUFFER_MAX_LENGTH];

uint32_t v0sensor0AveBuffer[AVERAGE_BUFFER_MAX_LENGTH];
uint32_t v1sensor1AveBuffer[AVERAGE_BUFFER_MAX_LENGTH];
uint32_t v2sensor2AveBuffer[AVERAGE_BUFFER_MAX_LENGTH];

uint32_t borderplane[100][100];
uint32_t x1[100];
uint32_t y1[100];
bool planeflag;

typedef struct _USER_DATA
{
    char buffer[MAX_CHARS + 1];
    uint8_t fieldCount;
    uint8_t fieldPosition[MAX_FIELDS];
    char fieldType[MAX_FIELDS];
} USER_DATA;

USER_DATA msg;

bool strcmp1(char *str, const char strCommand[])
{
    bool comp = 1;
    uint32_t i = 0;
    while (strCommand[i] != '\0')
    {
        if (*str != strCommand[i])
        {
            comp = 0;
            break;
        }
        i++;
        str++;
    }
    return comp;
}

bool isCommand(USER_DATA *data, const char strCommand[], uint8_t minArguments)
{
    uint8_t k = 0;
    uint8_t i = data->fieldPosition[0];
    uint16_t arg = 0;
    bool validation;
    while (data->buffer[i] != '\0' || strCommand[i] != '\0')
    {
        if (strCommand[k] != data->buffer[i])
        {
            validation = 0;
            break;
        }
        else
        {
            validation = 1;
        }
        k++;
        i++;
    }
    k = 1;
    if (validation == 1)
    {
        if (strCommand == "sensor" || strCommand == "reset"
                || strCommand == "beep")
        {
            while (k <= MAX_FIELDS)
            {
                if (data->fieldType[k] == 'n' || data->fieldType[k] == 'a')
                {
                    arg++;

                }
                k++;
            }
            if (data->fieldType[1] == 'a' && data->fieldType[2] == 'n'
                    && data->fieldType[3] == 'a')
            {
                arg = arg - 1;
            }
            if (arg == minArguments) //Check for >= arguments
            {
                validation = 1;
            }
            else
            {
                validation = 0;
            }
        }

        else if (strCommand == "sensor" || strCommand == "reset"
                || strCommand == "beep")
        {
            while (k <= MAX_FIELDS)
            {
                if (data->fieldType[k] == 'a')
                {
                    arg++;
                }
                k++;
            }
            if (arg == minArguments) //Check for >= arguments
            {
                validation = 1;
            }
            else
            {
                validation = 0;
            }
        }
    }
    return validation;
}

int32_t getFieldInteger(USER_DATA *data, uint8_t fieldNumber)
{
    uint32_t prev = 0;
    uint32_t s = 0;
    uint32_t i = data->fieldPosition[fieldNumber];
    if (data->fieldType[fieldNumber] == 'n')
    {
        while (data->buffer[i] != '\0')
        {
            s = (data->buffer[i] - '0') + (prev * 10);
            prev = s;
            i++;
        }
        return s;
    }
    else
    {
        return 0;
    }
}



void parseFields(USER_DATA *data)
{
    uint16_t i = 0;
    uint16_t j = 0;
    uint8_t f = 3;
    data->fieldCount = 0;
    for (i = 0; i < MAX_FIELDS; i++)
    {
        data->fieldPosition[i] = '\0';
        data->fieldType[i] = '\0';
    }
    i = 0;
    while (data->buffer[i] != '\0')
    {
        if (data->fieldCount < MAX_FIELDS)
        {
            if ((data->buffer[i] >= 65 && data->buffer[i] <= 90)
                    || (data->buffer[i] >= 97 && data->buffer[i] <= 122))
            {
                if (f != 1)
                {
                    data->fieldPosition[data->fieldCount] = i;
                    data->fieldType[data->fieldCount] = 'a';
                    data->fieldCount++;
                    f = 1;
                }
            }
            else if (data->buffer[i] >= 48 && data->buffer[i] <= 57)
            {
                if (f != 2)
                {
                    data->fieldPosition[data->fieldCount] = i;
                    data->fieldType[data->fieldCount] = 'n';
                    data->fieldCount++;
                    f = 2;
                }
            }
            else
            {
                if (f != 3)
                {
                    f = 3;
                }
            }
        }
        else
        {
            break;
        }
        i = i + 1;
    }
    for (j = 0; j <= MAX_CHARS; j++)
    {
        if ((data->buffer[j] <= 47)
                || (data->buffer[j] >= 58 && data->buffer[j] <= 64)
                || (data->buffer[j] >= 123 && data->buffer[j] <= 127))
        {
            data->buffer[j] = '\0';
        }
    }
}

void getsUart0(USER_DATA *data)
{
    uint32_t count = 0;
    char c;
    getc: ;
    c = getcUart0();
    if (c == 8 || c == 127)
    {
        if (count > 0)
        {
            count--;
            putcUart0(c);
            goto getc;
        }
        else
        {
            goto getc;
        }
    }
    else if (c == 10 || c == 13)
    {
        goto exit;
    }
    else if (c >= 32)
    {
        data->buffer[count] = c;
        count++;
        putcUart0(c);
        if (count == MAX_CHARS)
        {
            goto exit;
        }
        else
        {
            goto getc;
        }
    }
    else
    {
        goto getc;
    }
    exit: ;
    data->buffer[count] = '\0';
}



void planeDraw()
{
    x1[n] =(uint32_t)(x/10);
    y1[n] =(uint32_t)(y/10);
    n++;
    x=0;
    y=0;
    WTIMER5_TAILR_R = 400000000;
    WTIMER5_CTL_R |= TIMER_CTL_TAEN;
    NVIC_EN3_R |= 1 << (INT_WTIMER5A - 16 - 96);


}

void wideTimer5Isr()
{
    uint32_t temprow;
    uint32_t temp1col;
    uint32_t i=0;
    uint32_t temp2=(uint32_t)S2_Y/10;
    uint32_t temp3=(uint32_t)S1_X/10;
    putcUart0('\n');
    putcUart0('\r');
    for(temprow=0;temprow<temp2;temprow++)
    {
        for(temp1col=0;temp1col<temp3;temp1col++)
        {
            if(temp1col==0)
            {
                putcUart0('/');
            }


            for(i=0;i<100;i++)
            {

                if(x1[i]==temp1col && y1[i]==temprow && x1[i] != 0 &&y1[i] != 0)
                {
                    putcUart0('*');
                }
            }
            if(temprow==0)
            {
                putcUart0('/');
            }
            else
            putcUart0(' ');
        }
        putcUart0('\n');
        putcUart0('\r');
    }
    n=0;
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;
    WTIMER5_ICR_R = TIMER_ICR_TATOCINT;
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
    for(i=0;i<50;i++)
    {
        x1[i]=0;
        y1[i]=0;
    }
}


void findCordinates()
{
    Uy1=S2_Y-S0_Y;
    Ux1=S1_X-S0_X;
    ka=(avgValue0 * avgValue0);
    kb=(avgValue1 * avgValue1);
    kc=(avgValue2 * avgValue2);

    x=(((Ux1*Ux1)+ka-kb)/(2*Ux1));
    y=((ka-kc+(Uy1*Uy1))/(2*Uy1));

    sprintf(strm, "x= %f \n\n\r",x);
    putsUart0(strm);
    putsLcd(0, 0, strm);
    waitMicrosecond(1000000);
    sprintf(strm, "y= %f \n\n\r",y);
    putsUart0(strm);
    putsLcd(1, 0, strm);
    waitMicrosecond(1000000);


    Uy1=0;
    Ux1=0;

    ka=0;
    kb=0;
    kc=0;


}

void wideTimer0Isr()
{

    if (UL0 == 0) // zero counter for first period
    {
        ultrasonic0C = WTIMER0_TAV_R;
        count++;
        UL0 = 1;


    }
    WTIMER0_ICR_R |= TIMER_ICR_CAECINT;


}

void wideTimer1Isr()
{
    if (UL1 == 0)
    {
        ultrasonic1C = (WTIMER1_TAV_R);
        count++;
        UL1 = 1;
    }
    WTIMER1_ICR_R |= TIMER_ICR_CAECINT;
}

void wideTimer2Isr()
{
    if (UL2 == false)
    {
        ultrasonic2C = (WTIMER2_TAV_R);
        count++;
        UL2 = true;

    }
    WTIMER2_ICR_R |= TIMER_ICR_CAECINT;

}

void wideTimer3Isr()
{
    if (UL0 == 1)
    {
        s0c1=s0k1/1000;
        ultrasonic0C = ((s0c1) * (ultrasonic0C * 0.000025 * 341) - s0k0);
        sensor0AveBuffer[averageWriteIndex0] = ultrasonic0C;
        averageWriteIndex0 = (averageWriteIndex0 + 1)% AVERAGE_BUFFER_MAX_LENGTH;
    }

    if (UL1 == 1)
    {
        s1c1=s1k1/1000;
        ultrasonic1C = ((s1c1) * (ultrasonic1C * 0.000025 * 341) - s1k0);
        sensor1AveBuffer[averageWriteIndex1] = ultrasonic1C;
        averageWriteIndex1 = (averageWriteIndex1 + 1)% AVERAGE_BUFFER_MAX_LENGTH;
    }
    if (UL2 == 1)
    {
        s2c1=s2k1/1000;
        ultrasonic2C = ((s2c1) * (ultrasonic2C * 0.000025 * 341) - s2k0);
        sensor2AveBuffer[averageWriteIndex2] = ultrasonic2C;
        averageWriteIndex2 = (averageWriteIndex2 + 1)% AVERAGE_BUFFER_MAX_LENGTH;
    }
    if ((readEeprom(18) == averageWriteIndex0)|| (readEeprom(18) == averageWriteIndex1)|| (readEeprom(18) == averageWriteIndex2))
    {

        avgValue0 = 0;
        avgValue1 = 0;
        avgValue2 = 0;

        uint32_t i;

        //Just for an average
        for (i = 0; i <= averageWriteIndex0; i++)
        {
            avgValue0 += sensor0AveBuffer[i];
        }
        avgValue0 = avgValue0 / averageWriteIndex0;

        for (i = 0; i <= averageWriteIndex1; i++)
        {
            avgValue1 += sensor1AveBuffer[i];
        }
        avgValue1 = avgValue1 / averageWriteIndex1;
        for (i = 0; i <= averageWriteIndex2; i++)
        {
            avgValue2 += sensor2AveBuffer[i];
        }
        avgValue2 = avgValue2 / averageWriteIndex2;



        // Variance of S0

        for (i = 1; i <= averageWriteIndex0; i++)
        {
            v0 = v0+((sensor0AveBuffer[i] - avgValue0)*(sensor0AveBuffer[i] - avgValue0)); ///averageWriteIndex1);

        }
        v0 = v0 / averageWriteIndex0;

        for (i = 1; i <= averageWriteIndex1; i++)
        {
            v1 = v1+((sensor1AveBuffer[i] - avgValue1)
                    * (sensor1AveBuffer[i] - avgValue1)); ///averageWriteIndex1);

        }
        v1 = v1 / averageWriteIndex1;

        for (i = 1; i <= averageWriteIndex2; i++)
        {
            v2 = v2+((sensor2AveBuffer[i] - avgValue2)
                    * (sensor2AveBuffer[i] - avgValue2)); ///averageWriteIndex1);

        }
        v2 = v2 / averageWriteIndex2;
        uint32_t c=  readEeprom(25);

        // Variance checking with Average Value stored in EEprom

        if(v0< c && v1 <c && v2< c)
        {

        sprintf(strm, "avgValue0:  %u \n\n\r",avgValue0);
        putsUart0(strm);
        sprintf(strm, "avgValue1m:  %u \n\n\r",avgValue1);
        putsUart0(strm);
        sprintf(strm, "avgValue2:  %u \n\n\r",avgValue2);
        putsUart0(strm);

        sprintf(strm,"Avariance0:  %u \n\n\r",v0);
        putsUart0(strm);
        sprintf(strm,"Avariance1:  %u \n\n\r",v1);
        putsUart0(strm);
        sprintf(strm,"Avariance2:  %u \n\n\r",v2);
        putsUart0(strm);


        findCordinates();
        planeDraw();


        if(readEeprom(26) == 1)
        {  // beepWriteIndex++;

            putcUart0('f');
            SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;               // reset PWM1 module
            SYSCTL_SRPWM_R = 0;                             // leave reset state
            PWM1_1_CTL_R = 0;                               // turn-off PWM1 generator 2 (drives outs 4 and 5)
            PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
            PWM1_1_LOAD_R = readEeprom(27);                 // set frequency to 40 MHz sys clock / 2 /readEeprom(27)  = Taken value from Eeprom)
            PWM1_1_CMPA_R = readEeprom(27) * 0.5;
            WTIMER4_TAILR_R = readEeprom(28);
            WTIMER4_CTL_R |= TIMER_CTL_TAEN;
            NVIC_EN3_R |= 1 << (INT_WTIMER4A - 16 - 96);
            PWM1_1_CTL_R = PWM_1_CTL_ENABLE;
            PWM1_ENABLE_R = PWM_ENABLE_PWM2EN;
        }
        }
        v0 = 0;
        v1 = 0;
        v2 = 0;
        v = 0;
        uint32_t q = 0;
        averageWriteIndex0=0;
        averageWriteIndex1=0;
        averageWriteIndex2=0;
        for (q = 0; q < AVERAGE_BUFFER_MAX_LENGTH; q++)
        {
            sensor0AveBuffer[q] = 0;
            sensor1AveBuffer[q] = 0;
            sensor2AveBuffer[q] = 0;
        }
    }
    int e;
    for(e=1;e<=count;e++)
    {
        if(readEeprom(6+3*(e-1))==1)
        {
             beepBuffer[beepWriteIndex] = e;
             beepWriteIndex = (beepWriteIndex + 1) % BEEP_BUFFER_LENGTH;

        }
    }
    if(primepump)
    {
        NodSensor = beepBuffer[beepReadIndex];

                if (NodSensor == 1 && readEeprom(6) == 1)
                {
                    putcUart0('0');
                    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;               // reset PWM1 module
                    SYSCTL_SRPWM_R = 0;                             // leave reset state
                    PWM1_1_CTL_R = 0; // turn-off PWM1 generator 2 (drives outs 4 and 5)
                    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                    uint32_t c = readEeprom(7);
                    PWM1_1_LOAD_R = c; // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

                    PWM1_1_CMPA_R = c * 0.5;
                    WTIMER4_TAILR_R = readEeprom(8); // red off (0=always low, 1023=always high)
                    WTIMER4_CTL_R |= TIMER_CTL_TAEN;
                    NVIC_EN3_R |= 1 << (INT_WTIMER4A - 16 - 96);
                    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;         // turn-on PWM1 generator 2
                    PWM1_ENABLE_R = PWM_ENABLE_PWM2EN; //| PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                    beepReadIndex = (beepReadIndex + 1) % BEEP_BUFFER_LENGTH;
                }
                else if (NodSensor == 2 && readEeprom(9) == 1)
                {

                    putcUart0('1');
                    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;               // reset PWM1 module
                    SYSCTL_SRPWM_R = 0;                             // leave reset state
                    PWM1_1_CTL_R = 0; // turn-off PWM1 generator 2 (drives outs 4 and 5)
                    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                    PWM1_1_LOAD_R = readEeprom(10); // set frequency to 40 MHz sys clock / 2 / 80000 = 250 Hz

                    PWM1_1_CMPA_R = readEeprom(10) * 0.5;
                    WTIMER4_TAILR_R = readEeprom(11); // red off (0=always low, 1023=always high)
                    WTIMER4_CTL_R |= TIMER_CTL_TAEN;
                    NVIC_EN3_R |= 1 << (INT_WTIMER4A - 16 - 96);
                    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;         // turn-on PWM1 generator 2
                    PWM1_ENABLE_R = PWM_ENABLE_PWM2EN; //| PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                    beepReadIndex = (beepReadIndex + 1) % BEEP_BUFFER_LENGTH;
                }
                else if (NodSensor == 3 && readEeprom(12) == 1)
                {
                    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;               // reset PWM1 module
                    SYSCTL_SRPWM_R = 0;                             // leave reset state
                    PWM1_1_CTL_R = 0; // turn-off PWM1 generator 2 (drives outs 4 and 5)
                    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                    PWM1_1_LOAD_R = readEeprom(13); // set frequency to 40 MHz sys clock / 2 / 80000 = 250 Hz

                    PWM1_1_CMPA_R = readEeprom(13) * 0.5;
                    WTIMER4_TAILR_R = readEeprom(14); // red off (0=always low, 1023=always high)
                    WTIMER4_CTL_R |= TIMER_CTL_TAEN;
                    NVIC_EN3_R |= 1 << (INT_WTIMER4A - 16 - 96);
                    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;         // turn-on PWM1 generator 2
                    PWM1_ENABLE_R = PWM_ENABLE_PWM2EN; //| PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                    beepReadIndex = (beepReadIndex + 1) % BEEP_BUFFER_LENGTH;

                }
                else if (NodSensor == 4 && readEeprom(15) == 1)
                {
                    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;               // reset PWM1 module
                    SYSCTL_SRPWM_R = 0;                             // leave reset state
                    PWM1_1_CTL_R = 0; // turn-off PWM1 generator 2 (drives outs 4 and 5)
                    PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                    PWM1_1_LOAD_R = readEeprom(16); // set frequency to 40 MHz sys clock / 2 / 80000 = 250 Hz

                    PWM1_1_CMPA_R = readEeprom(16) * 0.5;
                    WTIMER4_TAILR_R = readEeprom(17); // red off (0=always low, 1023=always high)
                    WTIMER4_CTL_R |= TIMER_CTL_TAEN;
                    NVIC_EN3_R |= 1 << (INT_WTIMER4A - 16 - 96);
                    PWM1_1_CTL_R = PWM_1_CTL_ENABLE;         // turn-on PWM1 generator 2
                    PWM1_ENABLE_R = PWM_ENABLE_PWM2EN; //| PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                    beepReadIndex = (beepReadIndex + 1) % BEEP_BUFFER_LENGTH;
                }
                primepump=false;
    }
    WTIMER3_ICR_R |= TIMER_ICR_TATOCINT;

    count = 0;
    WTIMER0_TAV_R = 0;                          // zero counter for first period
    WTIMER1_TAV_R = 0;
    WTIMER2_TAV_R = 0;
    WTIMER3_TAV_R = 0;
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    enableNvicInterrupt(16);
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;

}




void wideTimer4Isr()
{
    SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;
    WTIMER4_ICR_R = TIMER_ICR_TATOCINT;
    WTIMER4_CTL_R &= ~TIMER_CTL_TAEN;
    NodSensor=beepBuffer[beepReadIndex];
    if(beepWriteIndex!=beepReadIndex)
    {

                    if (NodSensor == 1 && readEeprom(6) == 1)
                    {
//                        sprintf(strm, "IR ON and PWM OF IR  running\n\n\r");
//                       putsUart0(strm);
                        putcUart0('0');
                        SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;               // reset PWM1 module
                        SYSCTL_SRPWM_R = 0;                             // leave reset state
                        PWM1_1_CTL_R = 0; // turn-off PWM1 generator 2 (drives outs 4 and 5)
                        PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                        uint32_t c = readEeprom(7);
                        PWM1_1_LOAD_R = c; // set frequency to 40 MHz sys clock / 2 / 1024 = 19.53125 kHz

                        PWM1_1_CMPA_R = c * 0.5;
                        WTIMER4_TAILR_R = readEeprom(8); // red off (0=always low, 1023=always high)
                        WTIMER4_CTL_R |= TIMER_CTL_TAEN;
                        NVIC_EN3_R |= 1 << (INT_WTIMER4A - 16 - 96);
                        PWM1_1_CTL_R = PWM_1_CTL_ENABLE;         // turn-on PWM1 generator 2
                        PWM1_ENABLE_R = PWM_ENABLE_PWM2EN; //| PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                        beepReadIndex = (beepReadIndex + 1) % BEEP_BUFFER_LENGTH;
                    }
                    else if (NodSensor == 2 && readEeprom(9) == 1)
                    {
//                        sprintf(strm, "ERROR ON and PWM OF ERROR running \n\n\r");
//                        putsUart0(strm);
                        putcUart0('1');
                        SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;               // reset PWM1 module
                        SYSCTL_SRPWM_R = 0;                             // leave reset state
                        PWM1_1_CTL_R = 0; // turn-off PWM1 generator 2 (drives outs 4 and 5)
                        PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                        PWM1_1_LOAD_R = readEeprom(10); // set frequency to 40 MHz sys clock / 2 / 80000 = 250 Hz

                        PWM1_1_CMPA_R = readEeprom(10) * 0.5;
                        WTIMER4_TAILR_R = readEeprom(11); // red off (0=always low, 1023=always high)
                        WTIMER4_CTL_R |= TIMER_CTL_TAEN;
                        NVIC_EN3_R |= 1 << (INT_WTIMER4A - 16 - 96);
                        PWM1_1_CTL_R = PWM_1_CTL_ENABLE;         // turn-on PWM1 generator 2
                        PWM1_ENABLE_R = PWM_ENABLE_PWM2EN; //| PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                        beepReadIndex = (beepReadIndex + 1) % BEEP_BUFFER_LENGTH;
                    }
                    else if (NodSensor == 3 && readEeprom(12) == 1)
                    {
//                        sprintf(strm, "beeep2 ON and PWM OF beep2 running \n\n\r");
//                        putsUart0(strm);
                        putcUart0('2');
                        SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;               // reset PWM1 module
                        SYSCTL_SRPWM_R = 0;                             // leave reset state
                        PWM1_1_CTL_R = 0; // turn-off PWM1 generator 2 (drives outs 4 and 5)
                        PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                        PWM1_1_LOAD_R = readEeprom(13); // set frequency to 40 MHz sys clock / 2 / 80000 = 250 Hz

                        PWM1_1_CMPA_R = readEeprom(13) * 0.5;
                        WTIMER4_TAILR_R = readEeprom(14); // red off (0=always low, 1023=always high)
                        WTIMER4_CTL_R |= TIMER_CTL_TAEN;
                        NVIC_EN3_R |= 1 << (INT_WTIMER4A - 16 - 96);
                        PWM1_1_CTL_R = PWM_1_CTL_ENABLE;         // turn-on PWM1 generator 2
                        PWM1_ENABLE_R = PWM_ENABLE_PWM2EN; //| PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                        beepReadIndex = (beepReadIndex + 1) % BEEP_BUFFER_LENGTH;

                    }
                    else if (NodSensor == 4 && readEeprom(15) == 1)
                    {
//                        sprintf(strm, "beeep3 ON and PWM OF beep3 running \n\n\r");
//                        putsUart0(strm);
                        putcUart0('3');
                        SYSCTL_SRPWM_R = SYSCTL_SRPWM_R1;               // reset PWM1 module
                        SYSCTL_SRPWM_R = 0;                             // leave reset state
                        PWM1_1_CTL_R = 0; // turn-off PWM1 generator 2 (drives outs 4 and 5)
                        PWM1_1_GENA_R = PWM_1_GENA_ACTCMPAD_ONE | PWM_1_GENA_ACTLOAD_ZERO;
                        PWM1_1_LOAD_R = readEeprom(16); // set frequency to 40 MHz sys clock / 2 / 80000 = 250 Hz

                        PWM1_1_CMPA_R = readEeprom(16) * 0.5;
                        WTIMER4_TAILR_R = readEeprom(17); // red off (0=always low, 1023=always high)
                        WTIMER4_CTL_R |= TIMER_CTL_TAEN;
                        NVIC_EN3_R |= 1 << (INT_WTIMER4A - 16 - 96);
                        PWM1_1_CTL_R = PWM_1_CTL_ENABLE;         // turn-on PWM1 generator 2
                        PWM1_ENABLE_R = PWM_ENABLE_PWM2EN; //| PWM_ENABLE_PWM6EN | PWM_ENABLE_PWM7EN;
                        beepReadIndex = (beepReadIndex + 1) % BEEP_BUFFER_LENGTH;
                    }
      }
    else
    {
        primepump=true;
        reseteverything=0;
        for(reseteverything=0; reseteverything<AVERAGE_BUFFER_MAX_LENGTH; reseteverything++)
        {
        sensor0AveBuffer[reseteverything]=0;
        sensor1AveBuffer[reseteverything]=0;
        sensor2AveBuffer[reseteverything]=0;
        beepWriteIndex=0;
        beepReadIndex=0;
        averageWriteIndex0=0;
        averageWriteIndex1=0;
        averageWriteIndex2=0;
        }
        reseteverything=0;
    }

}

void initIRInterrupt()
{
    count = 0;
    enablePort(PORTA);
    _delay_cycles(3);
    selectPinDigitalInput(PORTA, 7);
    //   enablePinPullup(PORTA, 7);

    enableNvicInterrupt(16);
    disablePinInterrupt(PORTA, 7);
    selectPinInterruptLowLevel(PORTA, 7);
    enablePinInterrupt(PORTA, 7);
    clearPinInterrupt(PORTA, 7);
}

void GPIOA_isr()
{

    UL0 = 0;
    UL1 = 0;
    UL2 = 0;
    count = 0;
    WTIMER0_TAV_R = 0;                          // zero counter for first period
    WTIMER1_TAV_R = 0;
    WTIMER2_TAV_R = 0;
    WTIMER3_TAILR_R = 400000;
    WTIMER0_CTL_R |= TIMER_CTL_TAEN;                 // turn-on counter
    WTIMER1_CTL_R |= TIMER_CTL_TAEN;
    WTIMER2_CTL_R |= TIMER_CTL_TAEN;
    WTIMER3_CTL_R |= TIMER_CTL_TAEN;

    NVIC_EN2_R |= 1 << (INT_WTIMER0A - 16 - 64); // turn-on interrupt 112 (WTIMER1A)
    NVIC_EN3_R |= 1 << (INT_WTIMER1A - 16 - 96);
    NVIC_EN3_R |= 1 << (INT_WTIMER2A - 16 - 96);
    NVIC_EN3_R |= 1 << (INT_WTIMER3A - 16 - 96);
    count++;
    disableNvicInterrupt(16);

}

void initHw()
{
    // Initialize system clock to 40 MHz
    initSystemClockTo40Mhz();
    // Enable clocks
    SYSCTL_RCGCGPIO_R |= SYSCTL_RCGCGPIO_R5 | SYSCTL_RCGCGPIO_R3| SYSCTL_RCGCGPIO_R2 | SYSCTL_RCGCGPIO_R1 | SYSCTL_RCGCGPIO_R0; //PORT F,B,A

    _delay_cycles(3);
    // Enable clocks
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R0;  //UV0
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R1;  //UV1
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R2;  //UV2
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R3;  //Total
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R4;  //Buzzer Duration
    SYSCTL_RCGCWTIMER_R |= SYSCTL_RCGCWTIMER_R5;  // To draw a plane
    _delay_cycles(3);

    SYSCTL_RCGCPWM_R |= SYSCTL_RCGCPWM_R1;
    _delay_cycles(3);

    selectPinDigitalInput(PORTF, 4);
    enablePinPullup(PORTF, 4);
    WTIMER0_CTL_R &= ~TIMER_CTL_TAEN;   // turn-off counter before reconfiguring
    WTIMER1_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER2_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER3_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER4_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER5_CTL_R &= ~TIMER_CTL_TAEN;
    WTIMER0_CFG_R = 4;                   // configure as 32-bit counter (A only)
    WTIMER1_CFG_R = 4;
    WTIMER2_CFG_R = 4;
    WTIMER3_CFG_R = 4;
    WTIMER4_CFG_R = 4;
    WTIMER5_CFG_R = 4;
    WTIMER0_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER1_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER2_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_CAP | TIMER_TAMR_TACDIR; // configure for edge time mode, count up
    WTIMER3_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_1_SHOT; // configure for edge time mode, count down
    WTIMER4_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_1_SHOT; // configure for edge time mode, count down
    WTIMER5_TAMR_R = TIMER_TAMR_TACMR | TIMER_TAMR_TAMR_1_SHOT;
    WTIMER0_CTL_R = TIMER_CTL_TAEVENT_NEG; // measure time from positive edge to positive edge
    WTIMER1_CTL_R = TIMER_CTL_TAEVENT_NEG;
    WTIMER2_CTL_R = TIMER_CTL_TAEVENT_NEG;

    WTIMER0_IMR_R = TIMER_IMR_CAEIM;                 // turn-on interrupts
    WTIMER1_IMR_R = TIMER_IMR_CAEIM;
    WTIMER2_IMR_R = TIMER_IMR_CAEIM;
    WTIMER3_IMR_R = TIMER_IMR_TATOIM;
    WTIMER4_IMR_R = TIMER_IMR_TATOIM;       //Make A6 pin low in this Isr
    WTIMER5_IMR_R = TIMER_IMR_TATOIM;
    selectPinPushPullOutput(PORTA, 6);
    setPinAuxFunction(PORTA, 6, 5);
    selectPinDigitalInput(PORTC, 4);
    selectPinDigitalInput(PORTC, 6);
    selectPinDigitalInput(PORTD, 0);
    setPinAuxFunction(PORTC, 4, 7);
    setPinAuxFunction(PORTC, 6, 7);
    setPinAuxFunction(PORTD, 0, 7);

    averageWriteIndex0 = 0;
    averageWriteIndex1 = 0;
    averageWriteIndex2 = 0;
    primepump=true;
    planeflag=0;

}
/**
 * main.c
 */
void main(void)
{

    initHw();
    initUart0();
    initIRInterrupt();
    initI2c0();
    // Setup UART0 baud rate
    setUart0BaudRate(115200, 40e6);
    initEeprom();
    initLcd();

    S0_X = readEeprom(0);
    S0_Y = readEeprom(1);
    S1_X = readEeprom(2);
    S1_Y = readEeprom(3);
    S2_X = readEeprom(4);
    S2_Y = readEeprom(5);
    s0k0 = readEeprom(19);
    s0k1 = readEeprom(20);
    s1k0 = readEeprom(21);
    s1k1 = readEeprom(22);
    s2k0 = readEeprom(23);
    s2k1 = readEeprom(24);
    variance = readEeprom(25);

    sprintf(strm, "IR ON/OFF: %u \n\n\r", readEeprom(6));
    putsUart0(strm);
    sprintf(strm, "Error ON/OFF: %u \n\n\r", readEeprom(9));
    putsUart0(strm);
    sprintf(strm, "beep2 ON/OFF: %u \n\n\r", readEeprom(12));
    putsUart0(strm);
    sprintf(strm, "beep3 ON/OFF: %u \n\n\r", readEeprom(15));
    putsUart0(strm);
    sprintf(strm, "fix ON/OFF: %u \n\n\r", readEeprom(26));
    putsUart0(strm);

    while (true)
    {

        getsUart0(&msg);
        putcUart0('\n');
        putcUart0('\r');
        //Parse fields
        parseFields(&msg);
        if (isCommand(&msg, "reset", 0))
        {

            int32_t backup = NVIC_APINT_R & ~ NVIC_APINT_VECTKEY_M;
            NVIC_APINT_R = backup | NVIC_APINT_VECTKEY | NVIC_APINT_SYSRESETREQ; //Reset the microcontroller

        }

        if (isCommand(&msg, "sensor", 3))
        {

            sensorN = getFieldInteger(&msg, 1);
            if (sensorN == 0)
            {
                S0_X = getFieldInteger(&msg, 2);
                S0_Y = getFieldInteger(&msg, 3);
                writeEeprom(0, S0_X);
                writeEeprom(1, S0_Y);

            }
            else if (sensorN == 1)
            {
                S1_X = getFieldInteger(&msg, 2);
                S1_Y = getFieldInteger(&msg, 3);
                writeEeprom(2, S1_X);
                writeEeprom(3, S1_Y);
            }
            else if (sensorN == 2)
            {
                S2_X = getFieldInteger(&msg, 2);
                S2_Y = getFieldInteger(&msg, 3);
                writeEeprom(4, S2_X);
                writeEeprom(5, S2_Y);

            }
            else
            {

            }
        }

        if (isCommand(&msg, "beep", 2))
        {
            if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "ir"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[2]], "ON")))
            {
                writeEeprom(6, 1);
                sprintf(strm,
                        "ir status: %u, pwm freq: %u, pemperiod: %u \n\n\r",
                        readEeprom(6), readEeprom(7), readEeprom(8));
                putsUart0(strm);
            }
            else if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "ir"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[2]], "OFF")))
            {
                writeEeprom(6, 0);
            }
            else if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "error"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[2]], "ON")))
            {
                writeEeprom(9, 1);
                sprintf(strm,
                        "error status: %u, pwm freq: %u, pemperiod: %u \n\n\r",
                        readEeprom(9), readEeprom(10), readEeprom(11));
                putsUart0(strm);
            }
            else if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "error"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[2]], "OFF")))
            {
                writeEeprom(9, 0);
            }
            else if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "beep2"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[3]], "ON")))
            {
                writeEeprom(12, 1);
                sprintf(strm,
                        "Beep2 status: %u, pwm freq: %u, pemperiod: %u \n\n\r",
                        readEeprom(12), readEeprom(13), readEeprom(14));
                putsUart0(strm);
            }
            else if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "beep2"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[3]], "OFF")))
            {
                writeEeprom(12, 0);
            }
            else if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "beep3"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[3]], "ON")))
            {
                writeEeprom(15, 1);
                sprintf(strm,
                        "Beep3 status: %u, pwm freq: %u, pemperiod: %u \n\n\r",
                        readEeprom(15), readEeprom(16), readEeprom(17));
                putsUart0(strm);
            }
            else if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "beep3"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[3]], "OFF")))
            {
                writeEeprom(15, 0);
            }
            else if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "fix"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[2]], "ON")))
            {
                writeEeprom(26, 1);
                sprintf(strm,"fix status: %u, pwm freq: %u, pemperiod: %u \n\n\r",
                        readEeprom(26), readEeprom(27), readEeprom(28));
                putsUart0(strm);
            }
            else if ((strcmp1(&msg.buffer[msg.fieldPosition[1]], "fix"))
                    && (strcmp1(&msg.buffer[msg.fieldPosition[2]], "OFF")))
            {
                writeEeprom(26, 0);
            }
            else
            {
            }

        }

        if (isCommand(&msg, "beep", 3))
        {
            if (strcmp1(&msg.buffer[msg.fieldPosition[1]], "ir"))
            {
                uint32_t beepfreq = (uint32_t) getFieldInteger(&msg, 2);
                uint32_t beepdura = (uint32_t) getFieldInteger(&msg, 3);
                writeEeprom(7, beepfreq);
                writeEeprom(8, beepdura);
            }
            if (strcmp1(&msg.buffer[msg.fieldPosition[1]], "error"))
            {
                uint32_t beepfreq = (uint32_t) getFieldInteger(&msg, 2);
                uint32_t beepdura = (uint32_t) getFieldInteger(&msg, 3);
                writeEeprom(10, beepfreq);
                writeEeprom(11, beepdura);
            }
            if (strcmp1(&msg.buffer[msg.fieldPosition[1]], "fix"))
            {
                uint32_t beepfreq = (uint32_t) getFieldInteger(&msg, 2);
                uint32_t beepdura = (uint32_t) getFieldInteger(&msg, 3);
                writeEeprom(27, beepfreq);
                writeEeprom(28, beepdura);
            }
        }
        if (isCommand(&msg, "beep", 4))
        {
            if (strcmp1(&msg.buffer[msg.fieldPosition[1]], "beep2"))
            {
                uint32_t beepfreq = (uint32_t) getFieldInteger(&msg, 3);
                uint32_t beepdura = (uint32_t) getFieldInteger(&msg, 4);
                writeEeprom(13, beepfreq);
                writeEeprom(14, beepdura);
            }
            if (strcmp1(&msg.buffer[msg.fieldPosition[1]], "beep3"))
            {
                uint32_t beepfreq = (uint32_t) getFieldInteger(&msg, 3);
                uint32_t beepdura = (uint32_t) getFieldInteger(&msg, 4);
                writeEeprom(16, beepfreq);
                writeEeprom(17, beepdura);
            }

        }
        if (isCommand(&msg, "average", 1))
        {
            averageN = (uint32_t) getFieldInteger(&msg, 1);
            writeEeprom(18, averageN);
        }
        if (isCommand(&msg, "variance", 1))
        {
            variance = (uint32_t) getFieldInteger(&msg, 1);
            writeEeprom(25, variance);
        }
        if (isCommand(&msg, "correct", 3))
        {
            if ((uint32_t) getFieldInteger(&msg, 1) == 0)
            {
                s0k0 = (uint32_t) getFieldInteger(&msg, 2);
                s0k1 = (uint32_t) getFieldInteger(&msg, 3);
                writeEeprom(19, s0k0);
                writeEeprom(20, s0k1);
            }
            else if ((uint32_t) getFieldInteger(&msg, 1) == 1)
            {
                s1k0 = (uint32_t) getFieldInteger(&msg, 2);
                s1k1 = (uint32_t) getFieldInteger(&msg, 3);
                writeEeprom(21, s1k0);
                writeEeprom(22, s1k1);
            }
            else if ((uint32_t) getFieldInteger(&msg, 1) == 2)
            {
                s2k0 = (uint32_t) getFieldInteger(&msg, 2);
                s2k1 = (uint32_t) getFieldInteger(&msg, 3);
                writeEeprom(23, s2k0);
                writeEeprom(24, s2k1);
            }
        }


        if (isCommand(&msg, "distance", 0))
        {   ultrasonic0C=0;
            ultrasonic1C=0;
            ultrasonic2C=0;

            while (getPinValue(PORTF, 4))
            {
                sprintf(strm," Distance from S0 in mm : %u \n\n\r Distance from S1 in mm : %u \n\n\r Distance from s2 in mm: %u \n\n\r ",ultrasonic0C, ultrasonic1C, ultrasonic2C);
                putsUart0(strm);

            }

        }


    }
}

