#include "xc_configuration_bits.h"
#include "adc.h"
#include "timers.h"
#include "delays.h"
#include "math.h"
#include "pwm.h"

void move(int angle, int forward);
void config(void);
void Rmotor(int power);
void Lmotor(int power);

int x = 0;
int time360 = 700;

int main(void)
{
    config();

    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);

    move(0, 1); //forward
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    move(0, -1); //backward
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    move(0, 2);//fast forward
    Delay10KTCYx(20);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    move(90, 0); //tun right
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    move(-90, 0); //turn left
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    Delay10KTCYx(250);
    move(0, 3); //curve

    ClosePWM4();
    ClosePWM5();
    while (1);
}

void move(int angle, int forward)
{
    int exec_time_ms = ((double)sqrt(angle*angle)/360.0)*time360;
    if(forward == 3)
    {
        PORTHbits.RH3 = 1;
        Rmotor(600);
        Lmotor(800);
        Delay10KTCYx(250);
        Delay10KTCYx(100);
        PORTHbits.RH3 = 0;
    }
    if(forward == 2)
    {
        PORTHbits.RH3 = 1;
        Rmotor(800);
        Lmotor(800);
        Delay10KTCYx(100);
        PORTHbits.RH3 = 0;
    }
    if(forward == 1)
    {
        PORTHbits.RH3 = 1;
        Rmotor(700);
        Lmotor(700);
        Delay10KTCYx(100);
        PORTHbits.RH3 = 0;
    }
    else if(forward == -1)
    {
        PORTHbits.RH3 = 1;
        Rmotor(300);
        Lmotor(300);
        Delay10KTCYx(100);
        PORTHbits.RH3 = 0;
    }
    else if(forward == 0)
    {
        if(angle > 0)
        {
            PORTHbits.RH3 = 1;
            Rmotor(800);
            Lmotor(200);
            for(int i = 0; i < exec_time_ms; i++)
            {
                Delay10TCYx(250);
            }
            PORTHbits.RH3 = 0;
        }
        else
        {
            PORTHbits.RH3 = 1;
            Rmotor(200);
            Lmotor(800);
            for(int i = 0; i < exec_time_ms; i++)
            {
                Delay10TCYx(250);
            }
            PORTHbits.RH3 = 0;
        }
    }
}

void config(void)
{
    //pwm output
    TRISGbits.RG3 = 0;
    TRISGbits.RG4 = 0;

    TRISH = 0b10010100;

    //enable bit
    //PORTHbits.RH3 = 1;

    //unipolar setting
    PORTHbits.RH0 = 1;
    PORTHbits.RH1 = 1;

    //direction bits
    PORTHbits.RH5 = 0;
    PORTHbits.RH6 = 0;

    //timer configuration
    OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);

    //OpenPWM2
    OpenPWM4(249); // 1/10khz = (249+1)*4*Tosc*Ps
    OpenPWM5(249);
}

void Rmotor(int power)
{
    SetDCPWM4(power);
}
void Lmotor(int power)
{
    SetDCPWM5(power);
}

