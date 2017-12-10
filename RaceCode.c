// <editor-fold defaultstate="collapsed" desc="0. Code information">
    //Project: ESP - Group 13
    //Leader: Nicolo Frisiani
    //Year: 2017
    //Version: 5.4
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="1. File Inclusions">
#include "xc_configuration_bits.h"
#include "adc.h"
#include "timers.h"
#include "delays.h"
#include "pwm.h"
#include "capture.h"
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="2. Function Declarations">
 
// <editor-fold defaultstate="collapsed" desc="2.1 Configuration Functions">
    void config_PWM(void);
    void config_LS(void);
    void config_PS(void);
//</editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="2.2 Motors Functions">
    void stop(void);
    void move(int PID_error);
    void scan(int error);
    void turn180 (void);
    void Rmotor(int power);
    void Lmotor(int power);
//</editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="2.3 Line Sensors Functions">
    void LSarray_read(void);
    void LEDarray_write(unsigned char x);
//</editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="2.4 PID Functions">
    int computeError(void);
    int error_switch(int sum);
    int PID(int error);
//</editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="2.5 Proximity Sensor Functions">
    void interrupt isr(void);
    void enable_global_interrupts(void);
//</editor-fold>
//</editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="3. Global variables">
 
// <editor-fold defaultstate="collapsed" desc="3.1 Main Loop variables">
int error = 0;
int PID_error = 0;
int temp = 0;
int testing = 0;

//</editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="3.2 Turn 180 variables">
int t = 0;
//</editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="3.3 PID variables">
int prev_error = 0;
int integral = 0;
int P = 0;
int I = 0;
int D = 0;
int Kp = 22;
int Ki = 17;
int Kd = 220;
int prox_count = 0;
//</editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="3.4 Ultrasound variables">
volatile char y=0,s=0;
volatile unsigned int logic_high =0;
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="3.5 Line Sensors variables">
int LS_val[6] = {0, 0, 0, 0, 0, 0};
unsigned char LS_digital[6] = {0, 0, 0, 0, 0, 0};
int last_val = 0;
int value = 0;
// </editor-fold>
 
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="4. Main Line Code">
int main(void)
{
    //configuration
    config_LS();
    config_PS();
    config_PWM();
    enable_global_interrupts();
    OpenADC(ADC_FOSC_16 & ADC_RIGHT_JUST & ADC_12_TAD, ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, 0);
 
    //MLC
    while (1)
    {
        LSarray_read(); //updates the sensor reading array
        error = computeError();
        PID_error = PID(error);
        temp = 0;
 
        for(int i = 0; i < 6; i++)
        {
            temp += LS_digital[i];
        }
 
        if(temp == 0) //all sensors on black
        {
            Delay10KTCYx(50);
 
            LSarray_read(); //update readings
 
            for(int i = 0; i < 6; i++)
            {
                temp += LS_digital[i];
            }
 
            if(temp == 0)
            {
                stop();
                Delay10KTCYx(150); //moves backwards to ensure halt is within 30cm
                PORTHbits.RH3 = 0;
                INTCONbits.GIE = 0;
                INTCONbits.PEIE = 0;
                while(1);
            }
        }
 
        move(PID_error);
    }
}
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="5. Functions">
 
// <editor-fold defaultstate="collapsed" desc="5.1 Configuration Functions">
    void config_PWM(void)
    {
        TRISGbits.RG3 = 0;
        TRISGbits.RG4 = 0;
 
        TRISH = 0b10010100;
 
        //enable bit
        PORTHbits.RH3 = 0;
 
        //unipolar setting
        PORTHbits.RH0 = 1;
        PORTHbits.RH1 = 1;
 
        //direction bits
        PORTHbits.RH5 = 0;
        PORTHbits.RH6 = 0;
 
        //timer configuration
        OpenTimer2(TIMER_INT_OFF & T2_PS_1_1 & T2_POST_1_1);
 
        //OpenPWM2
        OpenPWM4(252);
        OpenPWM5(252);
    }
 
    void config_LS(void)
    {
        ADCON1 = 0x00;
        TRISA = 0x00;
    }
 
    void config_PS(void)
    {
        //used to calculate distance from wall
        OpenTimer3(TIMER_INT_OFF & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_1 & T1_OSC1EN_ON & T1_SYNC_EXT_OFF);
 
        //used to trigger sound sensor
        OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
        WriteTimer0(30000);
 
        //used for Capture Compare interrupt
        OpenCapture3(CAPTURE_INT_ON & C3_EVERY_RISE_EDGE);
 
        TRISB =0x00;
        TRISC = 0x00;
    }
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="5.2 Motor Functions">
 
    void Rmotor(int power)
    {
        SetDCPWM4(power);
    }
 
    void Lmotor(int power)
    {
        SetDCPWM5(power);
    }
 
    //moves backwards to ensure halt within 30 cm from the end of the line
    void stop(void)
    {
        Rmotor(350);
        Lmotor(350);
    }
 
    void move(int PIDerror)
    {
        PORTHbits.RH3 = 1;
 
        //ensures the error is not too big to overflow the PWM in opposite direction
        if(PIDerror > 1000)
        {
           PIDerror = 1000;
        }
        else if(PIDerror < -1000)
        {
            PIDerror = -1000;
        }
 
 
        if(PIDerror <= 0) //turn right
        {
            Rmotor(1000);
            Lmotor(1000 + PIDerror);
        }
        if(PIDerror > 0) //turn left
        {
            Rmotor(1000 - PIDerror);
            Lmotor(1000);
        }
    }
 
    void turn180(void)
    {
        //starts turning
        PORTHbits.RH3 = 1;
        Rmotor(300);
        Lmotor(700);
        Delay10KTCYx(200);
 
        //clears integral and disables interrupts
        integral = 0;
        INTCONbits.GIE = 0;
        INTCONbits.PEIE = 0;
        t = 0;
 
        //keeps turning until it detects the white line
        while(t == 0)
        {
            LSarray_read();
 
            t = LS_digital[1] + LS_digital[2] + LS_digital[3] + LS_digital[4];
        }
    }
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="5.3 Line Sensor Functions">
    void LSarray_read(void)
    {
        for(int i = 0; i < 6; i++)
        {
            value = 0;
 
            //switch case reads values from all sensors one at a time
            switch(i)
            {
                case 0: SetChanADC(ADC_CH5);
                break;
                case 1: SetChanADC(ADC_CH6);
                break;
                case 2: SetChanADC(ADC_CH7);
                break;
                case 3: SetChanADC(ADC_CH8);
                break;
                case 4: SetChanADC(ADC_CH9);
                break;
                case 5: SetChanADC(ADC_CH10);
                break;
                default:break;
            }
 
            ConvertADC();
            while(BusyADC());
            value = ReadADC();
 
            LS_val[i] = value; //updates sensor array
 
            //updates digital sensor array (i.e. Black or White)
            if(value > 800)
            {
                LS_digital[i] = 1;
            }
            else
            {
                LS_digital[i] = 0;
            }
 
        }
    }
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="5.4 Proximity Sensor Functions">
    void enable_global_interrupts(void)
    {
        INTCONbits.GIE = 1;
        INTCONbits.PEIE = 1;
    }
 
    void interrupt isr(void)
    {
        if(INTCONbits.TMR0IF) //Proximity trigger signal
        {
            INTCONbits.TMR0IF =0; //TMRO flag down
            s = s^1;              //switches between 1 and 0
            LATBbits.LATB0 = s;
            WriteTimer0(45000);
        }
 
        if(PIR3bits.CCP3IF == 1)
        {
            PIR3bits.CCP3IF = 0;  //CCP3 flag down
            y= y^1;              //switches between 1 and 0
 
            if(y==1)
            {
                CCP3CON = 4;      //configure CCP3 to interrupt on falling edge
                WriteTimer3(0);  //refresh timer3
            }
            else
            {
                logic_high = ReadTimer3();
                CCP3CON = 5;
                //checking if the wall is within ~3-10 cm
                if(logic_high < 3800 && logic_high > 2000)
                {
                    prox_count++;
                    //used to make sure the wall is actually there (avoid errors)
                    if(prox_count == 5)
                        turn180();
                }
            }
        }
    }
// </editor-fold>
 
// <editor-fold defaultstate="collapsed" desc="5.5 PID Functions">
    int read_line()
    {
        LSarray_read(); //updates the sensor array
 
        int sum = 0;
        int average = 0;
 
        //calculates a weighted average and an unweighted sum of analog sensors
        for(int i = 0; i < 6; i++)
        {
            if(LS_val[i] > 50)
            {
                average += 100*i*LS_val[i];
                sum += LS_val[i];
            }
        }
 
        //calculates the position of the line between 0 and 500
        int read_line_distance = average/sum;
        return read_line_distance;
    }
 
    int computeError(void)
    {
        //skewes the postion from being centred at 250 to being centred around 0
        int position = read_line();
        int error = position - 250;
        return error;
    }
 
    int PID(int error)
    {
        int output;
 
        integral += error;
 
        P = error*Kp;
        //to avoid using floats to achieve a Ki smaller than 1, integral is
        //devided by Ki (sacrifices precision due to int-int division for
        //efficiency)
        I = integral/Ki;
        D = Kd*(error - prev_error);
 
        prev_error = error;
 
        output = P + I + D;
 
        return output;
    }
// </editor-fold>
 
// </editor-fold>