// <editor-fold defaultstate="collapsed" desc="0. Code information">
    //Author: Nicolo Frisiani
    //Project: ESP-13
    //Year: 2017
    //Version: 5.0
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="1. File inclusions required">
#include "xc_configuration_bits.h"
#include "adc.h"
#include "timers.h"
#include "delays.h"
#include "math.h"
#include "pwm.h"
#include "capture.h"

#define array_size 300
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
    void LEDarray_on(void);
    void LEDarray_off(void);
    void LSarray_read(void);
    void LEDarray_write(unsigned char x);
    unsigned char LEDarray_breakdetected(void);
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
unsigned char testing = 0;
int last_errors[array_size];
int helper[array_size];
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="3.2 Turn 180 variables">
unsigned char t = 0;
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="3.3 PID variables">
int DTimer = 0;
int prev_error = 0;
int integral = 0;
int P = 0;
int I = 0;
int D = 0;
int Kp = 105;
int Ki = 45;
int Kd = 50;
//</editor-fold>

// <editor-fold defaultstate="collapsed" desc="3.4 Ultrasound variables">
volatile char y=0,s=0;
volatile unsigned int logic_high =0;
// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="3.5 Line Sensors variables">
unsigned char LS_val[6] = {0, 0, 0, 0, 0, 0};
unsigned char last_val = 0;
int value = 0;
// </editor-fold>

// </editor-fold>

// <editor-fold defaultstate="collapsed" desc="4. Main Line Code">

int main(void)
{
    config_LS();
    config_PS();
    config_PWM();
    enable_global_interrupts();
    OpenADC(ADC_FOSC_16 & ADC_RIGHT_JUST & ADC_12_TAD, ADC_CH0 & ADC_INT_OFF & ADC_VREFPLUS_VDD & ADC_VREFMINUS_VSS, 0);

    while (1)
    {
        LSarray_read();
        error = computeError();
        PID_error = PID(error);
        temp = 0;
        
        testing = 0;
        for(int k = 0; k < 6; k++)
        {
            testing += LS_val[k] << (5-k);
        }
        
        LEDarray_write(testing);
        
        for(int i = 0; i < 6; i++)
        {
            temp += LS_val[i];
        }

        if(temp == 0)
        {
            Delay10KTCYx(15);

            LSarray_read();
            
            for(int i = 0; i < 6; i++)
            {
                temp += LS_val[i];
            }
            
            if(temp == 0)
            {
                stop();
                Delay10KTCYx(20);
                INTCONbits.GIE = 0;
                INTCONbits.PEIE = 0;
                PORTHbits.RH3 = 0;
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
        TRISA = 0b00000000;
    }

    void config_PS(void)
    {
        OpenTimer3(TIMER_INT_OFF & T1_16BIT_RW & T1_SOURCE_INT & T1_PS_1_1 & T1_OSC1EN_ON & T1_SYNC_EXT_OFF);
        OpenTimer0(TIMER_INT_ON & T0_16BIT & T0_SOURCE_INT & T0_PS_1_1);
        WriteTimer0(12536);
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

    void stop(void)
    {
        Rmotor(350);
        Lmotor(350);
    }
    
    void move(int PIDerror)
    {
        PORTHbits.RH3 = 1;

        if(PIDerror > 1000)
        {
           PIDerror = 1000;
        }
        else if(PIDerror < -1000)
        {
            PIDerror = -1000;
        }
        
        if(PIDerror >= 0)
        {
            Rmotor(1000);
            Lmotor(1000-PIDerror);
        }
        else
        {
            Rmotor(1000+PIDerror);
            Lmotor(1000);
        }
        
    }
    
    void turn180(void)
    {
        PORTHbits.RH3 = 1;
        Rmotor(350);
        Lmotor(650);
        Delay10KTCYx(120);
        integral = 0;
        t = 0;
        
        while(t == 0)
        {
            LSarray_read();
            
            testing = 0;
            for(int k = 0; k < 6; k++)
            {
                testing += LS_val[k] << (5-k);
            }
        
            LEDarray_write(testing);
        
            t = LS_val[1] + LS_val[2] + LS_val[3] + LS_val[4];
        }
    }

// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="5.3 Line Sensor Functions">
    void LSarray_read(void)
    {
        for(int i = 0; i < 6; i++)
        {
            value = 0;
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

            if(value > 800)
                LS_val[i] = 1;
            else
                LS_val[i] = 0;
        }
    }
    
    void LEDarray_write(unsigned char x)
    {
        LATA = x;
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
            INTCONbits.TMR0IF =0;
            s = s^1;
            LATBbits.LATB0 = s;   //J2 13
            WriteTimer0(40563);
        }

        if(PIR3bits.CCP3IF == 1)
        {
            PIR3bits.CCP3IF = 0;  //CCP3 interrupt bit zeroed
            y= y^1;              //switched between 1 and 0

            if(y==1)
            {
                CCP3CON = 4;  //configure CCP3 to interrupt on falling edge
                WriteTimer3(0);  //refresh timer3
            }
            else
            {
                logic_high = ReadTimer3();
                CCP3CON = 5;     
                if(logic_high < 3000)
                {    
                    turn180();     
                }
            }
        }
    }

// </editor-fold>
    
// <editor-fold defaultstate="collapsed" desc="5.5 PID Functions">
    int computeError(void)
    {
        int close_left = 0;
        int mid_left = 0;
        int far_left = 0;
        int close_right = 0;
        int mid_right = 0;
        int far_right = 0;
        int error = 0;

        int sum = LS_val[0] + LS_val[1] + LS_val[2] + LS_val[3] + LS_val[4] + LS_val[5];

        if(sum == 1)
        {
            close_left = -1*LS_val[3];
            mid_left = -3*LS_val[4];
            far_left = -9*LS_val[5];
            close_right = 1*LS_val[2];
            mid_right = 3*LS_val[1];
            far_right = 9*LS_val[0];

            error = (close_left + mid_left + far_left + close_right + mid_right + far_right);
        }
        else if(sum == 2)
        {
            far_left = -3*LS_val[5] + -1*LS_val[4];
            close_left = -2*LS_val[4] + 0*LS_val[3];
            close_right = 0*LS_val[2] + 2*LS_val[1];
            far_right = 1*LS_val[1] + 3*LS_val[0];

            error = (close_left + far_left + close_right + far_right);
        }
        else if(sum == 3)
        {
            far_left = -2*LS_val[5] + -1*LS_val[4] + 0*LS_val[3];
            close_left = -1*LS_val[4] + 0*LS_val[3] + 0*LS_val[2];
            close_right = 1*LS_val[1] + 0*LS_val[2] + 0*LS_val[3];
            far_right = 0*LS_val[2] + 1*LS_val[1] + 2*LS_val[0];

            error = (close_left + far_left + close_right + far_right);
        }
        else if(sum == 4)
        {
            far_left = -2*LS_val[5] + 0*LS_val[4] + 0*LS_val[3] + 0*LS_val[2];
            far_right = 0*LS_val[2] + 0*LS_val[2] + 0*LS_val[1] + 2*LS_val[0];

            error = (close_left + far_left + close_right + far_right);
        }
        
        
        
        for(int i = 1; i < array_size; i++)
            last_errors[i] = helper[i - 1];
        
        last_errors[0] = error;
        
        int errors_sum = 0;
        
        for(int i = 0; i < array_size; i++){
            helper[i] = last_errors[i];
            errors_sum += last_errors[i];
        }
        
        if(errors_sum < 30 && errors_sum > -30)
        {
            Kp = 30;
            Kd = 10;
        }
        else if(errors_sum < 80 && errors_sum > 80)
        {
            Kp = 120;
            Kd = 40;
        }
        else if(errors_sum < 120 && errors_sum > -120)
        {
            Kp = 300;
            Kd = 70;
        }
        else
        {
            Kp = 500;
            Kd = 90;
        }
        
        return error;
    }

    int PID(int error)
    {
        int output;

        integral += (error - prev_error);

        P = Kp*error;
        I = 0;
        //I = (integral + (Ki - 1))/Ki;
        D = Kd*(error - prev_error);
        
        DTimer++;
        
        if(DTimer == 10)
        {
            prev_error = error;
            DTimer = 0;
        }
       
        
        output = P + I + D;

        return output;
    }
    
    // </editor-fold>

// </editor-fold>