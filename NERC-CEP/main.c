/*
 * PROJECT: NERC/CEP 2026 Autonomous Robot
 * VERSION: 2.0 (Engineering Final)
 * AUTHOR: Yazdan Ali Khan (2024665)
 * TARGET: PIC18F4550 @ 8MHz
 */

// Configuration Bits
#pragma config FOSC = INTOSCIO_EC, WDT = OFF, LVP = OFF, MCLRE = ON, PBADEN = OFF

#include <xc.h>

#define _XTAL_FREQ 8000000

// --- PIN ARCHITECTURE ---
// Diagnostics
#define LED_SYS_OK      LATDbits.LATD0 // Green: Mode 0 / System Ready
#define LED_BUSY        LATDbits.LATD1 // Amber: Mode 1 / Processing
#define LED_ERROR       LATDbits.LATD2 // Red: Mode 2 / Fault detected

// Actuators
#define MOTOR_LEFT_F    LATDbits.LATD4
#define MOTOR_LEFT_B    LATDbits.LATD5
#define MOTOR_RIGHT_F   LATDbits.LATD6
#define MOTOR_RIGHT_B   LATDbits.LATD7
#define SERVO_PWM       LATBbits.LATB3 // CCP2 for pallet deployment

// Color Sensor (TCS3200)
#define COLOR_S2        LATCbits.LATC2
#define COLOR_S3        LATBbits.LATB1
#define COLOR_OUT       PORTBbits.RB2 

// Line Sensors (Active Low)
#define SENSORS_PORT    PORTA
#define S_CENTER        PORTAbits.RA2

// --- GLOBAL STATE ---
volatile uint8_t robot_mode = 0;
volatile uint16_t pulse_count = 0;

// --- LOW-LEVEL HARDWARE ABSTRACTION ---
void set_servo_angle(uint8_t angle) {
    // Basic software PWM or CCP2 configuration
    // 1ms = 0 deg, 2ms = 180 deg (approx)
    for(int i=0; i<30; i++) {
        SERVO_PWM = 1;
        __delay_us(1000 + (angle * 5));
        SERVO_PWM = 0;
        __delay_ms(18);
    }
}

void drive(int8_t left, int8_t right) {
    MOTOR_LEFT_F  = (left > 0);  MOTOR_LEFT_B  = (left < 0);
    MOTOR_RIGHT_F = (right > 0); MOTOR_RIGHT_B = (right < 0);
}

// --- SELF-DIAGNOSTIC SUITE (POST) ---
void run_power_on_self_test(void) {
    // 1. LED Chase
    LED_SYS_OK = 1; __delay_ms(200);
    LED_BUSY = 1;   __delay_ms(200);
    LED_ERROR = 1;  __delay_ms(200);
    LATD &= 0xF8;   // Clear LEDs

    // 2. Servo Range Test
    set_servo_angle(0);
    __delay_ms(500);
    set_servo_angle(90);

    // 3. Sensor Check
    if (SENSORS_PORT == 0xFF) {
        // Warning: No sensors detected (All High/Open circuit)
        LED_ERROR = 1; 
    }
}

// --- COLOR DETECTION ---
uint8_t is_blue_slot(void) {
    // S2=Low, S3=High for Blue frequency
    COLOR_S2 = 0; COLOR_S3 = 1;
    __delay_ms(10);
    // Simple frequency thresholding would go here
    return 1; // Placeholder for logic
}

void __interrupt(high_priority) ISR(void) {
    if (INT0IF) {
        __delay_ms(20); // Debounce
        if (PORTBbits.RB0 == 0) {
            robot_mode = (robot_mode + 1) % 3;
            drive(0,0); // Safety stop
        }
        INT0IF = 0;
    }
}

void main(void) {
    OSCCON = 0x72;
    ADCON1 = 0x0F;
    
    TRISB = 0x05; // RB0, RB2 as In
    TRISD = 0x00;
    TRISA = 0xFF;
    
    run_power_on_self_test();
    
    INTCONbits.GIE = 1;
    INTCONbits.INT0IE = 1;

    while(1) {
        switch(robot_mode) {
            case 0: // STANDBY / CALIBRATION
                LED_SYS_OK = 1; LED_BUSY = 0; LED_ERROR = 0;
                // Monitor sensors on LEDs for manual alignment
                if(!S_CENTER) LED_BUSY = 1; else LED_BUSY = 0;
                break;
                
            case 1: // DIAGNOSTIC DRIVE
                LED_SYS_OK = 0; LED_BUSY = 1; LED_ERROR = 0;
                drive(1,1); __delay_ms(500); drive(0,0);
                break;
                
            case 2: // FULL AUTONOMOUS (NERC THEME)
                LED_SYS_OK = 0; LED_BUSY = 0; LED_ERROR = 1;
                // Implement Line Follow -> Junction -> Color Check -> Drop
                break;
        }
    }
}
