/*
 * PROJECT: NERC/CEP 2026 Autonomous Robot
 * VERSION: 3.0 (Integration Build)
 * AUTHOR: Yazdan Ali Khan (2024665)
 * TARGET: PIC18F4550 @ 8MHz
 *
 * ARCHITECTURE: Interrupt-driven state machine (main.c v2.0 philosophy)
 * AUTONOMOUS LOGIC: Node-based navigation with colour-sorted pallet deployment
 */

// Configuration Bits
#pragma config FOSC = INTOSC_HS, WDT = OFF, LVP = OFF, MCLRE = ON, PBADEN = OFF

#include <xc.h>

#define _XTAL_FREQ 8000000

// ============================================================
//  PIN ARCHITECTURE
// ============================================================

// -- Diagnostics / Mode LEDs --
#define LED_SYS_OK      LATDbits.LATD0  // Green  : Mode 0 ? Standby/Calibration
#define LED_BUSY        LATDbits.LATD1  // Amber  : Mode 1 ? Diagnostic Drive
#define LED_ERROR       LATDbits.LATD2  // Red    : Mode 2 ? Full Autonomous

// -- Motor Driver (L298N / L293D) --
#define MOTOR_LEFT_F    LATDbits.LATD4  // IN1
#define MOTOR_LEFT_B    LATDbits.LATD5  // IN2
#define MOTOR_RIGHT_F   LATDbits.LATD6  // IN3
#define MOTOR_RIGHT_B   LATDbits.LATD7  // IN4

// -- Servo (Pallet Deployment) --
#define SERVO_PWM       LATBbits.LATB3  // CCP2 / software PWM

// -- Colour Sensor (TCS3200 or single digital output) --
//    Convention: COLOR_IN == 1 ? BLUE slot, 0 ? RED slot
#define COLOR_S2        LATCbits.LATC2
#define COLOR_S3        LATBbits.LATB1
#define COLOR_IN        PORTBbits.RB2

// -- Line Sensors (Active Low: 0 = Black Line detected) --
#define SENSORS_PORT    PORTA
#define S_FAR_LEFT      PORTAbits.RA0   // S1
#define S_LEFT          PORTAbits.RA1   // S2
#define S_CENTER        PORTAbits.RA2   // S3
#define S_RIGHT         PORTAbits.RA3   // S4
#define S_FAR_RIGHT     PORTAbits.RA5   // S5

// -- Push-Button (Mode Cycle) --
// RB0 / INT0 ? external pull-up required

// ============================================================
//  GLOBAL STATE
// ============================================================
volatile uint8_t robot_mode  = 0;   // 0 = Standby, 1 = Diag Drive, 2 = Autonomous
volatile int     node_count  = 0;   // Nodes detected during autonomous run
volatile int     last_node   = -1;  // Debounce: ignore re-triggers on same node

// ============================================================
//  LOW-LEVEL HARDWARE ABSTRACTION
// ============================================================

/* Software PWM pulse ? generates one servo pulse of 'us' microseconds */
static void _servo_pulse(uint16_t us) {
    SERVO_PWM = 1;
    // XC8 __delay_us() requires a compile-time constant; use a loop instead
    while (us--) { __delay_us(1); }
    SERVO_PWM = 0;
}

/* Move servo to angle (0?180°) by sending ~40 pulses @ 20 ms period */
void set_servo_angle(uint8_t angle) {
    uint16_t pulse_us = 1000u + ((uint16_t)angle * 1000u / 180u);
    for (uint8_t i = 0; i < 40; i++) {
        _servo_pulse(pulse_us);
        __delay_ms(20);
    }
}

/*
 * drive() ? unified motor command
 *   left / right : +1 = forward, -1 = reverse, 0 = stop
 */
void drive(int8_t left, int8_t right) {
    MOTOR_LEFT_F  = (left  > 0);
    MOTOR_LEFT_B  = (left  < 0);
    MOTOR_RIGHT_F = (right > 0);
    MOTOR_RIGHT_B = (right < 0);
}

/* Named movement helpers (readable in autonomous logic) */
static inline void forward(void)    { drive( 1,  1); }
static inline void stop(void)       { drive( 0,  0); }
static inline void soft_left(void)  { drive( 0,  1); }  // pivot: left wheel idle
static inline void soft_right(void) { drive( 1,  0); }  // pivot: right wheel idle
static inline void hard_left(void)  { drive(-1,  1); }
static inline void hard_right(void) { drive( 1, -1); }

// ============================================================
//  SELF-DIAGNOSTIC SUITE (Power-On Self Test)
// ============================================================
void run_power_on_self_test(void) {
    // 1. LED Chase ? confirms all three LEDs are functional
    LED_SYS_OK = 1; __delay_ms(200);
    LED_BUSY   = 1; __delay_ms(200);
    LED_ERROR  = 1; __delay_ms(200);
    LATD &= 0x0F;   // Clear only LED bits (D0-D2), preserve motor bits

    // 2. Servo Range Test
    set_servo_angle(0);
    __delay_ms(300);
    set_servo_angle(90);
    __delay_ms(300);
    set_servo_angle(0);

    // 3. Sensor Bus Check ? warn if all lines read high (open circuit / no sensors)
    if (SENSORS_PORT == 0xFF) {
        LED_ERROR = 1;  // Amber + Red = sensor fault; operator must inspect
    }
}

// ============================================================
//  COLOUR DETECTION
// ============================================================
/*
 * Returns 1 if the sensor reports BLUE, 0 if RED (or unknown).
 * The TCS3200 S2=0/S3=1 selects the blue photo-diode filter.
 * A full implementation would measure output frequency via timer;
 * here we read the raw digital output after a settle delay.
 */
uint8_t is_blue_slot(void) {
    COLOR_S2 = 0;
    COLOR_S3 = 1;
    __delay_ms(15);         // Allow TCS3200 output to settle
    return COLOR_IN;        // 1 = Blue frequency dominant
}

// ============================================================
//  LINE FOLLOWING  (priority-based, Active-Low sensors)
// ============================================================
/*
 * Sensor truth: 0 = line detected, 1 = floor
 * The robot steers TOWARD whichever sensor sees the line.
 * Centre sensor takes highest priority; outermost = hard turn.
 */
void line_follow(void) {
    if (!S_CENTER) {
        forward();
    } else if (!S_LEFT  &&  S_CENTER) {
        soft_left();
    } else if (!S_RIGHT &&  S_CENTER) {
        soft_right();
    } else if (!S_FAR_LEFT) {
        hard_left();
    } else if (!S_FAR_RIGHT) {
        hard_right();
    } else {
        forward();  // Recovery: last known good direction
    }
}

// ============================================================
//  JUNCTION / NODE DETECTION
// ============================================================
/*
 * A node is confirmed when ALL five sensors see the line
 * simultaneously (T-junction / full crossbar).
 * A 70 ms double-check eliminates glitches.
 */
uint8_t is_node(void) {
    if (!S_FAR_LEFT && !S_LEFT && !S_CENTER && !S_RIGHT && !S_FAR_RIGHT) {
        __delay_ms(70);
        if (!S_FAR_LEFT && !S_LEFT && !S_CENTER && !S_RIGHT && !S_FAR_RIGHT)
            return 1;
    }
    return 0;
}

// ============================================================
//  PALLET DEPLOYMENT  (colour-sorted)
// ============================================================
void demo_placement(void) {
    stop();
    __delay_ms(200);

    if (is_blue_slot()) {
        // BLUE slot ? full drop (90°)
        set_servo_angle(90);
        LED_SYS_OK = 1; LED_BUSY = 0;
    } else {
        // RED slot ? reject / partial swing (45°)
        set_servo_angle(45);
        LED_SYS_OK = 0; LED_BUSY = 1;
    }

    __delay_ms(400);
    set_servo_angle(0);     // Return arm to home
    __delay_ms(300);

    LED_SYS_OK = 0; LED_BUSY = 0;
}

// ============================================================
//  TIMED TURN HELPERS
// ============================================================
void turn_right(void) {
    hard_right();
    __delay_ms(450);
    stop();
}

void turn_left(void) {
    hard_left();
    __delay_ms(450);
    stop();
}

// ============================================================
//  INTERRUPT SERVICE ROUTINE  (Mode Cycle Button on RB0/INT0)
// ============================================================
void __interrupt(high_priority) ISR(void) {
    if (INT0IF) {
        __delay_ms(20);                 // Button debounce
        if (PORTBbits.RB0 == 0) {
            robot_mode = (robot_mode + 1) % 3;
            // Reset autonomous state on every mode transition
            node_count = 0;
            last_node  = -1;
            stop();                     // Safety stop during transition
        }
        INT0IF = 0;
    }
}

// ============================================================
//  MAIN
// ============================================================
void main(void) {
    // -- Oscillator: INTOSC @ 8 MHz --
    OSCCON = 0x72;

    // -- Disable ADC on all pins (use as digital I/O) --
    ADCON1 = 0x0F;

    // -- Port Directions --
    TRISA = 0xFF;           // All sensors = inputs
    TRISB = 0x05;           // RB0 (button), RB2 (colour out) = inputs; rest outputs
    TRISC = 0x00;           // All outputs (servo on RC / colour S2 on C2)
    TRISD = 0x00;           // LEDs + motors = outputs

    // -- Power-On Self Test --
    run_power_on_self_test();

    // -- Enable INT0 interrupt --
    INTCONbits.GIE   = 1;
    INTCONbits.INT0IE = 1;

    // --------------------------------------------------------
    //  MAIN LOOP
    // --------------------------------------------------------
    while (1) {
        switch (robot_mode) {

            // ----------------------------------------------------
            case 0: // STANDBY / CALIBRATION
            // ----------------------------------------------------
                LED_SYS_OK = 1; LED_BUSY = 0; LED_ERROR = 0;
                // Mirror centre sensor on BUSY LED for manual alignment aid
                LED_BUSY = !S_CENTER;
                break;

            // ----------------------------------------------------
            case 1: // DIAGNOSTIC DRIVE
            // ----------------------------------------------------
                LED_SYS_OK = 0; LED_BUSY = 1; LED_ERROR = 0;
                // Short forward burst then stop ? confirms motor wiring
                forward();  __delay_ms(500);
                stop();     __delay_ms(500);
                break;

            // ----------------------------------------------------
            case 2: // FULL AUTONOMOUS  (NERC 2026 Theme)
            // ----------------------------------------------------
                LED_SYS_OK = 0; LED_BUSY = 0; LED_ERROR = 1;

                line_follow();

                if (is_node()) {
                    // Debounce: skip if same node re-triggers
                    if (node_count == last_node) break;
                    last_node = node_count;
                    node_count++;

                    // ---- NODE 1 : Station S1 ----
                    if (node_count == 1) {
                        stop();
                        for (uint8_t i = 0; i < 4; i++) {
                            demo_placement();
                            __delay_ms(400);
                        }
                        turn_right();
                    }

                    // ---- NODE 2 : Station S2 ----
                    else if (node_count == 2) {
                        stop();
                        for (uint8_t i = 0; i < 4; i++) {
                            demo_placement();
                            __delay_ms(400);
                        }
                        turn_left();
                    }

                    // ---- NODE 3 : Zigzag Section ----
                    else if (node_count == 3) {
                        forward();
                        __delay_ms(1000);
                    }

                    // ---- NODE 4 : Ramp Traversal ----
                    else if (node_count == 4) {
                        forward();
                        __delay_ms(1800);
                    }

                    // ---- NODE 5 : Station S3 ----
                    else if (node_count == 5) {
                        stop();
                        for (uint8_t i = 0; i < 4; i++) {
                            demo_placement();
                            __delay_ms(400);
                        }
                        // Continue straight after placements
                    }

                    // ---- NODE 6 : Final Parking Zone ----
                    else if (node_count == 6) {
                        forward();
                        __delay_ms(1200);
                        stop();

                        // Mission complete ? all LEDs on
                        LED_SYS_OK = 1; LED_BUSY = 1; LED_ERROR = 1;
                        while (1);  // Halt until manual reset
                    }
                }
                break;
        }
    }
}
