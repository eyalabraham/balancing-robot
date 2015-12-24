/* avr-control.c
 *
 * this program is the control program for a 2-wheel balancing robot.
 * the robot is managed by a RaspberryPi connected to an ATmega328P,
 * which runs this code.
 * the AVR performs 2 tasks:
 * AVR: (1) read MPU-6050 sensor
 *      (2) compute PID, and control motors
 *      (3) process commands coming from RPi
 *
 * RPi: send commands to AVR for changing PID constants and getting status
 *
 *                               +-----------+
 *                  +--< I2C >---+ MPU-6050A |
 * +-----+          |  +-----+   +-----------+
 * |     |          |  |     |
 * | RPi |          +--+ AVR |
 * |     +--< UART >---+     |
 * +-----+             +--+--+
 *                        |
 *              2x PWM channels to Servo
 *
 * ATmega AVR IO
 * ---------------
 * right motor PWM          OC0A        pin 12      out
 * left motor PWM           OC0B        pin 11      out
 * right motor direction    PB0..PB1    pin 14..15  out
 * PID timing               PB2         pin 16
 * left motor direction     PB6..PB7    pin 9..10   out
 *
 * Batt low/ok              PD2         pin 4       out
 * run/not-run              PD3         pin 5       out
 *
 * TWI                      SCL         pin 28      out
 * TWI                      SDA         pin 27      in/out
 * Batt sense               ADC0        pin 23      analog in
 *
 * Port B bit assignment
 *
 * b7 b6 b5 b4 b3 b2 b1 b0
 * |  |  |  |  |  |  |  |
 * |  |  |  |  |  |  |  +--- 'o' right motor dir \ 0, 3=stop, 1=fwd, 2=rev
 * |  |  |  |  |  |  +------ 'o' right motor dir /
 * |  |  |  |  |  +--------- 'o' PID cycle test point
 * |  |  |  |  +------------ \
 * |  |  |  +---------------  | in circuit serial programmer
 * |  |  +------------------ /
 * |  +--------------------- 'o' left motor dir  \ 0, 3=stop, 1=fwd, 2=rev
 * +------------------------ 'o' left motor dir  /
 *
 * Port D bit assignment
 *
 * b7 b6 b5 b4 b3 b2 b1 b0
 * |  |  |  |  |  |  |  |
 * |  |  |  |  |  |  |  +--- UART Rx
 * |  |  |  |  |  |  +------ UART Tx
 * |  |  |  |  |  +--------- 'o' batt. ok/low      0=batt.ok,1=batt.low (red LED)
 * |  |  |  |  +------------ 'o' run/not-run       0=not running,1=running (red LED)
 * |  |  |  +--------------- -
 * |  |  +------------------ '0' OC0A right PWM
 * |  +--------------------- '0' OC0B left PWM
 * +------------------------ -
 *
 * note: all references to data sheet are for ATmega 328P Rev. 8161D–AVR–10/09
 *
 * Command line list see source or type 'help'<CR>
 * 
 */

#include    <stdint.h>
#include    <stdlib.h>
#include    <string.h>
#include    <stdio.h>
#include    <stdarg.h>
#include    <math.h>

#include    <avr/pgmspace.h>
#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <avr/wdt.h>

#include    "i2c_drv.h"
#include    "uart_drv.h"

// debug print to UART port definition
// UART is assumed to be defined and initialized
//#define     __DEBUG_PRINT__

// Gyro/Accel I2C definitions
#define     MPU_ADD         0x68    // I2C bus address for Gyro/Accel unit

#define     WHO_AM_I        0x75    // MPU identity register, should be equal upper 6-bits of GYRO_ADD
#define     MPU_ID          0x68    // MPU ID in WHO_AM_I register

#define     PWR_MGT_1       0x6b
#define     PWR_MGT_2       0x6c
#define     CONFIG          0x1a    // configuration registers
#define     GYRO_CONFIG     0x1b
#define     ACCEL_CONFIG    0x1c
#define     ACCEL_X         0x3b    // accelerometers
#define     ACCEL_Y         0x3d
#define     ACCEL_Z         0x3f
#define     ACCEL_SCALER    16384.0 // divisor to scale accelerometer reading
#define     GYRO_X          0x43    // gyro registers
#define     GYRO_Y          0x45
#define     GYRO_Z          0x47
#define     GYRO_SCALER     131.0   // divisor to scale accelerometer reading
#define     UINT2C(v)       ( (v >= 0x8000) ?  -((65535 - v) + 1) : v )  // convert to signed integer

// PID constants default
#define     KP              0.0     // PID constants
#define     KI              0.0
#define     KD              0.0
#define     MAX_INTEG       100.0
#define     RESET_INTEG     0.25
#define     LEAN            0.0     // platform leaning in [deg]
#define     PID_ANGLE_LIM   30.0    // stop running PID outside this angle in [deg]

// PID frequency Timer1 constant (sec 15.9.2 page 126..126)
#define     PID_FREQ        25      // <------PID frequency in Hz
#define     TIM1_FREQ_CONST ((PRE_SCALER / PID_FREQ) -1)
#define     PRE_SCALER      7812    // 8MHz clock divided by 1024 pre scaler
#define     LED_BLINK_RATE  2       // LED blink rate in Hz

// complementary filter constant
#define     ALPHA           0.98

// kalman filter definitions
#define     PID_LOOP_TIME   ((float)(1/(float)PID_FREQ))

// IO ports B and D initialization
#define     PB_DDR_INIT     0xc7    // port data direction
#define     PB_PUP_INIT     0x00    // port input pin pull-up
#define     PB_INIT         0x00    // port initial values

#define     PD_DDR_INIT     0x6e    // enable PD5 & 6 as outputs for OCRx PWM, PD2 & 3 for LED signals
#define     PD_PUP_INIT     0x00    // port input pin pull-up
#define     PD_INIT         0x00    // port initial values

// motor direction/state bit masks
#define     MOTOR_CLR_RIGHT 0xfc    // clear right motor direction bits (AND mask)
#define     MOTOR_CLR_LEFT  0x3f    // clear left motor direction bits (AND mask)
#define     MOTOR_CLR_DIR   0x3c    // clear all direction bits (AND mask)
#define     MOTOR_REV_RIGHT 0x01    // right motor reverse (OR masks)
#define     MOTOR_FWD_RIGHT 0x02    // right motor forward
#define     MOTOR_REV_LEFT  0x40    // left motor reverse
#define     MOTOR_FWD_LEFT  0x80    // left motor forward
#define     MOTOR_PWM_MIN   0       // initial/minimum PWM value
#define     MOTOR_PWM_MAX   255

// misc masks
#define     STAT_TOGG_BATT  0x04    // toggle battery status (XOR mask)
#define     STAT_RUN        0x08    // toggle running status (XOR mask)
#define     LOOP_TEST_POINT 0x04    // toggle PID loop timing test point

// battery voltage macros
#define     BATT_LVL_65     164     // battery threshold levels
#define     BATT_LVL_60     152     // for 6 x NiCd battery pile
#define     BATT_LVL_55     139
#define     BATT_LVL_70     177     // for LiPo batteries
#define     BATT_CONVRT     0.0394749 // ADC0 battery voltage conversion ratio
#define     BATT_BIAS       0.0     // bias to correct battery reading

// UART command line processing
#define     CLI_BUFFER      80
#define     CR              0x0d    // carriage return
#define     LF              0x0a    // line feed
#define     BELL            0x07    // bell
#define     BS              0x08    // back space
#define     SPACE           0x20    // space
#define     MAX_TOKENS      3       // max number of command line tokens
#define     CMD_DELIM       " \t"   // command line white-space delimiters
#define     PROMPT          ">"
#define     PRINT_BUFFER    64      // output print buffer

#define     SYNTAX_ERR      "syntax error.\n"
#define     HELP_TEXT       "\n\
  help                         - help text\n\
  get <kp> | <ki> | <kd>       - PID constants\n\
  get lean                     - frame leaning\n\
  get batt                     - battery voltage\n\
  get run                      - go/no-go switch\n\
  get pwmmin                   - show pwm min value\n\
  get alpha                    - print comp. filter alpha\n\
  set prompt <on> | <off>      - set prompt\n\
  set [<kp> | <ki> | <kd>] <n> - set PID constants\n\
  set lean <lean>              - set frame leaning\n\
  set pwmmin <pwm_min>         - set min. pwm value\n\
  set run <1|0>                - set run flag to 1=go, 0=stop\n\
  set alpha <a>                - set filter alpha\n"

/****************************************************************************
  special function prototypes
****************************************************************************/
// This function is called upon a HARDWARE RESET:
void reset(void) __attribute__((naked)) __attribute__((section(".init3")));

/****************************************************************************
  Globals
****************************************************************************/
volatile    uint8_t uBattery = 0;   // battery voltage read from ADC
volatile    int     nSensorErr = 0; // MPU-6050 read error
uint8_t     runFlag = 0;            // run flag reflecting the go/no-go switch state
int         nDoPrompt = 1;          // print prompt
int         blink;                  // LED blink rate

// union to allow register burst read
#define     MPU_REGS        sizeof(struct mpuReg_t)  // MPU register data buffer size in bytes
union mpu6050_t                                 // union data structure
{
    struct mpuReg_t
    {
        uint16_t    ACCEL_XOUT;                 // register list
        uint16_t    ACCEL_YOUT;
        uint16_t    ACCEL_ZOUT;
        uint16_t    TEMP_OUT;
        uint16_t    GYRO_XOUT;
        uint16_t    GYRO_YOUT;
    } mpuReg;

    uint8_t     mpuData[MPU_REGS];              // data buffer reference
} mpu6050;

float Accel_x;                  // accelerometer X, TIMER1 ISR global variable
float Accel_y;                  // accelerometer Y, TIMER1 ISR global variable
float Accel_z;                  // accelerometer Z, TIMER1 ISR global variable
float Gyro_x;                   // gyroscope X, TIMER1 ISR global variable
float Gyro_y;                   // gyroscope X, TIMER1 ISR global variable

float alpha = (float) ALPHA;    // complementary filter constant

// PID formula variable
float Ek;
float SEk  = 0.0;
float DEk  = 0.0;
float Ek_1 = 0.0;
float Uk;

volatile    float Kp, Ki, Kd;   // PID factors
volatile    float lean;         // platform lean constant
volatile    int   pwm_min;

// Kalman filter variable
int   kalmanResetGuard = 0;     // Kalman filter reset guard
float Q_angle = 0.001;          // Process noise variance for the accelerometer
float Q_gyro  = 0.003;          // Process noise variance for the gyro bias
float R_angle = 0.03;           // Measurement noise variance - this is the variance of the measurement noise
float angle   = 0.0;
float bias    = 0.0;
float P_00    = 0.0,
      P_01    = 0.0,
      P_10    = 0.0,
      P_11    = 0.0;

/* ----------------------------------------------------------------------------
 * ioinit()
 *
 *  initialize IO interfaces
 *  timer and data rates calculated based on 4MHz internal clock
 *
 */
void ioinit(void)
{
    // reconfigure system clock scaler to 8MHz
    CLKPR = 0x80;   // change clock scaler (sec 8.12.2 p.37)
    CLKPR = 0x00;

    // initialize UART interface to 19200 BAUD, 8 bit, 1 stop, no parity
    uart_initialize();

    // initialize I2C interface for master mode
    i2c_m_initialize();

    // initialize Timer0 to provide 2 independent PWM signals for motor control
    // (section 14.7.3 p.101)
    // - base PWM frequency at 500Hz
    // - OC0A -> right motor
    // - OC0B -> left motor
    OCR0A  = MOTOR_PWM_MIN; // initialize output compare registers
    OCR0B  = MOTOR_PWM_MIN;
    TCNT0  = 0;    // initialize counter register
    TIMSK0 = 0;    // no interrupts
    TCCR0A = 0xa3; // 'fast PWM' clear OC0x on compare
    TCCR0B = 0x03; // compare on OCRx, use clock with scaler=3 (pwm Fc=500Hz) and start timer

    // initialize Timer1 to provide a periodic interrupt for PID
    // with Clear Timer on Compare Match (CTC) Mode (sec 15.9.2 p.125)
    // the interrupt routine will drive the PID control loop:
    // - read tilt/gyro
    // - calculate PID values
    // - drive PWM
    TCNT1  = 0;                 // zero initial counter value
    OCR1A  = TIM1_FREQ_CONST;   // OCR1A value for PID ISR frequency
    TCCR1A = 0x00;              // CTC mode with OC1x pins kept in normal IO port mode (not used by timer)
    TCCR1B = 0x0D;              // use OCR1A for compare and internal clock with scaler=1024 (256uSec resolution) and start timer
    TCCR1C = 0;
    TIMSK1 = 0x02;              // interrupt on OCR1A match

    // initialize ADC converter input ADC0
    ADMUX  = 0x60;  // external AVcc reference, left adjusted result, ADC0 source
    ADCSRA = 0xEF;  // enable auto-triggered conversion and interrupts, ADC clock 62.5KHz @ 8MKz system clock
    ADCSRB = 0x00;  // auto trigger source is free-running

    // initialize general IO PB and PD pins for output
    // - PB0, PB1: output, no pull-up, right and left motor fwd/rev control
    // -
    DDRB  = PB_DDR_INIT;            // PB pin directions
    PORTB = PB_INIT | PB_PUP_INIT;  // initial value of pins is '0', and input with pull-up

    DDRD   = PD_DDR_INIT;           // PD data direction
    PORTD  = PD_INIT | PD_PUP_INIT; // initial value of pins is '0', and input with pull-up
}

/* ----------------------------------------------------------------------------
 * reset()
 *
 *  Clear SREG_I on hardware reset.
 *  source: http://electronics.stackexchange.com/questions/117288/watchdog-timer-issue-avr-atmega324pa
 */
void reset(void)
{
     cli();
    // Note that for newer devices (any AVR that has the option to also
    // generate WDT interrupts), the watchdog timer remains active even
    // after a system reset (except a power-on condition), using the fastest
    // prescaler value (approximately 15 ms). It is therefore required
    // to turn off the watchdog early during program startup.
    MCUSR = 0; // clear reset flags
    wdt_disable();
}

/* ----------------------------------------------------------------------------
 * read_mpu_2c()
 *
 * read 2's complement accelerometer data
 *
 */
/*
int read_mpu_2c(uint8_t address, uint8_t command)
{
    static uint8_t     high;
    static uint8_t     low;
    static uint16_t    value;

    i2c_m_getByte(address, command, &high);     // read bytes
    i2c_m_getByte(address, command+1, &low);

    value = (high << 8) + low;                  // make into word

    if ( value >= 0x8000 )                      // convert to signed integer
        return -((65535 - value) + 1);
    else
        return value;
}
*/

/* ----------------------------------------------------------------------------
 * printstr()
 *
 * Send a NULL-terminated string down the UART Tx
 *
 */
void printstr(const char *s)
{

  while (*s)
    {
      if (*s == '\n')
          uart_putchr('\r');

      uart_putchr(*s++);
    }
}

/* ----------------------------------------------------------------------------
 * printstr_p()
 *
 * Same as printstr(), but the string is located in program memory,
 * so "lpm" instructions are needed to fetch it.
 *
 */
void printstr_p(const char *s)
{
  char c;

  for (c = pgm_read_byte(s); c; ++s, c = pgm_read_byte(s))
    {
      if (c == '\n')
          uart_putchr('\r');

      uart_putchr(c);
    }
}

/* ----------------------------------------------------------------------------
 * vprintfunc()
 *
 * print a formatter string
 *
 */
int vprintfunc(char *format, ...)
{
    char    text[PRINT_BUFFER] = {0};             // text buffer for printing messages
    va_list aptr;
    int     ret;

    va_start(aptr, format);
    ret = vsnprintf(text, PRINT_BUFFER, format, aptr);
    va_end(aptr);

    printstr(text);
    
    return(ret);
}

/* ----------------------------------------------------------------------------
 * printfloat()
 *
 * print a floating point number
 *
 */
void printfloat(float val)
{
    int     d1, d2;
    float   f2;
    char    sig;

    d1 = (int) val;
    f2 = val - d1;
    d2 = (int) (f2 * 1000.0);
    sig = (val < 0) ? '-' : '+';
    d1 = abs(d1);
    d2 = abs(d2);
    vprintfunc("%c%d.%03d", sig, d1, d2);
}

/* ----------------------------------------------------------------------------
 * process_cli()
 *
 * process the command line text and execute appropriate action
 * return '-1' on syntax error, otherwise '0'
 *
 */
int process_cli(char *commandLine)
{
    char    *tokens[MAX_TOKENS] = {0, 0, 0};
    char    *token;
    char    *tempCli;
    int     i, numTokens;
    float   fbatt;

    // separate command line into tokens
    tempCli = commandLine;
    for ( numTokens = 0; numTokens < MAX_TOKENS; numTokens++, tempCli = NULL)
    {
        token = strtok(tempCli, CMD_DELIM);
        if ( token == NULL )
            break;
        tokens[numTokens] = token;
    }

    // if nothing found then this is an empty line, just exit
    if ( numTokens == 0 )
        return 0;
    
    // parse and execute command
    if ( strcmp(tokens[0], "set") == 0 && numTokens == 3 )
    {
        // set prompt <on> | <off>  - set prompt state in 'nDoPrompt'
        if ( strcmp(tokens[1], "prompt") == 0 )
        {
            if ( strcmp(tokens[2], "on") == 0 )
                nDoPrompt = 1;
            else
                nDoPrompt = 0;
        }
        // set lean <lean>          - set frame leaning level with fixed-point number (format Q10.5)
        else if ( strcmp(tokens[1], "lean") == 0 )
        {
            lean = atof(tokens[2]);
        }
        // set [<kp> | <ki> | <kd>] <num>   - set one of the PID constants with fixed-point number (format Q10.5)
        else if ( strcmp(tokens[1], "kp") == 0 )
        {
            Kp = atof(tokens[2]);
        }
        else if ( strcmp(tokens[1], "ki") == 0 )
        {
            Ki = atof(tokens[2]);
        }
        else if ( strcmp(tokens[1], "kd") == 0 )
        {
            Kd = atof(tokens[2]);
        }
        // set Complementary alpha
        else if ( strcmp(tokens[1], "alpha") == 0 )
        {
            alpha = atof(tokens[2]);
        }
        // set run flag
        else if ( strcmp(tokens[1], "run") == 0 )
        {
            runFlag = atoi(tokens[2]);
            if ( runFlag > 1 || runFlag < 0 )   // fix flag to be '0' or '1'
                runFlag = 1;
        }
        // set pwmmin <pwm_min>         - set min. pwm value
        else if ( strcmp(tokens[1], "pwmmin") == 0 )
        {
            i = atoi(tokens[2]);

            // if out of range, then reset and return syntax error
            if ( i > MOTOR_PWM_MAX || i < 0 )
                return -1;
            else
                pwm_min = i;
        }
        else
            return -1;
    }
    else if ( strcmp(tokens[0], "get") == 0 && numTokens == 2 )
    {
        // get <kp> | <ki> | <kd>   - print one of the PID constants as: fixed-point, float
        if ( strcmp(tokens[1], "kp") == 0 )
        {
            printfloat(Kp);
            vprintfunc("\n");
        }
        else if ( strcmp(tokens[1], "ki") == 0 )
        {
            printfloat(Ki);
            vprintfunc("\n");

        }
        else if ( strcmp(tokens[1], "kd") == 0 )
        {
            printfloat(Kd);
            vprintfunc("\n");
        }
        // get lean                 - print frame leaning value as: fixed-point, float
        else if ( strcmp(tokens[1], "lean") == 0 )
        {
            printfloat(lean);
            vprintfunc("\n");
        }
        // get alpha                - print Complementary filter alpha
        else if ( strcmp(tokens[1], "alpha") == 0 )
        {
            printfloat(alpha);
            vprintfunc("\n");
        }
        // get batt                 - battery voltage
        else if ( strcmp(tokens[1], "batt") == 0 )
        {
            fbatt  = ((float) uBattery * (float) BATT_CONVRT) + (float) BATT_BIAS;
            printfloat(fbatt);
            vprintfunc("\n");
        }
        // get run                  - print go/no-go state: 1 or 0
        else if ( strcmp(tokens[1], "run") == 0 )
        {
            vprintfunc("%d\n", (runFlag ? 1 : 0));
        }
        // get pwmmin                   - show pwm min value
        else if ( strcmp(tokens[1], "pwmmin") == 0 )
        {
            vprintfunc("%d\n", pwm_min);
        }
        else
            return -1;
    }
    else if ( strcmp(tokens[0], "help") == 0 )
    {
        printstr_p(PSTR(HELP_TEXT));
        return 0;
    }
    else
        return -1;

    return 0;
}

/* ----------------------------------------------------------------------------
 * kalmanFilter()
 *
 *  Kalman filter module
 *  source: http://www.x-firm.com/?page_id=148
 *
 */
float kalmanFilter(float newAngle, float newRate, float looptime)
{
    static float y, S, K_0, K_1;

    angle += looptime * (newRate - bias);

    P_00  += looptime * (looptime * P_11 - P_01 - P_10 + Q_angle);
    P_01  -= looptime * P_11;
    P_10  -= looptime * P_11;
    P_11  += Q_gyro * looptime;

    y = newAngle - angle;
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;

    angle += K_0 * y;
    bias  += K_1 * y;

    P_00  -= K_0 * P_00;
    P_01  -= K_0 * P_01;
    P_10  -= K_1 * P_00;
    P_11  -= K_1 * P_01;

    return angle;
}

/* ----------------------------------------------------------------------------
 * resetKalmanFilter()
 *
 *  reset Kalman filter parameters
 *  source: http://www.x-firm.com/?page_id=148
 *
 */
void resetKalmanFilter(float angleInit)
{
    angle   = angleInit;
    bias    = 0.0;
    P_00    = 0.0;
    P_01    = 0.0;
    P_10    = 0.0;
    P_11    = 0.0;
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when Timer 1 compare indicates the time interval
 * for running the PID tasks:
 * - read tilt/gyro
 * - compute PID
 * - set motor PWMs
 *
 * measured execution time. approximately 1.8mSec
 *
 */
ISR(TIMER1_COMPA_vect)
{
    static uint8_t pwm, motor_dir, port_b;
    static int     motor_power, mp;
    static float   distance, pitch;

    if ( runFlag )
    {
        // toggle b7 to output a cycle-test signal
        PORTB ^= LOOP_TEST_POINT;

        // burst read accelerometer and gyro
        nSensorErr = i2c_m_burstRead(MPU_ADD, ACCEL_X, MPU_REGS, mpu6050.mpuData);

        pwm = mpu6050.mpuData[0];   // convert to little endian
        mpu6050.mpuData[0] = mpu6050.mpuData[1];
        mpu6050.mpuData[1] = pwm;

        pwm = mpu6050.mpuData[2];
        mpu6050.mpuData[2] = mpu6050.mpuData[3];
        mpu6050.mpuData[3] = pwm;

        pwm = mpu6050.mpuData[4];
        mpu6050.mpuData[4] = mpu6050.mpuData[5];
        mpu6050.mpuData[5] = pwm;

        pwm = mpu6050.mpuData[10];
        mpu6050.mpuData[10] = mpu6050.mpuData[9];
        mpu6050.mpuData[11] = pwm;

        // scale readings
        Accel_x = (float) UINT2C(mpu6050.mpuReg.ACCEL_XOUT) / (float) ACCEL_SCALER;
        Accel_y = (float) UINT2C(mpu6050.mpuReg.ACCEL_YOUT) / (float) ACCEL_SCALER;
        Accel_z = (float) UINT2C(mpu6050.mpuReg.ACCEL_ZOUT) / (float) ACCEL_SCALER;
        Gyro_y  = (float) UINT2C(mpu6050.mpuReg.GYRO_YOUT) / (float) GYRO_SCALER;   // gyro rate in [deg/sec]

        // rotation calculation (http://www.hobbytronics.co.uk/accelerometer-info)
        distance = sqrt(Accel_y*Accel_y + Accel_z*Accel_z);
        pitch = atan2(Accel_x, distance) * (float) 57.2957795;    // convert angle from [rad] to [deg]

        // check if platform is out of control-limits and inhibit PID
        // if it is, then blink 'run' LED and exit PID control loop
        if ( abs(pitch) > PID_ANGLE_LIM )
        {
            blink++;
            if ( blink > (PID_FREQ / LED_BLINK_RATE) )
            {
                PORTD ^= STAT_RUN;          // toggle 'run' LED
                blink = 0;
            }
            PORTB &= MOTOR_CLR_DIR;         // stop motors
            PORTB ^= LOOP_TEST_POINT;       // toggle cycle-test signal
            kalmanResetGuard = 0;           // reset once when existing PID inhibit state
            return;
        }
        else
        {
            PORTD |= STAT_RUN;              // turn on 'run' LED
            if ( !kalmanResetGuard )        // reset only once entering back into active PID
            {
                resetKalmanFilter(pitch);
                kalmanResetGuard = 1;
            }
            Ek   = 0.0;
            SEk  = 0.0;
            DEk  = 0.0;
            Ek_1 = pitch;
        }

        // Kalman filter
        Ek = kalmanFilter(pitch, Gyro_y, PID_LOOP_TIME);

        // Complementary Filer
        // http://ozzmaker.com/2013/04/18/success-with-a-balancing-robot-using-a-raspberry-pi/
        //Ek = (alpha * (Ek_1 + (Gyro_y * PID_LOOP_TIME))) + ((1 - alpha) * pitch);

        // PID calculation
        // source: https://en.wikipedia.org/wiki/PID_controller#Discrete_implementation
        Ek  += lean;                        // offset for frame leaning
        SEk += Ek;                          // sum of error for integral part
        DEk  = (Ek - Ek_1);                 // difference of errors for differential part
        Ek_1 = Ek;

        if ( SEk > MAX_INTEG )              // prevent integrator wind-up
            SEk = MAX_INTEG;

        if ( SEk < -MAX_INTEG )
            SEk = -MAX_INTEG;

        Uk   = Kp*Ek + Ki*SEk + Kd*DEk;     // calculate PID

        // setup motor turn direction
        motor_power = round(Uk);            // convert to integer
        if ( motor_power < 0 )
            motor_dir = MOTOR_REV_RIGHT + MOTOR_REV_LEFT;
        else if ( motor_power > 0 )
            motor_dir = MOTOR_FWD_RIGHT + MOTOR_FWD_LEFT;
        else
            motor_dir = 0;

        // setup motor pwm power
        mp = abs(motor_power);
        if ( mp > (MOTOR_PWM_MAX - pwm_min) )
            mp = MOTOR_PWM_MAX - pwm_min;

        pwm = ((uint8_t) mp) + pwm_min;

        // setup IO ports for motors
        port_b = PORTB;
        port_b &= MOTOR_CLR_DIR;
        port_b |= motor_dir;

        PORTB = port_b;
        OCR0A = pwm;
        OCR0B = pwm;

        // toggle cycle-test signal
        PORTB ^= LOOP_TEST_POINT;

#ifdef __DEBUG_PRINT__                      // if tracing is on then output some data
        printfloat(Ek);                     // print angle value (PV = Process Variable)
        vprintfunc(",");
        printfloat(Uk);                     // print control value (OP = Output)
        vprintfunc("\n");
#endif  // trace is 'on'
    }
    else
    {
        PORTB &= MOTOR_CLR_DIR;                 // stop motors
        PORTD &= (~(STAT_RUN) | PB_PUP_INIT);   // turn off 'run' LED
        kalmanResetGuard = 0;
        nSensorErr = 0;
    }
}

/* ----------------------------------------------------------------------------
 * This ISR will trigger when the ADC completes a conversion
 * conversions are auto-triggered and this ISR will trigger at 31.25KHz
 * ADC result is left adjusted, so only ADCH needs to be read
 *
 */
ISR(ADC_vect)
{
    uBattery = ADCH;  // read battery voltage from ADC register
}

/* ----------------------------------------------------------------------------
 * main() control functions
 *
 * initialize IO: TWI, UART, timer and IO pins for H-bridge control
 * - TWI interface in master mode to read angle info from MPU_6050
 * - timer0 provides 2 PWM signals for motor control
 * - IO pins to control H-bridge direction
 * - UART to send/receive commands from host RaspberryPi
 *
 */
int main(void)
{
    static char commandLine[CLI_BUFFER] = {0};

    uint8_t result;
    int     nCliIndex;
    uint8_t inChar;

    // initialize command line buffer
    memset(commandLine, 0, CLI_BUFFER);
    nCliIndex = 0;
    
    // initialize IO devices
    ioinit();

    // clear console and output message
    vprintfunc("%c[2J", 27);
    printstr_p(PSTR("RPi / AVR two-wheel robot\n"));
    vprintfunc("build %s %s\n", __DATE__, __TIME__);

    // get MPU identification
    i2c_m_getByte(MPU_ADD, WHO_AM_I, &result);
    if ( result == MPU_ID )
        printstr_p(PSTR("MPU-60X0 found\n"));
    else
        printstr_p(PSTR("MPU-60X0 not found\n"));

    // print working parameters
    vprintfunc("PID frequency %dHz\n", (uint8_t) PID_FREQ);
    vprintfunc("PID Kp, Ki, Kd: ");
    printfloat((float) KP);
    vprintfunc(", ");
    printfloat((float) KI);
    vprintfunc(", ");
    printfloat((float) KD);
    vprintfunc("\n");

    // bring MPU-6050 out of sleep mode
    // other setup: resolution/accuracy/filters?
    i2c_m_sendByte(MPU_ADD, PWR_MGT_1, 0x08);
    i2c_m_sendByte(MPU_ADD, PWR_MGT_2, 0x00);

    // enable interrupts
    sei();

    // setup constants
    Kp      = (float) KP;
    Ki      = (float) KI;
    Kd      = (float) KD;
    lean    = (float) LEAN;

    alpha   = (float) ALPHA;
    kalmanResetGuard = 0;

    pwm_min = (int) MOTOR_PWM_MIN;
    runFlag = 0;

    // print command line prompt
    printstr_p(PSTR(PROMPT));
    
    // loop forever and scan for commands from RPi host
    while ( 1 )
    {
        // sample battery voltage level and reflect in LED
        if ( uBattery < BATT_LVL_70 )
            PORTD |= STAT_TOGG_BATT;
        else
            PORTD &= (~(STAT_TOGG_BATT) | PB_PUP_INIT);
        
        // sensor error
        if ( nSensorErr < 0 )
            printstr_p(PSTR("*** sense error\n"));

        // sample UART input and act on command line
        if ( uart_ischar() )
        {
            inChar = uart_getchr();
            switch ( inChar )
            {
                case CR:
                    uart_putchr(inChar);                        // output a CR-LF
                    uart_putchr(LF);
                    if ( process_cli(commandLine) == -1 )       // -- process command --
                        printstr_p(PSTR(SYNTAX_ERR));
                    memset(commandLine, 0, CLI_BUFFER);         // reinitialize command line buffer for next command
                    nCliIndex = 0;
                    if ( nDoPrompt )
                        printstr_p(PSTR(PROMPT));               // output a prompt if required
                    break;
                    
                default:
                    if ( nCliIndex < CLI_BUFFER )
                    {
                        if ( inChar != BS )                     // is character a back-space?
                            commandLine[nCliIndex++] = inChar;  // no, then store in command line buffer
                        else if ( nCliIndex > 0 )               // yes, it is a back-space, but do we have characters to remove?
                        {
                            nCliIndex--;                        // yes, remove the character
                            commandLine[nCliIndex] = 0;
                            uart_putchr(BS);
                            uart_putchr(SPACE);
                        }
                        else
                            inChar = 0;                         // no, so do nothing
                    }
                    else
                        inChar = BELL;
                    
                    uart_putchr(inChar);                        // echo character to console
            }
        } /* UART character input */
    } /* endless while loop */

    return 0;
}
