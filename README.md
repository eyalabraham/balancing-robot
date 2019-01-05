# Two-wheel balancing robot
This AVR ATmega328p software is the control program for a 2-wheel balancing robot. The robot is managed by a RaspberryPi connected to an ATmega328P, which runs this code. The AVR manages all control loops and is supervised through a Raspberry Pi host.
AVR tasks inclyde:
1. read MPU-6050 sensor
2. read wheel encoder counters
3. compute PID, and control motors
4. process commands coming from RPi

Steps 1 to 3 are executed in a 50Hz ISR and step 4 is the main-loop program section

```
 *                               +-----------+
 *                  +--< I2C >---+ MPU-6050A |
 * +-----+          |  +-----+   +-----------+
 * |     |          |  |     |
 * | RPi |          +--+ AVR |
 * |     |             |     |
 * |     +--< UART >---+     |
 * |     |             |     |
 * |     +--< SPI  >---+     |
 * |     |             |     |
 * +-----+             +--+--+
 *                        |
 *              2x PWM channels to motor power board
 *              2x Hall Effect sensor inputs from wheel encoder
 *
```
## ATmega AVR IO
```
 *
 * Port B bit assignment
 *
 * b7 b6 b5 b4 b3 b2 b1 b0
 * |  |  |  |  |  |  |  |
 * |  |  |  |  |  |  |  +--- 'o' right motor dir \ 0 or 3=stop, 1=fwd, 2=rev
 * |  |  |  |  |  |  +------ 'o' right motor dir /
 * |  |  |  |  |  +--------- 'o' PID cycle test point
 * |  |  |  |  +------------ \
 * |  |  |  +---------------  | SPI and GPIO pins for in circuit serial programmer
 * |  |  +------------------ /
 * |  +--------------------- 'o' left motor dir  \ 0 or 3=stop, 1=fwd, 2=rev
 * +------------------------ 'o' left motor dir  /
 *
 * Port D bit assignment
 *
 * b7 b6 b5 b4 b3 b2 b1 b0
 * |  |  |  |  |  |  |  |
 * |  |  |  |  |  |  |  +--- UART Rx
 * |  |  |  |  |  |  +------ UART Tx
 * |  |  |  |  |  +--------- 'i' Right wheel Hall Effect sensor (INT0)
 * |  |  |  |  +------------ 'i' Left wheel Hall Effect sensor (INT1)
 * |  |  |  +--------------- 'o' batt. ok/low      0=batt.ok,1=batt.low (red LED)
 * |  |  +------------------ 'o' OC0A right PWM
 * |  +--------------------- 'o' OC0B left PWM
 * +------------------------ 'o' run/not-run       0=not running,1=running (red LED)
 *
 * note: all references to data sheet are for ATmega 328P Rev. 8161D–AVR–10/09
 *
 * Command line list see source or type 'help'<CR>
```
## Control loops
```

                |
'tilt_offset' -(+)
                |
 +-------------(+)          'orient_Uk'
 |              |               |
 |              |               |
 |         [ Tilt PID ]         |                                       +-----------+
 |              |               +---+                                   |           |
 |           'ang_Uk'           |   |                   'pwmLeft'       |           |
 |              |           +--(-)- | ---[ PWM ]----[ left motor  ]-----+           |
 |              |           |       |                                   |           |
 |              +-----------+   ----+                                   | Robot     |
 |                          |   |                       'pwmRight'      | Platform  |
 |                          +--(+)-------[ PWM ]----[ right motor ]-----+           |
 |                                                                      |           |
 |                                                                      |           |
 |                          +-[ Gyroscope     ]-------------------------+           |
 +----[ Kalman filter ]-----+                                           |           |
                            +-[ Accelerometer ]-------------------------+           |
                                                                        |           |
                                                                        +-----------+
```


