/*****************************************************************************
* i2c_drv.c
*
* Driver source for AVT TWI interface
*
* This is driver is interrupt driven.
* All functionality is controlled through passing information to and
* from functions.
*
* Created: December 14, 2014
*
*****************************************************************************/

#include    <stdint.h>
#include    <string.h>
#include    <avr/io.h>
#include    <avr/interrupt.h>
#include    <util/twi.h>
#include    <avr/cpufunc.h>

#include    "i2c_drv.h"

/****************************************************************************
  TWI State codes
****************************************************************************/
// General TWI Master status codes
#define TWI_START                  0x08  // START has been transmitted
#define TWI_REP_START              0x10  // Repeated START has been transmitted
#define TWI_ARB_LOST               0x38  // Arbitration lost

// TWI Master Transmitter status codes
#define TWI_MTX_ADR_ACK            0x18  // SLA+W has been transmitted and ACK received
#define TWI_MTX_ADR_NACK           0x20  // SLA+W has been transmitted and NACK received
#define TWI_MTX_DATA_ACK           0x28  // Data byte has been transmitted and ACK received
#define TWI_MTX_DATA_NACK          0x30  // Data byte has been transmitted and NACK received

// TWI Master Receiver status codes
#define TWI_MRX_ADR_ACK            0x40  // SLA+R has been transmitted and ACK received
#define TWI_MRX_ADR_NACK           0x48  // SLA+R has been transmitted and NACK received
#define TWI_MRX_DATA_ACK           0x50  // Data byte has been received and ACK transmitted
#define TWI_MRX_DATA_NACK          0x58  // Data byte has been received and NACK transmitted

// TWI Slave Transmitter status codes
#define TWI_STX_ADR_ACK            0xA8  // Own SLA+R has been received; ACK has been returned
#define TWI_STX_ADR_ACK_M_ARB_LOST 0xB0  // Arbitration lost in SLA+R/W as Master; own SLA+R has been received; ACK has been returned
#define TWI_STX_DATA_ACK           0xB8  // Data byte in TWDR has been transmitted; ACK has been received
#define TWI_STX_DATA_NACK          0xC0  // Data byte in TWDR has been transmitted; NOT ACK has been received
#define TWI_STX_DATA_ACK_LAST_BYTE 0xC8  // Last data byte in TWDR has been transmitted (TWEA = '0'); ACK has been received

// TWI Slave Receiver status codes
#define TWI_SRX_ADR_ACK            0x60  // Own SLA+W has been received ACK has been returned
#define TWI_SRX_ADR_ACK_M_ARB_LOST 0x68  // Arbitration lost in SLA+R/W as Master; own SLA+W has been received; ACK has been returned
#define TWI_SRX_GEN_ACK            0x70  // General call address has been received; ACK has been returned
#define TWI_SRX_GEN_ACK_M_ARB_LOST 0x78  // Arbitration lost in SLA+R/W as Master; General call address has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_ACK       0x80  // Previously addressed with own SLA+W; data has been received; ACK has been returned
#define TWI_SRX_ADR_DATA_NACK      0x88  // Previously addressed with own SLA+W; data has been received; NOT ACK has been returned
#define TWI_SRX_GEN_DATA_ACK       0x90  // Previously addressed with general call; data has been received; ACK has been returned
#define TWI_SRX_GEN_DATA_NACK      0x98  // Previously addressed with general call; data has been received; NOT ACK has been returned
#define TWI_SRX_STOP_RESTART       0xA0  // A STOP condition or repeated START condition has been received while still addressed as Slave

// TWI Miscellaneous status codes
#define TWI_NO_STATE               0xF8  // No relevant state information available; TWINT = '0'
#define TWI_BUS_ERROR              0x00  // Bus error due to an illegal START or STOP condition

// TWCR bit set/reset macro
#define set_s_TWCR(sta,sto,intr,ea) (((intr)<<TWINT)|   \
                                     ((ea)<<TWEA)|      \
                                     ((sta)<<TWSTA)|    \
                                     ((sto)<<TWSTO)|    \
                                     (0<<TWWC)|         \
                                     (1<<TWEN)|         \
                                     (1<<TWIE))

#define set_m_TWCR(sta,sto,intr,ea) (((intr)<<TWINT)|   \
                                     ((ea)<<TWEA)|      \
                                     ((sta)<<TWSTA)|    \
                                     ((sto)<<TWSTO)|    \
                                     (0<<TWWC)|         \
                                     (1<<TWEN)|         \
                                     (0<<TWIE))

/****************************************************************************
  Globals
****************************************************************************/

// TWI slave globals
static  volatile uint8_t i2c_genCallActive = 0;
static  volatile uint8_t i2c_rx_busy       = 0;   // busy receiving
static  volatile uint8_t i2c_tx_busy       = 0;   // busy transmitting
static           uint8_t txDataCount       = 0;   // bytes to transmit
static           uint8_t bytesToMove;

// TWI master globals
static           uint8_t slaveAddress;
static           uint8_t data[2];

// common mode globals
static  volatile uint8_t rxIndex           = 0;   // index to data in rxBuffers
static  volatile uint8_t txIndex           = 0;   // index to data in txBuffers
static           uint8_t rxBuffer[TWI_BUFF_LEN];  // receive data buffer
static           uint8_t txBuffer[TWI_BUFF_LEN];  // transmit data buffer

/* ---------------------------------------------------------------------------
 * i2c_m_initialize()
 *
 * setup SCL clock rate for master mode
 *
 */
void i2c_m_initialize(void)
{
    TWBR = 2;   // rate divisor to yield 400KHz SCL @ 8MHz CPU clock
    TWSR = 0;
}

/* ---------------------------------------------------------------------------
 * i2c_m_getData()
 *
 * implement a Master Receive mode using a polling method.
 * data is passed in with 'data' buffer from slave 'address'.
 * function returns data bytes received or -1 on error, and
 * number of bytes read on success
 *
 */
int i2c_m_getData(uint8_t address, uint8_t *data, int byteCount)
{
    if ( (byteCount > TWI_BUFF_LEN) || (byteCount <= 0) )
        return -1;

    slaveAddress = (address << 1) + TW_READ;
    rxIndex = 0;

    TWCR = (1<<TWEN)|                       // enable TWI-interface
           (0<<TWIE)|(1<<TWINT)|            // clear INT flag
           (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)| // send s START condition
           (0<<TWWC);

MASTER_RX_LOOP:
    while ((TWCR & _BV(TWINT)) == 0);       // wait for transmission

    switch (TWSR)
    {
    /*
     * Master Receive Modes
     */
    case TWI_START:                     // -- 0x08
    case TWI_REP_START:                 // -- 0x10
        TWDR = slaveAddress;            // send SLA+R
        TWCR = set_m_TWCR(0, 0, 1, 0);
        goto MASTER_RX_LOOP;
        break;

    case TWI_ARB_LOST:                  // -- 0x38
        TWCR = set_m_TWCR(1, 0, 1, 0);  // START condition will be transmitted when the bus becomes free
        goto MASTER_RX_LOOP;
        break;

    case TWI_MRX_ADR_ACK:               // -- 0x40
        if ( rxIndex < (byteCount-1))   // ACK received for SLA+R, wait for data if buffer not full
            TWCR = set_m_TWCR(0, 0, 1, 1); // byte will be received and ACK will be returned
        else
            TWCR = set_m_TWCR(0, 0, 1, 0); // byte will be received and NACK will be returned, buffer will be full (-> status 0x58)
        goto MASTER_RX_LOOP;
        break;

    case TWI_MRX_DATA_ACK:              // -- 0x50
        rxBuffer[rxIndex++] = TWDR;     // read byte from TWDR
        if ( rxIndex < (byteCount-1))   // last byte space left in buffer?
            TWCR = set_m_TWCR(0, 0, 1, 1); // no, next byte will be received and ACK will be returned
        else
            TWCR = set_m_TWCR(0, 0, 1, 0); // yes, next byte received and NACK will be returned, buffer will be full (-> status 0x58)
        goto MASTER_RX_LOOP;
        break;

    case TWI_MRX_DATA_NACK:             // -- 0x58
        rxBuffer[rxIndex++] = TWDR;     // read last byte from TWDR
        /* no break */
    case TWI_MRX_ADR_NACK:              // -- 0x48
        TWCR = set_m_TWCR(0, 1, 1, 0);  // STOP condition will be transmitted and TWSTO Flag will be reset

        memcpy((void*)data, (const void*)rxBuffer, rxIndex);
        return rxIndex;
        break;

    default:
        TWCR = set_m_TWCR(0, 1, 1, 0);  // STOP condition will be transmitted and TWSTO Flag will be reset
        return -1;
    }
}

/* ---------------------------------------------------------------------------
 * i2c_m_setData()
 *
 * implement a Master Transmit mode using a polling method.
 * data is passed out in 'data' buffer with 'byteCount' count,
 * and sent to slave 'address'.
 * function returns data bytes sent or -1 on error.
 *
 */
int i2c_m_setData(uint8_t address, uint8_t *data, int byteCount)
{
    if ( (byteCount > TWI_BUFF_LEN) || (byteCount <= 0) )
        return -1;

    slaveAddress = (address << 1) + TW_WRITE;
    memcpy((void*)txBuffer, (const void*)data, byteCount);
    txIndex = 0;

    TWCR = (1<<TWEN)|                           // enable TWI-interface
            (0<<TWIE)|(1<<TWINT)|               // clear INT flag
            (0<<TWEA)|(1<<TWSTA)|(0<<TWSTO)|    // send s START condition
            (0<<TWWC);

MASTER_TX_LOOP:
    while ((TWCR & _BV(TWINT)) == 0);           // wait for transmission ** not time-out check !! **

    switch (TWSR)
    {
    /*
     *   Master Transmit Modes
     */
    case TWI_START:                     // -- 0x08
    case TWI_REP_START:                 // -- 0x10
        TWDR = slaveAddress;            // send SLA+W
        TWCR = set_m_TWCR(0, 0, 1, 0);
        goto MASTER_TX_LOOP;
        break;

    case TWI_MTX_ADR_ACK:               // -- 0x18
    case TWI_MTX_DATA_ACK:              // -- 0x28
        if ( txIndex < byteCount )
        {
            TWDR = txBuffer[txIndex++]; // load data into TWDR
            TWCR = set_m_TWCR(0, 0, 1, 0); // data byte will be transmitted
            goto MASTER_TX_LOOP;
        }
        else
        {
            TWCR = set_m_TWCR(0, 1, 1, 0); // STOP condition will be transmitted and TWSTO Flag will be reset
            return txIndex;                // done transmitting, exit here
        }
        break;

    case TWI_MTX_ADR_NACK:              // -- 0x20
    case TWI_MTX_DATA_NACK:             // -- 0x30
        TWCR = set_m_TWCR(0, 1, 1, 0);  // STOP condition will be transmitted and TWSTO Flag will be reset
        return txIndex;
        break;

    case TWI_ARB_LOST:                  // -- 0x38
        TWCR = set_m_TWCR(1, 0, 1, 0);  // START condition will be transmitted when the bus becomes free
        goto MASTER_TX_LOOP;
        break;

    default:
        TWCR = set_m_TWCR(0, 1, 1, 0);  // STOP condition will be transmitted and TWSTO Flag will be reset
        return -1;
    }
}

/* ---------------------------------------------------------------------------
 * i2c_m_sendByte()
 *
 * send one byte of 'command' and one 'byte' of data to slave located
 * at I2C bus 'address'
 *
 */
int i2c_m_sendByte(uint8_t address, uint8_t command, uint8_t byte)
{
    data[0] = command;
    data[1] = byte;

    return i2c_m_setData(address, data, (int) (2 * sizeof(uint8_t)));
}

/* ---------------------------------------------------------------------------
 * i2c_m_getByte()
 *
 * read a data byte from the slave. this requires a slave 'address'
 * and a 'command' to instruct the slave device which data to send.
 * data is returned in 'byte'.
 * function returns '-1' 0n failure, and '1' on success.
 *
 */
int i2c_m_getByte(uint8_t address, uint8_t command, uint8_t *byte)
{
    if ( i2c_m_setData(address, &command, (int) sizeof(uint8_t)) > 0 )  // send the command
    {
        return i2c_m_getData(address, byte, (int) sizeof(uint8_t));     // if successful, then read returned data from slave
    }

    return -1;
}

/* ---------------------------------------------------------------------------
 * i2c_m_burstRead()
 *
 * burst read a data block from the slave. this requires a slave 'address'
 * and a 'command' to instruct the slave device which data to send.
 * data is returned in 'bytes' array.
 * function returns '-1' 0n failure, and byte count on success.
 *
 */
int i2c_m_burstRead(uint8_t address, uint8_t command, uint8_t count, uint8_t *bytes)
{
    if ( i2c_m_setData(address, &command, (int) sizeof(uint8_t)) > 0 )  // send the command
    {
        return i2c_m_getData(address, bytes, (int) count);              // if successful, then read returned data from slave
    }

    return -1;
}

/* ---------------------------------------------------------------------------
 * i2c_s_initialize()
 *
 * initialize slave mode with device 'address' and a flag for
 * noting 'answerGenCall' if slave should answer General Call address.
 * AVR TWI will enter a wait for address state and wait for master
 * to initiate communication.
 * The function must have a defined callback function to handle slave
 * transmits upon SLA+R commands
 * NOTE: an sei() call MUST be initiated after calling this function.
 *
 */
void i2c_s_initialize(uint8_t address, uint8_t answerGenCall)
{
    TWAR = ((address<<1) | (answerGenCall ? 1 : 0));
    TWSR = 0;

    // Set own TWI slave address. Accept TWI General Calls.
    TWCR = (1<<TWEN)|                           // enable TWI-interface
            (1<<TWIE)|(1<<TWINT)|               // interrupts enabled and clear INT flag
            (1<<TWEA)|(0<<TWSTA)|(0<<TWSTO)|    // ack on requests
            (0<<TWWC);
}

/* ---------------------------------------------------------------------------
 * i2c_s_getData()
 *
 * read 'byteCount' bytes from the input buffer into 'data' buffer.
 * this function blocks until an SLA+W transaction is complete and
 * then reads data from the input buffer (if any)
 *
 */
int i2c_s_getData(uint8_t *data, uint8_t byteCount)
{
    if ( byteCount > TWI_BUFF_LEN )
        return 0;

    while( i2c_rx_busy || rxIndex == 0 );  // block until transaction completes

    (rxIndex > byteCount) ? (bytesToMove = byteCount) : (bytesToMove = rxIndex);
    memcpy((void*)data, (const void*)rxBuffer, bytesToMove);
    rxIndex = 0;

    return bytesToMove;
}

/* ---------------------------------------------------------------------------
 * i2c_s_setData()
 *
 * write 'byteCount' bytes to the output buffer from 'data' buffer.
 * fill the output buffer with data to send upon receiving SLA+R.
 * if there is data in the buffer with no active SLA+R transaction the data will
 * be overwritten
 *
 */
int i2c_s_setData(uint8_t *data, uint8_t byteCount)
{
    if ( byteCount > TWI_BUFF_LEN )
        return 0;

    while ( i2c_tx_busy );  // block until current SLA+R transaction completes
                            // before overwriting the txBuffer with new data

    memcpy((void*)txBuffer, (const void*)data, byteCount);
    txDataCount = byteCount;
    return byteCount;
}

/* ---------------------------------------------------------------------------
 * interrupt service routine for AVR TWI (i2c) interface.
 *
 */
ISR(TWI_vect)
{
    switch (TWSR)
    {
    /*
     *   Slave Receiver Modes
     */
    case TWI_SRX_GEN_ACK:               // -- 0x70
    case TWI_SRX_GEN_ACK_M_ARB_LOST:    // -- 0x78
        i2c_genCallActive = 1;
        /* no break */
    case TWI_SRX_ADR_ACK:               // -- 0x60
    case TWI_SRX_ADR_ACK_M_ARB_LOST:    // -- 0x68
        i2c_rx_busy = 1;                // we're stating a new transaction
        rxIndex = 0;                    // initialize buffer index
        TWCR = set_s_TWCR(0,0,1,1);     // Data byte will be received and ACK will be returned
        break;

    case TWI_SRX_GEN_DATA_NACK:         // -- 0x98
    case TWI_SRX_ADR_DATA_NACK:         // -- 0x88
        i2c_rx_busy = 0;                // completed an SLA+W transaction
        /* no break */
    case TWI_SRX_GEN_DATA_ACK:          // -- 0x90
    case TWI_SRX_ADR_DATA_ACK:          // -- 0x80
        rxBuffer[rxIndex++] = TWDR;     // read data from input to buffer
        if ( rxIndex == TWI_BUFF_LEN )
            rxIndex--;                  // crude, but will prevent buffer overrun
        TWCR = set_s_TWCR(0,0,1,1);     // Data byte will be received and ACK will be returned
        break;

    case TWI_SRX_STOP_RESTART:          // -- 0xA0
        i2c_rx_busy = 0;                // successful completed an SLA+W transaction
        TWCR = set_s_TWCR(0,0,1,1);     // in addressed Slave mode own SLA will be recognized
        break;

    /*
     *   Slave Transmit Modes
     */
    case TWI_STX_ADR_ACK:               // -- 0xA8
    case TWI_STX_ADR_ACK_M_ARB_LOST:    // -- 0xB0
        i2c_tx_busy = 1;                // we're stating a new transaction
        txIndex = 0;                    // initialize buffer index
        TWDR = txBuffer[txIndex++];     // read data from buffer to output
        if ( txDataCount == txIndex )
        {
            TWCR = set_s_TWCR(0,0,1,0); // indicate last byte transmitted
            i2c_tx_busy = 0;            // successful completed an SLA+R transaction
        }
        else
            TWCR = set_s_TWCR(0,0,1,1); // Data byte will be transmitted and ACK should be returned
        break;

    case TWI_STX_DATA_ACK:              // -- 0xB8
        TWDR = txBuffer[txIndex++];     // read next data from buffer to output
        if ( txDataCount == txIndex )
        {
            TWCR = set_s_TWCR(0,0,1,0); // indicate last byte transmitted
            i2c_tx_busy = 0;            // successful completed an SLA+R transaction
        }
        else
            TWCR = set_s_TWCR(0,0,1,1); // Data byte will be transmitted and ACK should be returned
        break;

    case TWI_STX_DATA_NACK:             // -- 0xC0
        TWCR = set_s_TWCR(0,0,1,1);     // in addressed Slave mode own SLA will be recognized
        break;

    case TWI_STX_DATA_ACK_LAST_BYTE:    // -- 0xC8
        TWCR = set_s_TWCR(0,0,1,1);     // in addressed Slave mode own SLA will be recognized
        break;

    /*
     *   Other Slave States
     */
    case TWI_BUS_ERROR:                 // -- 0c00
        i2c_rx_busy = 0;                // reset any transaction indication
        i2c_tx_busy = 0;
        txIndex = 0;
        rxIndex = 0;
        TWCR = set_s_TWCR(0,1,1,0);     // internal reset
        TWCR = set_s_TWCR(0,0,1,1);     // in addressed Slave mode own SLA will be recognized
        break;

    case TWI_NO_STATE:                  // -- 0xF8
        // do nothing
        break;

    default:;
    }
}
