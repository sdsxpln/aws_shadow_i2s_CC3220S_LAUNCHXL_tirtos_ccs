/*
 *    ======== i2ctmp006.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>
#include <ti/drivers/I2C.h>
#include <ti/display/Display.h>

/* Example/Board Header files */
#include "Board.h"

extern Display_Handle display;

#define TMP006_DIE_TEMP     0x0001  /* Die Temp Result Register */

/*
 *  ======== mainThread ========
 */
void *i2SmainThread(void *arg0)
{
    unsigned int    i;
    uint16_t        temperature;
    uint8_t         txBuffer[1];
    uint8_t         rxBuffer[2];
    I2C_Handle      i2c;
    I2C_Params      i2cParams;
    I2C_Transaction i2cTransaction;

    /* Call driver init functions */
/*
    Display_init();
    GPIO_init();
    I2C_init();

    // Open the HOST display for output
    display = Display_open(Display_Type_UART, NULL);
    if (display == NULL) {
        while (1);
    }

    // Turn on user LED
    GPIO_write(Board_GPIO_LED0, Board_GPIO_LED_ON);
*/
    I2C_init();
    Display_printf(display, 0, 0, "Starting the i2ctmp006 example\n");

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        Display_printf(display, 0, 0, "Error Initializing I2C\n");
        while (1);
    }
    else {
        Display_printf(display, 0, 0, "I2C Initialized!\n");
    }

    /* Point to the T ambient register and read its 2 bytes */
    txBuffer[0] = TMP006_DIE_TEMP;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

    /* Take 20 samples and print them out onto the console */
    for (i = 0; i < 20; i++) {
        if (I2C_transfer(i2c, &i2cTransaction)) {
            /* Extract degrees C from the received data; see TMP102 datasheet */
            temperature = (rxBuffer[0] << 6) | (rxBuffer[1] >> 2);

            /*
             * If the MSB is set '1', then we have a 2's complement
             * negative value which needs to be sign extended
             */
            if (rxBuffer[0] & 0x80) {
                temperature |= 0xF000;
            }
           /*
            * For simplicity, divide the temperature value by 32 to get rid of
            * the decimal precision; see TI's TMP006 datasheet
            */
            temperature /= 32;

            Display_printf(display, 0, 0, "Sample %u: %d (C)\n", i, temperature);
        }
        else {
            Display_printf(display, 0, 0, "I2C Bus fault\n");
        }

        /* Sleep for 1 second */
        sleep(1);
    }

    /* Deinitialized I2C */
    I2C_close(i2c);
    Display_printf(display, 0, 0, "I2C closed!\n");

    return (0);
}
