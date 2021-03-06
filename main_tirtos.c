/*
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*
 *  ======== main_tirtos.c ========
 */
#include <stdint.h>

/* POSIX Header files */
#include <pthread.h>

/* RTOS header files */
#include <ti/sysbios/BIOS.h>
#include <ti/display/Display.h>
#include <ti/drivers/GPIO.h>
#include <ti/drivers/Power.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/PWM.h>
#include <ti/net/tls.h>

/* Example/Board Header files */
#include "Board.h"


#include "Board.h"
#include "certs.h"
#include "aws_iot_config.h"

/*
 * The following macro is disabled by default. This is done to prevent the
 * certificate files from being written to flash every time the program
 * is run.  If an update to the cert files are needed, just update the
 * corresponding arrays, and rebuild with this macro defined. Note
 * you must remember to disable it otherwise the files will keep being
 * overwritten each time.
 */
#ifdef OVERWRITE_CERTS
static bool overwriteCerts = true;
#else
static bool overwriteCerts = false;
#endif

Display_Handle AWSIOT_display;

extern void NetWiFi_init(void);
extern void runAWSClient(void);

extern void *i2SmainThread(void *arg0);

/*
 *  ======== flashCerts ========
 *  Utility function to flash the contents of a buffer (PEM format) into the
 *  filename/path specified by certName (DER format)
 */
void flashCerts(uint8_t *certName, uint8_t *buffer, uint32_t bufflen)
{
    int status = 0;
    int16_t slStatus = 0;
    SlFsFileInfo_t fsFileInfo;

    /* Check if the cert file already exists */
    slStatus = sl_FsGetInfo(certName, 0, &fsFileInfo);

    /* If the cert doesn't exist, write it (or overwrite if specified to) */
    if (slStatus == SL_ERROR_FS_FILE_NOT_EXISTS || overwriteCerts == true) {

        Display_printf(AWSIOT_display, 0, 0, "Flashing certificate file ...");

        /* Convert the cert to DER format and write to flash */
        status = TLS_writeDerFile(buffer, bufflen, TLS_CERT_FORMAT_PEM,
                (const char *)certName);

        if (status != 0) {
            Display_printf(AWSIOT_display, 0, 0,
                    "Error: Could not write file %s to flash (%d)\n",
                    certName, status);
            while(1);
        }
        Display_printf(AWSIOT_display, 0, 0, " successfully wrote file %s to flash\n",
                certName);
    }
}

/*
 *  ======== awsThreadFxn ========
 */
void *awsThreadFxn(void *arg0)
{
    /* Initialize SimpleLink */
    NetWiFi_init();

    /* Flash Certificate Files */
    flashCerts((uint8_t *)AWS_IOT_ROOT_CA_FILENAME, (uint8_t *)root_ca_pem,
            strlen(root_ca_pem));

    flashCerts((uint8_t *)AWS_IOT_CERTIFICATE_FILENAME,
            (uint8_t *)client_cert_pem, strlen(client_cert_pem));

    flashCerts((uint8_t *)AWS_IOT_PRIVATE_KEY_FILENAME,
            (uint8_t *)client_private_key_pem, strlen(client_private_key_pem));

    runAWSClient();

    return (NULL);
}

/*
 *  ======== main ========
 */
int main(void)
{
// start original
    pthread_attr_t pthreadAttrs;
    pthread_t slThread;
    pthread_t awsThread;
    struct sched_param  priParam;
    int status;

    Board_initGeneral();

    GPIO_init();
    PWM_init();
    SPI_init();
    Display_init();

    GPIO_write(Board_LED0, Board_LED_OFF);
    GPIO_write(Board_LED1, Board_LED_OFF);
    GPIO_write(Board_LED2, Board_LED_OFF);

    /* Open the display for output */
    AWSIOT_display = Display_open(Display_Type_UART, NULL);
    if (AWSIOT_display == NULL) {
        /* Failed to open display driver */
        while (1);
    }


    // GPIO_write(Board_LED0, Board_LED_ON);

    /* Create the sl_Task thread */
    pthread_attr_init(&pthreadAttrs);

    status = pthread_attr_setdetachstate(&pthreadAttrs, PTHREAD_CREATE_DETACHED);
    if (status != 0) {
        while (1);
    }

    priParam.sched_priority = 1;
    pthread_attr_setschedparam(&pthreadAttrs, &priParam);

    status = pthread_attr_setstacksize(&pthreadAttrs, 2048);
    if (status != 0) {
        /* Error setting stack size */
        while (1);
    }

    status = pthread_create(&slThread, &pthreadAttrs, sl_Task, NULL);
    if (status != 0) {
        /* Failed to create sl_Task thread */
        while (1);
    }

    /* Create the AWS thread */
    status = pthread_attr_setstacksize(&pthreadAttrs, 3328);
    if (status != 0) {
        /* Error setting stack size */
        while (1);
    }

    status = pthread_create(&awsThread, &pthreadAttrs, awsThreadFxn, NULL);
    if (status != 0) {
        /* Failed to create AWS thread */
        while (1);
    }

    pthread_attr_destroy(&pthreadAttrs);

    /*  To enable low power mode, uncomment the following line.
     *  Please be aware that your JTAG connection will be
     *  dropped when entering low power mode. You must reset the
     *  board in order to re-establish your JTAG connection.
     */
    /* Power_enablePolicy(); */

    BIOS_start();
// end original
    return (0);
}
