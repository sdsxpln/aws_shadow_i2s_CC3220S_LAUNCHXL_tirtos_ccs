#include <string.h>

#include <unistd.h>

#include <ti/drivers/I2C.h>

/* Example/Board Header files */
#include "Board.h"

#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_shadow_interface.h"
#include "aws_iot_shadow_json_data.h"
#include "aws_iot_config.h"
#include "aws_iot_mqtt_client_interface.h"

/*
 * The goal of this sample application is to demonstrate the capabilities of
 * shadow.
 * This device(say Connected Window) will open the window of a room based on
 * temperature.
 * It can report to the Shadow the following parameters:
 *  1. temperature of the room (double)
 *  2. status of the window (open or close)
 * It can act on commands from the cloud. In this case it will open or close
 * the window based on the json object "windowOpen" data[open/close]
 *
 * The two variables from a device's perspective are double temperature and
 * bool windowOpen
 * The device needs to act on only on windowOpen variable, so we will create
 * a primitiveJson_t object with callback
 *
 * The Json Document in the cloud will be
 * {
 *   "reported": {
 *     "temperature": 0,
 *     "windowOpen": false
 *   },
 *   "desired": {
 *     "windowOpen": false
 *   }
 * }
 */

#define ROOMTEMPERATURE_UPPERLIMIT 32.0f
#define ROOMTEMPERATURE_LOWERLIMIT 25.0f
#define STARTING_ROOMTEMPERATURE ROOMTEMPERATURE_LOWERLIMIT
#define MAX_LENGTH_OF_UPDATE_JSON_BUFFER 200

char HostAddress[255] = AWS_IOT_MQTT_HOST;
uint32_t port = AWS_IOT_MQTT_PORT;
uint8_t numPubs = 5;

#define TMP006_DIE_TEMP     0x0001  /* Die Temp Result Register */

uint8_t         txBuffer[1];
uint8_t         rxBuffer[2];
I2C_Handle      i2c;
I2C_Params      i2cParams;
I2C_Transaction i2cTransaction;

void initI2sTemperature();
void readI2sTemperature(float*);

void ShadowUpdateStatusCallback(const char *pThingName, ShadowActions_t action,
        Shadow_Ack_Status_t status, const char *pReceivedJsonDocument,
        void *pContextData)
{
    if (status == SHADOW_ACK_TIMEOUT) {
        IOT_INFO("Update Timeout--");
    } else if (status == SHADOW_ACK_REJECTED) {
        IOT_INFO("Update RejectedXX");
    } else if (status == SHADOW_ACK_ACCEPTED) {
        IOT_INFO("Update Accepted !!");
    }
}

void windowActuate_Callback(const char *pJsonString, uint32_t JsonStringDataLen,
      jsonStruct_t *pContext)
{
    if (pContext != NULL) {
        IOT_INFO("Delta - Window state changed to %d", *(bool *)(pContext->pData));
    }
}

void runAWSClient(void)
{
    // i2s
    // end i2s
    IoT_Error_t rc = SUCCESS;

    AWS_IoT_Client mqttClient;

    char JsonDocumentBuffer[MAX_LENGTH_OF_UPDATE_JSON_BUFFER];
    size_t sizeOfJsonDocumentBuffer = sizeof(JsonDocumentBuffer)
            / sizeof(JsonDocumentBuffer[0]);

    float fTemperature = 0.0;

    bool windowOpen = false;
    jsonStruct_t windowActuator;
    windowActuator.cb = windowActuate_Callback;
    windowActuator.pData = &windowOpen;
    windowActuator.pKey = "windowOpen";
    windowActuator.type = SHADOW_JSON_BOOL;

    jsonStruct_t temperatureHandler;
    temperatureHandler.cb = NULL;
    temperatureHandler.pKey = "temperature";
    temperatureHandler.pData = &fTemperature;
    temperatureHandler.type = SHADOW_JSON_FLOAT;

    IOT_INFO("\nAWS IoT SDK Version(dev) %d.%d.%d-%s\n", VERSION_MAJOR,
            VERSION_MINOR, VERSION_PATCH, VERSION_TAG);

    IOT_DEBUG("Using rootCA %s", AWS_IOT_ROOT_CA_FILENAME);
    IOT_DEBUG("Using clientCRT %s", AWS_IOT_CERTIFICATE_FILENAME);
    IOT_DEBUG("Using clientKey %s", AWS_IOT_PRIVATE_KEY_FILENAME);

    ShadowInitParameters_t sp = ShadowInitParametersDefault;
    sp.pHost = HostAddress;
    sp.port = port;
    sp.pClientCRT = AWS_IOT_CERTIFICATE_FILENAME;
    sp.pClientKey = AWS_IOT_PRIVATE_KEY_FILENAME;
    sp.pRootCA = AWS_IOT_ROOT_CA_FILENAME;
    sp.enableAutoReconnect = false;
    sp.disconnectHandler = NULL;

    IOT_INFO("Shadow Init");
    rc = aws_iot_shadow_init(&mqttClient, &sp);
    if (SUCCESS != rc) {
        IOT_ERROR("Shadow Initialization Error (%d)", rc);
        return;
    }

    ShadowConnectParameters_t scp = ShadowConnectParametersDefault;
    scp.pMyThingName = AWS_IOT_MY_THING_NAME;
    scp.pMqttClientId = AWS_IOT_MQTT_CLIENT_ID;
	scp.mqttClientIdLen = (uint16_t) strlen(AWS_IOT_MQTT_CLIENT_ID);

    IOT_INFO("Shadow Connect");
    rc = aws_iot_shadow_connect(&mqttClient, &scp);
    if (SUCCESS != rc) {
        IOT_ERROR("Shadow Connection Error (%d)", rc);
        return;
    }

    /*
     *  Enable Auto Reconnect functionality. Minimum and Maximum time of
     *  exponential backoff are set in aws_iot_config.h:
     *  #AWS_IOT_MQTT_MIN_RECONNECT_WAIT_INTERVAL
     *  #AWS_IOT_MQTT_MAX_RECONNECT_WAIT_INTERVAL
     */
    rc = aws_iot_shadow_set_autoreconnect_status(&mqttClient, true);
    if (SUCCESS != rc) {
        IOT_ERROR("Unable to set Auto Reconnect to true - %d", rc);
    }

    rc = aws_iot_shadow_register_delta(&mqttClient, &windowActuator);

    if (SUCCESS != rc) {
        IOT_ERROR("Shadow Register Delta Error (%d)", rc);
    }

    initI2sTemperature();

    /* loop and publish a change in temperature */
    while (NETWORK_ATTEMPTING_RECONNECT == rc || NETWORK_RECONNECTED == rc ||
            SUCCESS == rc) {
        rc = aws_iot_shadow_yield(&mqttClient, 1000);
        if (NETWORK_ATTEMPTING_RECONNECT == rc) {
            usleep(1000);
            /* If the client is attempting to reconnect, skip rest of loop */
            continue;
        }
        IOT_INFO("\n==========================================================\n");
        IOT_INFO("On Device: window state %s", windowOpen?"true":"false");

        readI2sTemperature(&fTemperature);

        rc = aws_iot_shadow_init_json_document(JsonDocumentBuffer,
                sizeOfJsonDocumentBuffer);
        if (rc == SUCCESS) {
            rc = aws_iot_shadow_add_reported(JsonDocumentBuffer,
                    sizeOfJsonDocumentBuffer, 2, &temperatureHandler,
                    &windowActuator);
            if (rc == SUCCESS) {
                rc = aws_iot_finalize_json_document(JsonDocumentBuffer,
                        sizeOfJsonDocumentBuffer);
                if (rc == SUCCESS) {
                    IOT_INFO("Update Shadow: %s", JsonDocumentBuffer);
                    rc = aws_iot_shadow_update(&mqttClient,
                            AWS_IOT_MY_THING_NAME, JsonDocumentBuffer,
                            ShadowUpdateStatusCallback, NULL, 4, true);
                }
            }
        }
        IOT_INFO("\n==========================================================\n");
        Display_doPrintf(AWSIOT_display, 0, 0, "." );
        sleep(1);
    }

    if (SUCCESS != rc) {
        IOT_ERROR("An error occurred in the loop %d", rc);
    }

    /* Deinitialized I2C */
    I2C_close(i2c);
    IOT_INFO("I2C closed!\n");

    IOT_INFO("Disconnecting");
    rc = aws_iot_shadow_disconnect(&mqttClient);

    if (SUCCESS != rc) {
        IOT_ERROR("Disconnect error %d", rc);
    }

}

void initI2sTemperature() {
    I2C_init();
    IOT_INFO("Starting the i2ctmp006 example\n");

    /* Create I2C for usage */
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;
    i2c = I2C_open(Board_I2C_TMP, &i2cParams);
    if (i2c == NULL) {
        IOT_ERROR("Error Initializing I2C\n");
        while (1);
    }
    else {
        IOT_INFO("I2C Initialized!\n");
    }

    /* Point to the T ambient register and read its 2 bytes */
    txBuffer[0] = TMP006_DIE_TEMP;
    i2cTransaction.slaveAddress = Board_TMP_ADDR;
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 2;

}

void readI2sTemperature(float* fTemperature ) {
    uint16_t temperature = (uint16_t)*fTemperature;
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

        IOT_INFO("Sample %u: (C)\n", temperature);
    }
    else {
        IOT_ERROR("I2C Bus fault\n");
    }
    *fTemperature = (float)temperature;
}
