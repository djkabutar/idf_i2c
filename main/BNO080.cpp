/*
  This is a library written for the BNO080123
  SparkFun sells these at its website: www.sparkfun.com
  Do you like this library? Help support SparkFun. Buy a board!
  https://www.sparkfun.com/products/14586

  Written by Nathan Seidle @ SparkFun Electronics, December 28th, 2017

  The BNO080 IMU is a powerful triple axis gyro/accel/magnetometer coupled with an ARM processor
  to maintain and complete all the complex calculations for various VR, inertial, step counting,
  and movement operations.

  This library handles the initialization of the BNO080 and is able to query the sensor
  for different readings.

  https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library

  Development environment specifics:
  Arduino IDE 1.8.3

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// #include <sys/stat.h>
// #include <sys/ioctl.h>
// #include <unistd.h>
// #include <linux/i2c-dev.h>
// #include <stdio.h>       Standard I/O functions
#include <fcntl.h>
// #include <syslog.h>     /* Syslog functionallity */
#include <inttypes.h>
#include <errno.h>
#include <math.h>

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_log.h"
// #include "driver/i2c.h"
#include "driver/gpio.h"
#include "sdkconfig.h"

#include "BNO080.h"
#include <cstring>
#include "driver/gpio.h"

#define GPIO_INPUT_IO_1    32
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_1))

#define SAMPLE_PERIOD_MS        200

#define I2C_SCL_IO              22  //19               /*!< gpio number for I2C master clock */
#define I2C_SDA_IO              21  //18               /*!< gpio number for I2C master data  */
#define I2C_FREQ_HZ             100000           /*!< I2C master clock frequency */
#define I2C_PORT_NUM            I2C_NUM_0        /*!< I2C port number for master dev */
#define I2C_TX_BUF_DISABLE      0                /*!< I2C master do not need buffer */
#define I2C_RX_BUF_DISABLE      0                /*!< I2C master do not need buffer */

// I2C common protocol defines
#define WRITE_BIT                          I2C_MASTER_WRITE /*!< I2C master write */
#define READ_BIT                           I2C_MASTER_READ  /*!< I2C master read */
#define ACK_CHECK_EN                       0x1              /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS                      0x0              /*!< I2C master will not check ack from slave */
#define ACK_VAL                            0x0              /*!< I2C ack value */
#define NACK_VAL                           0x1              /*!< I2C nack value */

static const char *TAG = "i2c_restart";

#define BNO080_I2C_ADDR 0x4A

// #define BNOO80_LOG_ENABLED

uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0};

static esp_err_t i2c_master_read_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return ESP_OK;
    }

    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_addr << 1 | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK) {
        ESP_LOGI(TAG, "!ESP_OK 1");
        return ret;
    }
    vTaskDelay(30 / portTICK_RATE_MS);
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_addr << 1 | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
        i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t) ACK_VAL);
    i2c_master_read_byte(cmd, data_rd + (size - 1), (i2c_ack_type_t)NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK)
        printf("Error in::i2c_master_read_slave_reg");
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t i2c_master_read_bytesslave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t* data_rd, size_t size)
{
    if (size == 0) {
        return !ESP_OK;
    }
    int ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, i2c_addr << 1 | READ_BIT, ACK_CHECK_EN);
    if (size > 1)
        i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t) ACK_VAL);
    i2c_master_read_byte(cmd, data_rd + (size - 1), (i2c_ack_type_t)NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK)
        printf("Error in::i2c_master_read_bytesslave_reg");
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t* data_wr, size_t size)
{

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    printf("Sending data \n" );
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 10 / portTICK_RATE_MS);
    if (ret != ESP_OK)
        ESP_LOGE(TAG, "Error in::i2c_master_write_slave_reg \n");
    else
        printf("data sent! \n");
    i2c_cmd_link_delete(cmd);
    return ret;
}


static esp_err_t i2c_master_write_slavePacket_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t* data_wr, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    // send register we want
    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK)
        printf("Error in::i2c_master_write_slavePacket_reg");
    i2c_cmd_link_delete(cmd);
    return ret;
}

/* Read contents of a MX register
---------------------------------------------------------------------------*/
esp_err_t rdBNO80( uint8_t reg, uint8_t *pdata, uint8_t count )
{
    return ( i2c_master_read_slave_reg( I2C_PORT_NUM, BNO080_I2C_ADDR,  reg, pdata, count ) );
}


/* Read contents of a MX register
---------------------------------------------------------------------------*/
esp_err_t rdBNO80Packet( uint8_t *pdata, uint8_t count )
{
    return ( i2c_master_read_bytesslave_reg( I2C_PORT_NUM, BNO080_I2C_ADDR, pdata, count ) );
}

/* Write value to specified MMA8451 register
---------------------------------------------------------------------------*/
esp_err_t wrBNO80(uint8_t *pdata, uint8_t count )
{
    return ( i2c_master_write_slave_reg( I2C_PORT_NUM, BNO080_I2C_ADDR, pdata, count ) );
}

esp_err_t wrBNO80Packet( uint8_t *pdata, uint8_t count )
{
    return ( i2c_master_write_slavePacket_reg( I2C_PORT_NUM, BNO080_I2C_ADDR, pdata, count ) );
}

static esp_err_t i2c_master_init()
{
    // ESP_LOGD(tag, ">> i2cScanner");
    gpio_set_direction(GPIO_NUM_33, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_level(GPIO_NUM_33, 0);
    vTaskDelay(pdMS_TO_TICKS(200));
    gpio_set_level(GPIO_NUM_33, 1);
    vTaskDelay(pdMS_TO_TICKS(200));

    gpio_set_direction(GPIO_NUM_19, GPIO_MODE_INPUT);
    gpio_set_pull_mode(GPIO_NUM_19, GPIO_PULLUP_ONLY);
    i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);

    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = 21;
    conf.scl_io_num = 22;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;
    conf.clk_flags = 0;
    i2c_param_config(I2C_NUM_0, &conf);

    esp_err_t clkStretchErr = i2c_set_timeout(I2C_NUM_0, 1000000);
    if (clkStretchErr != ESP_OK) {
        ESP_LOGE(TAG, "Clock stretching is not being enabled!");
    }

    // After BNO resets it just respondes for particular time , and it goes back to sleep again.// So we need to tell the BNO to be on mode instantly.
    int i;
    esp_err_t espRc;
    printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
    printf("00:         ");
    // vTaskDelay(pdMS_TO_TICKS(100));
    for (i = 3; i < 0x78; i++) {
        int d = 0x4A;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (d << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
        i2c_master_stop(cmd);

        espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
        if (i % 16 == 0) {
            printf("\n%.2x:", i);
        }
        if (espRc == 0) {
            uint8_t action = 2;                  // 1 = reset, 2 = on; 3 = sleep
            uint8_t reset_BNO[5] = {0, 1, 0, action};
            // wrBNO80(reset_BNO , 4);
            return ESP_OK;
            printf(" %.2x", i);
        } else {
            printf(" --");
        }
        //ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);
        i2c_cmd_link_delete(cmd);
    }
    printf("\n");

    return ESP_OK;

}


void configDevice() {
    uint8_t init[16] = {16, 0, 2, 1, 0xF2, 0, 0x04, 1, 0, 0, 0, 0, 0, 0, 0, 0 };
    wrBNO80(init, 16);
}


void gpioConfig() {
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_INPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}
//CTOR DTOR already implemented in .h

//Attempt communication with the device
//Return true if we got a 'Polo' back from Marco
bool BNO080::begin()
{

    i2c_master_init();

    printf("Initialization of the i2c driver \n");
    softReset();
    flushChannel(0);
    vTaskDelay(200 / portTICK_RATE_MS);
    // turnOn();
    // receivePacket();
    printf ("******** initialize **********\n");
    // vTaskDelay(20 / portTICK_RATE_MS);
    // receivePacket();                                                                  // to report response
    // receivePacket();
    //Check communication with device
    // vTaskDelay(200 / portTICK_RATE_MS);
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    shtpData[1] = 0; //Reserved
    // //Transmit packet on channel 2, 2 bytes
    sendPacket(CHANNEL_CONTROL, 2);


    // //Now we wait for response
    printf("Here recieving packet1\n");
    if (receivePacket())
    {
        printPacket();
        printf("Here recieving packet %d %2x %2x\n", shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE, shtpData[0], SHTP_REPORT_PRODUCT_ID_RESPONSE);
        if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
        {
            printf("SW Version Major: %2x \n", shtpData[2]);
            printf(" SW Version Minor:%2x \n", shtpData[3]);
            uint32_t SW_Part_Number = ((uint32_t)shtpData[7] << 24) | ((uint32_t)shtpData[6] << 16) | ((uint32_t)shtpData[5] << 8) | ((uint32_t)shtpData[4]);
            printf(" SW Part Number: %2x \n", SW_Part_Number);
            uint32_t SW_Build_Number = ((uint32_t)shtpData[11] << 24) | ((uint32_t)shtpData[10] << 16) | ((uint32_t)shtpData[9] << 8) | ((uint32_t)shtpData[8]);
            printf(" SW Build Number: %4x \n", SW_Build_Number);
            uint16_t SW_Version_Patch = ((uint16_t)shtpData[13] << 8) | ((uint16_t)shtpData[12]);
            printf(" SW Version Patch: %2x \n", SW_Version_Patch);
            gpioConfig();
            flushChannel(0);
            flushChannel(1);
            // receivePacket();
            // printPacket();
            vTaskDelay(20 / portTICK_RATE_MS);
            return (true);
        }
    } else
        ESP_LOGE(TAG, "Receive packet error");

    vTaskDelay(20 / portTICK_RATE_MS);
    receivePacket();
    return (false); //Something went wrong
}
//Updates the latest variables if possible
//Returns false if new readings are not available
bool BNO080::dataAvailable(void)
{
    // printf("%d \n",gpio_get_level((gpio_num_t)GPIO_INPUT_IO_1));
    if (gpio_get_level((gpio_num_t)GPIO_INPUT_IO_1))
        return false;

    // printf("dataAvailable() \n");

    if (receivePacket())
    {
        printf("dataAvailable %2x \n", shtpHeader[2]);
        //Check to see if this packet is a sensor reporting its data to us
        if (shtpHeader[2] == CHANNEL_REPORTS && shtpData[0] == SHTP_REPORT_BASE_TIMESTAMP)
        {
            parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
            return (true);
        } else if (shtpHeader[2] == CHANNEL_CONTROL)
        {
            return parseCommandReport(); //This will update responses to commands, calibrationStatus, etc.
        }
        else if (shtpHeader[2] == CHANNEL_GYRO)
        {
            // parseInputReport(); //This will update the rawAccelX, etc variables depending on which feature report is found
        }
    }
    printPacket();
    return (false);
}


uint16_t BNO080::parseCommandReport(void)
{
    printf("parseCommandReport() %2x\n", shtpData[0]);
    if (shtpData[0] == SHTP_REPORT_COMMAND_RESPONSE)
    {
        //The BNO080 responds with this report to command requests. It's up to use to remember which command we issued.
        uint8_t command = shtpData[2]; //This is the Command byte of the response

        if (command == COMMAND_ME_CALIBRATE)
        {
            calibrationStatus = shtpData[5 + 0]; //R0 - Status (0 = success, non-zero = fail)
        }
        return shtpData[0];
    }
    else
    {
        printPacket();
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
    }

    //TODO additional feature reports may be strung together. Parse them all.
    return 0;
}

//This function pulls the data from the input report
//The input reports vary in length so this function stores the various 16-bit values as globals

//Unit responds with packet that contains the following:
//shtpHeader[0:3]: First, a 4 byte header
//shtpData[0:4]: Then a 5 byte timestamp of microsecond clicks since reading was taken
//shtpData[5 + 0]: Then a feature report ID (0x01 for Accel, 0x05 for Rotation Vector)
//shtpData[5 + 1]: Sequence number (See 6.5.18.2)
//shtpData[5 + 2]: Status
//shtpData[3]: Delay
//shtpData[4:5]: i/accel x/gyro x/etc
//shtpData[6:7]: j/accel y/gyro y/etc
//shtpData[8:9]: k/accel z/gyro z/etc
//shtpData[10:11]: real/gyro temp/etc
//shtpData[12:13]: Accuracy estimate
void BNO080::parseInputReport(void)
{
    //Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)shtpHeader[1] << 8 | shtpHeader[0]);
    dataLength &= ~(1 << 15); //Clear the MSbit. This bit indicates if this package is a continuation of the last.
    //Ignore it for now. TODO catch this as an error and exit

    dataLength -= 4; //Remove the header bytes from the data count

    uint8_t status = shtpData[5 + 2] & 0x03; //Get status bits
    uint16_t data1 = (uint16_t)shtpData[5 + 5] << 8 | shtpData[5 + 4];
    uint16_t data2 = (uint16_t)shtpData[5 + 7] << 8 | shtpData[5 + 6];
    uint16_t data3 = (uint16_t)shtpData[5 + 9] << 8 | shtpData[5 + 8];
    uint16_t data4 = 0;
    uint16_t data5 = 0;

    if (dataLength - 5 > 9)
    {
        data4 = (uint16_t)shtpData[5 + 11] << 8 | shtpData[5 + 10];
    }
    if (dataLength - 5 > 11)
    {
        data5 = (uint16_t)shtpData[5 + 13] << 8 | shtpData[5 + 12];
    }

    //Store these generic values to their proper global variable
    if (shtpData[5] == SENSOR_REPORTID_ACCELEROMETER)
    {
        accelAccuracy = status;
        rawAccelX = data1;
        rawAccelY = data2;
        rawAccelZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_GYROSCOPE)
    {
        gyroAccuracy = status;
        rawGyroX = data1;
        rawGyroY = data2;
        rawGyroZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_MAGNETIC_FIELD)
    {
        magAccuracy = status;
        rawMagX = data1;
        rawMagY = data2;
        rawMagZ = data3;
    }
    else if (shtpData[5] == SENSOR_REPORTID_ROTATION_VECTOR || shtpData[5] == SENSOR_REPORTID_GAME_ROTATION_VECTOR)
    {
        quatAccuracy = status;
        rawQuatI = data1;
        rawQuatJ = data2;
        rawQuatK = data3;
        rawQuatReal = data4;
        rawQuatRadianAccuracy = data5; //Only available on rotation vector, not game rot vector
    }
    else if (shtpData[5] == SENSOR_REPORTID_STEP_COUNTER)
    {
        stepCount = data3; //Bytes 8/9
    }
    else if (shtpData[5] == SENSOR_REPORTID_STABILITY_CLASSIFIER)
    {
        stabilityClassifier = shtpData[5 + 4]; //Byte 4 only
    }
    else if (shtpData[5] == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER)
    {
        activityClassifier = shtpData[5 + 5]; //Most likely state

        //Load activity classification confidences into the array
        for (uint8_t x = 0 ; x < 9 ; x++) //Hardcoded to max of 9. TODO - bring in array size
            _activityConfidences[x] = shtpData[5 + 6 + x]; //5 bytes of timestamp, byte 6 is first confidence byte
    }
    else
    {
        //This sensor report ID is unhandled.
        //See reference manual to add additional feature reports as needed
    }

    //TODO additional feature reports may be strung together. Parse them all.
}

//Return the rotation vector quaternion I
float BNO080::getQuatI()
{
    float quat = qToFloat(rawQuatI, rotationVector_Q1);
    return (quat);
}

//Return the rotation vector quaternion J
float BNO080::getQuatJ()
{
    float quat = qToFloat(rawQuatJ, rotationVector_Q1);
    return (quat);
}

//Return the rotation vector quaternion K
float BNO080::getQuatK()
{
    float quat = qToFloat(rawQuatK, rotationVector_Q1);
    return (quat);
}

//Return the rotation vector quaternion Real
float BNO080::getQuatReal()
{
    float quat = qToFloat(rawQuatReal, rotationVector_Q1);
    return (quat);
}

//Return the rotation vector accuracy
float BNO080::getQuatRadianAccuracy()
{
    float quat = qToFloat(rawQuatRadianAccuracy, rotationVector_Q1);
    return (quat);
}

//Return the acceleration component
uint8_t BNO080::getQuatAccuracy()
{
    return (accelAccuracy);
}

//Return the acceleration component
float BNO080::getAccelX()
{
    float accel = qToFloat(rawAccelX, accelerometer_Q1);
    return (accel);
}

//Return the acceleration component
float BNO080::getAccelY()
{
    float accel = qToFloat(rawAccelY, accelerometer_Q1);
    return (accel);
}

//Return the acceleration component
float BNO080::getAccelZ()
{
    float accel = qToFloat(rawAccelZ, accelerometer_Q1);
    return (accel);
}

//Return the acceleration component
uint8_t BNO080::getAccelAccuracy()
{
    return (accelAccuracy);
}

//Return the gyro component
float BNO080::getGyroX()
{
    float gyro = qToFloat(rawGyroX, gyro_Q1);
    return (gyro);
}

//Return the gyro component
float BNO080::getGyroY()
{
    float gyro = qToFloat(rawGyroY, gyro_Q1);
    return (gyro);
}

//Return the gyro component
float BNO080::getGyroZ()
{
    float gyro = qToFloat(rawGyroZ, gyro_Q1);
    return (gyro);
}

//Return the gyro component
uint8_t BNO080::getGyroAccuracy()
{
    return (gyroAccuracy);
}

//Return the magnetometer component
float BNO080::getMagX()
{
    float mag = qToFloat(rawMagX, magnetometer_Q1);
    return (mag);
}

//Return the magnetometer component
float BNO080::getMagY()
{
    float mag = qToFloat(rawMagY, magnetometer_Q1);
    return (mag);
}

//Return the magnetometer component
float BNO080::getMagZ()
{
    float mag = qToFloat(rawMagZ, magnetometer_Q1);
    return (mag);
}

//Return the mag component
uint8_t BNO080::getMagAccuracy()
{
    return (magAccuracy);
}

//Return the step count
uint16_t BNO080::getStepCount()
{
    return (stepCount);
}

//Return the stability classifier
uint8_t BNO080::getStabilityClassifier()
{
    return (stabilityClassifier);
}

//Return the activity classifier
uint8_t BNO080::getActivityClassifier()
{
    return (activityClassifier);
}

//Given a record ID, read the Q1 value from the metaData record in the FRS (ya, it's complicated)
//Q1 is used for all sensor data calculations
int16_t BNO080::getQ1(uint16_t recordID)
{
    //Q1 is always the lower 16 bits of word 7
    uint16_t q = readFRSword(recordID, 7) & 0xFFFF; //Get word 7, lower 16 bits
    return (q);
}

//Given a record ID, read the Q2 value from the metaData record in the FRS
//Q2 is used in sensor bias
int16_t BNO080::getQ2(uint16_t recordID)
{
    //Q2 is always the upper 16 bits of word 7
    uint16_t q = readFRSword(recordID, 7) >> 16; //Get word 7, upper 16 bits
    return (q);
}

//Given a record ID, read the Q3 value from the metaData record in the FRS
//Q3 is used in sensor change sensitivity
int16_t BNO080::getQ3(uint16_t recordID)
{
    //Q3 is always the upper 16 bits of word 8
    uint16_t q = readFRSword(recordID, 8) >> 16; //Get word 8, upper 16 bits
    return (q);
}

//Given a record ID, read the resolution value from the metaData record in the FRS for a given sensor
float BNO080::getResolution(uint16_t recordID)
{
    //The resolution Q value are 'the same as those used in the sensor's input report'
    //This should be Q1.
    int16_t Q = getQ1(recordID);

    //Resolution is always word 2
    uint32_t value = readFRSword(recordID, 2); //Get word 2

    float resolution = qToFloat(value, Q);

    return (resolution);
}

//Given a record ID, read the range value from the metaData record in the FRS for a given sensor
float BNO080::getRange(uint16_t recordID)
{
    //The resolution Q value are 'the same as those used in the sensor's input report'
    //This should be Q1.
    int16_t Q = getQ1(recordID);

    //Range is always word 1
    uint32_t value = readFRSword(recordID, 1); //Get word 1

    float range = qToFloat(value, Q);

    return (range);
}

//Given a record ID and a word number, look up the word data
//Helpful for pulling out a Q value, range, etc.
//Use readFRSdata for pulling out multi-word objects for a sensor (Vendor data for example)
uint32_t BNO080::readFRSword(uint16_t recordID, uint8_t wordNumber)
{
    if (readFRSdata(recordID, wordNumber, 1) == true) //Get word number, just one word in length from FRS
        return (metaData[0]); //Return this one word

    return (0); //Error
}

//Ask the sensor for data from the Flash Record System
//See 6.3.6 page 40, FRS Read Request
void BNO080::frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize)
{
    shtpData[0] = SHTP_REPORT_FRS_READ_REQUEST; //FRS Read Request
    shtpData[1] = 0; //Reserved
    shtpData[2] = (readOffset >> 0) & 0xFF; //Read Offset LSB
    shtpData[3] = (readOffset >> 8) & 0xFF; //Read Offset MSB
    shtpData[4] = (recordID >> 0) & 0xFF; //FRS Type LSB
    shtpData[5] = (recordID >> 8) & 0xFF; //FRS Type MSB
    shtpData[6] = (blockSize >> 0) & 0xFF; //Block size LSB
    shtpData[7] = (blockSize >> 8) & 0xFF; //Block size MSB

    //Transmit packet on channel 2, 8 bytes
    sendPacket(CHANNEL_CONTROL, 8);
}

//Given a sensor or record ID, and a given start/stop bytes, read the data from the Flash Record System (FRS) for this sensor
//Returns true if metaData array is loaded successfully
//Returns false if failure
bool BNO080::readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead)
{
    uint8_t spot = 0;

    //First we send a Flash Record System (FRS) request
    frsReadRequest(recordID, startLocation, wordsToRead); //From startLocation of record, read a # of words

    //Read bytes until FRS reports that the read is complete
    while (1)
    {
        //Now we wait for response
        while (1)
        {
            uint8_t counter = 0;
            while (receivePacket() == false)
            {
                if (counter++ > 100) return (false); //Give up
            }

            //We have the packet, inspect it for the right contents
            //See page 40. Report ID should be 0xF3 and the FRS types should match the thing we requested
            if (shtpData[0] == SHTP_REPORT_FRS_READ_RESPONSE)
                if ( ( (uint16_t)shtpData[13] << 8 | shtpData[12]) == recordID)
                    break; //This packet is one we are looking for
        }

        uint8_t dataLength = shtpData[1] >> 4;
        uint8_t frsStatus = shtpData[1] & 0x0F;

        uint32_t data0 = (uint32_t)shtpData[7] << 24 | (uint32_t)shtpData[6] << 16 | (uint32_t)shtpData[5] << 8 | (uint32_t)shtpData[4];
        uint32_t data1 = (uint32_t)shtpData[11] << 24 | (uint32_t)shtpData[10] << 16 | (uint32_t)shtpData[9] << 8 | (uint32_t)shtpData[8];

        //Record these words to the metaData array
        if (dataLength > 0)
        {
            metaData[spot++] = data0;
        }
        if (dataLength > 1)
        {
            metaData[spot++] = data1;
        }

        if (spot >= MAX_METADATA_SIZE)
        {
            if (_printDebug == true) puts("metaData array over run. Returning.");
            return (true); //We have run out of space in our array. Bail.
        }

        if (frsStatus == 3 || frsStatus == 6 || frsStatus == 7)
        {
            return (true); //FRS status is read completed! We're done!
        }
    }
}

void  BNO080::flushChannel(uint8_t channel){
     shtpData[0] = SENSOR_FLUSH;
     shtpData[1] = channel;

     sendPacket(CHANNEL_CONTROL, 2);
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void BNO080::softReset(void)
{
    shtpData[0] = 1; //Reset

    //Attempt to start communication with sensor
    sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte
    vTaskDelay(100 / portTICK_RATE_MS);
    //Read all incoming data and flush it
    receivePacket();
    vTaskDelay(100 / portTICK_RATE_MS);
    receivePacket();

    // while (receivePacket() == true) ;
}

//Send command to reset IC
//Read all advertisement packets from sensor
//The sensor has been seen to reset twice if we attempt too much too quickly.
//This seems to work reliably.
void BNO080::turnOn(void)
{
    shtpData[0] = 2; //On

    //Attempt to start communication with sensor
    sendPacket(CHANNEL_EXECUTABLE, 1); //Transmit packet on channel 1, 1 byte
    //Read all incoming data and flush it
    // receivePacket();
    // receivePacket() == true);
}


//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO080::resetReason()
{
    shtpData[0] = SHTP_REPORT_PRODUCT_ID_REQUEST; //Request the product ID and reset info
    shtpData[1] = 0; //Reserved

    //Transmit packet on channel 2, 2 bytes
    sendPacket(CHANNEL_CONTROL, 2);

    //Now we wait for response
    if (receivePacket() == true)
    {
        if (shtpData[0] == SHTP_REPORT_PRODUCT_ID_RESPONSE)
        {
            return (shtpData[1]);
        }
    }

    return (0);
}

//Given a register value and a Q point, convert to float
//See https://en.wikipedia.org/wiki/Q_(number_format)
float BNO080::qToFloat(int16_t fixedPointValue, uint8_t qPoint)
{
    float qFloat = fixedPointValue;
    qFloat *= pow(2, qPoint * -1);
    return (qFloat);
}

//Sends the packet to enable the rotation vector
void BNO080::enableRotationVector(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_ROTATION_VECTOR, timeBetweenReports);
}


//Sends the packet to enable the rotation vector
void BNO080::enableGameRotationVector(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
void BNO080::enableAccelerometer(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the gyro
void BNO080::enableGyro(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_GYROSCOPE, timeBetweenReports);
}

//Sends the packet to enable the magnetometer
void BNO080::enableMagnetometer(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);
}

//Sends the packet to enable the step counter
void BNO080::enableStepCounter(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);
}

//Sends the packet to enable the Stability Classifier
void BNO080::enableStabilityClassifier(uint16_t timeBetweenReports)
{
    setFeatureCommand(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);
}

//Sends the packet to enable the various activity classifiers
void BNO080::enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable, uint8_t (&activityConfidences)[9])
{
    _activityConfidences = activityConfidences; //Store pointer to array

    setFeatureCommand(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToEnable);
}

//Sends the commands to begin calibration of the accelerometer
void BNO080::calibrateAccelerometer()
{
    sendCalibrateCommand(CALIBRATE_ACCEL);
}

//Sends the commands to begin calibration of the gyro
void BNO080::calibrateGyro()
{
    sendCalibrateCommand(CALIBRATE_GYRO);
}

//Sends the commands to begin calibration of the magnetometer
void BNO080::calibrateMagnetometer()
{
    sendCalibrateCommand(CALIBRATE_MAG);
}

//Sends the commands to begin calibration of the planar accelerometer
void BNO080::calibratePlanarAccelerometer()
{
    sendCalibrateCommand(CALIBRATE_PLANAR_ACCEL);
}

//See 2.2 of the Calibration Procedure document 1000-4044
void BNO080::calibrateAll()
{
    sendCalibrateCommand(CALIBRATE_ACCEL_GYRO_MAG);
}

void BNO080::endCalibration()
{
    sendCalibrateCommand(CALIBRATE_STOP); //Disables all calibrations
}


//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports)
{
    setFeatureCommand(reportID, timeBetweenReports, 0); //No specific config
}

//Given a sensor's report ID, this tells the BNO080 to begin reporting the values
//Also sets the specific config word. Useful for personal activity classifier
void BNO080::setFeatureCommand(uint8_t reportID, uint16_t timeBetweenReports, uint32_t specificConfig)
{
    long microsBetweenReports = (long)timeBetweenReports * 1000L;

    shtpData[0] = SHTP_REPORT_SET_FEATURE_COMMAND; //Set feature command. Reference page 55
    shtpData[1] = reportID; //Feature Report ID. 0x01 = Accelerometer, 0x05 = Rotation vector
    shtpData[2] = 0; //Feature flags
    shtpData[3] = 0; //Change sensitivity (LSB)
    shtpData[4] = 0; //Change sensitivity (MSB)
    shtpData[5] = (microsBetweenReports >> 0) & 0xFF; //Report interval (LSB) in microseconds. 0x7A120 = 500ms
    shtpData[6] = (microsBetweenReports >> 8) & 0xFF; //Report interval
    shtpData[7] = (microsBetweenReports >> 16) & 0xFF; //Report interval
    shtpData[8] = (microsBetweenReports >> 24) & 0xFF; //Report interval (MSB)
    shtpData[9] = 0; //Batch Interval (LSB)
    shtpData[10] = 0; //Batch Interval
    shtpData[11] = 0; //Batch Interval
    shtpData[12] = 0; //Batch Interval (MSB)
    shtpData[13] = (specificConfig >> 0) & 0xFF; //Sensor-specific config (LSB)
    shtpData[14] = (specificConfig >> 8) & 0xFF; //Sensor-specific config
    shtpData[15] = (specificConfig >> 16) & 0xFF; //Sensor-specific config
    shtpData[16] = (specificConfig >> 24) & 0xFF; //Sensor-specific config (MSB)

    //Transmit packet on channel 2, 17 bytes
    sendPacket(CHANNEL_CONTROL, 17);
}
//   BNO080::reportError()
void BNO080::reportError() {
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST;
    shtpData[1] = 2;
    shtpData[2] = SENSOR_REPORTID_ERROR_REPORT;
    shtpData[3] = 0x00;
    shtpData[4] = 0x00;
    shtpData[5] = 0x00;
    shtpData[6] = 0x00;
    shtpData[7] = 0x00;
    shtpData[8] = 0x00;
    shtpData[9] = 0x00;
    shtpData[10] = 0x00;
    shtpData[11] = 0x00;
    sendPacket(CHANNEL_CONTROL, 12);
}

//Tell the sensor to do a command
//See 6.3.8 page 41, Command request
//The caller is expected to set P0 through P8 prior to calling
void BNO080::sendCommand(uint8_t command)
{
    shtpData[0] = SHTP_REPORT_COMMAND_REQUEST; //Command Request
    shtpData[1] = commandSequenceNumber++; //Increments automatically each function call
    shtpData[2] = command; //Command

    //Caller must set these
    /*shtpData[3] = 0; //P0
    shtpData[4] = 0; //P1
    shtpData[5] = 0; //P2
    shtpData[6] = 0;
    shtpData[7] = 0;
    shtpData[8] = 0;
    shtpData[9] = 0;
    shtpData[10] = 0;
    shtpData[11] = 0;*/

    //Transmit packet on channel 2, 12 bytes
    sendPacket(CHANNEL_CONTROL, 12);
}

//This tells the BNO080 to begin calibrating
//See page 50 of reference manual and the 1000-4044 calibration doc
void BNO080::sendCalibrateCommand(uint8_t thingToCalibrate)
{
    /*shtpData[3] = 0; //P0 - Accel Cal Enable
    shtpData[4] = 0; //P1 - Gyro Cal Enable
    shtpData[5] = 0; //P2 - Mag Cal Enable
    shtpData[6] = 0; //P3 - Subcommand 0x00
    shtpData[7] = 0; //P4 - Planar Accel Cal Enable
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/
    for (uint8_t x = 3 ; x < 12 ; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    if (thingToCalibrate == CALIBRATE_ACCEL) shtpData[3] = 1;
    else if (thingToCalibrate == CALIBRATE_GYRO) shtpData[4] = 1;
    else if (thingToCalibrate == CALIBRATE_MAG) shtpData[5] = 1;
    else if (thingToCalibrate == CALIBRATE_PLANAR_ACCEL) shtpData[7] = 1;
    else if (thingToCalibrate == CALIBRATE_ACCEL_GYRO_MAG)
    {
        shtpData[3] = 1;
        shtpData[4] = 1;
        shtpData[5] = 1;
    }
    else if (thingToCalibrate == CALIBRATE_STOP) ; //Do nothing, bytes are set to zero

    //Using this shtpData packet, send a command
    sendCommand(COMMAND_ME_CALIBRATE);
}

//This tells the BNO080 to save the Dynamic Calibration Data (DCD) to flash
//See page 49 of reference manual and the 1000-4044 calibration doc
void BNO080::saveCalibration()
{
    /*shtpData[3] = 0; //P0 - Reserved
    shtpData[4] = 0; //P1 - Reserved
    shtpData[5] = 0; //P2 - Reserved
    shtpData[6] = 0; //P3 - Reserved
    shtpData[7] = 0; //P4 - Reserved
    shtpData[8] = 0; //P5 - Reserved
    shtpData[9] = 0; //P6 - Reserved
    shtpData[10] = 0; //P7 - Reserved
    shtpData[11] = 0; //P8 - Reserved*/

    for (uint8_t x = 3 ; x < 12 ; x++) //Clear this section of the shtpData array
        shtpData[x] = 0;

    //Using this shtpData packet, send a command
    sendCommand(COMMAND_DCD); //Save DCD command
}

//Wait a certain time for incoming I2C bytes before giving up
//Returns false if failed
bool BNO080::waitForI2C()
{
//  for (uint8_t counter = 0 ; counter < 100 ; counter++) //Don't got more than 255
//  {
    // if (i2c->available() > 0) return (true);

//  }

// if(_printDebug == true) _debugPort->println(F("I2C timeout"));
    return (false);
}

//Check to see if there is any new data available
//Read the contents of the incoming packet into the shtpData array
bool BNO080::receivePacket(void)
{
#ifdef BNOO80_LOG_ENABLED
    printf("receivePacket() \n");
#endif
    uint8_t new_data[4];
    esp_err_t err = rdBNO80Packet(new_data, 4);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "There is no data from the sensor!");
    }

    //Get the first four bytes, aka the packet header
    uint8_t packetLSB = new_data[0];
    uint8_t packetMSB = new_data[1];
    uint8_t channelNumber = new_data[2];
    uint8_t sequenceNumber = new_data[3]; //Not sure if we need to store this or not

#ifdef BNOO80_LOG_ENABLED
    for (int i = 0; i < 4; i++)
        printf("%d::%02x \n", i, new_data[i]);
#endif

    // //Store the header info.
    shtpHeader[0] = packetLSB;
    shtpHeader[1] = packetMSB;
    shtpHeader[2] = channelNumber;
    shtpHeader[3] = sequenceNumber;

    // //Calculate the number of data bytes in this packet
    int16_t dataLength = ((uint16_t)packetMSB << 8 | packetLSB);
    dataLength &= ~(1 << 15);
    printf("Data length is %d\n", dataLength);
#ifdef BNOO80_LOG_ENABLED

#endif
    // if (dataLength > 128)
    //     return true;//Clear the MSbit.
    //This bit indicates if this package is a continuation of the last. Ignore it for now.
    //TODO catch this as an error and exit
    if (dataLength == 0)
    {
        return (false); //All done
    }
    dataLength -= 4; //Remove the header bytes from the data count
#ifdef BNOO80_LOG_ENABLED

#endif
    printf("Packet length is :: %d \n", dataLength);
    getData(dataLength);

    printf("Returning()\n");
    return (true); //We're done!
}

//Sends multiple requests to sensor until all data bytes are received from sensor
//The shtpData buffer has max capacity of MAX_PACKET_SIZE. Any bytes over this amount will be lost.
//Arduino I2C read limit is 32 bytes. Header is 4 bytes, so max data we can read per interation is 28 bytes
bool BNO080::getData(uint16_t bytesRemaining)
{
    printf("calling getData() %d \n", bytesRemaining);
    memset(shtpData, 0, MAX_PACKET_SIZE);
    uint16_t dataSpot = 0; //Start at the beginning of shtpData array
    //Setup a series of chunked 32 byte reads
    while (bytesRemaining > 0)
    {
        uint16_t numberOfBytesToRead = bytesRemaining;
        if (numberOfBytesToRead > (I2C_BUFFER_LENGTH - 4))
            numberOfBytesToRead = (I2C_BUFFER_LENGTH - 4);

        uint8_t new_array[numberOfBytesToRead + 5];
        rdBNO80Packet(new_array, numberOfBytesToRead + 4); // i2c->write_byte(0, (uint8_t)(numberOfBytesToRead + 4));
        //The first four bytes are header bytes and are throw away
        // i2c->read_only();
        // i2c->read_only();
        // i2c->read_only();
        // i2c->read_only();
        for (uint8_t x = 4 ; x < numberOfBytesToRead + 4 ; x++)
        {
            // uint8_t incoming = i2c->read_only();
            if (dataSpot < MAX_PACKET_SIZE)
            {
                shtpData[dataSpot++] = new_array[x]; //Store data into the shtpData array
#ifdef BNOO80_LOG_ENABLED
                // if (numberOfBytesToRead <= 16)
                //     ESP_LOGI(TAG, "shtpData[%d] = %02x\n", dataSpot - 1, new_array[x]);
                // printf("shtpData[%d] = %02x\n", dataSpot - 1, new_array[x]);
#endif
            }
        }
        bytesRemaining -= numberOfBytesToRead;
    }
    return (true); //Done!
}

//Given the data packet, send the header then the data
//Returns false if sensor does not ACK
//TODO - Arduino has a max 32 byte send. Break sending into multi packets if needed.
bool BNO080::sendPacket(uint8_t channelNumber, uint8_t dataLength)
{
    printf("sendPacket():: %d \n", dataLength);

    uint8_t packetLength = dataLength + 4; //Add four bytes for the header
    if (packetLength > I2C_BUFFER_LENGTH) return (false); //You are trying to send too much. Break into smaller packets.
    // i2c->beginTransmission(_deviceAddress);
    // Send the 4 byte packet header
    uint8_t start_packet_data[4];
    start_packet_data[0] = packetLength & 0xFF;
    start_packet_data[1] = packetLength >> 8;
    start_packet_data[2] = channelNumber;
    start_packet_data[3] = sequenceNumber[channelNumber]++;
    // i2c->write_byte(0, packetLength & 0xFF); //Packet length LSB
    // i2c->write_byte(0, packetLength >> 8); //Packet length MSB
    // i2c->write_byte(0, channelNumber); //Channel number
    // i2c->write_byte(0, sequenceNumber[channelNumber]++); //Send the sequence number, increments with each packet sent, different counter for each channel
#ifdef BNOO80_LOG_ENABLED
    printf("header packet sent \n");
#endif

    uint8_t *packet_t = (uint8_t*) malloc( packetLength + 1 );
    if (packet_t == NULL)
        return false;

    memcpy(packet_t, start_packet_data , 4);

    for (int i = 0; i < dataLength; i++)
    {
        packet_t[i + 4] = shtpData[i];
    }
    wrBNO80(packet_t , packetLength);
    //Send the user's data packet
    // uint8_t new_data[255];
    // for (uint8_t i = 0 ; i < dataLength ; i++)
    // {
    //     new_data[i] = shtpData[i];

    //     // i2c->write_byte(shtpData[i]);
    // }
// #ifdef BNOO80_LOG_ENABLED
    for (int i = 4; i < packetLength; i++) {
        printf("%2x ", packet_t[i]);
        if (packetLength % 50 == 0)
            printf("\n");
    }
    printf("\n");
// #endif
    return (true);
}

//Pretty prints the contents of the current shtp header and data packets
void BNO080::printPacket(void)
{
    if (_printDebug == true)
    {
        uint16_t packetLength = (uint16_t)shtpHeader[1] << 8 | shtpHeader[0];

        //Print the four byte header
        puts("Header:");
        for (uint8_t x = 0 ; x < 4 ; x++)
        {
            // if (shtpHeader[x] < 0x10) puts("0");
            printf("%2x ", shtpHeader[x]);
        }

        uint8_t printLength = packetLength - 4;
        if (printLength > 40) printLength = 40; //Artificial limit. We don't want the phone book.

        puts("\n Body:");
        for (uint8_t x = 0 ; x < packetLength ; x++)
        {
            // if (shtpData[x] < 0x10) puts("0");
            printf("%2x ", shtpData[x]);
            if (packetLength % 50 == 0)
                printf("\n");
        }

        if (packetLength & 1 << 15)
        {
            puts(" [Continued packet] ");
            packetLength &= ~(1 << 15);
        }

        printf(" Length:%u", packetLength);

        puts(" Channel:");
        if (shtpHeader[2] == 0) puts("Command");
        else if (shtpHeader[2] == 1) puts("Executable");
        else if (shtpHeader[2] == 2) puts("Control");
        else if (shtpHeader[2] == 3) puts("Sensor-report");
        else if (shtpHeader[2] == 4) puts("Wake-report");
        else if (shtpHeader[2] == 5) puts("Gyro-vector");
        else printf("%x", shtpHeader[2]);

        puts("");
    }

}