#ifndef MX4405XC_H
#define MX4405XC_H

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include <functional>

#define SAMPLE_PERIOD_MS        200

#define I2C_SCL_IO              22  //19               /*!< gpio number for I2C master clock */
#define I2C_SDA_IO              21  //18               /*!< gpio number for I2C master data  */
#define I2C_FREQ_HZ             400000           /*!< I2C master clock frequency */
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


// MAX4005XC defines
#define MXC4005_REG_XOUT_UPPER      0x03
#define MXC4005_REG_XOUT_LOWER      0x04
#define MXC4005_REG_YOUT_UPPER      0x05
#define MXC4005_REG_YOUT_LOWER      0x06
#define MXC4005_REG_ZOUT_UPPER      0x07
#define MXC4005_REG_ZOUT_LOWER      0x08
#define MXC4005_REG_TOUT            0x09

#define MXC4005_REG_INT_MASK1       0x0B
#define MXC4005_REG_INT_MASK1_BIT_DRDYE 0x01

#define MXC4005_REG_INT_CLR0        0x00
// #define MXC4005_REG_INT_CLR1        0x01

#define MXC4005_REG_INT_CLR1        0x01
#define MXC4005_REG_INT_CLR1_BIT_DRDYC  0x01

#define MXC4005_REG_CONTROL     0x0D
#define MXC4005_REG_CONTROL_MASK_FSR    GENMASK(6, 5)
#define MXC4005_CONTROL_FSR_SHIFT   5

#define MXC4005_REG_DEVICE_ID       0x0E
#define MXC4005_WHO_I_AM       0x0C
#define MXC4005_2G_MODE 0x00
#define MXC4005_4G_MODE 0x20
#define MXC4005_8G_MODE 0x40

#define MX4005_I2C_ADDR 0x15


#define MXC4005_SHXP  0
#define MXC4005_SHXM  1
#define MXC4005_SHYP  2
#define MXC4005_SHYM  3
#define MXC4005_CHORXY 6
#define MXC4005_CHORZ 7


#define MXC4005_ORXY0 4 
#define MXC4005_ORXY1 5
#define MXC4005_ORZ   6

#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_19 19
#define MX4005_INT_PIN (1ULL << GPIO_19 )


typedef struct mxc4005x_accl {
	uint16_t x;
	uint16_t y;
	uint16_t z;
} dataMX;

class MXC4005x {
public:
	bool initialiSed = false;
	bool int_byte = false;

	bool max4005x_init();
	bool readXYZ(dataMX *loadData);
	uint8_t readTemp();
	uint8_t readSingleReg(uint8_t address);
	void writeSingelReg(uint8_t address, uint8_t data);
	bool readByte(uint8_t new_data, uint8_t pos) {
		bool number = (new_data >> pos)  & 0x01;
		return number;
	};

	uint8_t setBit(uint8_t new_data, uint8_t pos){
		new_data = new_data | 1 << pos;
		return new_data;
	}

	void checkMxc();
};
extern MXC4005x mxc;
#endif