#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "mx4005xc.h"
#include "driver/gpio.h"

// #define MMA8451_I2C_ADDR     0x1C

// #define MMA8451_OUT_X_MSB        0x01
// #define WHO_AM_I_REG         0x0D
// #define XYZ_DATA_CFG_REG     0x0E
// #define  CTRL_REG1               0x2A
// #define  CTRL_REG2               0x2B
// #define  CTRL_REG3               0x2C
// #define  CTRL_REG4               0x2D
// #define  CTRL_REG5               0x2E

// #define ASLP_RATE_20MS           0x00
// #define ACTIVE_MASK              0x01

// #define DATA_RATE_80MS        0x28
// #define FULL_SCALE_2G         0x00

// #define MODS_MASK             0x03
// #define MODS1_MASK            0x02
// #define MODS0_MASK            0x01

// #define PP_OD_MASK            0x01
// #define INT_EN_DRDY_MASK      0x01

// #define INT_CFG_DRDY_MASK     0x01

// Structure to hold accelerometer data

typedef struct ACCEL_DATA {
    int16_t X;
    int16_t Y;
    int16_t Z;
} stACCEL_DATA_t;

static const char *TAG = "i2c_restart";

extern "C" {

    // static void i2c_master_init()
    // {
    //     int i2c_master_port = I2C_PORT_NUM;
    //     i2c_config_t conf;
    //     conf.mode = I2C_MODE_MASTER;
    //     conf.sda_io_num = I2C_SDA_IO;
    //     conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    //     conf.scl_io_num = I2C_SCL_IO;
    //     conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    //     conf.master.clk_speed = I2C_FREQ_HZ;
    //     conf.clk_flags = 0;
    //     // i2c_param_config(i2c_master_port, &conf);

    //     esp_err_t err = i2c_param_config(I2C_PORT_NUM, &conf);
    //     if (err != ESP_OK) {
    //         ESP_LOGE( TAG , "Driver is not being initialised");
    //         return;
    //     }
    //     i2c_driver_install(i2c_master_port, conf.mode,
    //                        I2C_RX_BUF_DISABLE, I2C_TX_BUF_DISABLE, 0);
    // }


    static bool mxc4005_is_writeable_reg(unsigned int reg)
    {
        switch (reg) {
        case MXC4005_REG_INT_CLR1:
        case MXC4005_REG_INT_MASK1:
        case MXC4005_REG_CONTROL:
            return true;
        default:
            return false;
        }
    }


    static bool mxc4005_is_readable_reg(unsigned int reg)
    {
        switch (reg) {
        case MXC4005_REG_XOUT_UPPER:
        case MXC4005_REG_XOUT_LOWER:
        case MXC4005_REG_YOUT_UPPER:
        case MXC4005_REG_YOUT_LOWER:
        case MXC4005_REG_ZOUT_UPPER:
        case MXC4005_REG_ZOUT_LOWER:
        case MXC4005_REG_DEVICE_ID:
        case MXC4005_REG_CONTROL:
            return true;
        default:
            return false;
        }
    }

    /**
     * @brief test code to read i2c slave device with registered interface
     * _______________________________________________________________________________________________________
     * | start | slave_addr + rd_bit +ack | register + ack | read n-1 bytes + ack | read 1 byte + nack | stop |
     * --------|--------------------------|----------------|----------------------|--------------------|------|
     *
     */


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
        // i2c_master_read_byte(cmd, data_h, ACK_VAL);
        i2c_master_read_byte(cmd, data_rd, (i2c_ack_type_t)NACK_VAL);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        return ret;
    }

    /**
     * @brief Test code to write i2c slave device with registered interface
     *        Master device write data to slave(both esp32),
     *        the data will be stored in slave buffer.
     *        We can read them out from slave buffer.
     * ____________________________________________________________________________________
     * | start | slave_addr + wr_bit + ack | register + ack | write n bytes + ack  | stop |
     * --------|---------------------------|----------------|----------------------|------|
     *
     */
    static esp_err_t i2c_master_write_slave_reg(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t i2c_reg, uint8_t* data_wr, size_t size)
    {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        // first, send device address (indicating write) & register to be written
        i2c_master_write_byte(cmd, ( i2c_addr << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        // send register we want
        i2c_master_write_byte(cmd, i2c_reg, ACK_CHECK_EN);
        // write the data
        i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG , "i2c is not writing ");
        }
        return ret;
    }


    /* Read contents of a MX register
    ---------------------------------------------------------------------------*/
    esp_err_t rdMX4005x( uint8_t reg, uint8_t *pdata, uint8_t count )
    {
        // if (!mxc.initialiSed) return !ESP_OK;
        return ( i2c_master_read_slave_reg( I2C_PORT_NUM, MX4005_I2C_ADDR,  reg, pdata, count ) );
    }

    /* Write value to specified MMA8451 register
    ---------------------------------------------------------------------------*/
    esp_err_t wrMX4005x( uint8_t reg, uint8_t *pdata, uint8_t count )
    {
        // if (!mxc.initialiSed) return !ESP_OK;
        return ( i2c_master_write_slave_reg( I2C_PORT_NUM, MX4005_I2C_ADDR,  reg, pdata, count ) );
    }

    uint16_t byte_swap( uint16_t data )
    {
        return ( (data >> 8) | (data << 8));
    }

    static void IRAM_ATTR gpio_isr_handler(void* arg)
    {
        // printf("Interrupt occured \n");
        uint32_t gpio_num = (uint32_t) arg;
        if (gpio_num == 19) {
            mxc.int_byte = true;
        }
        // switch (gpio_num) {
        // case MX4005_INT_PIN:
        // {
        //     mxc.int_byte = true;
        // }
        // break;
        //     default:
        //      mxc.int_byte = false;
        // }

    }

    void gpio_init() {

    }

}

bool MXC4005x::readXYZ(dataMX *loadData) {
    printf("Reading XYC orientation of MXC4005x\n");

    uint8_t x_t[2], y_t[2], z_t[2];
    rdMX4005x( MXC4005_REG_XOUT_UPPER, x_t , 1 );
    rdMX4005x( MXC4005_REG_XOUT_LOWER, x_t + 1, 1 );

    printf("x0 : %u \n", x_t[0]);
    printf("x1 : %u \n", x_t[1]);

    // int16_t out = x_t[1] + (x_t[0] << 8);

    uint8_t new_data = x_t[0];
    uint8_t other_data = x_t[1] ;
    other_data = other_data >> 4;
    new_data = new_data << 4;
    other_data = other_data | new_data;

    int out = (other_data) + ((x_t[0] >> 4) << 8); // conversion

    printf(" x : %d \n", out);

    rdMX4005x( MXC4005_REG_YOUT_UPPER, y_t , 1 );
    rdMX4005x( MXC4005_REG_YOUT_LOWER, y_t + 1, 1 );

    printf("y0 : %u \n", y_t[0]);
    printf("y1 : %u \n", y_t[1]);


    new_data = y_t[0];
    other_data = y_t[1] ;
    other_data = other_data >> 4;
    new_data = new_data << 4;
    other_data = other_data | new_data;

    out = (other_data) + ((x_t[0] >> 4) << 8); // conversion

    printf(" y : %d \n", out);

    rdMX4005x( MXC4005_REG_ZOUT_UPPER, z_t , 1 );
    rdMX4005x( MXC4005_REG_ZOUT_LOWER, z_t + 1, 1 );

    printf("z0 : %u \n", z_t[0]);
    printf("z1 : %u \n", z_t[1]);

    new_data = z_t[0];
    other_data = z_t[1] ;
    other_data = other_data >> 4;
    new_data = new_data << 4;
    other_data = other_data | new_data;

    out = (other_data) + ((x_t[0] >> 4) << 8); // conversion

    printf(" z : %d \n", out);

    return true;
}


uint8_t MXC4005x::readTemp() {
    uint8_t temp;
    rdMX4005x(MXC4005_REG_TOUT , &temp , 1 );
    return temp;
}

void MXC4005x::checkMxc() {
    if (int_byte == true)
    {
        int_byte = false;
        printf("MX4005_INT_PIN interrupt \n");
        uint8_t CLR0_reg = readSingleReg(MXC4005_REG_INT_CLR0);
        bool clearReg = false;
        if (readByte(CLR0_reg , MXC4005_SHXP)) {
            printf("Shake event in X+ Direction detected\n");
            clearReg = true;
        }

        if (readByte(CLR0_reg , MXC4005_SHXM)) {
            printf("Shake event in X- Direction detected\n");
            clearReg = true;
        }

        if (readByte(CLR0_reg , MXC4005_SHYP)) {
            printf("Shake event in Y+ Direction detected\n");
            clearReg = true;
        }

        if (readByte(CLR0_reg , MXC4005_SHYM)) {
            printf("Shake event in Y- Direction detected\n");
            clearReg = true;
        }

        if (readByte(CLR0_reg , MXC4005_CHORXY)) {
            printf("Orientation event in XY Direction detected\n");
            uint8_t INT_SRC1_reg = readSingleReg(MXC4005_REG_INT_CLR1);

            if (!readByte(INT_SRC1_reg , MXC4005_ORXY0) && !readByte(INT_SRC1_reg , MXC4005_ORXY1)) {
                printf("Current Orientation Is in +X Direction\n");
            } else if (readByte(INT_SRC1_reg , MXC4005_ORXY0) && !readByte(INT_SRC1_reg , MXC4005_ORXY1)) {
                printf("Current Orientation Is in +Y Direction\n");
            } else if (!readByte(INT_SRC1_reg , MXC4005_ORXY0) && readByte(INT_SRC1_reg , MXC4005_ORXY1)) {
                printf("Current Orientation Is in -X Direction\n");
            } else {
                printf("Current Orientation Is in -Y Direction\n");
            }
            clearReg = true;
        }

        if (readByte(CLR0_reg , MXC4005_CHORZ)) {
            printf("Orientation event in Z Direction detected\n");
            clearReg = true;
        }

        if (clearReg)
            writeSingelReg(MXC4005_REG_INT_CLR0, 0xff); // clear interrupt
        // uint8_t number = ((new_data >> 4)  & 0x01);
        // printf("Reg :: %2x \n", mxc.readSingleReg(MXC4005_REG_INT_CLR0));
    }
}

bool MXC4005x::max4005x_init()
{
    ESP_LOGI( TAG , "starting the code");
    uint8_t val = 0x15;

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = MX4005_INT_PIN;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add((gpio_num_t)19, gpio_isr_handler, (void*) GPIO_19);
    // i2c_master_init();
    // // Before re-configuring, must enter 'standby' mode
    rdMX4005x( MXC4005_REG_DEVICE_ID, &(val), 1 );

    if (val == 0x02) {
        ESP_LOGI( TAG, "MXC4005x ID:0x%X (ok)", val );
    } else {
        ESP_LOGE( TAG, "MXC4005x ID:0x%X !!!! (NOT correct; should be 0x02)", val );
        return false;
    }

    uint8_t controlData = 0x00; // Range = 2g, PD bit = 0 , External clock select = 0

    wrMX4005x(MXC4005_REG_CONTROL, &controlData , 1);

    mxc.writeSingelReg(MXC4005_REG_INT_CLR0, 0x00);
    return true;
}

uint8_t MXC4005x::readSingleReg(uint8_t address) {
    uint8_t new_val;
    rdMX4005x(address , &(new_val) , 1);
    return new_val;
}

void MXC4005x::writeSingelReg(uint8_t address, uint8_t data) {
    uint8_t val = data;
    wrMX4005x(address , &data , 1);
}

MXC4005x mxc;
