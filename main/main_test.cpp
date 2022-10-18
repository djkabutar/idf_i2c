/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "BNO080.h"
#include "mx4005xc.h"
// #include "freertos/queue.h"
#include "driver/gpio.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

#define GPIO_INPUT_IO_0 GPIO_NUM_5
#define GPIO_INPUT_PIN_SEL (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

int flag = 0;

static QueueHandle_t gpio_evt_queue = NULL;

const char *TAG = "InfixAR";
extern "C"
{
  void app_main();
}

void i2c_read(uint8_t addr);
void i2c_write(uint8_t addr, uint8_t data);
void i2c_transactions();

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
          printf("GPIO[%d] intr\n", io_num);
          vTaskDelay(100 / portTICK_PERIOD_MS);
          i2c_transactions();
        }
    }
}

static void i2c_master_init()
{
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = 1;
  conf.scl_io_num = 2;
  conf.sda_pullup_en = GPIO_PULLUP_DISABLE;
  conf.scl_pullup_en = GPIO_PULLUP_DISABLE;
  conf.master.clk_speed = 100000;
  conf.clk_flags = 0;
  i2c_param_config(I2C_NUM_0, &conf);

  esp_err_t clkStretchErr = i2c_set_timeout(I2C_NUM_0, 10000000);
  if (clkStretchErr != ESP_OK)
  {
    ESP_LOGE(TAG, "Clock stretching is not being enabled!");
  }
}

void gpio_inits()
{
  gpio_set_direction(GPIO_NUM_33, GPIO_MODE_INPUT_OUTPUT_OD);
  gpio_set_level(GPIO_NUM_33, 0);
  vTaskDelay(pdMS_TO_TICKS(200));
  gpio_set_level(GPIO_NUM_33, 1);
  vTaskDelay(pdMS_TO_TICKS(200));

  gpio_set_direction(GPIO_NUM_19, GPIO_MODE_INPUT);
  gpio_set_pull_mode(GPIO_NUM_19, GPIO_PULLUP_ONLY);
  i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void app_main(void)
{

  // i2c_master_init();
  printf("Hello world!\n");

  gpio_inits();

  i2c_master_init();

  gpio_config_t io_conf = {};
  // interrupt of rising edge
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  // bit mask of the pins, use GPIO4/5 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  // set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  // enable pull-up mode
  io_conf.pull_up_en = (gpio_pullup_t) 1;
  gpio_config(&io_conf);

  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

  // vTaskDelay(pdMS_TO_TICKS(200)/);

  // int i;
  // esp_err_t espRc = 0;
  // printf("     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\n");
  // printf("00:  ");
  // // vTaskDelay(pdMS_TO_TICKS(100));
  // for (i = 3; i < 4; i++)
  // {
  //   int d = 0x14;
  //   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  //   // i2c_master_start(cmd);
  //   // i2c_master_write_byte(cmd, (d << 1) | I2C_MASTER_WRITE, 1 /* expect ack */);
  //   // i2c_master_stop(cmd);

  //   // espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
  //   if (i % 16 == 0)
  //   {
  //     printf("\n%.2x:", i);
  //   }
  //   if (espRc == 0)
  //   {
  // wrBNO80(reset_BNO , 4);
  // return ESP_OK;
  gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_POSEDGE);
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0);
  gpio_isr_handler_remove(GPIO_INPUT_IO_0);

  gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void *)GPIO_INPUT_IO_0);

  xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

  while (1)
  {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  // }

  // else
  // {
  //   printf(" --");
  // }

  // ESP_LOGD(tag, "i=%d, rc=%d (0x%x)", i, espRc, espRc);

  // printf("Finished !! \n");
  // while (1)
  // {
  //   // TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  //   // TIMERG0.wdt_feed = 1;
  //   // TIMERG0.wdt_wprotect = 0;
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }

  // if (!mxc.max4005x_init())
  // {
  //   // SENSOR IS NOT BEING FOUND
  // }

  // BNO080 tt(0, 0, 0);
  // tt.begin();
  // // tt.enableLinearAccelerometer(50); //Send data update every 50ms
  // // tt.enableGyro(50); //Send data update every 50ms
  // // tt.enableGameRotationVector(50);
  // // cout << "Enabled GRV" << endl;
  // // tt.begin();
  // // cout << "Begun" << endl;
  // // tt.receivePacket();
  // // tt.printPacket();
  // // // tt.enableMagnetometer(50);
  // // printf("Reporting error \n");
  // // tt.reportError();
  // // tt.printPacket();
  // // printf("Printing data every some seconds\n");
  // // // tt.enableRotationVector(50);
  // // // tt.enableGyro(50);
  // tt.enableAccelerometer(50);
  // // // // vTaskDelay(2000 / portTICK_PERIOD_MS);
  // for (;;) {
  //   TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
  //   TIMERG0.wdt_feed = 1;
  //   TIMERG0.wdt_wprotect = 0;
  //   mxc.checkMxc();
  //   tt.checkBno();
  // }

  // for (int i = 10; i >= 0; i--) {
  //   printf("Restarting in %d seconds...\n", i);
  //   vTaskDelay(1000 / portTICK_PERIOD_MS);
  // }
  // printf("Restarting now.\n");
  // fflush(stdout);
  // esp_restart();
  // return 0;
}

void i2c_read(uint8_t addr)
{
  i2c_cmd_handle_t cmd;
  // Setup write 0x56 for 0x01
  {
    uint8_t data_wr[1];
    data_wr[0] = addr;
    // data_wr[1] = 0xA0;
    // uint8_t data_rd[1];

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0x2b << 1) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write(cmd, data_wr, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);

    // i2c_master_start(cmd);
    // i2c_master_write_byte(cmd, (0x2b << 1) | READ_BIT, ACK_CHECK_EN);
    // i2c_master_read_byte(cmd, data_rd, (i2c_ack_type_t)NACK_VAL);
    // // i2c_master_write_byte(cmd, 0x01, ACK_CHECK_EN);
    // i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    if (ret != ESP_OK)
    {
      ESP_LOGI(TAG, "!ESP_OK 1");
      // return ret;
    }
  }

  // vTaskDelay(30 / portTICK_RATE_MS);
  // Setup read 0x57
  {
    uint8_t data_rd[1];
    cmd = i2c_cmd_link_create();
    size_t size = 1;

    data_rd[0] = 0x57;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (0X2b << 1) | READ_BIT, ACK_CHECK_EN);
    vTaskDelay(30 / portTICK_RATE_MS);
    if (size > 1)
      i2c_master_read(cmd, data_rd, size - 1, (i2c_ack_type_t)ACK_VAL);
    i2c_master_read_byte(cmd, data_rd + (size - 1), (i2c_ack_type_t)NACK_VAL);
    // i2c_master_read(cmd, data_rd, 1, (i2c_ack_type_t) ACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
    if (ret != ESP_OK)
    {
      printf("Error in::i2c_master_read_slave_reg");
      // printf(" %.2x %d", data_rd[0], ret);
    }
    i2c_cmd_link_delete(cmd);
    printf("%d: %d\n", addr, data_rd[0]);
  }
}

void i2c_write(uint8_t addr, uint8_t data)
{
  i2c_cmd_handle_t cmd;
  {
    uint8_t data_wr[2];
    data_wr[0] = addr;
    data_wr[1] = data;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, (0x2b << 1) | WRITE_BIT, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, 2, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    printf("Sending data \n");
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "Error in::i2c_master_write_slave_reg \n");
    else
      printf("data sent! \n");
    i2c_cmd_link_delete(cmd);
  }

  // Setup write 0x56 for 0xFF
  {
    uint8_t data_wr[1];
    data_wr[0] = addr;
    // data_wr[1] = 0xA0;

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    // first, send device address (indicating write) & register to be written
    i2c_master_write_byte(cmd, (0x2b << 1) | WRITE_BIT, ACK_CHECK_EN);
    // write the data
    i2c_master_write(cmd, data_wr, 1, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    printf("Sending data \n");
    esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_RATE_MS);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "Error in::i2c_master_write_slave_reg \n");
    else
      printf("data sent! \n");
    i2c_cmd_link_delete(cmd);
  }
}

void i2c_transactions()
{
  // I2C
  printf("I2C:\n");
  i2c_write(0xFF, 0x80);
  i2c_write(0xEE, 0x01);
  i2c_write(0xFF, 0xA0);

  i2c_read(0x00);
  i2c_read(0x01);

  printf("Pixel Clock: ");
  i2c_write(0xFF, 0xA0);
  i2c_write(0x34, 0x21);
  i2c_write(0xFF, 0xB8);
  i2c_read(0xB1);
  i2c_read(0xB2);
  i2c_read(0xB3);

  i2c_write(0xFF, 0x80);
  i2c_write(0xEE, 0x00);
}