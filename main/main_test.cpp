/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "BNO080.h"
#include "mx4005xc.h"
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"
const char * TAG = "InfixAR";
extern "C" {
  void app_main();
}

static void i2c_master_init()
{
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
}

void gpio_inits() {
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

  if (!mxc.max4005x_init())
  {
    // SENSOR IS NOT BEING FOUND
  }

  BNO080 tt(0, 0, 0);
  tt.begin();
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
  tt.enableAccelerometer(50);
  // // // vTaskDelay(2000 / portTICK_PERIOD_MS);
  for (;;) {
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    mxc.checkMxc();
    tt.checkBno();
  }

  for (int i = 10; i >= 0; i--) {
    printf("Restarting in %d seconds...\n", i);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  printf("Restarting now.\n");
  fflush(stdout);
  esp_restart();
}