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
#include "soc/timer_group_struct.h"
#include "soc/timer_group_reg.h"

extern "C" {

  void app_main();
}

void app_main(void)
{
  // i2c_master_init();
  printf("Hello world!\n");
  BNO080 tt(0, 0, 0);
  tt.begin();

  // tt.enableLinearAccelerometer(50); //Send data update every 50ms
  // tt.enableGyro(50); //Send data update every 50ms

  // tt.enableGameRotationVector(50);
  // cout << "Enabled GRV" << endl;
  // tt.begin();
  // cout << "Began" << endl;
  tt.receivePacket();
  tt.printPacket();
  // tt.enableMagnetometer(50);
  printf("Reporting error \n");
  tt.reportError();
  tt.printPacket();
  printf("Printing data every some seconds\n");
  // tt.enableRotationVector(50);
  // tt.enableGyro(50);
  tt.enableAccelerometer(50);
  // vTaskDelay(2000 / portTICK_PERIOD_MS);
  for (;;) {
    TIMERG0.wdt_wprotect = TIMG_WDT_WKEY_VALUE;
    TIMERG0.wdt_feed = 1;
    TIMERG0.wdt_wprotect = 0;
    // if (tt.receivePacket() == true)
    // {
    //   tt.printPacket();
    // }
    if (tt.dataAvailable() == true) {
      // printf("data :: %f\n", tt.getQuatI());
      // vTaskDelay(70 / portTICK_PERIOD_MS);
      // float x = tt.getGyroX();
      // float y = tt.getGyroY();
      // float z = tt.getGyroZ();
      float x = tt.getAccelX();
      float y = tt.getAccelY();
      float z = tt.getAccelZ();
      // byte linAccuracy = myIMU.getLinAccelAccuracy();
      printf("data X:: %f\n", x);
    }
  }

  for (int i = 10; i >= 0; i--) {
    printf("Restarting in %d seconds...\n", i);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  printf("Restarting now.\n");
  fflush(stdout);
  esp_restart();
}