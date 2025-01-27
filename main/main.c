#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include "driver/ledc.h"

/* BlindsPWM
    PWM Blinds is an automatic blinds operator powered by a servo motor
    The stepper has 2 modes: ON or OFF, meaning the blinds are shut or not
    These modes are dependent on the PWM signal from channel 0
    When altered the stepper motor will effectively switch the blinds.
*/

void servoRotate_task(void *args){

    uint16_t duty = 819; // varies form 819-4096 (2.5%-12.5% of 2^15)
    uint8_t step = 7;
    uint16_t total_cycles = 467;
    bool pos_direction = true; //up or down
    uint8_t iteration_time = 10; //10 ms

    ledc_timer_config_t ledc_timer = {
        .speed_mode          = LEDC_HIGH_SPEED_MODE,
        .timer_num           = LEDC_TIMER_0,
        .duty_resolution     = LEDC_TIMER_15_BIT,
        .freq_hz             = 50,  //1/50 = 0.002 = 20ms 
        .clk_cfg             = LEDC_AUTO_CLK  
    };

    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = 25,
        .duty       = duty,
        .hpoint     = 0
    };

    ledc_channel_config(&ledc_channel);

    int i;
    while(1) {
        for (i=0; i < total_cycles; i++){
            
            pos_direction ? (duty += step) : (duty -= step);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

            vTaskDelay(iteration_time/portTICK_PERIOD_MS);
        }
        pos_direction = !pos_direction;
    }
}


void app_main(){
    xTaskCreate(&servoRotate_task,"servoRotate_task", 2048, NULL, 5, NULL); //run task
}
