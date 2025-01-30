#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "sdkconfig.h"
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "sys/time.h"
#include <stdio.h>
#include <string.h>

/* BlindsPWM
 *   PWM Blinds is an automatic blinds operator powered by a servo motor
 *   The servo has 2 modes: ON or OFF, meaning the blinds are open or closed
 *   These modes are dependent on the PWM signal from channel 0
 *   When altered the servo motor will effectively switch the blinds.
 */

#define LED_PIN 18
#define PWM_PIN 25
#define OPEN_BLINDS_TIME "08:00:00"
#define CLOSE_BLINDS_TIME "16:00:00"

uint16_t duty = 819; // varies form 819-4096 (2.5%-12.5% of 2^15)
uint16_t step = 7; //each step in duty increment
uint16_t total_cycles = 467; //total steps
bool pos_direction = true; //true = open, false = closed 
uint8_t iteration_time = 10; //10 ms 
char time_buf[9]; //HH:MM:SS


 //gets the running real time and formats it
void get_time(){
    struct timeval tv;
    gettimeofday(&tv, NULL);

    struct tm * timeinfo;
    timeinfo = localtime(&tv.tv_sec);

    sprintf(time_buf,"%02d:%02d:%02d",timeinfo->tm_hour
                                     ,timeinfo->tm_min
                                     ,timeinfo->tm_sec);  
}

/*
 *    Waiting function polling and comparing the correct event times with real time.
 *    then returns resulting in resuming the program
 */
void wait_for_event_time(void){
    if(!pos_direction){ //Closed
        while(1){
            get_time();
            vTaskDelay(1000/portTICK_PERIOD_MS);
            if(strcmp(time_buf,OPEN_BLINDS_TIME)==0){ //Open blinds time
                return;
            }
        }
    }
    while(1){ //Open
        get_time();
        vTaskDelay(1000/portTICK_PERIOD_MS);
        if(strcmp(time_buf,CLOSE_BLINDS_TIME)==0){ //Close blinds time
            return;
        }
    }
}

/*
 * Main function loop of the program
 * configures ledc timer and ledc pwm channel
 * if servo position is true it moves counter-clockwise
 * incrementing the dutycycle changing its angle and vice versa
 * then waits for an event time
 * when event time has accured the servo position-direction changes
 * repeat
 */

void servoRotate_task(void *args){

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
        .gpio_num   = PWM_PIN,
        .duty       = duty,
        .hpoint     = 0
    };

    ledc_channel_config(&ledc_channel);
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    int i;
    while(1) {
        for (i=0; i < total_cycles; i++){
            
            pos_direction ? (duty += step) : (duty -= step);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);

            vTaskDelay(iteration_time/portTICK_PERIOD_MS);
        }
        wait_for_event_time();
        gpio_set_level(LED_PIN, 1);

        pos_direction = !pos_direction;

        vTaskDelay(1000/portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN, 0);
    }
}


void app_main(){
    xTaskCreate(&servoRotate_task,"servoRotate_task", 2048, NULL, 5, NULL); //run task
}
