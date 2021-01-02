/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "signal.h"

#ifdef CONFIG_IDF_TARGET_ESP32
#define CHIP_NAME "ESP32"
#endif

#ifdef CONFIG_IDF_TARGET_ESP32S2BETA
#define CHIP_NAME "ESP32-S2 Beta"
#endif

spi_device_handle_t spi;
spi_bus_config_t buscfg;
spi_device_interface_config_t devcfg;
const int NUM_LEDS = 72;
int setupSPI();
int sendSPI();
void setupGPIO();
void setLED(int);
void staticpattern(uint32_t intensity);
void staticpattern_dither(uint32_t intensity);
static void periodic_timer_callback(void* arg);
uint32_t data[256];
void genpattern(uint32_t);

void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU cores, WiFi%s%s, ",
            CHIP_NAME,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");
    
    printf("Setting up SPI now... [%d]\n", setupSPI());
    printf("Setting up GPIO\n");
    setupGPIO();
    
    printf("portTICK_PERIOD_MS: \t[%d]\n", portTICK_PERIOD_MS);
    
    // Generate test pattern
    //for (int i=0; i<sig_len; i++) {
    //    sig[i] = (i % 2) * 65535;
    //}
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */
    // Theoretical max update rate limited by PWM: 4.3KHz
    // Time in us
    //ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10000)); // 100Hz sample rate
    //ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 33367)); // 29.97Hz sample rate
    //ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 8333)); // 120.005Hz sample rate
    //ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 4167)); // 240.005Hz sample rate
    //ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 2083)); // 480.005Hz sample rate
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 10000));
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    for (int i = 10; i >= 0; i--) {
        staticpattern(i);
        sendSPI();
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
/*
uint8_t sin_table[1024];
static void calc_sin_table() {
    for (int i=0; i<1024; i++) {
        float x = sinf(i * 2 * (float)M_PI / 1024.0f);
        sin_table[i] = (uint8_t)(x * 128.0f + 127.0f);
}
*/

uint32_t gamma2linear(uint32_t x, uint32_t s) {
    return x*x / s;
}

/*
// sig
// sig_len
static uint32_t get_interpolate(uint32_t t, uint32_t denom) {
    uint32_t y1 = sig[t / denom];
    uint32_t y2 = sig[t / denom + 1];
    uint32_t w2 = t % denom;
    uint32_t w1 = denom - w2;
    uint32_t y = (y1 * w1 + y2 * w2) / denom; 
    return y;
}
*/

//uint32_t curr_intensity;
uint32_t curr_idx;
static void periodic_timer_callback(void* arg) {
    curr_idx = (curr_idx + 1) % (24*512);
    //staticpattern_dither((uint32_t)bri);
    genpattern(curr_idx);
    sendSPI();
    //setLED(1);
}

void genpattern(uint32_t tstep) {
    //memset(data, 0xff, sizeof(data));
    //data[0] = 0;
    //printf("%d\n", tstep);
    for (uint32_t i=0; i<144; i++) {
        //uint32_t intensity = gamma2linear((i+tstep)%64*65, 4095);
        //uint32_t intensity = gamma2linear((i+tstep)%64*65, 4095);
        uint32_t intensity = gamma2linear((i+tstep)%24*178, 4095);
        //uint32_t color = (float)((i+tstep)%256) / 255.0f;
        float t = ((i+tstep)%512) * (float)M_PI / 256.0f;
        uint32_t cr = (uint32_t)((sinf(t) + 1.0f) * 127.0f);
        uint32_t cg = (uint32_t)((sinf( t + (float)(M_PI * 2.0 / 3.0) ) + 1.0f) * 127.0f);
        uint32_t cb = (uint32_t)((sinf( t + (float)(M_PI * 4.0 / 3.0) ) + 1.0f) * 127.0f);
        //t = (t + 1.0f) / 2.0f;
        //int tmp = (int)(t * 65535.0f);
        
        //intensity >>= 1;
        uint32_t msbs = (intensity >> 7) & 0b11111;
        uint32_t lsbs;
        if (msbs == 0) {
            msbs = 1;
            lsbs = intensity & 0x7f;
        } else {
            lsbs = 0x80 + (intensity & 0x7f); 
        }
        //int lsbs = intensity & 0xff;
        uint32_t r = (lsbs * cr) >> 8;
        uint32_t g = (lsbs * cg) >> 8; //lsbs >> 1;
        uint32_t b = (lsbs * cb) >> 8;
        data[i] = r<<24 | g<<16 | b<<8 | 0b11100000 | msbs; // RRGGBB
    }
    for (uint32_t i=144; i<256; i++) {
        data[i] = 0; // RRGGBB
    }
}


void lin2wh_0_65535(uint32_t x, uint32_t* outw8, uint32_t* outh5) {
    x = x * 65535u;
    uint32_t height = x / 138543105u + 1; // [1, 31]
    uint32_t width = x % 138543105u;
    uint32_t width8 = (width + 138543104u*height - 138543104u) / height;
    
    //*outw8 = width8 / 543306u;
    *outw8 = width8 / 541185u;
    *outh5 = height;
}

void staticpattern_dither(uint32_t intensity) {
    uint32_t pwmval;
    uint32_t currentval;

    uint32_t r;
    uint32_t g;
    uint32_t b;
    uint32_t mydata;
    data[0] = 0;
    for (uint32_t i=1; i<NUM_LEDS+1; i++) {
        lin2wh_0_65535(intensity, &pwmval, &currentval);
        //r = 0;
        //g = 255;
        //b = 0;
        //currentval = 31;
        r = pwmval;
        g = pwmval >> 1;
        b = pwmval >> 2;
        mydata = r<<24 | g<<16 | b<<8 | 0b11100000 | currentval; // RRGGBB
        data[i] = mydata;
    }
    for (uint32_t i=NUM_LEDS+1; i<128; i++) {
        data[i] = 0;
    }
}

void staticpattern(uint32_t intensity) {
    uint32_t pwmval = intensity;
    uint32_t currentval = 15;

    uint32_t r = pwmval;
    uint32_t g = pwmval;
    uint32_t b = 0;
    
    uint32_t mydata = r<<24 | g<<16 | b<<8 | 0b11100000 | currentval; // RRGGBB
    data[0] = 0;
    for (uint32_t i=1; i<NUM_LEDS+1; i++) {
        data[i] = mydata;
    }
    for (uint32_t i=NUM_LEDS+1; i<128; i++) {
        data[i] = 0;
    }
}

#define BLINK_GPIO 12
void setupGPIO()
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(BLINK_GPIO, 0);
}

void setLED(int i)
{
    gpio_set_level(BLINK_GPIO, i);
}

/*
 *          SPI2    SPI3
 * Pin        GPIO Num
 * CS0      15      5
 * SCLK     14      18
 * MISO     12      19
 * MOSI     13      23
 * QUADWP   2       22
 * QUADHD   4       21
*/
// APB_CLK_FREQ = 80MHz
#define PIN_NUM_MOSI 13
#define PIN_NUM_CLK 14
#define maxSPIFrameInBytes 4096 // 409x
#define maxSPIFrequency (20*1000*1000)
int setupSPI()
{
    esp_err_t ret;
	//Set up the Bus Config struct
	buscfg.miso_io_num=-1;
	buscfg.mosi_io_num=PIN_NUM_MOSI;
	buscfg.sclk_io_num=PIN_NUM_CLK;
	buscfg.quadwp_io_num=-1;
	buscfg.quadhd_io_num=-1;
	buscfg.max_transfer_sz=maxSPIFrameInBytes;
	
	//Set up the SPI Device Configuration Struct
	devcfg.clock_speed_hz=maxSPIFrequency;
	devcfg.mode=0;                        
	devcfg.spics_io_num=-1;
	devcfg.queue_size=1;

	//Initialize the SPI driver
	ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);	
	//Add SPI port to bus
	ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
	return ret;
}

int sendSPI() {
    esp_err_t ret;
    //spi_transaction_t& t = spiTransObject;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    //t.length=72*4*8; // length in bits
    t.length=256*4*8; // length in bits
    t.tx_buffer=data;               //Data
    t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
    return ret;
}
