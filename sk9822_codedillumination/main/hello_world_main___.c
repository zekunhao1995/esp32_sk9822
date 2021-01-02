/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "esp_timer.h"

#define STACK_SIZE 8192
//SPI Vars
spi_device_handle_t spi;
//spi_transaction_t spiTransObject;
esp_err_t ret;
spi_bus_config_t buscfg;
spi_device_interface_config_t devcfg;

int setupSPI();
int sendSPI();
uint32_t data[512];
void genpattern(uint32_t tstep);
void genpattern2(uint32_t tstep);
void genpattern3(uint32_t tstep);
void staticpattern(uint32_t intensity, uint32_t rgain, uint32_t ggain, uint32_t bgain);
void LEDTask( void * pvParameters );
static void periodic_timer_callback(void* arg);

uint32_t rand_uint32();
//_Atomic uint32_t glob_brightness = 0;
uint32_t glob_brightness = 0;


void app_main()
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%dMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
            (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    
    printf("Setting up SPI now\t[%d]\n", setupSPI());
    
    printf("portTICK_PERIOD_MS: \t[%d]\n", portTICK_PERIOD_MS);
    
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &periodic_timer_callback,
        /* name is optional, but may help identify the timer when debugging */
        .name = "periodic"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    /* The timer has been created but is not running yet */
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, 33367));
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    /*
    uint32_t i = 0;
    while(1) {
        genpattern3(i);
        sendSPI();
        vTaskDelay(20 / portTICK_PERIOD_MS);
        i = (i + 1) % 6144;
    }*/
    
    TaskHandle_t xHandle = NULL;
    // Create the task, storing the handle.  Note that the passed parameter ucParameterToPass
    // must exist for the lifetime of the task, so in this case is declared static.  If it was just an
    // an automatic stack variable it might no longer exist, or at least have been corrupted, by the time
    // the new task attempts to access it.
    //xTaskCreate( LEDTask, "LED", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle );
    xTaskCreatePinnedToCore( LEDTask, "LED", STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle, 1);
    configASSERT( xHandle );
    while (1) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

    /* Disable buffering on stdin */
    ESP_ERROR_CHECK( uart_driver_install(CONFIG_CONSOLE_UART_NUM, 256, 0, 0, NULL, 0) );
    esp_vfs_dev_uart_use_driver(CONFIG_CONSOLE_UART_NUM);
    setvbuf(stdin, NULL, _IONBF, 0);
    char line[1024];
    while (1) {
        if( fgets (line, 1024, stdin)!=NULL ) {
            /* writing content to stdout */
            //glob_brightness = atoi(line);
            //fputs(line, stdout);
            char* p = line;
            uint32_t irgb[4];
            for (int i=0; i<4; i++) {
                irgb[i] = 0;
                while (*p >= '0' && *p <= '9') {
                    irgb[i] = irgb[i]*10 + (*p - '0');
                    p++;
                }
                while (*p != '\n' && (*p < '0' || *p > '9')) {
                    p++;
                }
            }
            staticpattern(irgb[0], irgb[1], irgb[2], irgb[3]);
            sendSPI();
        }
        //putchar('.');
        //eof = fgets(line, sizeof(line), stdin);
        //vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    
    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}

uint32_t intensity = 1;
static void periodic_timer_callback2(void* arg) {
    intensity = 1u-intensity;
    staticpattern(intensity * 65535, 256u, 256u, 256u);
    sendSPI();
}

static void periodic_timer_callback(void* arg) {
    uint32_t intensity = rand_uint32() >> 16;
    staticpattern(intensity, 256u, 256u, 256u);
    sendSPI();
}

void LEDTask( void * pvParameters ) {
    uint32_t i = 0;
    uint32_t rgbgain[3];
    uint32_t intensity;
    while(1) {
        float t = sinf(i * 2 * (float)M_PI / 512.0f);
        t = (t + 1.0f) / 2.0f;
        int tmp = (int)(t * 65535.0f);
        intensity = (tmp < 0) ? 0 : (tmp > 65535 ? 65535 : tmp);
        intensity = rand_uint32() >> 16;
        //intensity = tmp;
        //printf("H %d\n", tmp < 0);
        //staticpattern0(i);
        //rgbgain[0] = (i % 3 == 0) ? 256u : 0u;
        //rgbgain[1] = (i % 3 == 1) ? 256u : 0u;
        //rgbgain[2] = (i % 3 == 2) ? 256u : 0u;
        staticpattern(intensity, 256u, 256u, 256u);
        sendSPI();
        //vTaskDelay(20 / portTICK_PERIOD_MS);
        
        for (int j=0; j<1000000; j++) {
            asm volatile ("nop");
        }
        i = (i + 1) % 2048;
    }
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
#define maxSPIFrameInBytes 8192 // 409x
#define maxSPIFrequency (16*1000*1000)
int setupSPI()
{
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

uint32_t gamma2linear(uint32_t x, uint32_t s) {
    return x*x / s;
}

void lin2wh_0_7935(uint32_t x, uint32_t* outw8, uint32_t* outh5) {
    uint32_t msbs = ((x >> 8) + 1) & 0b11111; // 1 to 31
    uint32_t lsbs = x & 0xff;
    *outw8 = (lsbs + 255*msbs - 255) / msbs;
    *outh5 = msbs;
}

void lin2wh_0_65535_old(uint32_t x, uint32_t* outw8, uint32_t* outh5) {
    uint32_t height = x / 2115 + 1; // [1, 31]
    uint32_t width = x % 2115;
    uint32_t width8 = (width + 2114*height - 2114) / height;
    
    *outw8 = width8 * 7 / 58;
    *outh5 = height;
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

void genpattern2(uint32_t tstep) {
    //memset(data, 0xff, sizeof(data));
    //data[0] = 0;
    for (uint32_t i=0; i<144; i++) {
        uint32_t lsbs = gamma2linear((i+tstep)%24*178, 4095) >> 4;
        uint32_t msbs = 16;
        uint32_t r = lsbs;
        uint32_t g = lsbs; //lsbs >> 1;
        uint32_t b = lsbs;
        data[i] = r<<24 | g<<16 | b<<8 | 0b11100000 | msbs; // RRGGBB
    }
    for (uint32_t i=144; i<512; i++) {
        data[i] = 0; // RRGGBB
    }
}

void genpattern3(uint32_t tstep) {
    //memset(data, 0xff, sizeof(data));
    //data[0] = 0;
    for (uint32_t i=0; i<144; i++) {
        //uint32_t intensity = gamma2linear((i+tstep)%24*343, 7843); // 0 to 7935
        //uint32_t intensity = gamma2linear((i+tstep)%48*168, 7857); // 0 to 7935
        uint32_t intensity = gamma2linear((i+tstep)%48*1394, 65501); // 0 to 7935
        uint32_t pwmval, currentval;
        //intensity >>= 4;
        lin2wh_0_65535(intensity, &pwmval, &currentval);
        //uint32_t msbs = ((intensity >> 8) + 1) & 0b11111; // 1 to 31
        //uint32_t lsbs = intensity & 0xff;
        //uint32_t pwmval = (lsbs + 255*msbs - 255) / msbs;

        //int lsbs = intensity & 0xff;
        uint32_t r = pwmval;
        uint32_t g = 0; //pwmval >> 1; //lsbs >> 1;
        uint32_t b = 0; //pwmval >> 2;
        data[i] = r<<24 | g<<16 | b<<8 | 0b11100000 | currentval; // RRGGBB
    }
    for (uint32_t i=144; i<512; i++) {
        data[i] = 0; // RRGGBB
    }
}

void genpattern(uint32_t tstep) {
    //memset(data, 0xff, sizeof(data));
    //data[0] = 0;
    for (uint32_t i=0; i<144; i++) {
        uint32_t intensity = gamma2linear((i+tstep)%24*178, 4095);
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
        uint32_t r = lsbs;
        uint32_t g = lsbs; //lsbs >> 1;
        uint32_t b = lsbs;
        data[i] = r<<24 | g<<16 | b<<8 | 0b11100000 | msbs; // RRGGBB
    }
    for (uint32_t i=144; i<512; i++) {
        data[i] = 0; // RRGGBB
    }
}

void staticpattern0(uint32_t tstep) {
    volatile uint32_t brightness = glob_brightness;
    for (uint32_t i=0; i<144; i++) {
        //uint32_t intensity = gamma2linear((i+tstep)%24*343, 7843); // 0 to 7935
        //uint32_t intensity = gamma2linear((i+tstep)%48*168, 7857); // 0 to 7935
        //uint32_t intensity = gamma2linear((i+tstep)%48*1394, 65501); // 0 to 7935
        //intensity = intensity * brightness / 65535u;
        uint32_t intensity = brightness;
        uint32_t pwmval, currentval;
        lin2wh_0_65535(intensity, &pwmval, &currentval);

        uint32_t r = pwmval;
        uint32_t g = 0; //pwmval >> 1; //lsbs >> 1;
        uint32_t b = 0; //pwmval >> 2;
        data[i] = r<<24 | g<<16 | b<<8 | 0b11100000 | currentval; // RRGGBB
    }
    for (uint32_t i=144; i<148; i++) {
        data[i] = 0; // RRGGBB
    }
    /*
    for (uint32_t i=144; i<512; i++) {
        data[i] = 0; // RRGGBB
    }
    */
}

void staticpattern(uint32_t intensity, uint32_t rgain, uint32_t ggain, uint32_t bgain) {
    uint32_t pwmval, currentval;
    lin2wh_0_65535(intensity, &pwmval, &currentval);

    uint32_t r = pwmval * rgain / 256;
    uint32_t g = pwmval * ggain / 256; //pwmval >> 1; //lsbs >> 1;
    uint32_t b = pwmval * bgain / 256; //pwmval >> 2;
    
    uint32_t mydata = r<<24 | g<<16 | b<<8 | 0b11100000 | currentval; // RRGGBB
    for (uint32_t i=0; i<144; i++) {
        data[i] = mydata;
    }
    for (uint32_t i=144; i<148; i++) {
        data[i] = 0; // RRGGBB
    }
}

int sendSPI() {
    //spi_transaction_t& t = spiTransObject;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    t.length=148*4*8; // length in bits
    t.tx_buffer=data;               //Data
    //t.user=(void*)1;                //D/C needs to be set to 1
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
    return ret;
}

uint64_t m_s = 88172645463325252ULL;
uint32_t rand_uint32() {
    m_s ^= m_s << 13;
    m_s ^= m_s >> 7;
    m_s ^= m_s << 17;
    return (uint32_t)(m_s >> 32);
}


