/*
 * MIT License
 *
 * Copyright (c) 2017 David Antliff
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"

#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"

#define FORCE_LEGACY_FOR_RMT_ESP_IDF_VERSION_5    1
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0) && FORCE_LEGACY_FOR_RMT_ESP_IDF_VERSION_5 == 0
#include "driver/rmt_tx.h"
#include "driver/rmt_rx.h"
#else
//#pragma GCC diagnostic ignored "-Wcpp"
#include "driver/rmt.h"
//#pragma GCC diagnostic pop
#endif


#define TAG "drv_ds18b20"

#define GPIO_DS18B20_0          (CONFIG_DRV_DS18B20_ONE_WIRE_GPIO_0)
#define GPIO_DS18B20_1          (CONFIG_DRV_DS18B20_ONE_WIRE_GPIO_1)
#define GPIO_DS18B20_2          (CONFIG_DRV_DS18B20_ONE_WIRE_GPIO_2)
#define MAX_DEVICES             (8)
#define ONE_WIRE_BUS_CHANNELS   (3)
#define DS18B20_RESOLUTION      (DS18B20_RESOLUTION_12_BIT)
#define SAMPLE_PERIOD           (1000)   // milliseconds
#define MAX_DEVICES_PER_CHANNEL (8)


static int owb_gpio_num[ONE_WIRE_BUS_CHANNELS] = 
{
    GPIO_DS18B20_0,
    #if ONE_WIRE_BUS_CHANNELS > 1
    GPIO_DS18B20_1,
    #endif
    #if ONE_WIRE_BUS_CHANNELS > 2
    GPIO_DS18B20_2,
    #endif
};

static int ds18b20_devices_count = 0;
static float ds18b20_temperature[MAX_DEVICES] = {NAN};
static int ds18b20_errors[MAX_DEVICES] = {0};

int drv_ds18b20_devices_count_get(void)
{
    return ds18b20_devices_count;
}

float drv_ds18b20_temperature_get(int index)
{
    if (index < ds18b20_devices_count)
    {
        return ds18b20_temperature[index];
    }
    return NAN;
}

int drv_ds18b20_errors_get(int index)
{
    if (index < ds18b20_devices_count)
    {
        return ds18b20_errors[index];
    }
    return -1;
}

void drv_ds18b20_task(void* arg)
{
    // set log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
    esp_log_level_set("owb", ESP_LOG_INFO);
    esp_log_level_set("owb_rmt", ESP_LOG_INFO);
    esp_log_level_set("ds18b20", ESP_LOG_INFO);

    // To debug, use 'make menuconfig' to set default Log level to DEBUG, then uncomment:
    //esp_log_level_set("owb", ESP_LOG_DEBUG);
    //esp_log_level_set("ds18b20", ESP_LOG_DEBUG);

    // Stable readings require a brief period before communication
    //vTaskDelay(2000.0 / portTICK_PERIOD_MS);  //to do use esp_timer_read if needed instead delay here (microseconds from reset)

    // Create a 1-Wire bus, using the RMT timeslot driver
    OneWireBus * owb[ONE_WIRE_BUS_CHANNELS];
    owb_rmt_driver_info rmt_driver_info[ONE_WIRE_BUS_CHANNELS];
    int num_devices[ONE_WIRE_BUS_CHANNELS] = {0};
    int num_devices_total = 0;

    DS18B20_Info * devices[ONE_WIRE_BUS_CHANNELS][MAX_DEVICES_PER_CHANNEL] = {0};
    DS18B20_Info * devices_total[MAX_DEVICES] = {0};
    int devices_total_index_counter = 0;

    for (int index = 0; index < ONE_WIRE_BUS_CHANNELS; index++)
    {
        owb[index] = owb_rmt_initialize(&rmt_driver_info[index], owb_gpio_num[index], RMT_CHANNEL_1 + index*2, RMT_CHANNEL_0 + index*2);
        owb_use_crc(owb[index], true);  // enable CRC check for ROM code

        // Find all connected devices
        ESP_LOGI(TAG, "Find devices: OWB channel[%d]", index);
        OneWireBus_ROMCode device_rom_codes[MAX_DEVICES] = {0};
        
        OneWireBus_SearchState search_state = {0};
        bool found = false;
        owb_search_first(owb[index], &search_state, &found);
        while (found)
        {
            char rom_code_s[17];
            owb_string_from_rom_code(search_state.rom_code, rom_code_s, sizeof(rom_code_s));
            ESP_LOGI(TAG, "channel[%d]  %d : %s", index, num_devices[index], rom_code_s);
            device_rom_codes[num_devices[index]] = search_state.rom_code;
            ++num_devices[index];
            ++num_devices_total;
            owb_search_next(owb[index], &search_state, &found);
        }
        ESP_LOGI(TAG, "Found %d device%s channel[%d]", num_devices[index], num_devices[index] == 1 ? "" : "s", index);


        // In this example, if a single device is present, then the ROM code is probably
        // not very interesting, so just print it out. If there are multiple devices,
        // then it may be useful to check that a specific device is present.

        if (num_devices[index] == 0)
        {
            owb_uninitialize(owb[index]);
        }
        else
        if (num_devices[index] == 1)
        {
            // For a single device only:
            OneWireBus_ROMCode rom_code[ONE_WIRE_BUS_CHANNELS];
            owb_status status = owb_read_rom(owb[index], &rom_code[index]);
            if (status == OWB_STATUS_OK)
            {
                char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
                owb_string_from_rom_code(rom_code[index], rom_code_s, sizeof(rom_code_s));
                ESP_LOGI(TAG, "Single device %s present channel[%d]", rom_code_s, index);
            }
            else
            {
                ESP_LOGI(TAG, "An error occurred reading ROM code: %d channel[%d]", status, index);
            }
        }
        else
        {
            // Search for a known ROM code (LSB first):
            // For example: 0x1502162ca5b2ee28
            OneWireBus_ROMCode known_device = {
                .fields.family = { 0x28 },
                .fields.serial_number = { 0xee, 0xb2, 0xa5, 0x2c, 0x16, 0x02 },
                .fields.crc = { 0x15 },
            };
            char rom_code_s[OWB_ROM_CODE_STRING_LENGTH];
            owb_string_from_rom_code(known_device, rom_code_s, sizeof(rom_code_s));
            bool is_present = false;

            owb_status search_status = owb_verify_rom(owb[index], known_device, &is_present);
            if (search_status == OWB_STATUS_OK)
            {
                ESP_LOGI(TAG, "Device %s is %s  channel[%d]", rom_code_s, is_present ? "present" : "not present", index);
            }
            else
            {
                ESP_LOGI(TAG, "An error occurred searching for known device: %d channel[%d]", search_status, index);
            }
        }

        // Create DS18B20 devices on the 1-Wire bus
        for (int i = 0; i < num_devices[index]; ++i)
        {
            DS18B20_Info * ds18b20_info = ds18b20_malloc();  // heap allocation
            devices[index][i] = ds18b20_info;
            devices_total[devices_total_index_counter] = ds18b20_info;
            devices_total_index_counter++;

            if (num_devices[index] == 1)
            {
                ESP_LOGI(TAG, "Single device optimisations enabled");
                ds18b20_init_solo(ds18b20_info, owb[index]);          // only one device on bus
            }
            else
            {
                ds18b20_init(ds18b20_info, owb[index], device_rom_codes[i]); // associate with bus and device
            }
            ds18b20_use_crc(ds18b20_info, true);           // enable CRC check on all reads
            ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION);
        }

    //    // Read temperatures from all sensors sequentially
    //    while (1)
    //    {
    //        ESP_LOGI(TAG, "Temperature readings (degrees C):");
    //        for (int i = 0; i < num_devices; ++i)
    //        {
    //            float temp = ds18b20_get_temp(devices[i]);
    //            ESP_LOGI(TAG, "  %d: %.3f", i, temp);
    //        }
    //        vTaskDelay(1000 / portTICK_PERIOD_MS);
    //    }

        // Check for parasitic-powered devices
        bool parasitic_power = false;
        ds18b20_check_for_parasite_power(owb[index], &parasitic_power);
        if (parasitic_power) {
            ESP_LOGI(TAG, "Parasitic-powered devices detected");
        }

        // In parasitic-power mode, devices cannot indicate when conversions are complete,
        // so waiting for a temperature conversion must be done by waiting a prescribed duration
        owb_use_parasitic_power(owb[index], parasitic_power);

    #ifdef CONFIG_DRV_DS18B20_ENABLE_STRONG_PULLUP_GPIO
        // An external pull-up circuit is used to supply extra current to OneWireBus devices
        // during temperature conversions.
        owb_use_strong_pullup_gpio(owb, CONFIG_DRV_DS18B20_STRONG_PULLUP_GPIO);
    #endif

    }

    ds18b20_devices_count = num_devices_total;
    for (int index = 0; index < ds18b20_devices_count; index++)
    {
        ds18b20_temperature[index] = NAN;
        ds18b20_errors[index] = 0;

    }


    // Read temperatures more efficiently by starting conversions on all devices at the same time
    int errors_count[MAX_DEVICES] = {0};
    int sample_count = 0;
    if (num_devices_total > 0)
    {
        TickType_t last_wake_time = xTaskGetTickCount();

        while (1)
        {
            for (int index = 0; index < ONE_WIRE_BUS_CHANNELS; index++)
            {
                if (num_devices[index])
                {
                    ds18b20_convert_all(owb[index]);
                }
                
            }

            // In this application all devices use the same resolution,
            // so use the first device to determine the delay
            for (int index = 0; index < ONE_WIRE_BUS_CHANNELS; index++)
            {
                if (num_devices[index])
                {
                    ds18b20_wait_for_conversion(devices[index][0]);
                }
                
            }
            

            // Read the results immediately after conversion otherwise it may fail
            // (using printf before reading may take too long)
            float readings[MAX_DEVICES] = { 0 };
            DS18B20_ERROR errors[MAX_DEVICES] = { 0 };

            for (int i = 0; i < num_devices_total; ++i)
            {
                errors[i] = ds18b20_read_temp(devices_total[i], &readings[i]);
                ds18b20_temperature[i] = readings[i];
                ds18b20_errors[i] = errors[i];
            }

            // Print results in a separate loop, after all have been read
            ESP_LOGD(TAG, "Temperature readings (degrees C): sample %d", ++sample_count);
            for (int i = 0; i < num_devices_total; ++i)
            {
                if (errors[i] != DS18B20_OK)
                {
                    ++errors_count[i];
                }

                ESP_LOGD(TAG, "  %d: %.1f    %d errors", i, readings[i], errors_count[i]);
            }

            vTaskDelayUntil(&last_wake_time, SAMPLE_PERIOD / portTICK_PERIOD_MS);
        }
    }
    else
    {
        ESP_LOGI(TAG, "No DS18B20 devices detected!");
    }

    for (int index = 0; index < ONE_WIRE_BUS_CHANNELS; index++)
    {
        if (num_devices[index])
        {
            // clean up dynamically allocated data
            for (int i = 0; i < num_devices[index]; ++i)
            {
                ds18b20_free(&devices[index][i]);
            }
            owb_uninitialize(owb[index]);
        }
    }

    ESP_LOGW(TAG, "Exiting DS18B20 Task.");
    // fflush(stdout);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
    // esp_restart();
    vTaskDelete(NULL);
}


void drv_ds18b20_init(void)
{
    xTaskCreate(drv_ds18b20_task, "ds18b20", 2048+1024, NULL, 5,NULL);
}

