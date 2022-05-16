#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "driver/dac.h"
#include "sdkconfig.h"
#include "esp_log.h"
#include "eeprom.h"


#define	EEPROM_MODEL LC040A

static const char TAG[] = "SPI_DAC";

void dump(uint8_t *dt, int n)
{
	uint16_t clm = 0;
	uint8_t data;
	uint32_t saddr =0;
	uint32_t eaddr =n-1;

	printf("--------------------------------------------------------\n");
	uint32_t addr;
	for (addr = saddr; addr <= eaddr; addr++) {
		data = dt[addr];
		if (clm == 0) {
			printf("%05x: ",addr);
		}

		printf("%02x ",data);
		clm++;
		if (clm == 16) {
			printf("| \n");
			clm = 0;
		}
	}
	printf("--------------------------------------------------------\n");
}


void app_main(void)
{

	dac_output_enable(DAC_CHANNEL_2); //ativar a saida da DAC no GPIO26
    
    ESP_LOGI(TAG, "EEPROM_MODEL=%s", "24LC040A");
	EEPROM_t dev;
	spi_master_init(&dev);
	int32_t totalBytes = eeprom_TotalBytes(&dev);
	ESP_LOGI(TAG, "totalBytes=%d Bytes",totalBytes);

	// Get Status Register
	uint8_t reg;
	esp_err_t ret;
	ret = eeprom_ReadStatusReg(&dev, &reg);
	if (ret != ESP_OK) {
		ESP_LOGI(TAG, "ReadStatusReg Fail %d",ret);
		while(1) { vTaskDelay(1); }
	} 
	ESP_LOGI(TAG, "readStatusReg : 0x%02x", reg);

	uint8_t wdata[128];
	int len;
	
    write all bits 1
    for (int i=0; i<128; i++) {
		wdata[i]=0xff;	
	}  

    //write increase
	// for (int i=0; i<128; i++) {
	// 	wdata[i]=i;	
	// }  

    // //write decrease
	// for (int i=0; i<128; i++) {
	// 	wdata[i]=0xff-i;	
	// }

	for (int addr=0; addr<128;addr++) {
		len =  eeprom_WriteByte(&dev, addr, wdata[addr]);
		ESP_LOGI(TAG, "WriteByte(addr=%d) len=%d", addr, len);
		if (len != 1) {
			ESP_LOGI(TAG, "WriteByte Fail addr=%d", addr);
			while(1) { vTaskDelay(1); }
		}
	}

	// Read 128 byte from Address=0
	uint8_t rbuf[128];
	memset(rbuf, 0, 128);
	len =  eeprom_Read(&dev, 0, rbuf, 128);
	if (len != 128) {
		ESP_LOGI(TAG, "Read Fail");
		while(1) { vTaskDelay(1); }
	}
	ESP_LOGI(TAG, "Read Data: len=%d", len);
	dump(rbuf, 128);


	uint8_t val;
	for (int i = 0; i < 128; i++)
	{
		val = rbuf[i];
		dac_output_voltage(DAC_CHANNEL_2, 255 - i); //colocar voltagem na saída do canal DAC do GPIO25 
        vTaskDelay(30 / portTICK_PERIOD_MS); 		//pequeno delay
	}
	
}

