#ifndef INC_stm32F103xx_DHT11_H
#define INC_stm32F103xx_DHT11_H

#include"stm32f103xx.h"
#include"stm32f103xx_core.h"
#include"stm32f103xx_memory_map.h"
#include"stm32f103xx_gpio.h"



#define DHT11_TIMEOUT                      10000
#define DHT11_STATUS_OK                    0
#define DHT11_STATUS_ERROR                 1
#define DHT11_STATUS_CHECKSUM_ERROR        2

//DHT11 Handle Structure
typedef struct
{
    GPIO_RegDef* pGPIOx;
    uint8_t      gpio_pin;
    uint8_t      last_status;
    uint8_t      raw_data[5];
    float        temperature;
    float        humidity;
    float        heat_index;
}DHT11_Handle;



//API's
DHT11_Handle* DHT11_Init(GPIO_RegDef* pGPIOx, uint8_t gpio_pin);
uint8_t DHT11_ReadDataRaw(float* ptemp, float* phum);
uint8_t DHT11_Read_Data();
float DHT11_HeatIndex_Calculate(float tempC, float humidity);


#endif