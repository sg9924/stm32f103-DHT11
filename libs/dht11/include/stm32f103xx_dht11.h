#ifndef INC_stm32F103xx_DHT11_H
#define INC_stm32F103xx_DHT11_H

#include"stm32f103xx.h"
#include"stm32f103xx_core.h"
#include"stm32f103xx_gpio.h"

#define DHT11_SAMPLE_DELAY_US              40U     //sample point after a rising edge
#define DHT11_BIT_TIMEOUT_US               120U    //max wait for any single data edge
#define DHT11_FRAME_TIMEOUT_US             8000U   //cap on the whole 40-bit frame
#define DHT11_TIMEOUT_DEFAULT_MS           30U

#define DWT_CYCLES_PER_US                (RCC_Get_SYSCLK() / 1000000U)
#define DWT_US_TO_CYCLES(us)             ((uint32_t)(us) * DWT_CYCLES_PER_US)


//Status
typedef enum
{
    DHT11_OK = 0,
    DHT11_ERR_TIMEOUT,
    DHT11_ERR_CHECKSUM,
    DHT11_ERR_IMPLAUSIBLE,
} DHT11_Status_t;



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
void DHT11_Init(DHT11_Handle* h, GPIO_RegDef* pGPIOx, uint8_t gpio_pin);
uint8_t DHT11_ReadDataRaw(DHT11_Handle* h, float* ptemp, float* phum, uint32_t response_timeout_ms);
uint8_t DHT11_Read_Data(DHT11_Handle* h, uint32_t response_timeout_ms);
float DHT11_HeatIndex_Calculate(float tempC, float humidity);


#endif