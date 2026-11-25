#include"stm32f103xx_dht11.h"
#include"stm32f103xx_init.h"
#include"stm32f103xx_systick.h"
#include"stm32f103xx_dwt.h"
#include"math.h"
#include"math_lib.h"
#include"stdbool.h"


//Handle
static DHT11_Handle hDHT11;

//Helper Functions
static uint8_t dht11_start_internal(uint32_t response_timeout_us);
static DHT11_Status_t dht11_read_frame(uint8_t data[5]);
static inline DHT11_Status_t dht11_read_byte(uint8_t* byte);
static bool dht11_wait_for_level(uint8_t level, uint32_t timeout_us);
static inline uint8_t dht11_pin_level(void);



DHT11_Handle* DHT11_Init(GPIO_RegDef* pGPIOx, uint8_t gpio_pin)
{
    hDHT11.pGPIOx      = pGPIOx;
    hDHT11.gpio_pin    = gpio_pin;
    hDHT11.temperature = 0;
    hDHT11.humidity    = 0;
    
    //Initialize the gpio pin as output
    GPIO_Handle g;
    GPIO_Config(&g, pGPIOx, GPIO_MODE_OP, GPIO_CONFIG_GP_OP_PP, gpio_pin, GPIO_OP_SPEED_50);
    GPIO_Init(&g);

    //Set it HIGH initially
    GPIO_BIT_SET(pGPIOx, gpio_pin);

	return &hDHT11;
}



uint8_t DHT11_Read_Data(uint32_t response_timeout_ms)
{
	response_timeout_ms = (response_timeout_ms == 0)?DHT11_TIMEOUT_DEFAULT_MS:response_timeout_ms;
	hDHT11.last_status  = DHT11_ReadDataRaw(&hDHT11.temperature, &hDHT11.humidity, response_timeout_ms);

	return hDHT11.last_status;
}



uint8_t DHT11_ReadDataRaw(float* ptemp, float* phum, uint32_t response_timeout_ms)
{
    hDHT11.raw_data[0] = 0;
	hDHT11.raw_data[1] = 0;
	hDHT11.raw_data[2] = 0;
	hDHT11.raw_data[3] = 0;
	hDHT11.raw_data[4] = 0;

	DHT11_Status_t st;
    
    //Initialize DHT11
	st = dht11_start_internal(response_timeout_ms * 1000);
	
    if(st == DHT11_OK)
    {
       	//Read the full frame of 5 bytes
		DISABLE_IRQ();
		st = dht11_read_frame(hDHT11.raw_data);
		ENABLE_IRQ();
    }
    
	*phum  = 0.0f;
    *ptemp = 0.0f;

	if (st == DHT11_OK)
    {
		//Checksum success
		if((uint8_t)(hDHT11.raw_data[0] + hDHT11.raw_data[1] + hDHT11.raw_data[2] + hDHT11.raw_data[3]) == hDHT11.raw_data[4])
		{
			//Convert Data to Float
			*phum  = (float)hDHT11.raw_data[0] + (float)hDHT11.raw_data[1]/10.0;
			*ptemp = (float)hDHT11.raw_data[2] + (float)hDHT11.raw_data[3]/10.0;

			//log status as Success
			st = DHT11_OK;
		}
		//Checksum fail
		else
		{
			//log status as checksum error
			st = DHT11_ERR_CHECKSUM;
		}
	}
	return st;
}



float DHT11_HeatIndex_Calculate(float tempC, float humidity)
{
    //Convert Celsius to Fahrenheit
    float T = (tempC * 1.8f) + 32.0f;
    float R = humidity;
    
    //Simple formula for lower temperatures
    float hi = 0.5f * (T + 61.0f + ((T - 68.0f) * 1.2f) + (R * 0.094f));
    
    //If Heat Index is > 80F, use the full regression formula
    if (hi > 80.0f)
    {
        hi = -42.379f + 2.04901523f*T + 10.14333127f*R - 0.22475541f*T*R -
             0.00683783f*T*T - 0.05481717f*R*R + 0.00122874f*T*T*R +
             0.00085282f*T*R*R - 0.00000199f*T*T*R*R;
             
        //Adjustments for specific conditions
        if (R < 13.0f && T >= 80.0f && T <= 112.0f)
        {
            hi -= ((13.0f - R) / 4.0f) * sqrtf((17.0f - ABS(T - 95.0f)) / 17.0f);
        } else if (R > 85.0f && T >= 80.0f && T <= 87.0f)
        {
            hi += ((R - 85.0f) / 10.0f) * ((87.0f - T) / 5.0f);
        }
    }

    //Convert back to Celsius for the final result
    return (hi - 32.0f) * 0.5555f;
}





static uint8_t dht11_start_internal(uint32_t response_timeout_us)
{
	uint16_t timeout;
    GPIO_Handle g;

	//initialize GPIO Pin
	GPIO_Config(&g, hDHT11.pGPIOx, GPIO_MODE_OP, GPIO_CONFIG_GP_OP_PP, hDHT11.gpio_pin, GPIO_OP_SPEED_2);
	GPIO_Init(&g);

	//Send Init Signal to DHT11 Sensor
	//GPIO is acting as OP
	//          __________
	//   LOW   |   HIGH   |
	//_________|          |_____......
	//         ^20ms      ^50ms
	//<--20ms--><--30ms--->

	//Pull the Pin LOW
	GPIO_BIT_RESET(hDHT11.pGPIOx,hDHT11.gpio_pin);
	//wait for 20 ms
	tim_delay_ms(20);
	//pull pin HIGH
	GPIO_BIT_SET(hDHT11.pGPIOx,hDHT11.gpio_pin);
	//wait for 30 us
	tim_delay_us(30);

	//COnfigure GPIO to be Input
	g.GPIOx_PinConfig.PinMode       = GPIO_MODE_IP;
	g.GPIOx_PinConfig.PinConfigType = GPIO_CONFIG_PU;
	GPIO_Init(&g);


	//GPIO is now acting as IP
	//          <-----DHT11 RESPONSE------>
	//....______              ____________
	//          |    LOW     |    HIGH    |   LOW
	//          |____________|            |_________....
	//<--wait--><----80us----><----80us--->

	//Response: LOW ~80 us, HIGH ~80 us, LOW ~50 us before data
	//wait for DHT11 to pull LOW (80us)
	if (!dht11_wait_for_level(0, response_timeout_us)) return DHT11_ERR_TIMEOUT;
	//wait for DHT11 to pull HIGH (80us)
    if (!dht11_wait_for_level(1, DHT11_BIT_TIMEOUT_US)) return DHT11_ERR_TIMEOUT;
	//wait for DHT11 to pull LOW (50us)
    if (!dht11_wait_for_level(0, DHT11_BIT_TIMEOUT_US)) return DHT11_ERR_TIMEOUT;

	return DHT11_OK;
}



static DHT11_Status_t dht11_read_frame(uint8_t data[5])
{
	uint32_t frame_deadline = DWT_GetCount() + DWT_US_TO_CYCLES(DHT11_FRAME_TIMEOUT_US);

	for(uint8_t i=0; i<5; i++)
	{
		dht11_read_byte(&data[i]);

		if ((int32_t)(DWT_GetCount() - frame_deadline) >= 0)
        	return DHT11_ERR_TIMEOUT;
	}
	return DHT11_OK;
}



static inline DHT11_Status_t dht11_read_byte(uint8_t* byte)
{
	for (uint8_t a=0; a<8; a++)
	{
		//wait for DHT11 LOW response to end (50 us)
		//wait till DHT11 response goes HIGH
		if (!dht11_wait_for_level(1, DHT11_BIT_TIMEOUT_US))
            return DHT11_ERR_TIMEOUT;

		//the DHT11 response is now HIGH
		//we need to measure how long its HIGH
		//26-28us -> bit 0
		//70us -> bit 1
		//wait for 40us and then sample
		uint32_t rise = DWT_GetCount();
        while ((DWT_GetCount() - rise) < DWT_US_TO_CYCLES(DHT11_SAMPLE_DELAY_US))
		{}

		//its a bit 0 - HIGH for 28us
		if (!(GPIO_PIN_READ(hDHT11.pGPIOx, hDHT11.gpio_pin)))
			*byte &= ~(1<<(7-a));
		else //its a bit 0 - HIGH for 28us
			*byte |= (1<<(7-a));

		//wait for the DHT11 response to go LOW
		//Falling Edge marks the end of a bit
		if (!dht11_wait_for_level(0, DHT11_BIT_TIMEOUT_US))
            return DHT11_ERR_TIMEOUT;
	}
	return DHT11_OK;
}



static bool dht11_wait_for_level(uint8_t level, uint32_t timeout_us)
{
    uint32_t deadline = DWT_GetCount() + DWT_US_TO_CYCLES(timeout_us);

	//wait for the desired level
    while (dht11_pin_level() != level)
    {
        if ((int32_t)(DWT_GetCount() - deadline) >= 0)
            return false;
    }
    return true;
}



static inline uint8_t dht11_pin_level(void)
{
    return (GPIO_PIN_READ(hDHT11.pGPIOx, hDHT11.gpio_pin) != 0U) ? 1U : 0U;
}