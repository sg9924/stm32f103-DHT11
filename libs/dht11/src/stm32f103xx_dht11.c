#include"stm32f103xx_dht11.h"
#include"stm32f103xx_init.h"
#include"stm32f103xx_systick.h"
#include"stm32f103xx_dwt.h"
#include"math.h"
#include"math_lib.h"


//Handle
static DHT11_Handle hDHT11;

//Helper Functions
static uint8_t dht11_start_internal();
static int16_t dht11_read_byte();



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



uint8_t DHT11_ReadDataRaw(float* ptemp, float* phum)
{
    hDHT11.raw_data[0] = 0;
	hDHT11.raw_data[1] = 0;
	hDHT11.raw_data[2] = 0;
	hDHT11.raw_data[3] = 0;
	hDHT11.raw_data[4] = 0;
    
    //Initialize DHT11
    if (!dht11_start_internal())
    {
        //Timeout
        return DHT11_STATUS_ERROR;
    }

    //Read 5 bytes
    //IRQ should be disabled
	DISABLE_IRQ();
    for (int i = 0; i < 5; i++)
    {
        int16_t v = dht11_read_byte();
		if (v < 0)
		{
			ENABLE_IRQ();
			return DHT11_STATUS_ERROR;
		}
		hDHT11.raw_data[i] = (uint8_t)v;
    }
    //IRQ should be enabled
	ENABLE_IRQ();

    //Checksum success
    if((uint8_t)(hDHT11.raw_data[0] + hDHT11.raw_data[1] + hDHT11.raw_data[2] + hDHT11.raw_data[3]) == hDHT11.raw_data[4])
    {
        //Convert Data to Float
        *phum  = (float)hDHT11.raw_data[0] + (float)hDHT11.raw_data[1]/10.0;
        *ptemp = (float)hDHT11.raw_data[2] + (float)hDHT11.raw_data[3]/10.0;

        //log status as Success
        return DHT11_STATUS_OK;
    }
    //Checksum fail
    else
    {
        //log status as checksum error
        return DHT11_STATUS_CHECKSUM_ERROR;
    }
}



uint8_t DHT11_Read_Data()
{
	hDHT11.last_status = DHT11_ReadDataRaw(&hDHT11.temperature, &hDHT11.humidity);
	return hDHT11.last_status;
}





static uint8_t dht11_start_internal()
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
	//Systick_delay(20);
	//pull pin HIGH
	GPIO_BIT_SET(hDHT11.pGPIOx,hDHT11.gpio_pin);
	//wait for 30 us
	tim_delay_us(30);

	//COnfigure GPIO to be Input
	g.GPIOx_PinConfig.PinMode = GPIO_MODE_IP;
	g.GPIOx_PinConfig.PinConfigType = GPIO_CONFIG_PU;
	GPIO_Init(&g);


	//GPIO is now acting as IP
	//          <-----DHT11 RESPONSE------>
	//....______              ____________
	//          |    LOW     |    HIGH    |   LOW
	//          |____________|            |_________....
	//<--wait--><----80us----><----80us--->

	//wait for DHT11 to pull LOW (80us)
	timeout = 0;
	while(GPIO_PIN_READ(hDHT11.pGPIOx, hDHT11.gpio_pin))
	{
		if(timeout++ > DHT11_TIMEOUT) return 0;
	}

	//wait for DHT11 to pull HIGH (80us)
	timeout = 0;
	while(!GPIO_PIN_READ(hDHT11.pGPIOx, hDHT11.gpio_pin))
	{
		if(timeout++ > DHT11_TIMEOUT) return 0;
	}

	//wait for DHT11 to pull LOW again
	timeout = 0;
	while(GPIO_PIN_READ(hDHT11.pGPIOx, hDHT11.gpio_pin))
	{
		if(timeout++ > DHT11_TIMEOUT) return 0;
	}

	return 1;
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



static int16_t dht11_read_byte()
{
	uint8_t  b = 0;
	uint16_t timeout;
	uint32_t edge_cycles;

	for (uint8_t a=0; a<8; a++)
	{
		//wait for DHT11 LOW response to end (50 us)
		//wait till DHT11 response goes HIGH
		timeout = 0;
		while(!(GPIO_PIN_READ(hDHT11.pGPIOx, hDHT11.gpio_pin)))
		{
			if(timeout++ > DHT11_TIMEOUT) return -1;
		}

		//the DHT11 response is now HIGH
		//we need to measure how long its HIGH
		//26-28us -> bit 0
		//70us -> bit 1
		//wait for 40us and then sample
		edge_cycles = DWT_GetCount();

		//sample at a fixed offset from the edge, not from when this call started
		//2880 cycles = 40 us at 72 MHz
		while ((DWT_GetCount() - edge_cycles) < 2880);

		//its a bit 0 - HIGH for 28us
		if (!(GPIO_PIN_READ(hDHT11.pGPIOx, hDHT11.gpio_pin)))
			b &= ~(1<<(7-a));
		else //its a bit 0 - HIGH for 28us
			b |= (1<<(7-a));

		//wait for the DHT11 response to go LOW
		timeout = 0;
		while(GPIO_PIN_READ(hDHT11.pGPIOx, hDHT11.gpio_pin))
		{
			if(timeout++ > DHT11_TIMEOUT) return -1;
		}
	}
	return (int16_t)b;
}