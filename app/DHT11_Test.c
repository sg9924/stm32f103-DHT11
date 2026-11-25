#include "stm32f103xx_init.h"
#include "stm32f103xx_gpio.h"
#include "stm32f103xx_serial.h"
#include "stm32f103xx_dht11.h"

DHT11_Handle* D1;

int main()
{
    board_init();
    SET_LOG_LEVEL(LOG_LEVEL_INFO);

    D1 = DHT11_Init(GPIOA, GPIO_PIN7);
    
    while(1)
	{
        if(DHT11_Read_Data(DHT11_TIMEOUT_DEFAULT_MS) == DHT11_OK)
        {
            led_toggle();
            Serialprintln("DHT11: Temp: %.2fC | Hum: %.2f%%", INFO, D1->temperature, D1->humidity);
        }
        
        tim_delay_ms(2000);
    }
    return 0;
}