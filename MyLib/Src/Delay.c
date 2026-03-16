#include "Delay.h"

void Delay_us(uint32_t us)
{
    // 每个循环约6-10个时钟周期，需根据实际CPU频率校准
    uint32_t loops = us * (SystemCoreClock / 1000000) / 5;
    
    while(loops--)
    {
        __NOP();
    }
}

void Delay_ms(uint32_t ms)
{
	while(ms--)
	{
		Delay_us(1000);
	}
}

void Delay_s(uint32_t s)
{
	while(s--)
	{
		Delay_ms(1000);
	}
}