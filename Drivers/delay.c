/*
 * delay.c
 *
 *  Created on: 17.04.2020
 *      Author: bboeck
 */




void delay_us(unsigned int us)
{
    while (us)
    {
        __delay_cycles(1);      // 1 for 1 MHz set 16 for 16 MHz
        us--;
    }
}

void delay_ms(unsigned int ms)
{
    while (ms)
    {
        __delay_cycles(1000);   // 1000 for 1MHz and 16000 for 16MHz
        ms--;
    }
}





