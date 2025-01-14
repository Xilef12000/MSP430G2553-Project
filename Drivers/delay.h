/*
 * delay.h
 *
 *  Created on: 17.04.2020
 *      Author: bboeck
 */

#ifndef DELAY_H_
#define DELAY_H_



/** @brief Delay for a given number of microseconds
* @param  us    : delay in microseconds (1...2^16)
*
*/
void delay_us(unsigned int us);


/** @brief Delay for a given number of milliseconds
* @param  ms    : delay in milliseconds (1...2^16)
*
*/
void delay_ms(unsigned int ms);




#endif /* DELAY_H_ */
