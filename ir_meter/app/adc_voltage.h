/*
 * adc_volatage.h
 *
 *  Created on: 2014骞�11鏈�10鏃�
 *      Author: BHEscaper
 */

#ifndef ADC_VOLATAGE_H_
#define ADC_VOLATAGE_H_

#include <stdint.h>

#define ADC_GPIO_TYPE       GPIOB
#define ADC_GPIO_PIN        GPIO_Pin_0

#define ADC_POWER_GPIO_TYPE GPIOB
#define ADC_POWER_GPIO_PIN  GPIO_Pin_1

typedef struct {
	uint16_t b95;
	uint16_t b90;
	uint16_t b85;
	uint16_t b80;
	uint16_t b75;
	uint16_t b70;
	uint16_t b65;
	uint16_t b60;
	uint16_t b55;
	uint16_t b50;
	uint16_t b45;
	uint16_t b40;
	uint16_t b35;
	uint16_t b30;
	uint16_t b25;
	uint16_t b20;
	uint16_t b15;
	uint16_t b10;
	uint16_t b05;
	uint16_t b00;
} ADC_sample;

extern const ADC_sample default_sample;

void ADC_init(void);
uint16_t ADC_get(void);
void adc2ascii(uint16_t adc, char *ascii, const ADC_sample *sample);

inline void adc2ascii_default(uint16_t adc, char *ascii) {
	adc2ascii(adc, ascii, &default_sample);
}

inline void adc_on(void) {
	GPIO_SetBits(GPIOB, GPIO_Pin_0);
}

inline void adc_off(void) {
	GPIO_ResetBits(GPIOB, GPIO_Pin_0);
}

#endif /* ADC_VOLATAGE_H_ */
