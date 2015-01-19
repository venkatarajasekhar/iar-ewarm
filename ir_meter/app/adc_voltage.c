#include "stm32f10x_adc.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "adc_voltage.h"
#include "seg.h"

uint8_t ADC_Reference_Calibration;
uint8_t ADC_Sample_Calibration;

/*
 const ADC_sample default_sample =
 { 		0xE00, // 3.5
 0xD44, // 3.317
 0xD41, // 3.314
 0xD3C, // 3.309
 0xD38, // 3.305
 0xD32, // 3.299
 0xD2E, // 3.295
 0xD2B, // 3.292
 0xD27, // 3.289
 0xD22, // 3.284
 0xD1E, // 3.280
 0xD1C, // 3.278
 0xD15, // 3.271
 0xD0F, // 3.265
 0xD06, // 3.256
 0xCF9, // 3.244
 0xCE9, // 3.228
 0xCD2, // 3.206
 0xCCD, // 3.201
 0xCC1, // 3.189
 // 0xCA2  // 3.159
 };
 */

void ADC_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;

	//enable ADC1 clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	//ADC1 configuration
	//select independent conversion mode (single)
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	//We will convert single channel only
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	//we will convert one time
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
	//select no external triggering
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	//right 12-bit data alignment in ADC data register
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	//single channel conversion
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	//load structure values to control and status registers
	ADC_Init(ADC1, &ADC_InitStructure);
	//wake up temperature sensor
	ADC_TempSensorVrefintCmd(ENABLE);
	//ADC1 channel8 configuration
	ADC_RegularChannelConfig(ADC1, ADC_Channel_9, 1, ADC_SampleTime_41Cycles5);

	ADC_Reference_Calibration = *((uint16_t *) 0x0801FC00) & 0x00ff;
	ADC_Sample_Calibration = (*((uint16_t *) 0x0801FC00) & 0xff00) >> 8;
}

uint16_t ADC_get(void)
{
//clear EOC flag
	ADC_ClearFlag(ADC1, ADC_FLAG_EOC);
//Start ADC1 Software Conversion
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);
//wait for conversion complete
	if (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != RESET)
		return ADC_GetConversionValue(ADC1);
	else
		return 0;
}

/*
 void adc2ascii(uint16_t adc, char *ascii, const ADC_sample *sample)
 {
 if (sample->b95 <= adc)
 {
 ascii[0] = 'F';
 ascii[1] = 'U';
 }
 else if (sample->b90 < adc && adc <= sample->b95)
 {
 ascii[0] = '9';
 ascii[1] = '5';
 }
 else if (sample->b85 < adc && adc <= sample->b90)
 {
 ascii[0] = '9';
 ascii[1] = '0';
 }
 else if (sample->b80 < adc && adc <= sample->b85)
 {
 ascii[0] = '8';
 ascii[1] = '5';
 }
 else if (sample->b75 < adc && adc <= sample->b80)
 {
 ascii[0] = '8';
 ascii[1] = '0';
 }
 else if (sample->b70 < adc && adc <= sample->b75)
 {
 ascii[0] = '7';
 ascii[1] = '5';
 }
 else if (sample->b65 < adc && adc <= sample->b70)
 {
 ascii[0] = '7';
 ascii[1] = '0';
 }
 else if (sample->b60 < adc && adc <= sample->b65)
 {
 ascii[0] = '6';
 ascii[1] = '5';
 }
 else if (sample->b55 < adc && adc <= sample->b60)
 {
 ascii[0] = '6';
 ascii[1] = '0';
 }
 else if (sample->b50 < adc && adc <= sample->b55)
 {
 ascii[0] = '5';
 ascii[1] = '5';
 }
 else if (sample->b45 < adc && adc <= sample->b50)
 {
 ascii[0] = '5';
 ascii[1] = '0';
 }
 else if (sample->b40 < adc && adc <= sample->b45)
 {
 ascii[0] = '4';
 ascii[1] = '5';
 }
 else if (sample->b35 < adc && adc <= sample->b40)
 {
 ascii[0] = '4';
 ascii[1] = '0';
 }
 else if (sample->b30 < adc && adc <= sample->b35)
 {
 ascii[0] = '3';
 ascii[1] = '5';
 }
 else if (sample->b25 < adc && adc <= sample->b30)
 {
 ascii[0] = '3';
 ascii[1] = '0';
 }
 else if (sample->b20 < adc && adc <= sample->b25)
 {
 ascii[0] = '2';
 ascii[1] = '5';
 }
 else if (sample->b15 < adc && adc <= sample->b20)
 {
 ascii[0] = '2';
 ascii[1] = '0';
 }
 else if (sample->b10 < adc && adc <= sample->b15)
 {
 ascii[0] = '1';
 ascii[1] = '5';
 }
 else if (sample->b05 < adc && adc <= sample->b10)
 {
 ascii[0] = '1';
 ascii[1] = '0';
 }
 else if (sample->b00 < adc && adc <= sample->b05)
 {
 ascii[0] = '0';
 ascii[1] = '5';
 }
 else if (adc && adc <= sample->b00)
 {
 ascii[0] = '0';
 ascii[1] = '0';
 }
 else
 ;
 }
 */
void adc2ascii(uint16_t adc, char *ascii)
{
	uint16_t value;
	if (adc < 0xca8)
	{
		ascii[0] = 'L';
		ascii[1] = 'o';
	}
	else
	{
		value = (adc - 0xca8) / 2;
	}
	if (value < 100)
	{
		ascii[0] = '0';
		ascii[1] = '0';
		seg_itoa(value, ascii, 2, 10);
	}
	else
	{
		ascii[0] = 'F';
		ascii[1] = 'U';
	}
}
