/*
 * seg.c
 */

#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "seg.h"

const seg_map seg_map_default = {
		0x80, // seg_DP

		0x3F, // seg_0
		0x06, // seg_1
		0x5B, // seg_2
		0x4F, // seg_3
		0x66, // seg_4
		0x6D, // seg_5
		0x7D, // seg_6
		0x07, // seg_7
		0x7F, // seg_8
		0x6F, // seg_9

		0x77, // seg_A
		0x39, // seg_C
		0x79, // seg_E
		0x71, // seg_F
		0x3D, // seg_G
		0x76, // seg_H
		0x1E, // seg_J
		0x38, // seg_L
		0x37, // seg_N
		0x73, // seg_P
		0x3E, // seg_U
		0x6E, // seg_Y

		0x5F, // seg_a
		0x7C, // seg_b
		0x58, // seg_c
		0x5E, // seg_d
		0x7b, // seg_e
		0x74, // seg_h
		0x10, // seg_i
		0x0E, // seg_j
		0x06, // seg_l
		0x54, // seg_n
		0x5C, // seg_o
		0x67, // seg_q
		0x50, // seg_r
		0x78, // seg_t
		0x1C, // seg_u
		};

void seg_init(const seg_config *cfg) {
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(cfg->rcc, ENABLE);

	GPIO_InitStructure.GPIO_Mode = cfg->mode;
	GPIO_InitStructure.GPIO_Speed = cfg->speed;

	for (uint8_t i = 0; i < SEG_WIDTH + SEG_DIG; i++) {
		GPIO_InitStructure.GPIO_Pin = cfg->pin[i];
		GPIO_Init(cfg->pin_type[i], &GPIO_InitStructure);
	}
}

uint8_t ascii2seg(char ascii, const seg_map *map) {
	uint8_t DP_flag;

	DP_flag = ((ascii & 0x80) == 0x80) ? map->seg_DP : 0x0;

	switch (ascii & 0x7F) {
	case ' ':
		return DP_flag;
	case '0':
		return map->seg_0 | DP_flag;
	case '1':
		return map->seg_1 | DP_flag;
	case '2':
		return map->seg_2 | DP_flag;
	case '3':
		return map->seg_3 | DP_flag;
	case '4':
		return map->seg_4 | DP_flag;
	case '5':
		return map->seg_5 | DP_flag;
	case '6':
		return map->seg_6 | DP_flag;
	case '7':
		return map->seg_7 | DP_flag;
	case '8':
		return map->seg_8 | DP_flag;
	case '9':
		return map->seg_9 | DP_flag;
	case 'A':
		return map->seg_A | DP_flag;
	case 'C':
		return map->seg_C | DP_flag;
	case 'E':
		return map->seg_E | DP_flag;
	case 'F':
		return map->seg_F | DP_flag;
	case 'G':
		return map->seg_G | DP_flag;
	case 'H':
		return map->seg_H | DP_flag;
	case 'J':
		return map->seg_J | DP_flag;
	case 'L':
		return map->seg_L | DP_flag;
	case 'N':
		return map->seg_N | DP_flag;
	case 'P':
		return map->seg_P | DP_flag;
	case 'U':
		return map->seg_U | DP_flag;
	case 'Y':
		return map->seg_Y | DP_flag;
	case 'a':
		return map->seg_a | DP_flag;
	case 'b':
		return map->seg_b | DP_flag;
	case 'c':
		return map->seg_c | DP_flag;
	case 'd':
		return map->seg_d | DP_flag;
	case 'e':
		return map->seg_e | DP_flag;
	case 'h':
		return map->seg_h | DP_flag;
	case 'i':
		return map->seg_i | DP_flag;
	case 'j':
		return map->seg_j | DP_flag;
	case 'l':
		return map->seg_l | DP_flag;
	case 'n':
		return map->seg_n | DP_flag;
	case 'o':
		return map->seg_o | DP_flag;
	case 'q':
		return map->seg_q | DP_flag;
	case 'r':
		return map->seg_r | DP_flag;
	case 't':
		return map->seg_t | DP_flag;
	case 'u':
		return map->seg_u | DP_flag;
	default:
		return 0;
	}
}

void seg_flush(volatile seg_type *seg, const seg_config *cfg) {
	int i;

	if (seg->display == SEG_DISPLAY_ON) {

		for (i = ((seg->idx + 1) % SEG_DIG); i != seg->idx;
				i = ((i + 1) % SEG_DIG))
			if (cfg->seg_en == SEG_RESET_EN)
				GPIO_SetBits(cfg->pin_type[SEG_WIDTH + i],
						cfg->pin[SEG_WIDTH + i]);
			else /* SEG_SET_EN */
				GPIO_ResetBits(cfg->pin_type[SEG_WIDTH + i],
						cfg->pin[SEG_WIDTH + i]);

		for (i = 0; i < SEG_WIDTH; i++)
			if ((seg->dig8[seg->idx] >> i) & 0x1)
				if (cfg->dig_en == DIG_RESET_EN)
					GPIO_ResetBits(cfg->pin_type[i], cfg->pin[i]);
				else /* DIG_SET_EN */
					GPIO_SetBits(cfg->pin_type[i], cfg->pin[i]);
			else
				if (cfg->dig_en == DIG_RESET_EN)
					GPIO_SetBits(cfg->pin_type[i], cfg->pin[i]);
				else /* DIG_SET_EN */
					GPIO_ResetBits(cfg->pin_type[i], cfg->pin[i]);

		if (cfg->seg_en == SEG_RESET_EN)
			GPIO_ResetBits(cfg->pin_type[SEG_WIDTH + seg->idx],
					cfg->pin[SEG_WIDTH + seg->idx]);
		else /* SEG_SET_EN */
			GPIO_SetBits(cfg->pin_type[SEG_WIDTH + seg->idx],
					cfg->pin[SEG_WIDTH + seg->idx]);

		seg->idx++;
		seg->idx %= SEG_DIG;

	} else { /* SEG_DISPLAY_OFF */
		for (i = 0; i < SEG_WIDTH; i++)
			if (cfg->dig_en == DIG_RESET_EN)
				GPIO_SetBits(cfg->pin_type[i], cfg->pin[i]);
			else /* DIG_SET_EN */
				GPIO_ResetBits(cfg->pin_type[i], cfg->pin[i]);

		for (i = 0; i < SEG_DIG; i++)
			if (cfg->seg_en == SEG_RESET_EN)
				GPIO_SetBits(cfg->pin_type[SEG_WIDTH + i],
						cfg->pin[SEG_WIDTH + i]);
			else /* SEG_SET_EN */
				GPIO_ResetBits(cfg->pin_type[SEG_WIDTH + i],
						cfg->pin[SEG_WIDTH + i]);
	}
}

int seg_itoa(int num, char *str, int len, int base) {
	int sum = num;
	int i = len;
	int digit;

	if (len == 0)
		return -1;

	do {
		digit = sum % base;
		if (digit < 0xA)
			str[--i] = (char) ('0' + digit);
		else
			str[--i] = (char) ('A' + digit - 0xA);
		sum /= base;
	} while ((sum > 0) && (i > 0));

	if ((i == 0) && (sum > 0))
		return -1;

	return 0;
}
