/*
 * seg.h
 */

#ifndef SEG_H_
#define SEG_H_

#include <stdint.h>

#define SEG_WIDTH 8
#define SEG_DIG   2
#define SEG_PIN   (SEG_WIDTH + SEG_DIG)

enum {
	SEG_SET_EN = 0, SEG_RESET_EN = !SEG_SET_EN
};

enum {
	DIG_SET_EN = 0, DIG_RESET_EN = !DIG_SET_EN
};

enum {
	SEG_DISPLAY_OFF = 0, SEG_DISPLAY_ON = !SEG_DISPLAY_OFF
};

typedef struct {
	uint8_t seg_DP;

	uint8_t seg_0;
	uint8_t seg_1;
	uint8_t seg_2;
	uint8_t seg_3;
	uint8_t seg_4;
	uint8_t seg_5;
	uint8_t seg_6;
	uint8_t seg_7;
	uint8_t seg_8;
	uint8_t seg_9;

	uint8_t seg_A;
	uint8_t seg_C;
	uint8_t seg_E;
	uint8_t seg_F;
	uint8_t seg_G;
	uint8_t seg_H;
	uint8_t seg_J;
	uint8_t seg_L;
	uint8_t seg_N;
	uint8_t seg_P;
	uint8_t seg_U;
	uint8_t seg_Y;

	uint8_t seg_a;
	uint8_t seg_b;
	uint8_t seg_c;
	uint8_t seg_d;
	uint8_t seg_e;
	uint8_t seg_h;
	uint8_t seg_i;
	uint8_t seg_j;
	uint8_t seg_l;
	uint8_t seg_n;
	uint8_t seg_o;
	uint8_t seg_q;
	uint8_t seg_r;
	uint8_t seg_t;
	uint8_t seg_u;
} seg_map;

extern const seg_map seg_map_default;

typedef struct {
	/*************************
	 * pin[0] A    pin[5] F  *
	 * pin[1] B    pin[6] G  *
	 * pin[2] C    pin[7] DP *
	 * pin[3] D    pin[8] A1 *
	 * pin[4] E    pin[9] A2 *
	 *************************/
	uint16_t pin[SEG_PIN];
	GPIO_TypeDef *pin_type[SEG_PIN];
	GPIOSpeed_TypeDef speed;
	GPIOMode_TypeDef mode;
	uint8_t seg_en; /* SEG_SET_EN or SEG_RESET_EN */
	uint8_t dig_en; /* DIG_SET_EN or DIG_RESET_EN */
	uint32_t rcc;
} seg_config;

typedef struct {
	uint8_t display; /* SEG_DISPLAY_OFF or SEG_DISPLAY_ON */
	uint8_t idx;
	uint8_t dig8[SEG_DIG];
} seg_type;

void seg_init(const seg_config *cfg);
uint8_t ascii2seg(char c, const seg_map *map);
void seg_flush(volatile seg_type *seg, const seg_config *cfg);
int seg_itoa(int num, char *str, int len, int base);

inline uint8_t ascii2seg_default(char c) {
	return ascii2seg(c, &seg_map_default);
}

#endif /* SEG_H_ */
