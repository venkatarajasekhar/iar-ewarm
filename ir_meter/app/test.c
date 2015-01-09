/*
 * test.c
 *
 *  Created on: Nov 12, 2014
 *      Author: BHEscaper
 */

#include <stdint.h>
#include "test.h"
#include "platform_config.h"

#include "seg.h"
extern seg_config seg_cfg;
extern volatile seg_type segment;
//extern volatile uint32_t wakeup_cd;
//extern volatile uint32_t shutdown_cd;

extern uint8_t biao_addr[20];
extern uint8_t com_add_cmd07[40];
extern uint8_t com_add_cmd97[40];
extern uint8_t com_cmd07[18][25];
extern uint8_t com_cmd97[18][25];
extern uint8_t scan_var;
extern uint32_t t2_cont;
extern uint8_t scan_en;
extern uint8_t key_test_flag;

int test07(void)
{
	uint8_t buf[RXD_SUM];
	uint8_t len = 0;
	uint8_t i, j, k, biao_sum;
	uint8_t chk = 0;

	put_string(2, &com_add_cmd07[1], com_add_cmd07[0]); //发07命令 仅第1条命令

	segment.display = SEG_DISPLAY_ON;
	if (key_test_flag == 1) {
		segment.dig8[0] = ascii2seg_default('t');
		segment.dig8[1] = ascii2seg_default('E');
	} else {
		segment.dig8[0] = ascii2seg_default(' ' | 0x80);
		segment.dig8[1] = ascii2seg_default(' ' | 0x80);
	}

	t2_cont = 0;
	while (t2_cont <= 2000) //等回复，超时时间500ms。
	{
		seg_flush(&segment, &seg_cfg);

		WWDG_WEI();
		len = get_string(2, buf);
		if (len > 7) //收到数据
		{
			for (i = 6; i < len - 3; i++)
			{
				if (buf[i] == 0x68 && buf[i + 1] == 0x91)
				{
					scan_var = 1; //1 [1200 07] 2 [1200 97]  3 [2400 07] 4 [2400 97] 其他设置认为无效。
					biao_addr[0] = buf[i - 6];
					biao_addr[1] = buf[i - 5];
					biao_addr[2] = buf[i - 4];
					biao_addr[3] = buf[i - 3];
					biao_addr[4] = buf[i - 2];
					biao_addr[5] = buf[i - 1];
					for (k = 0; k < 18; k++)
					{
						for (j = 0; j < 6; j++)
						{
							com_cmd07[k][6 + j] = biao_addr[j];
						}
						biao_sum = 0;
						for (j = 5; j < 19; j++)
						{
							biao_sum += com_cmd07[k][j];
						}
						com_cmd07[k][19] = biao_sum;
					}
#if 1
					for (k = 0; k < 6; k++)
					{
						put_string(2, &com_cmd07[k][1], com_cmd07[k][0]);
						t2_cont = 0;
						while (t2_cont <= 2000) //等回复，超时时间500ms。
						{
							seg_flush(&segment, &seg_cfg);

							WWDG_WEI();
							len = get_string(2, buf);
							if (len > 7) //收到数据
							{
								for (j = 6; j < len - 3; j++)
								{
									if (buf[j] == 0x68 && buf[j + 1] == 0x91)
									{
										chk++;
									}
								}
							}
						}
					}
					return (chk + 1);  //成功，则返回。
#else
					return 0;
#endif
				}
			}
		}
	}
	return 0;
}

int test97(void)
{
	uint8_t buf[RXD_SUM];
	uint8_t len = 0;
	uint8_t i, j, k, biao_sum;
	uint8_t chk = 0;

	put_string(2, &com_add_cmd97[1], com_add_cmd97[0]);  //发97命令 仅第1条命令

	segment.display = SEG_DISPLAY_ON;
	if (key_test_flag == 1) {
		segment.dig8[0] = ascii2seg_default('t');
		segment.dig8[1] = ascii2seg_default('E');
	} else {
		segment.dig8[0] = ascii2seg_default(' ' | 0x80);
		segment.dig8[1] = ascii2seg_default(' ' | 0x80);
	}

	t2_cont = 0;
	while (t2_cont <= 2000)  //等回复，超时时间500ms。
	{
		seg_flush(&segment, &seg_cfg);

		WWDG_WEI();
		len = get_string(2, buf);
		if (len > 7)  //收到数据
		{
			for (i = 6; i < len - 3; i++)
			{
				if (buf[i] == 0x68 && buf[i + 1] == 0x81)
				{
					scan_var = 2; //1 [1200 07] 2 [1200 97]  3 [2400 07] 4 [2400 97] 其他设置认为无效。
					biao_addr[0] = buf[i - 6];
					biao_addr[1] = buf[i - 5];
					biao_addr[2] = buf[i - 4];
					biao_addr[3] = buf[i - 3];
					biao_addr[4] = buf[i - 2];
					biao_addr[5] = buf[i - 1];
					for (k = 0; k < 18; k++)
					{
						for (j = 0; j < 6; j++)
						{
							com_cmd97[k][6 + j] = biao_addr[j];
						}
						biao_sum = 0;
						for (j = 5; j < 17; j++)
						{
							biao_sum += com_cmd97[k][j];
						}
						com_cmd97[k][17] = biao_sum;
					}
#if 1
					for (k = 0; k < 6; k++)
					{
						put_string(2, &com_cmd97[k][1], com_cmd97[k][0]);
						t2_cont = 0;
						while (t2_cont <= 2000) //等回复，超时时间500ms。
						{
							seg_flush(&segment, &seg_cfg);

							WWDG_WEI();
							len = get_string(2, buf);
							if (len > 7) //收到数据
							{
								for (j = 6; j < len - 3; j++)
								{
									if (buf[j] == 0x68 && buf[j + 1] == 0x81)
									{
										chk++;
									}
								}
							}
						}
					}
					return (chk + 1);  //成功，则返回。
#else
					return 0;
#endif
				}
			}
		}
	}
	return 0;
}
