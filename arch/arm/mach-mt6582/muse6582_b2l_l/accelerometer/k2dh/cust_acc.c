#include <linux/types.h>
#include <cust_acc.h>
#include <mach/mt_pm_ldo.h>
#include <mach/board_lge.h>

#if defined(TARGET_S4)
extern void mtk_sensor_power(int on_off);

/*---------------------------------------------------------------------------*/
int cust_acc_power(struct acc_hw *hw, unsigned int on, char* devname)
{
    if (on)
        mtk_sensor_power(1);
    else
        mtk_sensor_power(0);

    return 1;
}
#endif
/*---------------------------------------------------------------------------*/
extern int K2HH; //for K2HH

static struct acc_hw cust_acc_hw = {
    .i2c_num = 2, 
    .direction = 4,
    .power_id = MT65XX_POWER_NONE,  /*!< LDO is not used */
    .power_vol= VOL_DEFAULT,        /*!< LDO is not used */
    .firlen = 16,                   /*!< don't enable low pass fileter */
#if defined(TARGET_S4)
    .power = cust_acc_power,
#endif   
};
/*---------------------------------------------------------------------------*/
struct acc_hw* get_cust_acc_hw(void) 
{
  if(lge_get_board_revno() >= HW_REV_1_0)
  	{
     cust_acc_hw.direction = 7;
     K2HH=1;
  	}
  else
    cust_acc_hw.direction = 4;
    return &cust_acc_hw;
}
