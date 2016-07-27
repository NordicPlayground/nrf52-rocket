#ifndef __DRV_PRESSURE_MPL3115A2_H__
#define __DRV_PRESSURE_MPL3115A2_H__

#include <stdint.h>

#define MPL3115A2_STATUS                0x00
#define MPL3115A2_OUT_P_MSB             0x01
#define MPL3115A2_OUT_P_CSB             0x02
#define MPL3115A2_OUT_P_LSB             0x03
#define MPL3115A2_OUT_T_MSB             0x04
#define MPL3115A2_OUT_T_LSB             0x05
#define MPL3115A2_DR_STATUS             0x06
#define MPL3115A2_OUT_P_DELTA_MSB       0x07 
#define MPL3115A2_OUT_P_DELTA_CSB       0x08
#define MPL3115A2_OUT_P_DELTA_LSB       0x09
#define MPL3115A2_OUT_T_DELTA_MSB       0x0A
#define MPL3115A2_OUT_T_DELTA_LSB       0x0B
#define MPL3115A2_WHO_AM_I              0x0C
#define MPL3115A2_F_STATUS              0x0D
#define MPL3115A2_F_DATA                0x0E
#define MPL3115A2_F_SETUP               0x0F
#define MPL3115A2_TIME_DLY              0x10
#define MPL3115A2_SYSMOD                0x11
#define MPL3115A2_INT_SOURCE            0x12
#define MPL3115A2_PT_DATA_CFG           0x13
#define MPL3115A2_BAR_IN_MSB            0x14
#define MPL3115A2_BAR_IN_LSB            0x15
#define MPL3115A2_P_TGT_MSB             0x16
#define MPL3115A2_P_TGT_LSB             0x17
#define MPL3115A2_T_TGT                 0x18
#define MPL3115A2_P_WND_MSB             0x19
#define MPL3115A2_P_WND_LSB             0x1A
#define MPL3115A2_T_WND                 0x1B
#define MPL3115A2_P_MIN_MSB             0x1C
#define MPL3115A2_P_MIN_CSB             0x1D
#define MPL3115A2_P_MIN_LSB             0x1E
#define MPL3115A2_T_MIN_MSB             0x1F
#define MPL3115A2_T_MIN_LSB             0x20
#define MPL3115A2_P_MAX_MSB             0x21
#define MPL3115A2_P_MAX_CSB             0x22
#define MPL3115A2_P_MAX_LSB             0x23
#define MPL3115A2_T_MAX_MSB             0x24
#define MPL3115A2_T_MAX_LSB             0x25
#define MPL3115A2_CTRL_REG1             0x26
#define MPL3115A2_CTRL_REG2             0x27
#define MPL3115A2_CTRL_REG3             0x28
#define MPL3115A2_CTRL_REG4             0x29
#define MPL3115A2_CTRL_REG5             0x2A
#define MPL3115A2_OFF_P                 0x2B
#define MPL3115A2_OFF_T                 0x2C
#define MPL3115A2_OFF_H                 0x2D
#define MPL3115A2_STATUS_TDR            (1 << 1)
#define MPL3115A2_STATUS_PDR            (1 << 2)
#define MPL3115A2_STATUS_PTDR           (1 << 3)
#define MPL3115A2_STATUS_TOW            (1 << 5)
#define MPL3115A2_STATUS_POW            (1 << 6)
#define MPL3115A2_STATUS_PTOW           (1 << 7)

#define MPL3115A2_PT_DATA_CFG_TDEFE     (1 << 0)
#define MPL3115A2_PT_DATA_CFG_PDEFE     (1 << 1)
#define MPL3115A2_PT_DATA_CFG_DREM      (1 << 2)

#define MPL3115A2_CTRL_REG1_SBYB_MASK   (1 << 0)
#define MPL3115A2_CTRL_REG1_OST         (1 << 1)
#define MPL3115A2_CTRL_REG1_RST         (1 << 2)

#define MPL3115A2_CTRL_REG1_OS_BITS     0x7
#define MPL3115A2_CTRL_REG1_OS_POS      3
#define MPL3115A2_CTRL_REG1_OS_MASK     (MPL3115A2_CTRL_REG1_OS_BITS << MPL3115A2_CTRL_REG1_OS_POS)

#define MPL3115A2_CTRL_REG1_RAW         (1 << 6)
#define MPL3115A2_CTRL_REG1_ALT_MASK    (1 << 7)

#define MPL3115A2_CTRL_REG2_ST_BITS     0xF
#define MPL3115A2_CTRL_REG2_ST_POS      0
#define MPL3115A2_CTRL_REG2_ST_MASK     (MPL3115A2_CTRL_REG2_ST_BITS << MPL3115A2_CTRL_REG2_ST_POS)

#define MPL3115A2_CTRL_REG2_ALARM_SEL   (1 << 4)
#define MPL3115A2_CTRL_REG2_LOAD_OPUT   (1 << 5)

#define MPL3115A2_CTRL_REG3_PP_OD2      (1 << 0)
#define MPL3115A2_CTRL_REG3_IPOL2       (1 << 1)
#define MPL3115A2_CTRL_REG3_PP_OD1      (1 << 4)
#define MPL3115A2_CTRL_REG3_IPOL1       (1 << 5)

#define MPL3115A2_CTRL_REG4_INT_EN_TCHG (1 << 0)
#define MPL3115A2_CTRL_REG4_INT_EN_PCHG (1 << 1)
#define MPL3115A2_CTRL_REG4_INT_EN_TTH  (1 << 2)
#define MPL3115A2_CTRL_REG4_INT_EN_PTH  (1 << 3)
#define MPL3115A2_CTRL_REG4_INT_EN_TW   (1 << 4)
#define MPL3115A2_CTRL_REG4_INT_EN_PW   (1 << 5)
#define MPL3115A2_CTRL_REG4_INT_EN_FIFO (1 << 6)
#define MPL3115A2_CTRL_REG4_INT_EN_DRDY (1 << 7)

#define MPL3115A2_CTRL_REG5_INT_CFG_TCHG    (1 << 0)
#define MPL3115A2_CTRL_REG5_INT_CFG_PCHG    (1 << 1)
#define MPL3115A2_CTRL_REG5_INT_CFG_TTH     (1 << 2)
#define MPL3115A2_CTRL_REG5_INT_CFG_PTH     (1 << 3)
#define MPL3115A2_CTRL_REG5_INT_CFG_TW      (1 << 4)
#define MPL3115A2_CTRL_REG5_INT_CFG_PW      (1 << 5)
#define MPL3115A2_CTRL_REG5_INT_CFG_FIFO    (1 << 6)
#define MPL3115A2_CTRL_REG5_INT_CFG_DRDY    (1 << 7)

#define MPL3115A2_ID                    0xC4

#define MPL3115A2_ADDR                  0x60

typedef struct
{
    uint8_t msb;
    uint8_t csb;
    uint8_t lsb;
}drv_mp3115a2_pressure_data_t;

typedef struct
{
    uint8_t msb;
    uint8_t lsb;
}drv_mp3115a2_temp_data_t;

typedef struct
{
    uint8_t                      status;
    drv_mp3115a2_pressure_data_t pressure;
    drv_mp3115a2_temp_data_t     temp;
}drv_mp3115a2_data_t;

#define DRV_PRESSURE_TX_BUF_SIZE        1
#define DRV_PRESSURE_RX_BUF_SIZE        2

#endif
