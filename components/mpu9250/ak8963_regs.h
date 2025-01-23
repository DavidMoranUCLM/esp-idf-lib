// ak8963_regs.h
// Register definitions for AK8963

#ifndef AK8963_REGS_H
#define AK8963_REGS_H

// Register Addresses
#define AK8963_WIA_ADDR    0x00 // Device ID
#define AK8963_INFO_ADDR   0x01 // Information
#define AK8963_ST1_ADDR    0x02 // Status 1
#define AK8963_HXL_ADDR    0x03 // X-axis data LSB
#define AK8963_HXH_ADDR    0x04 // X-axis data MSB
#define AK8963_HYL_ADDR    0x05 // Y-axis data LSB
#define AK8963_HYH_ADDR    0x06 // Y-axis data MSB
#define AK8963_HZL_ADDR    0x07 // Z-axis data LSB
#define AK8963_HZH_ADDR    0x08 // Z-axis data MSB
#define AK8963_ST2_ADDR    0x09 // Status 2
#define AK8963_CNTL1_ADDR  0x0A // Control 1
#define AK8963_CNTL2_ADDR  0x0B // Control 2
#define AK8963_ASTC_ADDR   0x0C // Self-test
#define AK8963_TS1_ADDR    0x0D // Test 1 (Do not access)
#define AK8963_TS2_ADDR    0x0E // Test 2 (Do not access)
#define AK8963_I2CDIS_ADDR 0x0F // I2C disable
#define AK8963_ASAX_ADDR   0x10 // X-axis sensitivity adjustment value
#define AK8963_ASAY_ADDR   0x11 // Y-axis sensitivity adjustment value
#define AK8963_ASAZ_ADDR   0x12 // Z-axis sensitivity adjustment value
#define AK8963_RSV_ADDR    0x13 // Reserved (Do not access)

// Register Bits
#define AK8963_ST1_DRDY_MASK   0x01 // Data Ready
#define AK8963_ST1_DOR_MASK    0x02 // Data Overrun

#define AK8963_ST2_HOFL_MASK   0x08 // Overflow
#define AK8963_ST2_BITM_MASK   0x10 // Magnetic sensor overflow

#define AK8963_CNTL1_MODE_MASK 0x0F // Mode Selection
#define AK8963_CNTL1_BIT_MASK  0x10 // Bit Setting

#define AK8963_CNTL2_SRST_MASK 0x01 // Soft Reset

#define AK8963_ASTC_SELF_MASK  0x40 // Self-Test Enable

// Sensitivity Adjustment Value Masks
#define AK8963_ASAX_MASK       0xFF
#define AK8963_ASAY_MASK       0xFF
#define AK8963_ASAZ_MASK       0xFF

// Reserved Registers
#define AK8963_RESERVED_TS1_MASK 0xFF // Test register 1 (Do not use)
#define AK8963_RESERVED_TS2_MASK 0xFF // Test register 2 (Do not use)
#define AK8963_RESERVED_RSV_MASK 0xFF // Reserved register (Do not use)

#endif // AK963C_REGS_H
