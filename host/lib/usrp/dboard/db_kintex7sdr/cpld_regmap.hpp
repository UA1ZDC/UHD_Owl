// ==============================
// File: cpld_regmap.hpp
// ==============================
#ifndef DB_KINTEX7SDR_CPLD_REGMAP_HPP
#define DB_KINTEX7SDR_CPLD_REGMAP_HPP

#include <cstdint>
#include <functional>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {
namespace cpld {

/***********************************************************************
 * CPLD SPI protocol (32-bit frame, MODE0, MSB-first):
 *  [31:24] CMD:
 *    bit7   W/R (1=write, 0=read)
 *    bit6:4 BANK[2:0]
 *    bit3:0 REG[3:0]
 *  [23:0]  DATA (24-bit)
 *
 * Readback returns:
 *  [31:24] echo CMD
 *  [23:0]  DATA (24-bit)
 **********************************************************************/

using spi_xfer32_fn = std::function<uint32_t(uint32_t /*tx32*/)>;

static constexpr inline uint8_t make_cmd(bool wr, uint8_t bank, uint8_t reg)
{
    return uint8_t((wr ? 0x80 : 0x00) | ((bank & 0x7) << 4) | (reg & 0xF));
}

static constexpr inline uint32_t pack_frame(uint8_t cmd, uint32_t data24)
{
    return (uint32_t(cmd) << 24) | (data24 & 0x00FFFFFFu);
}

static constexpr inline uint32_t unpack_data24(uint32_t rx32)
{
    return (rx32 & 0x00FFFFFFu);
}

/***********************************************************************
 * Banks / Registers
 **********************************************************************/
enum : uint8_t {
    BANK_ID     = 0x0,
    BANK_CTRL   = 0x1,
    BANK_ATT2   = 0x2,
    BANK_GPIO   = 0x3,
};

enum : uint8_t {
    // BANK_ID (RO)
    REG_ID0       = 0x0,
    REG_ID1       = 0x1,
    REG_ID2       = 0x2,
    REG_VER_MAJOR = 0x3,
    REG_VER_MINOR = 0x4,
    REG_VER_PATCH = 0x5,
    REG_STATUS0   = 0x6,
    REG_STATUS1   = 0x7,

    // BANK_CTRL (RW)
    REG_CTRL0     = 0x0,
    REG_CTRL1     = 0x1,

    // BANK_ATT2 (RW)
    REG_ATT2_CODE = 0x0,
    REG_ATT2_CTRL = 0x1,

    // BANK_GPIO (RO)
    REG_GPIO_IN0  = 0x0,
    REG_GPIO_IN1  = 0x1,
};

/***********************************************************************
 * STATUS0 bits (BANK_ID/REG_STATUS0)
 **********************************************************************/
enum : uint32_t {
    STATUS0_STAT_LTC5594  = (1u << 0),
    STATUS0_STAT_LTC6948  = (1u << 1),
    STATUS0_STAT_ATT2     = (1u << 2),
    STATUS0_STAT_PLL_SYS  = (1u << 3),
    STATUS0_STAT_PLL_REF  = (1u << 4),
};

/***********************************************************************
 * STATUS1 bits (BANK_ID/REG_STATUS1)
 **********************************************************************/
enum : uint32_t {
    STATUS1_ERR_INVALID_SEL = (1u << 0), // SPI_ADDR changed during active CS window (sticky)
};

/***********************************************************************
 * CTRL0 bits (BANK_CTRL/REG_CTRL0)
 **********************************************************************/
enum : uint32_t {
    CTRL0_TPS_EN     = (1u << 0),
    CTRL0_LED        = (1u << 1),
    CTRL0_ATT1_C1    = (1u << 2),
    CTRL0_ATT1_C2    = (1u << 3),
    CTRL0_AUTOLATCH  = (1u << 4), // when 1: write ATT2_CODE autoloads to PE43711 (no extra APPLY)
    CTRL0_LE_POL     = (1u << 5),
    CTRL0_SOFT_RST   = (1u << 6), // synchronous soft reset (pulse)
};

/***********************************************************************
 * ATT2_CTRL bits (BANK_ATT2/REG_ATT2_CTRL)
 **********************************************************************/
enum : uint32_t {
    ATT2_CTRL_APPLY  = (1u << 0), // pulse to apply stored code (used when AUTOLATCH=0)
};

class cpld_iface
{
public:
    explicit cpld_iface(spi_xfer32_fn xfer32) : _xfer32(std::move(xfer32)) {}

    uint32_t read_reg(uint8_t bank, uint8_t reg)
    {
        const uint8_t cmd = make_cmd(false, bank, reg);
        const uint32_t rx = _xfer32(pack_frame(cmd, 0));
        return unpack_data24(rx);
    }

    void write_reg(uint8_t bank, uint8_t reg, uint32_t data24)
    {
        const uint8_t cmd = make_cmd(true, bank, reg);
        (void)_xfer32(pack_frame(cmd, data24));
    }

private:
    spi_xfer32_fn _xfer32;
};

} // namespace cpld
}}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // DB_KINTEX7SDR_CPLD_REGMAP_HPP
