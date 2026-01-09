// ==============================
// File: ltc5594_regmap.hpp
// ==============================
#ifndef DB_KINTEX7SDR_LTC5594_REGMAP_HPP
#define DB_KINTEX7SDR_LTC5594_REGMAP_HPP

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <functional>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {
namespace ltc5594 {

/***********************************************************************
 * LTC5594 SPI protocol (MODE0, MSB-first):
 *  1st byte: [7]=W/R (1=write, 0=read), [6:0]=ADDR (0x00..0x17)
 *  2nd byte: DATA (8-bit)
 *
 * For read: send addr byte (W/R=0) then dummy byte; data is returned in 2nd byte.
 **********************************************************************/
using spi_xfer16_fn = std::function<uint16_t(uint16_t /*tx16*/)>;

static constexpr inline uint8_t addr_wr(uint8_t a) { return uint8_t(0x80u | (a & 0x7Fu)); }
static constexpr inline uint8_t addr_rd(uint8_t a) { return uint8_t(0x00u | (a & 0x7Fu)); }

/***********************************************************************
 * Register addresses used by helpers
 **********************************************************************/
enum : uint8_t {
    REG_PHA_MSB = 0x14, // PHA[8:1]
    REG_PHA_LSB = 0x15, // bit7 = PHA[0]
    REG_EN      = 0x16, // enables + reset + SDO_MODE
    REG_MISC    = 0x17, // CHIPID[1:0] at [7:6]
};

/***********************************************************************
 * REG_EN (0x16) bits
 *  [7] EDEM, [6] EDC, [5] EADJ, [4] EAMP, [3] SRST, [2] SDO_MODE, [1:0] don't care
 **********************************************************************/
enum : uint8_t {
    EN_EAMP     = (1u << 4),
    EN_SRST     = (1u << 3),
    EN_SDO_MODE = (1u << 2),
};

/***********************************************************************
 * REG_MISC (0x17) bits
 **********************************************************************/
enum : uint8_t {
    MISC_CHIPID_MASK = 0xC0u, // [7:6]
    MISC_CHIPID_SHIFT = 6,
};

class ltc5594_iface
{
public:
    explicit ltc5594_iface(spi_xfer16_fn xfer16) : _xfer16(std::move(xfer16)) {}

    void write_reg(uint8_t addr, uint8_t data)
    {
        const uint16_t tx = (uint16_t(addr_wr(addr)) << 8) | uint16_t(data);
        (void)_xfer16(tx);
    }

    uint8_t read_reg(uint8_t addr)
    {
        const uint16_t tx = (uint16_t(addr_rd(addr)) << 8) | 0x00u;
        const uint16_t rx = _xfer16(tx);
        return uint8_t(rx & 0xFFu);
    }

    void soft_reset()
    {
        uint8_t v = read_reg(REG_EN);
        write_reg(REG_EN, uint8_t(v | EN_SRST));
        write_reg(REG_EN, uint8_t(v & ~EN_SRST));
    }

    void enable_sdo_readback(bool en)
    {
        uint8_t v = read_reg(REG_EN);
        v = en ? uint8_t(v | EN_SDO_MODE) : uint8_t(v & ~EN_SDO_MODE);
        write_reg(REG_EN, v);
    }

    void enable_if_amp(bool en)
    {
        uint8_t v = read_reg(REG_EN);
        v = en ? uint8_t(v | EN_EAMP) : uint8_t(v & ~EN_EAMP);
        write_reg(REG_EN, v);
    }

    // PHA code: 9-bit, default 0x100 => 0 degrees
    // Datasheet range ~[-2.5°, +2.5°], practical symmetric mapping around 0x100 with LSB=2.5/256.
    void set_phase_error_deg(double deg)
    {
        const double lsb_deg = 2.5 / 256.0;
        const int code = int(std::lround(0x100 + (deg / lsb_deg)));
        const int clamped = std::max(0, std::min(0x1FF, code));
        const uint16_t pha = uint16_t(clamped);

        // REG_PHA_MSB: PHA[8:1]
        write_reg(REG_PHA_MSB, uint8_t((pha >> 1) & 0xFFu));

        // REG_PHA_LSB bit7: PHA[0] (preserve the rest)
        uint8_t v15 = read_reg(REG_PHA_LSB);
        v15 = (pha & 0x1) ? uint8_t(v15 | 0x80u) : uint8_t(v15 & ~0x80u);
        write_reg(REG_PHA_LSB, v15);
    }

    uint8_t read_chip_id()
    {
        const uint8_t v = read_reg(REG_MISC);
        return uint8_t((v & MISC_CHIPID_MASK) >> MISC_CHIPID_SHIFT);
    }

private:
    spi_xfer16_fn _xfer16;
};

} // namespace ltc5594
}}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // DB_KINTEX7SDR_LTC5594_REGMAP_HPP
