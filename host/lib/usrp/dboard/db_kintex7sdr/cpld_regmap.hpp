#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_CPLD_REGMAP_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_CPLD_REGMAP_HPP

#include <cstdint>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr { namespace cpld {

/***********************************************************************
 * SPI routing destinations (GPIO[SPI_ADDR] = 3 бита)
 * Должно совпадать с CPLD-логикой:
 *   CPLD_DEST = 0  -> доступ к регистрам CPLD
 *   остальные -> pass-through на внешние SPI-микросхемы
 **********************************************************************/
enum spi_dest_t : uint8_t {
    SPI_DEST_CPLD    = 0x0,
    SPI_DEST_LTC5594 = 0x1,
    SPI_DEST_LTC6948 = 0x2,
    // 0x3..0x7 reserved
};

/***********************************************************************
 * CPLD register map (если у тебя адреса другие — поменяй здесь,
 * но ДРАЙВЕР от этого собираться не перестанет).
 **********************************************************************/
enum reg_t : uint8_t {
    REG_ID         = 0x00, // read-only: версия/ID (если реализовано)
    REG_CTRL       = 0x01, // control bits
    REG_ATT2_CODE  = 0x02, // 7-bit code в младших битах
    REG_ATT2_MODE  = 0x03, // mode: auto-latch / direct / etc
    REG_STATUS     = 0x04  // sticky flags / status
};

// REG_CTRL bit definitions (должны совпадать с regmap_core)
static constexpr uint32_t CTRL_ATT1_C1  = (1u << 0);
static constexpr uint32_t CTRL_ATT1_C2  = (1u << 1);
static constexpr uint32_t CTRL_ATT1_MASK = (CTRL_ATT1_C1 | CTRL_ATT1_C2);

// Упаковка 32-битного слова под наш CPLD SPI engine: [CMD][DATA24]
// CMD: bit7 = 1 write / 0 read, bits[6:0] = reg
static inline uint32_t make_cmd(const bool is_write, const uint8_t reg7)
{
    return (uint32_t(is_write) << 7) | (reg7 & 0x7F);
}

static inline uint32_t make_frame_wr(const uint8_t reg7, const uint32_t data24)
{
    return (make_cmd(true, reg7) << 24) | (data24 & 0x00FFFFFFu);
}

static inline uint32_t make_frame_rd(const uint8_t reg7)
{
    return (make_cmd(false, reg7) << 24);
}

}}}}} // namespace uhd::usrp::dboard::db_kintex7sdr::cpld

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_CPLD_REGMAP_HPP
