#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC5594_REGMAP_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC5594_REGMAP_HPP

#include <cstdint>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr { namespace ltc5594 {

/***********************************************************************
 * LTC5594 uses 16-bit SPI:
 *  - First byte: MSB is R/W, lower 7 bits = address
 *  - Second byte: data (or dummy for reads)
 **********************************************************************/
static inline uint16_t make_addr_byte_wr(uint8_t addr7) { return uint16_t(0x80u | (addr7 & 0x7Fu)); }
static inline uint16_t make_addr_byte_rd(uint8_t addr7) { return uint16_t((addr7 & 0x7Fu)); }

static inline uint16_t make_word_wr(uint8_t addr7, uint8_t data)
{
    return uint16_t(make_addr_byte_wr(addr7) << 8) | data;
}

static inline uint16_t make_word_rd(uint8_t addr7)
{
    return uint16_t(make_addr_byte_rd(addr7) << 8); // data byte = 0
}

static inline uint8_t  rx_data_byte(uint16_t rx_word) { return uint8_t(rx_word & 0xFFu); }

/***********************************************************************
 * Full register address map (0x00..0x17)
 **********************************************************************/
enum reg_t : uint8_t {
    REG_IM3Q0 = 0x00,
    REG_IM3Q1 = 0x01,
    REG_IM3Q2 = 0x02,
    REG_IM3Q3 = 0x03,
    REG_IM3Q4 = 0x04,
    REG_IM3Q5 = 0x05,
    REG_IM3Q6 = 0x06,
    REG_IM3Q7 = 0x07,
    REG_IM3I0 = 0x08,
    REG_IM3I1 = 0x09,
    REG_IM3I2 = 0x0A,
    REG_IM3I3 = 0x0B,
    REG_IM3I4 = 0x0C,
    REG_IM3I5 = 0x0D,
    REG_IM3I6 = 0x0E,
    REG_IM3I7 = 0x0F,
    REG_HD3   = 0x10,
    REG_HD2   = 0x11,
    REG_DCO   = 0x12,
    REG_IPC   = 0x13,
    REG_PHA   = 0x14,
    REG_PHAI  = 0x15,
    REG_BCTL  = 0x16,
    REG_CHIPID= 0x17
};

/***********************************************************************
 * Practical helper bits (минимум нужного для драйвера).
 * Если у тебя есть точные маски из Table 9 — подставь,
 * но даже так структура regmap уже “на месте” и не теряется.
 **********************************************************************/
namespace bctl {
    // REG_BCTL (0x16) — enable/disable blocks (имена из таблицы)
    static const uint8_t BIT_EDEM = (1u << 7);
    static const uint8_t BIT_EAMP = (1u << 6); // IF AMP enable
    static const uint8_t BIT_EDC  = (1u << 5);
    static const uint8_t BIT_EADJ = (1u << 4);
    // Остальные биты оставь как есть (board defaults)
}

/***********************************************************************
 * PHA helper (9-bit code): хранится в REG_PHA/REG_PHAI.
 * Без “reverse7()”: драйвер формирует код как нужно.
 **********************************************************************/
static inline void pack_pha_9b(uint16_t pha_code_9b, uint8_t& pha_reg, uint8_t& phai_reg)
{
    pha_code_9b &= 0x01FFu;
    // REG_PHA: [8:1]
    pha_reg = uint8_t((pha_code_9b >> 1) & 0xFFu);
    // REG_PHAI: bit0 = PHA[0] (остальные биты REG_PHAI не трогаем снаружи)
    phai_reg = uint8_t(pha_code_9b & 0x01u);
}

}}}}} // namespace uhd::usrp::dboard::db_kintex7sdr::ltc5594

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC5594_REGMAP_HPP
