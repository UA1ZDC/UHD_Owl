#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP

#include <cstdint>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr { namespace ltc6948 {

/***********************************************************************
 * LTC6948 SPI: 16 clocks per register
 * First byte: [A6..A0][R/W]  (R/W is LSB!)
 * Second byte: data
 **********************************************************************/
static inline uint8_t make_addr_byte_wr(uint8_t addr7) { return uint8_t((addr7 & 0x7Fu) << 1); }
static inline uint8_t make_addr_byte_rd(uint8_t addr7) { return uint8_t(((addr7 & 0x7Fu) << 1) | 0x01u); }

static inline uint16_t make_word_wr(uint8_t addr7, uint8_t data)
{
    return uint16_t(make_addr_byte_wr(addr7) << 8) | data;
}

static inline uint16_t make_word_rd(uint8_t addr7)
{
    return uint16_t(make_addr_byte_rd(addr7) << 8); // data byte = 0
}

static inline uint8_t rx_data_byte(uint16_t rx_word) { return uint8_t(rx_word & 0xFFu); }

/***********************************************************************
 * Register addresses (основные, можно расширить)
 **********************************************************************/
enum reg_t : uint8_t {
    REG0 = 0x00,
    REG1 = 0x01,
    REG2 = 0x02,
    REG3 = 0x03,
    REG4 = 0x04,
    REG5 = 0x05,
    REG6 = 0x06,
    REG7 = 0x07,
    REG8 = 0x08,
    REG9 = 0x09,
    REGA = 0x0A,
    REGB = 0x0B,
    REGC = 0x0C,
    REGD = 0x0D,
    REGE = 0x0E
};

}}}}} // namespace uhd::usrp::dboard::db_kintex7sdr::ltc6948

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP
