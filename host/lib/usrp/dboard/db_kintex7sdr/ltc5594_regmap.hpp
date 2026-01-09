#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC5594_REGMAP_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC5594_REGMAP_HPP

#include <cstdint>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

namespace ltc5594 {

// Регистр CHIPID — 0x17 (по даташиту LTC5594 таблица регистров):contentReference[oaicite:2]{index=2}
enum : uint8_t {
    REG_CHIPID = 0x17,
};

// Формирование 16-бит SPI транзакций:
// cmd[7]=1 write, cmd[7]=0 read, cmd[6:0]=addr, далее data (или dummy для read)
static inline constexpr uint16_t make_write_frame(uint8_t addr, uint8_t data)
{
    return (uint16_t(uint8_t(0x80u | (addr & 0x7Fu))) << 8) | uint16_t(data);
}

static inline constexpr uint16_t make_read_frame(uint8_t addr)
{
    return (uint16_t(uint8_t(addr & 0x7Fu)) << 8); // data=dummy=0
}

} // namespace ltc5594

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC5594_REGMAP_HPP
