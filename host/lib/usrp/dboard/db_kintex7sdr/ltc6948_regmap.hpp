#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP

#include <cstdint>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

namespace ltc6948 {

// Минимально нужные регистры (расширишь под set_freq):
enum : uint8_t {
    REG_STATUS   = 0x00,
    REG_STATMASK = 0x01,
    REG_PD       = 0x02,
    REG_ID       = 0x0E, // ROM byte for device identification:contentReference[oaicite:3]{index=3}
};

// Типичный формат команд у LTC6948: 16 бит (addr/data). Реальную упаковку уточни по твоему драйверу/UBX.
static inline constexpr uint16_t make_write_frame(uint8_t addr, uint8_t data)
{
    // Заглушка-упаковка: [addr][data]
    return (uint16_t(addr) << 8) | uint16_t(data);
}

static inline constexpr uint16_t make_read_frame(uint8_t addr)
{
    // Заглушка: [addr][dummy]
    return (uint16_t(addr) << 8);
}

} // namespace ltc6948

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP
