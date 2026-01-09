#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_RX_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_RX_HPP

#include <uhd/usrp/dboard_base.hpp>
#include <uhd/types/direction.hpp>
#include <uhd/utils/log.hpp>

#include <cstdint>
#include <map>
#include <mutex>
#include <type_traits>
#include <utility>

#include "cpld_regmap.hpp"
#include "ltc5594_regmap.hpp"
#include "ltc6948_regmap.hpp"

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

// ===== ID платы =====
// Чтобы у тебя СЕЙЧАС собралось — дефолт 0xFFFF (потом поменяешь на реальный RX EEPROM ID).
static constexpr uint16_t DB_KINTEX7SDR_RX_ID      = 0xFFFF;
static constexpr uint16_t DB_KINTEX7SDR_TX_ID_NONE = 0xFFFF;

// ===== SPI routing через GPIO[2:0] (SPI_ADDR) =====
// ВНИМАНИЕ: значения должны совпадать с декодером в CPLD.
enum spi_dest_t : uint8_t {
    SPI_DEST_CPLD     = 0x0,
    SPI_DEST_LTC5594  = 0x1,
    SPI_DEST_LTC6948  = 0x2,
    SPI_DEST_AD7922   = 0x3,
    SPI_DEST_ATT2     = 0x4,
    SPI_DEST_NONE     = 0x7, // безопасное “никого не выбрали”
};

// ===== GPIO fields (то, что реально у тебя заведено как FPGA_OUT на UNIT_RX) =====
enum class gpio_field_id : uint8_t {
    SPI_ADDR,
    CPLD_RST_N,
};

struct gpio_field_info {
    gpio_field_id         id;
    dboard_iface::unit_t   unit;
    uint32_t               shift;
    uint32_t               mask;
    uint32_t               width;
    bool                   is_output;
};

class db_kintex7sdr_rx : public rx_dboard_base
{
public:
    explicit db_kintex7sdr_rx(ctor_args_t args);
    ~db_kintex7sdr_rx() override = default;

private:
    dboard_iface::sptr _iface;
    std::map<gpio_field_id, gpio_field_info> _gpio;
    std::mutex _spi_mutex;

    // ---- GPIO helpers ----
    void _init_gpio();
    void _write_gpio_field(gpio_field_id id, uint32_t value);
    void _set_spi_route(spi_dest_t dest);

    // ---- SPI helpers ----
    void _spi_write(uint32_t v, uint8_t nbits);

    // read_write_spi() есть НЕ во всех UHD — делаем мягкую компиляцию:
    template <typename IFACE>
    static auto _rw_spi(IFACE& iface,
                        dboard_iface::unit_t unit,
                        spi_config_t::spi_edge_t edge,
                        uint32_t v,
                        uint8_t nbits,
                        int)
        -> decltype(iface.read_write_spi(unit, edge, v, nbits))
    {
        return iface.read_write_spi(unit, edge, v, nbits);
    }

    template <typename IFACE>
    static uint32_t _rw_spi(IFACE& iface,
                            dboard_iface::unit_t unit,
                            spi_config_t::spi_edge_t edge,
                            uint32_t v,
                            uint8_t nbits,
                            long)
    {
        // fallback: только write
        iface.write_spi(unit, edge, v, nbits);
        return 0;
    }

    uint32_t _spi_readwrite(uint32_t v, uint8_t nbits);

    // ---- LTC5594 ----
    void _ltc5594_write_reg(uint8_t addr, uint8_t data);
    uint8_t _ltc5594_read_reg(uint8_t addr);
    void _log_ltc5594_chip_id();
};

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_RX_HPP
