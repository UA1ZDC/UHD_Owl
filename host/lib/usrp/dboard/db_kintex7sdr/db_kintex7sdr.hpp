// ==============================
// File: db_kintex7sdr_rx.hpp
// ==============================
#ifndef DB_KINTEX7SDR_RX_HPP
#define DB_KINTEX7SDR_RX_HPP

#include "cpld_regmap.hpp"
#include "ltc5594_regmap.hpp"
#include "ltc6948_regmap.hpp"

#include <uhd/types/ranges.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/usrp/dboard_base.hpp>

#include <cstdint>
#include <mutex>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

class db_kintex7sdr_rx : public rx_dboard_base
{
public:
    explicit db_kintex7sdr_rx(dboard_iface::sptr iface);
    ~db_kintex7sdr_rx() override = default;

private:
    // 3-bit SPI_ADDR routes (GPIO[2:0]); must match CPLD logic: CPLD_DEST == 0.
    enum class spi_dest_t : uint8_t {
        CPLD     = 0x0,
        LTC5594  = 0x1,
        LTC6948  = 0x2,
        RESERVED3 = 0x3,
        RESERVED4 = 0x4,
        RESERVED5 = 0x5,
        RESERVED6 = 0x6,
        RESERVED7 = 0x7,
    };

    enum class gpio_field_id : uint8_t {
        SPI_ADDR,
        CPLD_RST_N,
    };

    struct gpio_field_info {
        gpio_field_id id;
        dboard_iface::unit_t unit;
        uint8_t offset;
        uint32_t mask;
        uint8_t width;
        bool ddr_one_means_output_to_db; // true for controllable signals (input to DB from FPGA)
    };

    struct gpio_reg_state {
        uint32_t out = 0;
        uint32_t ddr = 0;
    };

    void _init_gpio();
    void _write_gpio();
    void _set_gpio_field(gpio_field_id id, uint32_t value);

    void _route_spi(spi_dest_t dest);

    void _spi_write(uint32_t v, uint8_t nbits);
    uint32_t _spi_xfer(uint32_t v, uint8_t nbits);

    // High-level controls
    double _set_rx_freq(double freq_hz);
    double _set_rx_gain(double att_db);

    uhd::sensor_value_t _get_lo_locked();

    void _cpld_reset_pulse();
    void _cpld_apply_ctrl0(uint32_t ctrl0);
    void _set_att1_code(uint8_t code2b);
    void _set_att2_code(uint8_t code7b);

private:
    dboard_iface::sptr _iface;
    std::mutex _spi_mutex;

    gpio_reg_state _gpio;
    spi_dest_t _cur_dest = spi_dest_t::CPLD;

    // Shadowed CTRL0 to do clean RMW
    uint32_t _ctrl0_shadow = 0;

    // Regmap helpers (callbacks route + xfer)
    cpld::cpld_iface _cpld;
    ltc5594::ltc5594_iface _ltc5594;
    ltc6948::ltc6948_iface _ltc6948;

    // Ranges
    uhd::meta_range_t _rx_freq_range;
    uhd::meta_range_t _rx_gain_range;
};

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // DB_KINTEX7SDR_RX_HPP
