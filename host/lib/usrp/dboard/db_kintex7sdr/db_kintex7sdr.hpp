#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_HPP

#include "db_kintex7sdr_ids.hpp"
#include "cpld_regmap.hpp"
#include "ltc5594_regmap.hpp"
#include "ltc6948_regmap.hpp"

#include <uhd/usrp/dboard_base.hpp>
#include <uhd/usrp/dboard_iface.hpp>
#include <uhd/types/ranges.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/types/serial.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/safe_call.hpp>

#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <type_traits>
#include <utility>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

class db_kintex7sdr_rx : public uhd::usrp::rx_dboard_base
{
public:
    explicit db_kintex7sdr_rx(uhd::usrp::dboard_base::ctor_args_t args);
    ~db_kintex7sdr_rx(void) override;

    // UHD property coercers (пока только кэш + TODO на реальное железо)
    double set_rx_frequency(double freq);
    double set_rx_gain(double gain);

/*
    struct gain_profile {
        double gain_db;
        uint8_t att1_code;
        uint8_t att2_code;
    };
*/

private:
    // GPIO fields (минимум нужного сейчас)
    enum gpio_field_id : uint8_t {
        GPIO_SPI_ADDR   = 0,
        GPIO_CPLD_RST_N = 1,
        // при необходимости добавишь тут LOCKED/EN/etc
    };

    struct gpio_field_info {
        gpio_field_id id;
        uhd::usrp::dboard_iface::unit_t unit;
        uint32_t offset;
        uint32_t mask;
        uint8_t  width;
        bool     fpga_drives; // true => FPGA drives pin ("INPUT" со стороны платы)
    };

    struct gpio_reg_cache {
        bool dirty;
        uint32_t value;
        uint32_t mask;
        uint32_t ddr;
    };

    struct cpld_cache_entry {
        uint32_t value;
        bool valid;
    };

    // GPIO helpers
    void _init_gpio_map();
    void _set_gpio_field(gpio_field_id id, uint32_t v);
    uint32_t _get_gpio_field(gpio_field_id id);
    void _flush_gpio();

    // --- SPI helpers (по образцу: route+spi под ОДНИМ mutex) ---

private:
    uhd::usrp::dboard_iface::sptr _iface;
    uhd::spi_config_t _spi_cfg;

    std::mutex _spi_mutex;
    std::mutex _cpld_mutex;

    std::map<gpio_field_id, gpio_field_info> _gpio_map;
    gpio_reg_cache _rx_gpio;
    std::map<uint8_t, cpld_cache_entry> _cpld_cache;

    // кэш текущего SPI destination (чтобы не дёргать GPIO каждый раз)
    bool _spi_dest_valid{false};
    uint32_t _spi_dest3{0};

    double _rx_freq;
    double _rx_gain;
};

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_HPP
