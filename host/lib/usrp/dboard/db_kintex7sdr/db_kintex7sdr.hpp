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

#include <array>
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

    enum gpio_field_id : uint8_t {
        GPIO_SPI_ADDR   = 0,
        GPIO_CPLD_RST_N = 1,
		RX_LO_LOCKED	= 2,
		RX_EN			= 3,
		TPS_EN			= 4
    };

    struct gpio_field_info_t {
    	gpio_field_id id;
    	uhd::usrp::dboard_iface::unit_t unit;
    	uint32_t offset;
    	uint32_t mask;
    	uint8_t  width;
    	enum { fpga_OUTPUT, fpga_INPUT } direction;
    	bool is_atr_controlled;
    	uint32_t atr_idle;
    	uint32_t atr_tx;
    	uint32_t atr_rx;
    	uint32_t atr_full_duplex;
    };

private:
    static const std::array<gpio_field_info_t, 5> gpio_field_info;

    struct gpio_reg_cache {
        bool dirty;
        uint32_t value;
        uint32_t mask;
        uint32_t ddr;
        uint32_t atr_mask;
        uint32_t atr_idle;
        uint32_t atr_tx;
        uint32_t atr_rx;
        uint32_t atr_full_duplex;
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

    sensor_value_t _get_locked(const std::string& pll_name);

    struct ltc6948_pll_config {
        uint8_t rd;
        uint8_t od;
        uint16_t nd;
        uint32_t num;
        double fpfd_hz;
        double fvco_hz;
        double actual_freq_hz;
        double error_hz;
    };

    void _ltc6948_init();
    uint8_t _ltc6948_read_reg(uint8_t addr);
    void _ltc6948_write_reg(uint8_t addr, uint8_t value, bool force = false);
    void _ltc6948_update_bits(uint8_t addr, uint8_t mask, uint8_t value);
    uint8_t _ltc6948_read_part_code();
    bool _ltc6948_resolve_pll(double target_freq, ltc6948_pll_config& cfg);
    void _ltc6948_apply_pll_config(const ltc6948_pll_config& cfg);

private:
    uhd::usrp::dboard_iface::sptr _iface;
    uhd::spi_config_t _spi_cfg;

    std::mutex _spi_mutex;
    std::mutex _cpld_mutex;

    std::map<gpio_field_id, gpio_field_info_t> _gpio_map;
    gpio_reg_cache _rx_gpio = {false, 0, 0, 0, 0, 0, 0, 0, 0};
    std::map<uint8_t, cpld_cache_entry> _cpld_cache;

    // кэш текущего SPI destination (чтобы не дёргать GPIO каждый раз)
    bool _spi_dest_valid{false};
    uint32_t _spi_dest3{0};

    // публичный для чиповых функций: lock + route + xfer
    uint32_t _spi_xfer_to(uint32_t dest3, uint32_t word, size_t nbits);

    std::array<uint8_t, ltc6948::NUM_REGS> _ltc6948_regs{};
    bool _ltc6948_initialized{false};
    uint8_t _ltc6948_part_code{0};
    double _ltc6948_vco_min_hz{2240e6};
    double _ltc6948_vco_max_hz{6390e6};

    double _rx_freq;
    double _rx_gain;

    const double _PFD_freq = 50e6;
    const double _REF_freq = ( _PFD_freq * 2.0 );

    bool _rxlo_locked = false;
};

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_HPP
