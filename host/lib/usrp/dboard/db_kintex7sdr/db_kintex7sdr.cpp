 // ==============================
// File: db_kintex7sdr_rx.cpp
// ==============================
#include "db_kintex7sdr.hpp"
#include "db_kintex7sdr_ids.hpp"

#include <uhd/types/direction.hpp>
#include <uhd/types/sensors.hpp>
#include <uhd/usrp/dboard_manager.hpp>
#include <uhd/utils/log.hpp>
#include <uhd/utils/safe_call.hpp>
#include <uhd/utils/static.hpp>

#include <boost/bind.hpp>

#include <chrono>
#include <thread>

using namespace uhd;
using namespace uhd::usrp;
using namespace uhd::usrp::dboard;
using namespace uhd::usrp::dboard::db_kintex7sdr;

static const db_kintex7sdr_rx::gpio_field_info gpio_info[] = {
    // id                 unit                   off mask        width ddr(1=output to DB)
    {db_kintex7sdr_rx::gpio_field_id::SPI_ADDR,  dboard_iface::UNIT_RX, 0, 0x7u << 0, 3, true},
    {db_kintex7sdr_rx::gpio_field_id::CPLD_RST_N,dboard_iface::UNIT_RX, 3, 0x1u << 3, 1, true},
};

db_kintex7sdr_rx::db_kintex7sdr_rx(dboard_iface::sptr iface)
    : rx_dboard_base(iface)
    , _iface(iface)
    , _cpld([this](uint32_t tx32) -> uint32_t {
        _route_spi(spi_dest_t::CPLD);
        return _spi_xfer(tx32, 32);
    })
    , _ltc5594([this](uint16_t tx16) -> uint16_t {
        _route_spi(spi_dest_t::LTC5594);
        return uint16_t(_spi_xfer(uint32_t(tx16), 16) & 0xFFFFu);
    })
    , _ltc6948([this](uint16_t tx16) -> uint16_t {
        _route_spi(spi_dest_t::LTC6948);
        return uint16_t(_spi_xfer(uint32_t(tx16), 16) & 0xFFFFu);
    })
{
    UHD_SAFE_CALL(_init_gpio();)

    // CPLD reset pulse
    UHD_SAFE_CALL(_cpld_reset_pulse();)

    // Default CTRL0:
    // - TPS_EN on
    // - AUTOLATCH on (so ATT2_CODE writes latch immediately)
    _ctrl0_shadow = 0;
    _ctrl0_shadow |= cpld::CTRL0_TPS_EN;
    _ctrl0_shadow |= cpld::CTRL0_AUTOLATCH;
    _cpld_apply_ctrl0(_ctrl0_shadow);

    // LTC5594: enable SDO readback, read CHIPID
    UHD_SAFE_CALL({
        _ltc5594.enable_sdo_readback(true);
        const auto chipid = _ltc5594.read_chip_id();
        UHD_LOG_INFO("DB_KINTEX7SDR", "LTC5594 CHIPID[1:0] = " + std::to_string(chipid));
    })

    // LTC6948: power up + default freq
    UHD_SAFE_CALL({
        _ltc6948.power_up();
        (void)_set_rx_freq(1000e6); // default 1 GHz
    })

    // Property tree (RX)
    _rx_freq_range = meta_range_t(300e6, 2200e6, 1e6);
    // Treat "gain" as attenuation (dB): [0 .. 55.75] step 0.25 (ATT2 0..31.75 + coarse)
    _rx_gain_range = meta_range_t(0.0, 55.75, 0.25);

    auto rx = get_rx_subtree();
    rx->create<std::string>("name").set("db_kintex7sdr_rx");

    rx->create<meta_range_t>("freq/range").set(_rx_freq_range);
    rx->create<double>("freq/value")
        .set(1000e6)
        .set_coercer(boost::bind(&db_kintex7sdr_rx::_set_rx_freq, this, _1));

    rx->create<meta_range_t>("gains/ATT/range").set(_rx_gain_range);
    rx->create<double>("gains/ATT/value")
        .set(0.0)
        .set_coercer(boost::bind(&db_kintex7sdr_rx::_set_rx_gain, this, _1));

    rx->create<sensor_value_t>("sensors/lo_locked")
        .set_publisher(boost::bind(&db_kintex7sdr_rx::_get_lo_locked, this));
}

void db_kintex7sdr_rx::_init_gpio()
{
    // Build DDR/out defaults
    _gpio = gpio_reg_state{};
    for (const auto& info : gpio_info) {
        if (info.ddr_one_means_output_to_db) {
            _gpio.ddr |= info.mask; // UHD GPIO DDR: 1 -> drive to DB
        }
    }

    // Defaults
    _set_gpio_field(gpio_field_id::SPI_ADDR, uint32_t(spi_dest_t::CPLD));
    _set_gpio_field(gpio_field_id::CPLD_RST_N, 1);

    // Apply
    _iface->set_gpio_ddr(dboard_iface::UNIT_RX, _gpio.ddr);
    _iface->set_gpio_out(dboard_iface::UNIT_RX, _gpio.out);
}

void db_kintex7sdr_rx::_write_gpio()
{
    _iface->set_gpio_out(dboard_iface::UNIT_RX, _gpio.out);
}

void db_kintex7sdr_rx::_set_gpio_field(gpio_field_id id, uint32_t value)
{
    for (const auto& info : gpio_info) {
        if (info.id != id) continue;
        const uint32_t shift = info.offset;
        const uint32_t masked = (value << shift) & info.mask;
        _gpio.out = (_gpio.out & ~info.mask) | masked;
        return;
    }
}

void db_kintex7sdr_rx::_route_spi(spi_dest_t dest)
{
    if (_cur_dest == dest) return;
    _cur_dest = dest;

    _set_gpio_field(gpio_field_id::SPI_ADDR, uint32_t(dest));
    _write_gpio();
}

void db_kintex7sdr_rx::_spi_write(uint32_t v, uint8_t nbits)
{
    std::lock_guard<std::mutex> lock(_spi_mutex);
    _iface->write_spi(dboard_iface::UNIT_RX, spi_config_t::EDGE_RISE, v, nbits);
}

uint32_t db_kintex7sdr_rx::_spi_xfer(uint32_t v, uint8_t nbits)
{
    std::lock_guard<std::mutex> lock(_spi_mutex);
    return _iface->read_spi(dboard_iface::UNIT_RX, spi_config_t::EDGE_RISE, v, nbits);
}

void db_kintex7sdr_rx::_cpld_reset_pulse()
{
    _set_gpio_field(gpio_field_id::CPLD_RST_N, 0);
    _write_gpio();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    _set_gpio_field(gpio_field_id::CPLD_RST_N, 1);
    _write_gpio();
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
}

void db_kintex7sdr_rx::_cpld_apply_ctrl0(uint32_t ctrl0)
{
    _ctrl0_shadow = (ctrl0 & 0x7Fu); // keep only defined bits [6:0]
    _cpld.write_reg(cpld::BANK_CTRL, cpld::REG_CTRL0, _ctrl0_shadow);
}

void db_kintex7sdr_rx::_set_att1_code(uint8_t code2b)
{
    code2b &= 0x3;
    // Map code2b -> CTRL0 bits [3:2]
    uint32_t ctrl0 = _ctrl0_shadow;
    ctrl0 &= ~(cpld::CTRL0_ATT1_C1 | cpld::CTRL0_ATT1_C2);
    if (code2b & 0x1) ctrl0 |= cpld::CTRL0_ATT1_C1;
    if (code2b & 0x2) ctrl0 |= cpld::CTRL0_ATT1_C2;
    _cpld_apply_ctrl0(ctrl0);
}

void db_kintex7sdr_rx::_set_att2_code(uint8_t code7b)
{
    code7b &= 0x7F;
    // With AUTOLATCH=1 this write shifts code into PE43711 and pulses LE at frame end
    _cpld.write_reg(cpld::BANK_ATT2, cpld::REG_ATT2_CODE, uint32_t(code7b));
}

double db_kintex7sdr_rx::_set_rx_freq(double freq_hz)
{
    // Quantize to 1 MHz
    const double f = std::min(2200e6, std::max(300e6, std::round(freq_hz / 1e6) * 1e6));

    // Use dboard clock as reference
    const double fref = _iface->get_clock_rate(dboard_iface::UNIT_RX);
    const double actual = _ltc6948.set_frequency(f, fref);

    return actual;
}

double db_kintex7sdr_rx::_set_rx_gain(double att_db)
{
    // Two attenuators in the schematic:
    // - ATT1: coarse 2-bit control (CTRL0 bits 3:2). Step is board-specific; keep a simple coarse ladder.
    // - ATT2: 7-bit PE43711 via CPLD autolatch, 0.25 dB/LSB assumed (0..31.75 dB).
    //
    // You can retune these constants later without changing CPLD/SPI plumbing.
    static const double att1_steps_db[4] = {0.0, 8.0, 16.0, 24.0}; // coarse ladder (adjust to real HW)

    double req = std::max(0.0, std::min(55.75, att_db));

    // Choose best ATT1 (largest not exceeding req)
    uint8_t best_att1 = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (att1_steps_db[i] <= req) best_att1 = i;
    }
    const double att1_db = att1_steps_db[best_att1];

    // Remaining to ATT2
    double rem = req - att1_db;
    rem = std::max(0.0, std::min(31.75, rem));
    const uint8_t att2_code = uint8_t(std::lround(rem / 0.25)); // 0.25 dB/LSB
    const double att2_db = double(att2_code) * 0.25;

    _set_att1_code(best_att1);
    _set_att2_code(att2_code);

    return att1_db + att2_db;
}

sensor_value_t db_kintex7sdr_rx::_get_lo_locked()
{
    const uint32_t st = _cpld.read_reg(cpld::BANK_ID, cpld::REG_STATUS0);
    const bool locked = (st & cpld::STATUS0_STAT_LTC6948) != 0;
    return sensor_value_t("lo_locked", locked, "locked");
}

/***********************************************************************
 * Dboard registration (RX-only)
 **********************************************************************/
static dboard_base::sptr make_db_kintex7sdr_rx(dboard_base::ctor_args_t args)
{
    return dboard_base::sptr(new db_kintex7sdr_rx(args.db_iface));
}

UHD_STATIC_BLOCK(register_db_kintex7sdr_rx)
{
    // RX-only: TX id = 0x0000 (absent)
	dboard_manager::register_dboard(
	    DB_KINTEX7SDR_RX_ID, &make_db_kintex7sdr_rx, "db_kintex7sdr_rx");
}
