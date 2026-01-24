#include "db_kintex7sdr.hpp"

#include <uhd/usrp/dboard_manager.hpp>
#include <uhd/utils/static.hpp>

#include <uhd/types/sensors.hpp>

#include <boost/format.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <limits>
#include <sstream>
#include <thread>
#include <vector>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

/***********************************************************************
 * UBX Constants
 **********************************************************************/
constexpr double fMHz = (1000000.0);

// ------------------------------------------------------------------
// Дефолты (замени под свой тракт/плату)
// ------------------------------------------------------------------
static const uhd::freq_range_t KINTEX7SDR_RX_FREQ_RANGE(300e6, 2.2e9);
static const uhd::freq_range_t KINTEX7SDR_RX_BW_RANGE(100e6, 100e6);
static const uhd::gain_range_t KINTEX7SDR_RX_GAIN_RANGE(0.0, 31.5, 0.5);
static const std::vector<std::string> KINTEX7SDR_RX_ANTENNAS{"RX1"};

// На большинстве dboard-дизайнов "ничего не выбрано" для 3-битного SPI_ADDR = 0b111.
// Если в твоём CPLD другое соглашение — поменяй здесь.
//static const uint32_t SPI_DEST_NONE_3B = 0x7u;
//static const uint32_t SPI_DEST_LTC5594 = 0x2;

enum spi_dest_t {
	SPI_DEST_CPLD = 0x0, // 0x00: TXLO1, the main TXLO from 400MHz to 6000MHz
	SPI_DEST_LTC6948 = 0x1, // 0x01: TXLO2, the low band mixer TXLO 10MHz to 400MHz
	SPI_DEST_LTC5594 = 0x2, // 0x02: RXLO1, the main RXLO from 400MHz to 6000MHz
	SPI_DEST_AD7922 = 0x3, // 0x03: RXLO2, the low band mixer RXLO 10MHz to 400MHz
	SPI_DEST_AD7922_2  = 0x4, // 0x04: CPLD SPI Register
	SPI_DEST_NONE_3B = 0x7u
};

/*static const std::array<db_kintex7sdr_rx::gain_profile, 6> KINTEX7SDR_GAIN_TABLE{{
    {0.0,  0b00, 0x00},
    {6.0,  0b01, 0x10},
    {12.0, 0b10, 0x20},
    {18.0, 0b11, 0x30},
    {24.0, 0b11, 0x40},
    {30.0, 0b11, 0x50},
}};*/

const std::array<db_kintex7sdr_rx::gpio_field_info_t, 5>
db_kintex7sdr_rx::gpio_field_info = {{
	//Field         										Unit			Offset	Mask	Width   					Direction						ATR    IDLE,TX,RX,FDX
	{db_kintex7sdr_rx::GPIO_SPI_ADDR,		uhd::usrp::dboard_iface::UNIT_RX,	0,	0x7u << 0,	3,	db_kintex7sdr_rx::gpio_field_info_t::fpga_OUTPUT,	false,	0,	0,	0,	0	},
    {db_kintex7sdr_rx::GPIO_CPLD_RST_N,		uhd::usrp::dboard_iface::UNIT_RX,	3,	0x1u << 3,	1,	db_kintex7sdr_rx::gpio_field_info_t::fpga_OUTPUT,	false,	0,	0,	0,	0	},
	{db_kintex7sdr_rx::RX_LO_LOCKED,		uhd::usrp::dboard_iface::UNIT_RX,	4,	0x1u << 4,	1,	db_kintex7sdr_rx::gpio_field_info_t::fpga_INPUT,	false,	0,	0,	0,	0	},
	{db_kintex7sdr_rx::RX_EN,				uhd::usrp::dboard_iface::UNIT_RX,	5,	0x1u << 5,	1,	db_kintex7sdr_rx::gpio_field_info_t::fpga_OUTPUT,	true,	0,	0,	1,	0	},
	{db_kintex7sdr_rx::TPS_EN,				uhd::usrp::dboard_iface::UNIT_RX,	6,	0x1u << 6,	1,	db_kintex7sdr_rx::gpio_field_info_t::fpga_OUTPUT,	false,	0,	0,	0,	0	}
}};

// ============================================================================
// db_kintex7sdr_rx
// ============================================================================

db_kintex7sdr_rx::db_kintex7sdr_rx(dboard_base::ctor_args_t args)
    : uhd::usrp::rx_dboard_base(args)
    , _iface(get_iface())
    , _spi_cfg(uhd::spi_config_t::EDGE_RISE)
    , _rx_gpio{false, 0, 0, 0, 0, 0, 0, 0, 0}
    , _rx_freq(0.0)
    , _rx_gain(0.0)
{
    // 1) Описываем GPIO-поля и собираем DDR
    _init_gpio_map();

    // Set direction of GPIO pins (1 is input to UBX, 0 is output)
    _iface->set_gpio_ddr(dboard_iface::UNIT_RX, _rx_gpio.ddr);

    // 3) Безопасные дефолты
    _set_gpio_field(GPIO_SPI_ADDR, SPI_DEST_NONE_3B);
    _set_gpio_field(GPIO_CPLD_RST_N, 0);
    _flush_gpio();

    _set_gpio_field(RX_EN, 0);
    _flush_gpio();

    // Configure ATR
    _iface->set_atr_reg(
        dboard_iface::UNIT_RX, gpio_atr::ATR_REG_IDLE, _rx_gpio.atr_idle);
    _iface->set_atr_reg(
        dboard_iface::UNIT_RX, gpio_atr::ATR_REG_RX_ONLY, _rx_gpio.atr_rx);

    // Engage ATR control (1 is ATR control, 0 is manual control)
    _iface->set_pin_ctrl(dboard_iface::UNIT_RX, _rx_gpio.atr_mask);

    // bring CPLD out of reset
    std::this_thread::sleep_for(
        std::chrono::milliseconds(20)); // hold CPLD reset for minimum of 20 ms

    // 4) CPLD reset sequence
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    _set_gpio_field(GPIO_CPLD_RST_N, 1);
    _flush_gpio();


    _spi_xfer_to(SPI_DEST_LTC5594,ltc5594::make_word_wr(ltc5594::REG_BCTL,0x08),16);

    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    uint16_t rx = _spi_xfer_to(SPI_DEST_LTC5594,ltc5594::make_word_rd(ltc5594::REG_CHIPID), 16);

    std::ostringstream oss;
    oss << "LTC5594 CHIPID = 0x" << std::hex << std::uppercase
    		<< std::setw(2) << std::setfill('0') << unsigned(ltc5594::rx_data_byte(rx));
    UHD_LOG_INFO("DB_KINTEX7SDR_RX", oss.str());

    _spi_xfer_to(SPI_DEST_LTC5594,ltc5594::make_word_wr(ltc5594::REG_BCTL, uint8_t( ltc5594::bctl::BIT_EAMP | ltc5594::bctl::BIT_EDEM  \
    		| ltc5594::bctl::BIT_EDC ) ), 16);
    rx = _spi_xfer_to(SPI_DEST_LTC5594,ltc5594::make_word_rd(ltc5594::REG_BCTL), 16);

    oss.str("");
    oss << "LTC5594 REG_BCTL = 0x" << std::hex << std::uppercase
    		<< std::setw(2) << std::setfill('0') << unsigned(ltc5594::rx_data_byte(rx));
    UHD_LOG_INFO("DB_KINTEX7SDR_RX", oss.str());

    _ltc6948_init();

    //Регистрируем UHD properties
    {
        using namespace std::placeholders;

        get_rx_subtree()->create<std::string>("name").set("DB_KINTEX7SDR RX");

        get_rx_subtree()
            ->create<double>("freq/value")
            .set_coercer(std::bind(&db_kintex7sdr_rx::set_rx_frequency, this, _1))
            .set(KINTEX7SDR_RX_FREQ_RANGE.start());
        get_rx_subtree()->create<uhd::meta_range_t>("freq/range").set(KINTEX7SDR_RX_FREQ_RANGE);

        get_rx_subtree()
            ->create<double>("gains/PGA0/value")
            .set_coercer(std::bind(&db_kintex7sdr_rx::set_rx_gain, this, _1))
            .set(0.0);
        get_rx_subtree()->create<uhd::meta_range_t>("gains/PGA0/range").set(KINTEX7SDR_RX_GAIN_RANGE);

        get_rx_subtree()
            ->create<std::string>("antenna/value")
            .set(KINTEX7SDR_RX_ANTENNAS.at(0));
        get_rx_subtree()
            ->create<std::vector<std::string>>("antenna/options")
            .set(KINTEX7SDR_RX_ANTENNAS);


        get_rx_subtree()->create<std::string>("connection").set("IQ");
        get_rx_subtree()->create<bool>("enabled").set(true);

        get_rx_subtree()->create<bool>("use_lo_offset").set(false);

        get_rx_subtree()
            ->create<sensor_value_t>("sensors/lo_locked")
            .set_publisher(std::bind(&db_kintex7sdr_rx::_get_locked, this, "RXLO"));

        get_rx_subtree()->create<double>("bandwidth/value").set(KINTEX7SDR_RX_BW_RANGE.start());
        get_rx_subtree()->create<uhd::meta_range_t>("bandwidth/range").set(KINTEX7SDR_RX_BW_RANGE);
    }

    try {
    	_iface->set_clock_rate(dboard_iface::UNIT_RX, _REF_freq);
    } catch (const uhd::not_implemented_error&) {
    	UHD_LOG_WARNING(
    			"KINTEX7SDR_RX", "Unable to set dboard clock rate - phase will vary");
    }

    double clock_rate = _iface->get_clock_rate(dboard_iface::UNIT_RX);
    oss.str("");
    oss << "UNIT_RX REF clock frequency: "  << std::fixed << std::setprecision(1) << double(clock_rate/fMHz) << "MHz";
    UHD_LOG_INFO("DB_KINTEX7SDR_RX", oss.str());

    _iface->set_clock_enabled(dboard_iface::UNIT_RX, true);
    //_iface->set_clock_enabled(dboard_iface::UNIT_TX, true);


    _get_locked("RXLO");

	UHD_LOG_WARNING(
			"KINTEX7SDR_RX", "I'm running");
};

db_kintex7sdr_rx::~db_kintex7sdr_rx(void)
{
	UHD_SAFE_CALL(
			// Engage ATR control (1 is ATR control, 0 is manual control)
			_iface->set_pin_ctrl(dboard_iface::UNIT_RX, uint32_t(0));

	// Возвращаем линии в безопасное состояние
	_set_gpio_field(GPIO_SPI_ADDR, SPI_DEST_NONE_3B);
	_set_gpio_field(GPIO_CPLD_RST_N, 0);
	_set_gpio_field(RX_EN, 0);
	_flush_gpio();

	UHD_LOG_WARNING(
			"KINTEX7SDR_RX", "I'm toast i'm done");
	)
}

// ============================================================================
// GPIO map + helpers
// ============================================================================

void db_kintex7sdr_rx::_init_gpio_map()
{
    for (const auto& info : gpio_field_info) {
        _gpio_map[info.id] = info;
        if (info.direction == gpio_field_info_t::fpga_OUTPUT) {
            _rx_gpio.ddr |= info.mask;
        }
        if (info.is_atr_controlled) {
            _rx_gpio.atr_mask |= info.mask;
            _rx_gpio.atr_idle |= (info.atr_idle << info.offset) & info.mask;
            _rx_gpio.atr_tx |= (info.atr_tx << info.offset) & info.mask;
            _rx_gpio.atr_rx |= (info.atr_rx << info.offset) & info.mask;
            _rx_gpio.atr_full_duplex |= (info.atr_full_duplex << info.offset) & info.mask;
        }
    }
}

void db_kintex7sdr_rx::_set_gpio_field(gpio_field_id id, uint32_t v)
{
    auto it = _gpio_map.find(id);
    if (it == _gpio_map.end()) {
        return;
    }

    const auto& f = it->second;

    // защита от "вылета" за ширину поля
    if (f.width < 32) {
        const uint32_t maxv = (1u << f.width) - 1u;
        v &= maxv;
    }

    uint32_t newv = _rx_gpio.value;
    newv &= ~f.mask;
    newv |= (v << f.offset) & f.mask;

    if (newv != _rx_gpio.value) {
        _rx_gpio.value = newv;
        _rx_gpio.mask |= f.mask;
        _rx_gpio.dirty = true;
    }
}

uint32_t db_kintex7sdr_rx::_get_gpio_field(gpio_field_id id)
{
    auto it = _gpio_map.find(id);
    if (it == _gpio_map.end()) {
        return 0;
    }

    const auto& f = it->second;

    // Если FPGA drives (т.е. мы сами выставляем) — читаем из кеша.
    // Важно: если позже включишь ATR, то кеш может не отражать реальное состояние
    // во время TX/RX переключений, тогда лучше читать read_gpio().
    if (f.direction == gpio_field_info_t::fpga_OUTPUT) {
        return (_rx_gpio.value & f.mask) >> f.offset;
    }

    const uint32_t v = _iface->read_gpio(f.unit);
    return (v & f.mask) >> f.offset;
}

void db_kintex7sdr_rx::_flush_gpio()
{
    if (_rx_gpio.dirty) {
        _iface->set_gpio_out(dboard_iface::UNIT_RX, _rx_gpio.value, _rx_gpio.mask);
        _rx_gpio.dirty = false;
        _rx_gpio.mask  = 0;
    }
}

// ============================================================================
// SPI helpers (ключевая правка относительно твоей текущей версии)
//   ВАЖНО: route (SPI_ADDR) и сама SPI транзакция должны быть под ОДНИМ mutex,
//   иначе два потока могут перемешать dest и послать слово "не туда".
//   Именно так сделано в образце (ROUTE_SPI + WRITE_SPI внутри lock).
// ============================================================================


uint32_t db_kintex7sdr_rx::_spi_xfer_to(uint32_t dest3, uint32_t word, size_t nbits)
{
    std::lock_guard<std::mutex> lock(_spi_mutex);

    dest3 &= 0x7u;
    if (!_spi_dest_valid || _spi_dest3 != dest3) {
        _set_gpio_field(GPIO_SPI_ADDR, dest3);
        _flush_gpio();
        _spi_dest3 = dest3;
        _spi_dest_valid = true;
    }

    return _iface->read_write_spi(dboard_iface::UNIT_RX, _spi_cfg, word, nbits);
}


// ============================================================================
// Chip-level helpers
// ============================================================================

/*void db_kintex7sdr_rx::_cpld_wr(uint8_t reg7, uint32_t data24)
{
    const uint32_t data = data24 & 0x00FFFFFFu;
    std::lock_guard<std::mutex> lock(_cpld_mutex);
    auto& entry = _cpld_cache[reg7];
    if (entry.valid && entry.value == data) {
        return;
    }

    (void)_spi_xfer_to(uint32_t(cpld::SPI_DEST_CPLD), cpld::make_frame_wr(reg7, data), 32);
    entry.value = data;
    entry.valid = true;
}

uint32_t db_kintex7sdr_rx::_cpld_rd(uint8_t reg7)
{
    const uint32_t rx = _spi_xfer_to(uint32_t(cpld::SPI_DEST_CPLD), cpld::make_frame_rd(reg7), 32);
    const uint32_t data = rx & 0x00FFFFFFu;
    std::lock_guard<std::mutex> lock(_cpld_mutex);
    auto& entry = _cpld_cache[reg7];
    entry.value = data;
    entry.valid = true;
    return data;
}

void db_kintex7sdr_rx::_cpld_update_bits(uint8_t reg7, uint32_t mask, uint32_t value)
{
    uint32_t base = 0;
    bool cache_valid = false;
    {
        std::lock_guard<std::mutex> lock(_cpld_mutex);
        auto it = _cpld_cache.find(reg7);
        if (it != _cpld_cache.end() && it->second.valid) {
            base = it->second.value;
            cache_valid = true;
        }
    }

    if (!cache_valid) {
        base = _cpld_rd(reg7);
    }

    const uint32_t next = (base & ~mask) | (value & mask);
    _cpld_wr(reg7, next);
}*/

namespace {
constexpr std::array<double, 4> LTC6948_VCO_MIN_HZ = {2240e6, 3080e6, 3840e6, 4200e6};
constexpr std::array<double, 4> LTC6948_VCO_MAX_HZ = {3740e6, 4910e6, 5790e6, 6390e6};
constexpr uint8_t LTC6948_PART_MIN = 1;
constexpr uint8_t LTC6948_PART_MAX = 4;
constexpr uint8_t LTC6948_PART_MASK = 0x0Fu;
constexpr uint8_t LTC6948_OD_MASK = ltc6948::REGB_OD_MASK;
constexpr uint8_t LTC6948_CP_LINEAR_MASK = static_cast<uint8_t>(
    ltc6948::REGD_CPCHI | ltc6948::REGD_CPCLO | ltc6948::REGD_CPMID | ltc6948::REGD_CPRST
    | ltc6948::REGD_CPUP | ltc6948::REGD_CPDN);
} // namespace

uint8_t db_kintex7sdr_rx::_ltc6948_read_reg(uint8_t addr)
{
    const auto rx = static_cast<uint16_t>(
        _spi_xfer_to(SPI_DEST_LTC6948, ltc6948::make_word_rd(addr), 16));
    return ltc6948::rx_data_byte(rx);
}

void db_kintex7sdr_rx::_ltc6948_write_reg(uint8_t addr, uint8_t value, bool force)
{
    if (addr >= ltc6948::NUM_REGS) {
        return;
    }

    if (!force && _ltc6948_initialized && _ltc6948_regs[addr] == value) {
        return;
    }

    _spi_xfer_to(SPI_DEST_LTC6948, ltc6948::make_word_wr(addr, value), 16);
    _ltc6948_regs[addr] = value;
}

void db_kintex7sdr_rx::_ltc6948_update_bits(uint8_t addr, uint8_t mask, uint8_t value)
{
    if (addr >= ltc6948::NUM_REGS) {
        return;
    }

    const uint8_t base = _ltc6948_initialized ? _ltc6948_regs[addr]
                                               : ltc6948::DEFAULT_REGS[addr];
    const uint8_t next = static_cast<uint8_t>((base & ~mask) | (value & mask));
    _ltc6948_write_reg(addr, next);
}

uint8_t db_kintex7sdr_rx::_ltc6948_read_part_code()
{
    const uint8_t reg_e = _ltc6948_read_reg(ltc6948::REGE);
    const uint8_t part_code = reg_e & LTC6948_PART_MASK;
    _ltc6948_part_code = part_code;

    if (part_code >= LTC6948_PART_MIN && part_code <= LTC6948_PART_MAX) {
        const std::size_t idx = part_code - 1;
        _ltc6948_vco_min_hz = LTC6948_VCO_MIN_HZ[idx];
        _ltc6948_vco_max_hz = LTC6948_VCO_MAX_HZ[idx];
    } else {
        _ltc6948_vco_min_hz = LTC6948_VCO_MIN_HZ.front();
        _ltc6948_vco_max_hz = LTC6948_VCO_MAX_HZ.back();
    }

    UHD_LOG_INFO("DB_KINTEX7SDR_RX",
        (boost::format("LTC6948 part code: 0x%1$X, VCO range: %2$.0f..%3$.0f MHz")
            % unsigned(part_code)
            % (_ltc6948_vco_min_hz / fMHz)
            % (_ltc6948_vco_max_hz / fMHz))
            .str());

    return part_code;
}

void db_kintex7sdr_rx::_ltc6948_init()
{
    if (_ltc6948_initialized) {
        return;
    }

    _ltc6948_regs = ltc6948::DEFAULT_REGS;

    // Board-specific defaults:
    //  * unmute output but keep mute-during-calibration enabled
    //  * keep BD default, use LDOV=2 (как было 0x46)
    //  * enable CP clamps, clear CP tristate
    _ltc6948_regs[ltc6948::REG2] = static_cast<uint8_t>(
        (_ltc6948_regs[ltc6948::REG2]
            & static_cast<uint8_t>(~(ltc6948::REG2_PDALL | ltc6948::REG2_PDPLL
                | ltc6948::REG2_PDVCO | ltc6948::REG2_PDOUT | ltc6948::REG2_PDFN
                | ltc6948::REG2_OMUTE | ltc6948::REG2_POR)))
        | ltc6948::REG2_MTCAL);

    _ltc6948_regs[ltc6948::REG4] = static_cast<uint8_t>(
        (_ltc6948_regs[ltc6948::REG4] & static_cast<uint8_t>(~ltc6948::REG4_LDOV_MASK)) | 0x02u);
    _ltc6948_regs[ltc6948::REG4] = static_cast<uint8_t>(_ltc6948_regs[ltc6948::REG4]
        & static_cast<uint8_t>(~ltc6948::REG4_CPLE));

    _ltc6948_regs[ltc6948::REGD] = static_cast<uint8_t>(ltc6948::REGD_CPCHI | ltc6948::REGD_CPCLO);

    // Program non-frequency-dependent registers first (avoid extra autocal runs).
    for (const uint8_t addr : {ltc6948::REG1, ltc6948::REG2, ltc6948::REG3, ltc6948::REG4,
             ltc6948::REG5, ltc6948::REGB, ltc6948::REGC, ltc6948::REGD}) {
        _ltc6948_write_reg(addr, _ltc6948_regs[addr], true);
    }

    _ltc6948_read_part_code();
    _ltc6948_initialized = true;
}

bool db_kintex7sdr_rx::_ltc6948_resolve_pll(double target_freq, ltc6948_pll_config& cfg)
{
    _ltc6948_init();

    if (_ltc6948_part_code == 0) {
        _ltc6948_read_part_code();
    }

    const double min_freq = _ltc6948_vco_min_hz / ltc6948::OD_MAX;
    const double max_freq = _ltc6948_vco_max_hz / ltc6948::OD_MIN;
    double target = target_freq;
    if (target < min_freq) {
        target = min_freq;
    } else if (target > max_freq) {
        target = max_freq;
    }
    if (target != target_freq) {
        UHD_LOG_WARNING("DB_KINTEX7SDR_RX",
            (boost::format("Requested %.3f MHz clipped to %.3f MHz by VCO range")
                % (target_freq / fMHz)
                % (target / fMHz))
                .str());
    }

    constexpr double k_modulus = static_cast<double>(ltc6948::MODULUS);
    double best_error = std::numeric_limits<double>::infinity();
    bool found = false;

    for (uint8_t od = ltc6948::OD_MIN; od <= ltc6948::OD_MAX; od++) {
        const double fvco_target = target * od;
        if (fvco_target < _ltc6948_vco_min_hz || fvco_target > _ltc6948_vco_max_hz) {
            continue;
        }

        for (uint8_t rd = ltc6948::RD_MIN; rd <= ltc6948::RD_MAX; rd++) {
            const double fpfd = _REF_freq / rd;
            const double ratio = fvco_target / fpfd;
            auto nd = static_cast<uint16_t>(std::floor(ratio));
            if (nd < ltc6948::ND_MIN || nd > ltc6948::ND_MAX) {
                continue;
            }

            double frac = ratio - nd;
            uint32_t num = static_cast<uint32_t>(std::llround(frac * k_modulus));
            if (num >= ltc6948::MODULUS) {
                num = 0;
                nd = static_cast<uint16_t>(nd + 1);
            }
            if (nd < ltc6948::ND_MIN || nd > ltc6948::ND_MAX) {
                continue;
            }

            frac = static_cast<double>(num) / k_modulus;
            const double fvco_actual = fpfd * (static_cast<double>(nd) + frac);
            const double actual = fvco_actual / od;
            const double error = std::abs(actual - target);

            if (!found || error < best_error
                || (error == best_error && (rd < cfg.rd || (rd == cfg.rd && od < cfg.od)))) {
                cfg = {rd, od, nd, num, fpfd, fvco_actual, actual, error};
                best_error = error;
                found = true;
            }
        }
    }

    if (!found) {
        UHD_LOG_WARNING("DB_KINTEX7SDR_RX",
            (boost::format("Unable to resolve LTC6948 PLL for %.3f MHz") % (target / fMHz)).str());
    }

    return found;
}

void db_kintex7sdr_rx::_ltc6948_apply_pll_config(const ltc6948_pll_config& cfg)
{
    _ltc6948_init();

    const uint32_t num = std::min<uint32_t>(cfg.num, ltc6948::NUM_MAX);

    uint8_t reg3 = static_cast<uint8_t>(_ltc6948_regs[ltc6948::REG3] & ~ltc6948::REG3_INTN);
    if (num == 0) {
        reg3 = static_cast<uint8_t>(reg3 | ltc6948::REG3_INTN);
    }

    const uint8_t reg6 = static_cast<uint8_t>(((cfg.rd << ltc6948::REG6_RD_SHIFT) & ltc6948::REG6_RD_MASK)
        | ((cfg.nd >> 8) & ltc6948::REG6_ND_MSB_MASK));
    const uint8_t reg7 = static_cast<uint8_t>(cfg.nd & 0xFFu);
    const uint8_t reg8 = static_cast<uint8_t>((num >> 12) & ltc6948::REG8_NUM_MSB_MASK);
    const uint8_t reg9 = static_cast<uint8_t>((num >> 4) & 0xFFu);
    const uint8_t rega = static_cast<uint8_t>((num & 0x0Fu) << ltc6948::REGA_NUM_LSB_SHIFT);
    const uint8_t regb = static_cast<uint8_t>((_ltc6948_regs[ltc6948::REGB] & ~LTC6948_OD_MASK)
        | (cfg.od & LTC6948_OD_MASK));

    _ltc6948_write_reg(ltc6948::REG3, reg3);
    _ltc6948_write_reg(ltc6948::REGB, regb);

    // Writing REG6..REGA triggers AUTOCAL when REG3[AUTOCAL]=1 (default).
    _ltc6948_write_reg(ltc6948::REG6, reg6, true);
    _ltc6948_write_reg(ltc6948::REG7, reg7, true);
    _ltc6948_write_reg(ltc6948::REG8, reg8, true);
    _ltc6948_write_reg(ltc6948::REG9, reg9, true);
    _ltc6948_write_reg(ltc6948::REGA, rega, true);

    // Keep CP clamps enabled and CP out of tristate.
    _ltc6948_update_bits(ltc6948::REGD, LTC6948_CP_LINEAR_MASK,
        static_cast<uint8_t>(ltc6948::REGD_CPCHI | ltc6948::REGD_CPCLO));

    UHD_LOG_INFO("DB_KINTEX7SDR_RX",
        (boost::format("LTC6948 set: f=%.3f MHz (err=%.3f Hz), RD=%u ND=%u NUM=%u OD=%u")
            % (cfg.actual_freq_hz / fMHz)
            % cfg.error_hz
            % unsigned(cfg.rd)
            % unsigned(cfg.nd)
            % unsigned(num)
            % unsigned(cfg.od))
            .str());
}

// ============================================================================
// UHD coercers
// ============================================================================

double db_kintex7sdr_rx::set_rx_frequency(double freq)
{
    constexpr double k_step_hz = 1e3;

    freq = KINTEX7SDR_RX_FREQ_RANGE.clip(freq);
    freq = std::round(freq / k_step_hz) * k_step_hz;

    ltc6948_pll_config cfg{};
    if (!_ltc6948_resolve_pll(freq, cfg)) {
        return _rx_freq;
    }

    _ltc6948_apply_pll_config(cfg);
    _rx_freq = cfg.actual_freq_hz;
    return _rx_freq;
}

/***********************************************************************
 * Board Control Handling
 **********************************************************************/
sensor_value_t db_kintex7sdr_rx::_get_locked([[maybe_unused]] const std::string& pll_name = "RXLO")
{
	std::lock_guard<std::mutex> lock(_spi_mutex);

	_rxlo_locked = (_get_gpio_field(RX_LO_LOCKED) != 0);

	UHD_LOG_INFO("DB_KINTEX7SDR_RX",
	    std::string("LO lock status: [") + (_rxlo_locked ? "LOCKED" : "----") + "]");

	return sensor_value_t("RXLO", _rxlo_locked, "locked", "unlocked");
}

double db_kintex7sdr_rx::set_rx_gain(double gain)
{
    // ВНИМАНИЕ: соответствие битов REG_CTRL и карт усиления должно
    // совпадать с regmap_core в CPLD.
    gain = KINTEX7SDR_RX_GAIN_RANGE.clip(gain);

/*    const gain_profile* profile = &KINTEX7SDR_GAIN_TABLE.front();
    for (const auto& entry : KINTEX7SDR_GAIN_TABLE) {
        if (gain >= entry.gain_db) {
            profile = &entry;
        }
    }

    uint32_t att1_value = 0;
    if (profile->att1_code & 0x2) {
        att1_value |= cpld::CTRL_ATT1_C1;
    }
    if (profile->att1_code & 0x1) {
        att1_value |= cpld::CTRL_ATT1_C2;
    }

    _cpld_update_bits(cpld::REG_CTRL, cpld::CTRL_ATT1_MASK, att1_value);
    _cpld_wr(cpld::REG_ATT2_CODE, profile->att2_code);*/

    _rx_gain = gain;
    return _rx_gain;
}

/***********************************************************************
 * Registration
 **********************************************************************/
static uhd::usrp::dboard_base::sptr make_db_kintex7sdr_rx(uhd::usrp::dboard_base::ctor_args_t args)
{
    return uhd::usrp::dboard_base::sptr(new db_kintex7sdr_rx(args));
}

UHD_STATIC_BLOCK(register_db_kintex7sdr_rx)
{
    // Пытаемся сначала overload с одним RX_ID, если его нет — используем (rx_id, tx_none)
	dboard_manager::register_dboard(DB_KINTEX7SDR_RX_ID, &make_db_kintex7sdr_rx, "DB_KINTEX7SDR_RX");
}

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr
