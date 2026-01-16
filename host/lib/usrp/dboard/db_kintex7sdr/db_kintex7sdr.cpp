#include "db_kintex7sdr.hpp"

#include <uhd/usrp/dboard_manager.hpp>
#include <uhd/utils/static.hpp>

#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <thread>
#include <vector>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

/***********************************************************************
 * UBX Constants
 **********************************************************************/
//#define fMHz (1000000.0)

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

// ============================================================================
// db_kintex7sdr_rx
// ============================================================================

db_kintex7sdr_rx::db_kintex7sdr_rx(uhd::usrp::dboard_base::ctor_args_t args)
    : uhd::usrp::rx_dboard_base(args)
    , _iface(get_iface())
    , _spi_cfg(uhd::spi_config_t::EDGE_RISE)
    , _rx_gpio{false, 0, 0, 0}
    , _rx_freq(0.0)
    , _rx_gain(0.0)
{
    // 1) Описываем GPIO-поля и собираем DDR
    _init_gpio_map();

    // 2) Применяем DDR: 1 = FPGA drives pin (т.е. это "выход" со стороны USRP/FPGA)
    _iface->set_gpio_ddr(uhd::usrp::dboard_iface::UNIT_RX, _rx_gpio.ddr);

    // 3) Безопасные дефолты
    _set_gpio_field(GPIO_SPI_ADDR, SPI_DEST_NONE_3B);
    _set_gpio_field(GPIO_CPLD_RST_N, 0);
    _flush_gpio();

    // 4) CPLD reset sequence
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    _set_gpio_field(GPIO_CPLD_RST_N, 1);
    _flush_gpio();

    // Дай железу чуть времени выйти в рабочий режим (особенно если SPI через CPLD)

    // 5) Регистрируем UHD properties (как в образце), чтобы UHD реально вызывал наши coercers
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

        // bandwidth пока фиксированная заглушка (подставь реальную полосу тракта)
        get_rx_subtree()->create<double>("bandwidth/value").set(KINTEX7SDR_RX_BW_RANGE.start());
        get_rx_subtree()->create<uhd::meta_range_t>("bandwidth/range").set(KINTEX7SDR_RX_BW_RANGE);
    }


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


    _iface->get_clock_rate(dboard_iface::UNIT_RX);

    try {
    	_iface->set_clock_rate(dboard_iface::UNIT_RX, _REF_freq);
    } catch (const uhd::not_implemented_error&) {
    	UHD_LOG_WARNING(
    			"KINTEX7SDR", "Unable to set dboard clock rate - phase will vary");
    }

    _iface->set_clock_enabled(dboard_iface::UNIT_RX, true);
}

db_kintex7sdr_rx::~db_kintex7sdr_rx(void)
{
    UHD_SAFE_CALL(
        // Возвращаем линии в безопасное состояние
        _set_gpio_field(GPIO_SPI_ADDR, SPI_DEST_NONE_3B);
        _set_gpio_field(GPIO_CPLD_RST_N, 0);
        _flush_gpio();
    )
}

// ============================================================================
// GPIO map + helpers
// ============================================================================

void db_kintex7sdr_rx::_init_gpio_map()
{
    // ВАЖНО: unit/offset/mask должны соответствовать твоим FPGA constraints.
    // Здесь минимально: 3 бита SPI_ADDR и CPLD_RST_N.
    const gpio_field_info fields[] = {
        {GPIO_SPI_ADDR,   uhd::usrp::dboard_iface::UNIT_RX, 0, 0x7u << 0, 3, true},
        {GPIO_CPLD_RST_N, uhd::usrp::dboard_iface::UNIT_RX, 3, 0x1u << 3, 1, true},
    };

    for (const auto& f : fields) {
        _gpio_map[f.id] = f;
        if (f.fpga_drives) {
            _rx_gpio.ddr |= f.mask;
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
    if (f.fpga_drives) {
        return (_rx_gpio.value & f.mask) >> f.offset;
    }

    const uint32_t v = _iface->read_gpio(f.unit);
    return (v & f.mask) >> f.offset;
}

void db_kintex7sdr_rx::_flush_gpio()
{
    if (_rx_gpio.dirty) {
        _iface->set_gpio_out(uhd::usrp::dboard_iface::UNIT_RX, _rx_gpio.value, _rx_gpio.mask);
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

    return _iface->read_write_spi(uhd::usrp::dboard_iface::UNIT_RX, _spi_cfg, word, nbits);
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

/*void db_kintex7sdr_rx::_program_ltc6948_integer_n(uint16_t n_div, uint8_t r_div)
{
    constexpr uint16_t k_nd_min = 32;
    constexpr uint16_t k_nd_max = 1023;
    constexpr uint8_t k_rd_min = 1;
    constexpr uint8_t k_rd_max = 31;

    if (n_div < k_nd_min) {
        n_div = k_nd_min;
    } else if (n_div > k_nd_max) {
        n_div = k_nd_max;
    }

    if (r_div < k_rd_min) {
        r_div = k_rd_min;
    } else if (r_div > k_rd_max) {
        r_div = k_rd_max;
    }

    uint8_t reg3 = ltc6948::REG3_DEFAULT;
    reg3 |= ltc6948::REG3_INTN;
    reg3 &= static_cast<uint8_t>(~ltc6948::REG3_DITHEN);

    const uint8_t reg6 = static_cast<uint8_t>(((r_div << ltc6948::REG6_RD_SHIFT)
        & ltc6948::REG6_RD_MASK)
        | ((n_div >> 8) & ltc6948::REG6_ND_MSB_MASK));
    const uint8_t reg7 = static_cast<uint8_t>(n_div & 0xFFu);

    _ltc6948_xfer16(ltc6948::make_word_wr(ltc6948::REG3, reg3));
    _ltc6948_xfer16(ltc6948::make_word_wr(ltc6948::REG6, reg6));
    _ltc6948_xfer16(ltc6948::make_word_wr(ltc6948::REG7, reg7));
}*/

// ============================================================================
// UHD coercers (пока заглушки)
// ============================================================================

double db_kintex7sdr_rx::set_rx_frequency(double freq)
{
    constexpr double k_min_freq = 300e6;
    constexpr double k_max_freq = 2200e6;
    constexpr double k_step_hz = 1e6;
    constexpr double k_ref_hz = _REF_freq;
    constexpr double k_nd_min = 32.0;

    if (freq < k_min_freq) {
        freq = k_min_freq;
    } else if (freq > k_max_freq) {
        freq = k_max_freq;
    }

    freq = std::round(freq / k_step_hz) * k_step_hz;

    double r_div = std::ceil((k_nd_min * k_ref_hz) / freq);
    if (r_div < 1.0) {
        r_div = 1.0;
    }

    auto r_div_u8 = static_cast<uint8_t>(r_div);

    if (r_div_u8 < 1) {
        r_div_u8 = 1;
    } else if (r_div_u8 > 31) {
        r_div_u8 = 31;
    }

    auto n_div = static_cast<uint16_t>(std::round((freq * r_div_u8) / k_ref_hz));

    if (n_div < 32) {
        n_div = 32;
    } else if (n_div > 1023) {
        n_div = 1023;
    }

    //_program_ltc6948_integer_n(n_div, r_div_u8);

    _rx_freq = (k_ref_hz * n_div) / r_div_u8;
    return _rx_freq;
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
