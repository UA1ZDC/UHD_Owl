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
 * Constants
 **********************************************************************/
constexpr double fMHz = 1e6;

// Board ranges (adjust to your HW)
static const uhd::freq_range_t KINTEX7SDR_RX_FREQ_RANGE(300e6, 2.2e9);
static const uhd::freq_range_t KINTEX7SDR_RX_BW_RANGE(100e6, 100e6);
static const uhd::gain_range_t KINTEX7SDR_RX_GAIN_RANGE(0.0, 31.5, 0.5);
static const std::vector<std::string> KINTEX7SDR_RX_ANTENNAS{"RX1"};

enum spi_dest_t {
    SPI_DEST_CPLD     = 0x0,
    SPI_DEST_LTC6948  = 0x1,
    SPI_DEST_LTC5594  = 0x2,
    SPI_DEST_AD7922   = 0x3,
    SPI_DEST_AD7922_2 = 0x4,
    SPI_DEST_NONE_3B  = 0x7u
};

const std::array<db_kintex7sdr_rx::gpio_field_info_t, 5>
db_kintex7sdr_rx::gpio_field_info = {{
    // Field                     Unit                          Offset Mask        Width Dir                                ATR   IDLE TX RX FDX
    {GPIO_SPI_ADDR,              uhd::usrp::dboard_iface::UNIT_RX, 0,    0x7u << 0, 3,    gpio_field_info_t::fpga_OUTPUT, false, 0,   0, 0, 0},
    {GPIO_CPLD_RST_N,            uhd::usrp::dboard_iface::UNIT_RX, 3,    0x1u << 3, 1,    gpio_field_info_t::fpga_OUTPUT, false, 0,   0, 0, 0},
    {RX_LO_LOCKED,               uhd::usrp::dboard_iface::UNIT_RX, 4,    0x1u << 4, 1,    gpio_field_info_t::fpga_INPUT,  false, 0,   0, 0, 0},
    {RX_EN,                      uhd::usrp::dboard_iface::UNIT_RX, 5,    0x1u << 5, 1,    gpio_field_info_t::fpga_OUTPUT, true,  0,   0, 1, 0},
    {TPS_EN,                     uhd::usrp::dboard_iface::UNIT_RX, 6,    0x1u << 6, 1,    gpio_field_info_t::fpga_OUTPUT, false, 0,   0, 0, 0}
}};

// ----------------------------------------------------------------------------
// LTC6948 part table and board defaults
// ----------------------------------------------------------------------------
namespace {
constexpr std::array<double, 4> LTC6948_VCO_MIN_HZ = {2240e6, 3080e6, 3840e6, 4200e6};
constexpr std::array<double, 4> LTC6948_VCO_MAX_HZ = {3740e6, 4910e6, 5790e6, 6390e6};
constexpr uint8_t LTC6948_PART_MIN = 1;
constexpr uint8_t LTC6948_PART_MAX = 4;
constexpr uint8_t LTC6948_PART_MASK = 0x0Fu;

constexpr uint8_t LTC6948_CP_LINEAR_MASK = static_cast<uint8_t>(
    ltc6948::REGD_CPCHI | ltc6948::REGD_CPCLO | ltc6948::REGD_CPMID | ltc6948::REGD_CPRST
    | ltc6948::REGD_CPUP | ltc6948::REGD_CPDN);

// Board-validated defaults:
constexpr bool k_ltc6948_bst = true;
constexpr ltc6948::filt_t k_ltc6948_filt = ltc6948::filt_t::f0; // your validated FILT=0 for 100MHz ref
constexpr ltc6948::rfo_t  k_ltc6948_rfo  = ltc6948::rfo_t::lvl3;

constexpr uint8_t k_ltc6948_bd = 4;
constexpr ltc6948::ldov_t k_ltc6948_ldov = ltc6948::ldov_t::v2;
constexpr bool k_ltc6948_ldoen = true;

constexpr ltc6948::mode_t   k_ltc6948_mode = ltc6948::mode_t::fractional; // always frac-N
constexpr ltc6948::dither_t k_ltc6948_dither = ltc6948::dither_t::on;      // keep as default/on
} // namespace

// ============================================================================
// db_kintex7sdr_rx
// ============================================================================

db_kintex7sdr_rx::db_kintex7sdr_rx(dboard_base::ctor_args_t args)
    : uhd::usrp::rx_dboard_base(args)
    , _iface(get_iface())
    , _spi_cfg(uhd::spi_config_t::EDGE_RISE)
{
    _init_gpio_map();

    _iface->set_gpio_ddr(dboard_iface::UNIT_RX, _rx_gpio.ddr);

    // Safe defaults
    _set_gpio_field(GPIO_SPI_ADDR, SPI_DEST_NONE_3B);
    _set_gpio_field(GPIO_CPLD_RST_N, 0);
    _flush_gpio();

    _set_gpio_field(RX_EN, 0);
    _flush_gpio();

    // Configure ATR
    _iface->set_atr_reg(dboard_iface::UNIT_RX, gpio_atr::ATR_REG_IDLE, _rx_gpio.atr_idle);
    _iface->set_atr_reg(dboard_iface::UNIT_RX, gpio_atr::ATR_REG_RX_ONLY, _rx_gpio.atr_rx);
    _iface->set_pin_ctrl(dboard_iface::UNIT_RX, _rx_gpio.atr_mask);

    // Reset CPLD
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    _set_gpio_field(GPIO_CPLD_RST_N, 1);
    _flush_gpio();

    // IQ demod bring-up (LTC5594): reset + enable blocks.
    _ltc5594_init();

    // UHD props
    using namespace std::placeholders;
    get_rx_subtree()->create<std::string>("name").set("DB_KINTEX7SDR RX");

    get_rx_subtree()->create<double>("bandwidth/value").set(KINTEX7SDR_RX_BW_RANGE.start());
    get_rx_subtree()->create<uhd::meta_range_t>("bandwidth/range").set(KINTEX7SDR_RX_BW_RANGE);

    get_rx_subtree()->create<double>("gains/PGA0/value")
        .set_coercer(std::bind(&db_kintex7sdr_rx::set_rx_gain, this, _1))
        .set(0.0);
    get_rx_subtree()->create<uhd::meta_range_t>("gains/PGA0/range").set(KINTEX7SDR_RX_GAIN_RANGE);

    get_rx_subtree()->create<std::string>("antenna/value").set(KINTEX7SDR_RX_ANTENNAS.at(0));
    get_rx_subtree()->create<std::vector<std::string>>("antenna/options").set(KINTEX7SDR_RX_ANTENNAS);

    get_rx_subtree()->create<std::string>("connection").set("QI");
    get_rx_subtree()->create<bool>("enabled").set(true);
    get_rx_subtree()->create<bool>("use_lo_offset").set(false);

    try {
        _iface->set_clock_rate(dboard_iface::UNIT_RX, _REF_freq);
    } catch (const uhd::not_implemented_error&) {
        UHD_LOG_WARNING("KINTEX7SDR_RX", "Unable to set dboard clock rate - phase will vary");
    }

    _iface->set_clock_enabled(dboard_iface::UNIT_RX, true);

    get_rx_subtree()->create<sensor_value_t>("sensors/lo_locked")
        .set_publisher(std::bind(&db_kintex7sdr_rx::_get_locked, this, "RXLO"));

    const double clock_rate = _iface->get_clock_rate(dboard_iface::UNIT_RX);
    std::ostringstream oss;
    oss.str("");
    oss << "UNIT_RX REF clock frequency: " << std::fixed << std::setprecision(1) << double(clock_rate / fMHz) << "MHz";
    UHD_LOG_INFO("DB_KINTEX7SDR_RX", oss.str());

    _ltc6948_init();

    get_rx_subtree()->create<uhd::meta_range_t>("freq/range").set(KINTEX7SDR_RX_FREQ_RANGE);

    get_rx_subtree()->create<double>("freq/value")
        .set_coercer(std::bind(&db_kintex7sdr_rx::set_rx_frequency, this, _1))
        .set(800e6);

    _get_locked("RXLO");

    UHD_LOG_WARNING("KINTEX7SDR_RX", "I'm running");
}

db_kintex7sdr_rx::~db_kintex7sdr_rx(void)
{
    UHD_SAFE_CALL(
        _iface->set_pin_ctrl(dboard_iface::UNIT_RX, uint32_t(0));
        _set_gpio_field(GPIO_SPI_ADDR, SPI_DEST_NONE_3B);
        _set_gpio_field(GPIO_CPLD_RST_N, 0);
        _set_gpio_field(RX_EN, 0);
        _flush_gpio();
        UHD_LOG_WARNING("KINTEX7SDR_RX", "I'm toast i'm done");
    )
}

// ============================================================================
// GPIO helpers
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
    const auto it = _gpio_map.find(id);
    if (it == _gpio_map.end()) return;

    const auto& f = it->second;

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
    const auto it = _gpio_map.find(id);
    if (it == _gpio_map.end()) return 0;

    const auto& f = it->second;

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
// SPI helper: route + xfer under one mutex
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
// LTC5594 low-level reg access + LO-matching “calibration” (from datasheet table)
// ============================================================================

uint8_t db_kintex7sdr_rx::_ltc5594_read_reg(uint8_t addr)
{
    if (addr >= ltc5594::NUM_REGS) {
        return 0;
    }
    const auto rx = static_cast<uint16_t>(
        _spi_xfer_to(SPI_DEST_LTC5594, ltc5594::make_word_rd(addr), 16));
    return ltc5594::rx_data_byte(rx);
}

void db_kintex7sdr_rx::_ltc5594_write_reg(uint8_t addr, uint8_t value, bool force)
{
    if (addr >= ltc5594::NUM_REGS) {
        return;
    }

    if (!force && _ltc5594_reg_valid[addr] && _ltc5594_regs[addr] == value) {
        return;
    }

    _spi_xfer_to(SPI_DEST_LTC5594, ltc5594::make_word_wr(addr, value), 16);
    _ltc5594_regs[addr] = value;
    _ltc5594_reg_valid[addr] = true;
}

void db_kintex7sdr_rx::_ltc5594_init()
{
    if (_ltc5594_initialized) {
        return;
    }

    _ltc5594_regs.fill(0);
    _ltc5594_reg_valid.fill(false);

    // Soft reset: set SRST briefly, then enable the required blocks.
    _ltc5594_write_reg(ltc5594::REG_BCTL,
        ltc5594::pack_bctl(ltc5594::bctl::ENABLE_ALL, true),
        true /*force*/);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    _ltc5594_write_reg(ltc5594::REG_BCTL,
        ltc5594::pack_bctl(ltc5594::bctl::ENABLE_ALL, false),
        true /*force*/);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    const uint8_t chipid = _ltc5594_read_reg(ltc5594::REG_CHIPID);
    UHD_LOG_INFO("DB_KINTEX7SDR_RX",
        (boost::format("LTC5594 CHIPID = 0x%1$02X") % unsigned(chipid)).str());

    _ltc5594_initialized = true;
}

void db_kintex7sdr_rx::_ltc5594_maybe_run_autocal(double /*lo_hz*/)
{
    // Placeholder: real closed-loop IQ auto-calibration will live here.
    // Intentionally no-op for now.
}

void db_kintex7sdr_rx::_ltc5594_apply_for_lo(double lo_hz)
{
    if (!_ltc5594_initialized) {
        _ltc5594_init();
    }

    const auto match = ltc5594::resolve_lo_match(lo_hz, _ltc5594_lo_mode);
    if (!match.valid) {
        UHD_LOG_WARNING("DB_KINTEX7SDR_RX",
            (boost::format("LTC5594: no LO-matching entry for f_LO=%.3f MHz")
                % (lo_hz / fMHz)).str());
        return;
    }

    const bool same_bucket = (_ltc5594_last_match_idx == match.table_index);
    const bool same_regs   = (same_bucket && _ltc5594_last_reg12 == match.reg12 && _ltc5594_last_reg13 == match.reg13);

    if (!same_regs) {
        _ltc5594_write_reg(ltc5594::REG_LVCM_CF1, match.reg12);
        _ltc5594_write_reg(ltc5594::REG_BAND_LF1_CF2, match.reg13);

        _ltc5594_last_match_idx = match.table_index;
        _ltc5594_last_reg12 = match.reg12;
        _ltc5594_last_reg13 = match.reg13;

        /*UHD_LOG_INFO("DB_KINTEX7SDR_RX",
            (boost::format("LTC5594 LO-match: f_LO=%.3f MHz, bucket=%u, REG12=0x%02X REG13=0x%02X")
                % (lo_hz / fMHz)
                % unsigned(match.table_index)
                % unsigned(match.reg12)
                % unsigned(match.reg13)).str());*/
    }

    // Apply cached per-bucket calibration values (reserved for future autocal).
    // Right now the cache is expected to be empty (valid=false everywhere),
    // but the wiring is in place.
    const auto it = _ltc5594_cal_cache.find(match.table_index);
    if (it != _ltc5594_cal_cache.end() && it->second.valid) {
        const auto& cal = it->second;

        // DC offsets
        _ltc5594_write_reg(ltc5594::REG_DCOI, cal.dcoi);
        _ltc5594_write_reg(ltc5594::REG_DCOQ, cal.dcoq);

        // IQ gain error: preserve IP3CC[1:0] bits
        uint8_t reg11 = 0;
        if (_ltc5594_reg_valid[ltc5594::REG_GERR_IP3CC]) {
            reg11 = _ltc5594_regs[ltc5594::REG_GERR_IP3CC];
        } else {
            reg11 = _ltc5594_read_reg(ltc5594::REG_GERR_IP3CC);
        }
        _ltc5594_write_reg(ltc5594::REG_GERR_IP3CC, ltc5594::pack_reg11_gerr(reg11, cal.gerr_6b));

        // IQ phase: preserve amplifier settings in REG_PHA0_MISC (0x15)
        uint8_t reg15 = 0;
        if (_ltc5594_reg_valid[ltc5594::REG_PHA0_MISC]) {
            reg15 = _ltc5594_regs[ltc5594::REG_PHA0_MISC];
        } else {
            reg15 = _ltc5594_read_reg(ltc5594::REG_PHA0_MISC);
        }

        uint8_t reg14 = 0;
        uint8_t reg15_new = 0;
        ltc5594::pack_pha_9b(cal.pha_9b, reg14, reg15, reg15_new);
        _ltc5594_write_reg(ltc5594::REG_PHA_8_1, reg14);
        _ltc5594_write_reg(ltc5594::REG_PHA0_MISC, reg15_new);
    }

    _ltc5594_maybe_run_autocal(lo_hz);
}

// ============================================================================
// LTC6948 low-level reg access
// ============================================================================

uint8_t db_kintex7sdr_rx::_ltc6948_read_reg(uint8_t addr)
{
    const auto rx = static_cast<uint16_t>(_spi_xfer_to(SPI_DEST_LTC6948, ltc6948::make_word_rd(addr), 16));
    return ltc6948::rx_data_byte(rx);
}

void db_kintex7sdr_rx::_ltc6948_write_reg(uint8_t addr, uint8_t value, bool force)
{
    if (addr >= ltc6948::NUM_REGS) return;

    if (!force && _ltc6948_initialized && _ltc6948_regs[addr] == value) {
        return;
    }
    _spi_xfer_to(SPI_DEST_LTC6948, ltc6948::make_word_wr(addr, value), 16);
    _ltc6948_regs[addr] = value;
}

void db_kintex7sdr_rx::_ltc6948_update_bits(uint8_t addr, uint8_t mask, uint8_t value)
{
    if (addr >= ltc6948::NUM_REGS) return;
    const uint8_t base = _ltc6948_initialized ? _ltc6948_regs[addr] : ltc6948::DEFAULT_REGS[addr];
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
            % unsigned(part_code) % (_ltc6948_vco_min_hz / fMHz) % (_ltc6948_vco_max_hz / fMHz)).str());

    return part_code;
}

void db_kintex7sdr_rx::_ltc6948_init()
{
    if (_ltc6948_initialized) return;

    _ltc6948_regs = ltc6948::DEFAULT_REGS;

    // REG2: keep mute-during-calibration, but unmute output, clear POR/powerdowns
    _ltc6948_regs[ltc6948::REG2] = static_cast<uint8_t>(
        (_ltc6948_regs[ltc6948::REG2]
            & static_cast<uint8_t>(~(ltc6948::REG2_PDALL | ltc6948::REG2_PDPLL | ltc6948::REG2_PDVCO
                | ltc6948::REG2_PDOUT | ltc6948::REG2_PDFN | ltc6948::REG2_OMUTE | ltc6948::REG2_POR)))
        | ltc6948::REG2_MTCAL);

    // REG3: start from datasheet default, but make mode/dither explicit
    _ltc6948_regs[ltc6948::REG3] = ltc6948::pack_reg3(ltc6948::DEFAULT_REGS[ltc6948::REG3], k_ltc6948_mode, k_ltc6948_dither);

    // REG4: make LDO/BD/CPLE explicit (CPLE starts OFF; we enable it only after LOCK)
    _ltc6948_regs[ltc6948::REG4] = ltc6948::pack_reg4(k_ltc6948_bd, k_ltc6948_ldoen, k_ltc6948_ldov, /*cple=*/false);

    // REGD: CP normal, clamps ON
    _ltc6948_regs[ltc6948::REGD] = ltc6948::regd_cp_normal_clamps_on();

    // REGB: explicit BST/FILT/RFO (OD is placeholder, will be updated per frequency)
    _ltc6948_regs[ltc6948::REGB] = ltc6948::pack_regb(/*od=*/3, k_ltc6948_bst, k_ltc6948_filt, k_ltc6948_rfo);

    // Program non-frequency-dependent regs first
    for (const uint8_t addr : {ltc6948::REG1, ltc6948::REG2, ltc6948::REG3, ltc6948::REG4,
                               ltc6948::REG5, ltc6948::REGB, ltc6948::REGC, ltc6948::REGD}) {
        _ltc6948_write_reg(addr, _ltc6948_regs[addr], true);
    }

    _ltc6948_read_part_code();
    _ltc6948_initialized = true;
}

// Choose OD/ND/NUM within +/-tol_hz. Criterion:
//  1) VCO closest to mid-range (robustness)
//  2) if almost equal, prefer larger OD (often better output phase noise due to division)
bool db_kintex7sdr_rx::_ltc6948_resolve_pll(double target_freq, double tol_hz, ltc6948_pll_config& cfg)
{
    _ltc6948_init();
    if (_ltc6948_part_code == 0) _ltc6948_read_part_code();

    const double min_freq = _ltc6948_vco_min_hz / ltc6948::OD_MAX;
    const double max_freq = _ltc6948_vco_max_hz / ltc6948::OD_MIN;

    double target = target_freq;
    if (target < min_freq) target = min_freq;
    else if (target > max_freq) target = max_freq;

    // Fixed PFD: REF=100MHz, PFD=50MHz => RD=2
    uint8_t rd = static_cast<uint8_t>(std::lround(_REF_freq / _PFD_freq));
    rd = std::max<uint8_t>(ltc6948::RD_MIN, std::min<uint8_t>(ltc6948::RD_MAX, rd));
    const double fpfd = _REF_freq / rd;

    const double vco_mid = 0.5 * (_ltc6948_vco_min_hz + _ltc6948_vco_max_hz);
    constexpr double k_modulus = static_cast<double>(ltc6948::MODULUS);
    constexpr double k_vco_score_tie_hz = 25e6;

    bool found = false;
    double best_score = std::numeric_limits<double>::infinity();

    for (uint8_t od = ltc6948::OD_MIN; od <= ltc6948::OD_MAX; od++) {
        const double fvco_target = target * od;
        if (fvco_target < _ltc6948_vco_min_hz || fvco_target > _ltc6948_vco_max_hz) continue;

        const double ratio = fvco_target / fpfd;
        auto nd = static_cast<uint16_t>(std::floor(ratio));
        if (nd < ltc6948::ND_MIN || nd > ltc6948::ND_MAX) continue;

        const double frac = ratio - nd;
        uint32_t num = static_cast<uint32_t>(std::llround(frac * k_modulus));
        if (num >= ltc6948::MODULUS) {
            num = 0;
            nd = static_cast<uint16_t>(nd + 1);
        }
        if (nd < ltc6948::ND_MIN || nd > ltc6948::ND_MAX) continue;

        const double fvco_actual = fpfd * (static_cast<double>(nd) + static_cast<double>(num) / k_modulus);
        const double actual = fvco_actual / od;
        const double err = actual - target;

        if (std::abs(err) > tol_hz) continue;

        const double score = std::abs(fvco_actual - vco_mid);
        const bool tie = found && (std::abs(score - best_score) <= k_vco_score_tie_hz);

        if (!found || score < best_score || (tie && od > cfg.od)) {
            cfg = {rd, od, nd, num, fpfd, fvco_actual, actual, err};
            best_score = score;
            found = true;
        }
    }

    if (!found) {
        UHD_LOG_WARNING("DB_KINTEX7SDR_RX",
            (boost::format("Unable to resolve LTC6948 PLL for %.3f MHz within ±%.3f MHz")
                % (target / fMHz) % (tol_hz / fMHz)).str());
    }
    return found;
}

void db_kintex7sdr_rx::_ltc6948_apply_pll_config(const ltc6948_pll_config& cfg)
{
    _ltc6948_init();

    const uint32_t num = std::min<uint32_t>(cfg.num, ltc6948::NUM_MAX);

    // REG3: preserve base bits, but ensure fractional mode (INTN=0) and desired dither setting.
    uint8_t reg3 = _ltc6948_regs[ltc6948::REG3];
    reg3 = ltc6948::pack_reg3(reg3, ltc6948::mode_t::fractional,  ltc6948::dither_t::off);

    // REG6/7: RD + ND
    const uint8_t reg6 = ltc6948::pack_reg6(cfg.rd, cfg.nd);
    const uint8_t reg7 = static_cast<uint8_t>(cfg.nd & 0xFFu);

    // REG8/9/A: NUM + keep REGA lower nibble (control bits) as-is
    uint8_t reg8 = 0, reg9 = 0, rega_num = 0;
    ltc6948::num_to_regs(num, reg8, reg9, rega_num);
    const uint8_t rega = static_cast<uint8_t>(rega_num | (_ltc6948_regs[ltc6948::REGA] & 0x0Fu));

    // REGB: preserve BST/FILT/RFO, update only OD.
    const uint8_t regb = ltc6948::regb_set_od(_ltc6948_regs[ltc6948::REGB], cfg.od);

    // CP must be normal.
    _ltc6948_update_bits(ltc6948::REGD, LTC6948_CP_LINEAR_MASK, ltc6948::regd_cp_normal_clamps_on());

    _ltc6948_write_reg(ltc6948::REG3, reg3);
    _ltc6948_write_reg(ltc6948::REGB, regb);

    // Writing REG6..REGA triggers AUTOCAL (default behaviour)
    _ltc6948_write_reg(ltc6948::REG6, reg6, true);
    _ltc6948_write_reg(ltc6948::REG7, reg7, true);
    _ltc6948_write_reg(ltc6948::REG8, reg8, true);
    _ltc6948_write_reg(ltc6948::REG9, reg9, true);
    _ltc6948_write_reg(ltc6948::REGA, rega, true);

    // Wait LOCK up to 20ms (LOCK pin routed to GPIO)
    bool locked = false;
    using clock = std::chrono::steady_clock;
    const auto deadline = clock::now() + std::chrono::milliseconds(20);
    while (clock::now() < deadline) {
        if (_get_gpio_field(RX_LO_LOCKED) != 0) { locked = true; break; }
        std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
    _rxlo_locked = locked;

    // enable dither after lock
    uint8_t reg3_on = ltc6948::pack_reg3(_ltc6948_regs[ltc6948::REG3],
    		ltc6948::mode_t::fractional,
			k_ltc6948_dither);
    _ltc6948_write_reg(ltc6948::REG3, reg3_on);

    // REG4: enable CPLE only after LOCK; otherwise force it off
    const bool cple_en = locked;
    const uint8_t reg4 = ltc6948::reg4_set_cple(_ltc6948_regs[ltc6948::REG4], cple_en);
    _ltc6948_write_reg(ltc6948::REG4, reg4);

    if (!locked) {
        const uint8_t st0 = _ltc6948_read_reg(ltc6948::REG0);
        UHD_LOG_WARNING("DB_KINTEX7SDR_RX",
            (boost::format("LTC6948: no LOCK in 20ms, skip CPLE. REG0=0x%1$02X (%2%)")
                % unsigned(st0) % ltc6948::status_to_string(st0)).str());
    }

/*    UHD_LOG_INFO("DB_KINTEX7SDR_RX",
        (boost::format("LTC6948 set: req=%.3f MHz act=%.3f MHz (err=%.3f kHz), RD=%u ND=%u NUM=%u OD=%u, REGB(BST=%u FILT=%u RFO=%u)")
            % ((cfg.actual_freq_hz - cfg.error_hz) / fMHz)
            % (cfg.actual_freq_hz / fMHz)
            % (cfg.error_hz / 1e3)
            % unsigned(cfg.rd)
            % unsigned(cfg.nd)
            % unsigned(num)
            % unsigned(cfg.od)
            % unsigned(ltc6948::get_bst(regb))
            % unsigned(ltc6948::filt_bits(ltc6948::get_filt(regb)))
            % unsigned(ltc6948::rfo_bits(ltc6948::get_rfo(regb)))).str());*/
}

// ============================================================================
// UHD coercers
// ============================================================================

double db_kintex7sdr_rx::set_rx_frequency(double freq)
{
    constexpr double k_step_hz = 1e3;
    constexpr double k_cache_tol_hz = 0.5e6; // +/-0.5 MHz

    freq = KINTEX7SDR_RX_FREQ_RANGE.clip(freq);
    freq = std::round(freq / k_step_hz) * k_step_hz;

    // Cache shortcut: if already within +/-0.5 MHz, don't touch PLL
    if (_rxlo_cfg_valid && std::abs(freq - _rxlo_last_cfg.actual_freq_hz) <= k_cache_tol_hz) {
        // Still ensure the IQ demod (LTC5594) is configured for the current LO bucket.
        _ltc5594_apply_for_lo(_rxlo_last_cfg.actual_freq_hz);
        return _rxlo_last_cfg.actual_freq_hz;
    }

    ltc6948_pll_config cfg{};
    if (!_ltc6948_resolve_pll(freq, k_cache_tol_hz, cfg)) {
        return _rx_freq;
    }

    _ltc6948_apply_pll_config(cfg);

    // Configure LTC5594 LO matching (and apply cached per-bucket calibration if present).
    _ltc5594_apply_for_lo(cfg.actual_freq_hz);

    _rx_freq = cfg.actual_freq_hz;
    _rxlo_last_cfg = cfg;
    _rxlo_cfg_valid = true;

    return _rx_freq;
}

sensor_value_t db_kintex7sdr_rx::_get_locked([[maybe_unused]] const std::string& pll_name)
{
    std::lock_guard<std::mutex> lock(_spi_mutex);

    _rxlo_locked = (_get_gpio_field(RX_LO_LOCKED) != 0);
    UHD_LOG_INFO("DB_KINTEX7SDR_RX", std::string("LO lock status: [") + (_rxlo_locked ? "LOCKED" : "----") + "]");

    return sensor_value_t("RXLO", _rxlo_locked, "locked", "unlocked");
}

double db_kintex7sdr_rx::set_rx_gain(double gain)
{
    gain = KINTEX7SDR_RX_GAIN_RANGE.clip(gain);
    _rx_gain = gain;
    return _rx_gain;
}

// Registration
static uhd::usrp::dboard_base::sptr make_db_kintex7sdr_rx(uhd::usrp::dboard_base::ctor_args_t args)
{
    return uhd::usrp::dboard_base::sptr(new db_kintex7sdr_rx(args));
}

UHD_STATIC_BLOCK(register_db_kintex7sdr_rx)
{
    dboard_manager::register_dboard(DB_KINTEX7SDR_RX_ID, &make_db_kintex7sdr_rx, "DB_KINTEX7SDR_RX");
}

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr
