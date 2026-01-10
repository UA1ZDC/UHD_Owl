#include "db_kintex7sdr.hpp"

#include <uhd/usrp/dboard_manager.hpp>
#include <uhd/utils/static.hpp>

#include <chrono>
#include <thread>
#include <type_traits>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

db_kintex7sdr_rx::db_kintex7sdr_rx(uhd::usrp::dboard_base::ctor_args_t args)
    : uhd::usrp::rx_dboard_base(args)
    , _iface(get_iface())
    , _spi_cfg(uhd::spi_config_t::EDGE_RISE)
    , _rx_gpio{false, 0, 0, 0}
    , _rx_freq(0.0)
    , _rx_gain(0.0)
{
    _init_gpio_map();

    // Настраиваем DDR: 1 = FPGA drives pin (input to dboard)
    _iface->set_gpio_ddr(uhd::usrp::dboard_iface::UNIT_RX, _rx_gpio.ddr);

    // Дефолты
    _set_gpio_field(GPIO_SPI_ADDR, 0);
    _set_gpio_field(GPIO_CPLD_RST_N, 0);
    _flush_gpio();

    // Держим CPLD в reset
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    _set_gpio_field(GPIO_CPLD_RST_N, 1);
    _flush_gpio();

    // Пример: прочитать ID LTC5594 (если SDO подключён и read_write_spi есть)
    log_ltc5594_chip_id();
}

db_kintex7sdr_rx::~db_kintex7sdr_rx(void)
{
    UHD_SAFE_CALL(
        // Вернуть SPI_ADDR в “ничего не выбрано” (если нужно)
        _set_gpio_field(GPIO_SPI_ADDR, 0);
        _set_gpio_field(GPIO_CPLD_RST_N, 0);
        _flush_gpio();
    )
}

void db_kintex7sdr_rx::_init_gpio_map()
{
    // Минимальная карта под твой текущий UNIT_RX
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
    if (it == _gpio_map.end()) return;

    const auto& f = it->second;
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
    if (it == _gpio_map.end()) return 0;

    const auto& f = it->second;
    // Если FPGA drives — читаем из кеша
    if (f.fpga_drives) {
        return (_rx_gpio.value & f.mask) >> f.offset;
    }

    // Иначе читаем реально
    uint32_t v = _iface->read_gpio(f.unit);
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

void db_kintex7sdr_rx::_route_spi(cpld::spi_dest_t dest)
{
    _set_gpio_field(GPIO_SPI_ADDR, uint32_t(dest));
    _flush_gpio();
}

template<typename IFACE>
auto db_kintex7sdr_rx::_has_readwrite(int) -> decltype(
    std::declval<IFACE&>().read_write_spi(
        std::declval<uhd::usrp::dboard_iface::unit_t>(),
        std::declval<const uhd::spi_config_t&>(),
        uint32_t{}, size_t{}),
    std::true_type{})
{
    return {};
}

template<typename IFACE>
std::false_type db_kintex7sdr_rx::_has_readwrite(...)
{
    return {};
}

uint32_t db_kintex7sdr_rx::_spi_xfer(uint32_t word, size_t nbits)
{
    std::lock_guard<std::mutex> lock(_spi_mutex);

    // Если есть read_write_spi — используем (нужен для чтения ID и т.п.)
    if (decltype(_has_readwrite<uhd::usrp::dboard_iface>(0))::value) {
        return _iface->read_write_spi(uhd::usrp::dboard_iface::UNIT_RX, _spi_cfg, word, nbits);
    }

    // Иначе только write (чтение вернём 0, но сборка проходит)
    _iface->write_spi(uhd::usrp::dboard_iface::UNIT_RX, _spi_cfg, word, nbits);
    return 0;
}

void db_kintex7sdr_rx::_cpld_wr(uint8_t reg7, uint32_t data24)
{
    _route_spi(cpld::SPI_DEST_CPLD);
    _spi_xfer(cpld::make_frame_wr(reg7, data24), 32);
}

uint16_t db_kintex7sdr_rx::_ltc5594_xfer16(uint16_t w)
{
    _route_spi(cpld::SPI_DEST_LTC5594);
    return uint16_t(_spi_xfer(uint32_t(w), 16) & 0xFFFFu);
}

uint16_t db_kintex7sdr_rx::_ltc6948_xfer16(uint16_t w)
{
    _route_spi(cpld::SPI_DEST_LTC6948);
    return uint16_t(_spi_xfer(uint32_t(w), 16) & 0xFFFFu);
}

void db_kintex7sdr_rx::log_ltc5594_chip_id()
{
    const uint16_t rx = _ltc5594_xfer16(ltc5594::make_word_rd(ltc5594::REG_CHIPID));
    const uint8_t chipid = ltc5594::rx_data_byte(rx);
    UHD_LOG_INFO("DB_KINTEX7SDR_RX", "LTC5594 CHIPID = 0x" + std::to_string(unsigned(chipid)));
}

double db_kintex7sdr_rx::set_rx_frequency(double freq)
{
    // TODO: будет через LTC6948-1 (300..2200 MHz, step 1 MHz)
    _rx_freq = freq;
    return _rx_freq;
}

double db_kintex7sdr_rx::set_rx_gain(double gain)
{
    // TODO: учесть два аттенюатора (ATT1 + ATT2)
    _rx_gain = gain;
    return _rx_gain;
}

/***********************************************************************
 * Registration
 * Предпочитаем overload “только RX id”, если он есть в твоём UHD.
 * Иначе используем пару (rx_id, tx_none).
 **********************************************************************/
static uhd::usrp::dboard_base::sptr make_db_kintex7sdr_rx(uhd::usrp::dboard_base::ctor_args_t args)
{
    return uhd::usrp::dboard_base::sptr(new db_kintex7sdr_rx(args));
}

template <typename MGR>
static auto _register_db(int) -> decltype(
    MGR::register_dboard(DB_KINTEX7SDR_RX_ID, &make_db_kintex7sdr_rx, "db_kintex7sdr_rx"),
    void())
{
    MGR::register_dboard(DB_KINTEX7SDR_RX_ID, &make_db_kintex7sdr_rx, "db_kintex7sdr_rx");
}

template <typename MGR>
static auto _register_db(long) -> decltype(
    MGR::register_dboard(DB_KINTEX7SDR_RX_ID, DB_KINTEX7SDR_TX_ID_NONE, &make_db_kintex7sdr_rx, "db_kintex7sdr_rx"),
    void())
{
    MGR::register_dboard(DB_KINTEX7SDR_RX_ID, DB_KINTEX7SDR_TX_ID_NONE, &make_db_kintex7sdr_rx, "db_kintex7sdr_rx");
}

UHD_STATIC_BLOCK(register_db_kintex7sdr_rx)
{
    // попробуем 1-id overload, если нет — упадём на 2-id
    _register_db<uhd::usrp::dboard_manager>(0);
}

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr
