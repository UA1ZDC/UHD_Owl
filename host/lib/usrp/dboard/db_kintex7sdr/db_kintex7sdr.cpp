#include "db_kintex7sdr_rx.hpp"

#include <uhd/usrp/dboard_manager.hpp>
#include <uhd/utils/static.hpp>
#include <boost/format.hpp>

#include <vector>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

// ===== GPIO map (UNIT_RX) =====
static const gpio_field_info gpio_info[] = {
    // id                      unit                 shift mask          width out
    {gpio_field_id::SPI_ADDR,  dboard_iface::UNIT_RX, 0,    (0x7u << 0), 3,    true},
    {gpio_field_id::CPLD_RST_N,dboard_iface::UNIT_RX, 3,    (0x1u << 3), 1,    true},
};

db_kintex7sdr_rx::db_kintex7sdr_rx(ctor_args_t args)
    : rx_dboard_base(args)
{
    _iface = get_iface(); // ВАЖНО: ctor_args_t у тебя = void*, iface берём через базовый класс.

    _init_gpio();

    // безопасно: никого не выбираем
    _set_spi_route(SPI_DEST_NONE);

    // выводим CPLD из reset (если активный 0)
    _write_gpio_field(gpio_field_id::CPLD_RST_N, 1);

    // читаем/логируем CHIPID LTC5594
    _log_ltc5594_chip_id();
}

void db_kintex7sdr_rx::_init_gpio()
{
    _gpio.clear();
    for (const auto& f : gpio_info) {
        _gpio.emplace(f.id, f);

        // DDR: выставляем как output только нужные биты
        if (f.is_output) {
            // set_gpio_ddr(unit, value, mask)
            _iface->set_gpio_ddr(f.unit, f.mask, f.mask);
        }
    }
}

void db_kintex7sdr_rx::_write_gpio_field(gpio_field_id id, uint32_t value)
{
    const auto it = _gpio.find(id);
    if (it == _gpio.end()) {
        UHD_LOG_ERROR("DB_KINTEX7SDR_RX", "GPIO field not found");
        return;
    }
    const auto& f = it->second;

    const uint32_t v = (value << f.shift) & f.mask;
    _iface->write_gpio(f.unit, v, f.mask);
}

void db_kintex7sdr_rx::_set_spi_route(spi_dest_t dest)
{
    _write_gpio_field(gpio_field_id::SPI_ADDR, static_cast<uint32_t>(dest));
}

void db_kintex7sdr_rx::_spi_write(uint32_t v, uint8_t nbits)
{
    _iface->write_spi(dboard_iface::UNIT_RX, spi_config_t::EDGE_RISE, v, nbits);
}

uint32_t db_kintex7sdr_rx::_spi_readwrite(uint32_t v, uint8_t nbits)
{
    // Пытаемся read_write_spi(), если его нет в твоей UHD — будет fallback на write-only.
    return _rw_spi(*_iface, dboard_iface::UNIT_RX, spi_config_t::EDGE_RISE, v, nbits, 0);
}

// ===== LTC5594 low-level =====
void db_kintex7sdr_rx::_ltc5594_write_reg(uint8_t addr, uint8_t data)
{
    std::lock_guard<std::mutex> lock(_spi_mutex);
    _set_spi_route(SPI_DEST_LTC5594);
    _spi_write(ltc5594::make_write_frame(addr, data), 16);
    _set_spi_route(SPI_DEST_NONE);
}

uint8_t db_kintex7sdr_rx::_ltc5594_read_reg(uint8_t addr)
{
    std::lock_guard<std::mutex> lock(_spi_mutex);
    _set_spi_route(SPI_DEST_LTC5594);
    const uint32_t resp = _spi_readwrite(ltc5594::make_read_frame(addr), 16);
    _set_spi_route(SPI_DEST_NONE);
    return static_cast<uint8_t>(resp & 0xFFu);
}

void db_kintex7sdr_rx::_log_ltc5594_chip_id()
{
    const uint8_t reg = _ltc5594_read_reg(ltc5594::REG_CHIPID);
    const uint8_t chipid = (reg >> 6) & 0x03; // CHIPID[1:0] — верхние биты (по карте регистра)

    UHD_LOG_INFO("DB_KINTEX7SDR_RX",
                 (boost::format("LTC5594 reg 0x%02X = 0x%02X, CHIPID=0x%X")
                  % unsigned(ltc5594::REG_CHIPID)
                  % unsigned(reg)
                  % unsigned(chipid))
                     .str());
}

// ===== Factory + registration =====
static dboard_base::sptr make_db_kintex7sdr_rx(dboard_base::ctor_args_t args)
{
    return dboard_base::sptr(new db_kintex7sdr_rx(args));
}

// SFINAE: если в твоём UHD есть перегрузка register_dboard(id, ctor, name) — используем её.
// иначе падаем обратно на (rx_id, tx_id, ctor, name)
static auto _register_one_id(int) -> decltype(
    dboard_manager::register_dboard(DB_KINTEX7SDR_RX_ID, &make_db_kintex7sdr_rx, "db_kintex7sdr_rx"),
    void())
{
    dboard_manager::register_dboard(DB_KINTEX7SDR_RX_ID, &make_db_kintex7sdr_rx, "db_kintex7sdr_rx");
}

static void _register_one_id(long)
{
    dboard_manager::register_dboard(DB_KINTEX7SDR_RX_ID,
                                    DB_KINTEX7SDR_TX_ID_NONE,
                                    &make_db_kintex7sdr_rx,
                                    "db_kintex7sdr_rx");
}

UHD_STATIC_BLOCK(register_db_kintex7sdr_rx)
{
    _register_one_id(0);
}

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr
