#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_IDS_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_IDS_HPP

#include <uhd/usrp/dboard_id.hpp>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

// ВАЖНО: поставь сюда РЕАЛЬНЫЙ RX ID из EEPROM твоей платы.
// 0xFFFF = none (никогда не матчится), 0x0000 тоже плохо (может матчить “мусор”).
static const uhd::usrp::dboard_id_t DB_KINTEX7SDR_RX_ID(0x0001); // <-- TODO: заменить на свой

// Для RX-only платы TX отсутствует:
static const uhd::usrp::dboard_id_t DB_KINTEX7SDR_TX_ID_NONE = uhd::usrp::dboard_id_t::none();

}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_IDS_HPP
