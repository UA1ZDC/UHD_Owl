#ifndef DB_KINTEX7SDR_IDS_HPP
#define DB_KINTEX7SDR_IDS_HPP

#include <uhd/usrp/dboard_id.hpp> // dboard_id_t

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {

/*!
 * RX-only board ID.
 * ДОЛЖНО совпадать с тем, что прошито в RX EEPROM.
 */
static const uhd::usrp::dboard_id_t DB_KINTEX7SDR_RX_ID = 0xXXXX; // <-- поставь свой ID

/*!
 * TX slot отсутствует -> используем “none”.
 * Это лучше, чем магические 0x0000/0xFFFF в коде.
 */
static const uhd::usrp::dboard_id_t DB_KINTEX7SDR_TX_ID_NONE = uhd::usrp::dboard_id_t::none;

}}}} // namespaces

#endif // DB_KINTEX7SDR_IDS_HPP
