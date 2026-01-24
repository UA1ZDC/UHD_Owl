#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP

#include <array>
#include <cstddef>
#include <cstdint>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr { namespace ltc6948 {

/***********************************************************************
 * LTC6948 SPI: 16 clocks per register
 * First byte: [A6..A0][R/W]  (R/W is LSB!)
 * Second byte: data
 **********************************************************************/
static inline uint8_t make_addr_byte_wr(uint8_t addr7) { return uint8_t((addr7 & 0x7Fu) << 1); }
static inline uint8_t make_addr_byte_rd(uint8_t addr7) { return uint8_t(((addr7 & 0x7Fu) << 1) | 0x01u); }

static inline uint16_t make_word_wr(uint8_t addr7, uint8_t data)
{
    return uint16_t(make_addr_byte_wr(addr7) << 8) | data;
}

static inline uint16_t make_word_rd(uint8_t addr7)
{
    return uint16_t(make_addr_byte_rd(addr7) << 8); // data byte = 0
}

static inline uint8_t rx_data_byte(uint16_t rx_word) { return uint8_t(rx_word & 0xFFu); }

/***********************************************************************
 * Register addresses (Table 15, datasheet)
 **********************************************************************/
enum reg_t : uint8_t {
    REG0 = 0x00,
    REG1 = 0x01,
    REG2 = 0x02,
    REG3 = 0x03,
    REG4 = 0x04,
    REG5 = 0x05,
    REG6 = 0x06,
    REG7 = 0x07,
    REG8 = 0x08,
    REG9 = 0x09,
    REGA = 0x0A,
    REGB = 0x0B,
    REGC = 0x0C,
    REGD = 0x0D,
    REGE = 0x0E
};

static constexpr std::size_t NUM_REGS = 15;
static constexpr uint32_t MODULUS = (1u << 18);

// Datasheet defaults for writable registers (REG0/REGE are read-only).
static constexpr std::array<uint8_t, NUM_REGS> DEFAULT_REGS = {
    0x00, // REG0 (read-only status)
    0x04, // REG1
    0x06, // REG2
    0x3E, // REG3
    0x47, // REG4
    0x11, // REG5
    0x08, // REG6
    0xFA, // REG7
    0x3F, // REG8
    0xFF, // REG9
    0xF0, // REGA
    0xF9, // REGB
    0x4F, // REGC
    0xE4, // REGD
    0x00  // REGE (read-only part/revision)
};

// REG2 bits (power, mute, reset)
static constexpr uint8_t REG2_PDALL = (1u << 7);
static constexpr uint8_t REG2_PDPLL = (1u << 6);
static constexpr uint8_t REG2_PDVCO = (1u << 5);
static constexpr uint8_t REG2_PDOUT = (1u << 4);
static constexpr uint8_t REG2_PDFN  = (1u << 3);
static constexpr uint8_t REG2_MTCAL = (1u << 2);
static constexpr uint8_t REG2_OMUTE = (1u << 1);
static constexpr uint8_t REG2_POR   = (1u << 0);

// REG3 bits
static constexpr uint8_t REG3_INTN    = 0x01u;
static constexpr uint8_t REG3_DITHEN  = 0x02u;
static constexpr uint8_t REG3_DEFAULT = DEFAULT_REGS[REG3];

// REG4 bits
static constexpr uint8_t REG4_BD_MASK = 0xF0u;
static constexpr uint8_t REG4_CPLE    = (1u << 3);
static constexpr uint8_t REG4_LDOEN   = (1u << 2);
static constexpr uint8_t REG4_LDOV_MASK = 0x03u;

// REG6 packing
static constexpr uint8_t REG6_RD_SHIFT = 3u; // RD[4:0] -> bits[7:3]
static constexpr uint8_t REG6_RD_MASK  = 0xF8u;
static constexpr uint8_t REG6_ND_MSB_MASK = 0x03u; // ND[9:8] -> bits[1:0]

// REG8/REGA packing for NUM[17:0]
static constexpr uint8_t REG8_NUM_MSB_MASK = 0x3Fu; // NUM[17:12]
static constexpr uint8_t REGA_NUM_LSB_SHIFT = 4u;   // NUM[3:0] -> bits[7:4]
static constexpr uint8_t REGA_NUM_LSB_MASK = 0xF0u;

// REGB bits
static constexpr uint8_t REGB_OD_MASK = 0x07u;

// REGD bits (charge pump control)
static constexpr uint8_t REGD_CPCHI = (1u << 7);
static constexpr uint8_t REGD_CPCLO = (1u << 6);
static constexpr uint8_t REGD_CPMID = (1u << 5);
static constexpr uint8_t REGD_CPINV = (1u << 4);
static constexpr uint8_t REGD_CPWIDE = (1u << 3);
static constexpr uint8_t REGD_CPRST = (1u << 2);
static constexpr uint8_t REGD_CPUP  = (1u << 1);
static constexpr uint8_t REGD_CPDN  = (1u << 0);

// Ranges from datasheet / Linduino helpers.
static constexpr uint8_t RD_MIN = 1;
static constexpr uint8_t RD_MAX = 31;
static constexpr uint16_t ND_MIN = 32;
static constexpr uint16_t ND_MAX = 1023;
static constexpr uint32_t NUM_MAX = MODULUS - 1;
static constexpr uint8_t OD_MIN = 1;
static constexpr uint8_t OD_MAX = 6;

}}}}} // namespace uhd::usrp::dboard::db_kintex7sdr::ltc6948

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP
