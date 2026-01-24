#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

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
 * Register addresses
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

/***********************************************************************
 * REG0 (status) bits
 **********************************************************************/
static constexpr uint8_t REG0_TLO    = (1u << 0);
static constexpr uint8_t REG0_THI    = (1u << 1);
static constexpr uint8_t REG0_LOCK   = (1u << 2);
static constexpr uint8_t REG0_ALCLO  = (1u << 3);
static constexpr uint8_t REG0_ALCHI  = (1u << 4);
static constexpr uint8_t REG0_UNLOCK = (1u << 5);

static inline std::string status_to_string(uint8_t reg0)
{
    std::string s;
    s.reserve(64);
    s += (reg0 & REG0_LOCK) ? "LOCK " : "---- ";
    if (reg0 & REG0_UNLOCK) s += "UNLOCK ";
    if (reg0 & REG0_TLO)    s += "TLO ";
    if (reg0 & REG0_THI)    s += "THI ";
    if (reg0 & REG0_ALCLO)  s += "ALCLO ";
    if (reg0 & REG0_ALCHI)  s += "ALCHI ";
    if (!s.empty() && s.back() == ' ') s.pop_back();
    return s;
}

/***********************************************************************
 * Bitfields / masks
 **********************************************************************/
// REG2 bits (power, mute, reset)
static constexpr uint8_t REG2_PDALL = (1u << 7);
static constexpr uint8_t REG2_PDPLL = (1u << 6);
static constexpr uint8_t REG2_PDVCO = (1u << 5);
static constexpr uint8_t REG2_PDOUT = (1u << 4);
static constexpr uint8_t REG2_PDFN  = (1u << 3);
static constexpr uint8_t REG2_MTCAL = (1u << 2);
static constexpr uint8_t REG2_OMUTE = (1u << 1);
static constexpr uint8_t REG2_POR   = (1u << 0);

// REG3 bits (we only touch INTN/DITHEN; other bits preserved from base value)
static constexpr uint8_t REG3_INTN    = (1u << 0); // 1=int-N, 0=frac-N
static constexpr uint8_t REG3_DITHEN  = (1u << 1); // 1=dither enabled
static constexpr uint8_t REG3_DEFAULT = DEFAULT_REGS[REG3];

// REG4 bits
static constexpr uint8_t REG4_BD_MASK   = 0xF0u;  // BD[3:0] -> bits[7:4]
static constexpr uint8_t REG4_CPLE      = (1u << 3);
static constexpr uint8_t REG4_LDOEN     = (1u << 2);
static constexpr uint8_t REG4_LDOV_MASK = 0x03u;

// REG6 packing
static constexpr uint8_t REG6_RD_SHIFT      = 3u;    // RD[4:0] -> bits[7:3]
static constexpr uint8_t REG6_RD_MASK       = 0xF8u;
static constexpr uint8_t REG6_ND_MSB_MASK   = 0x03u; // ND[9:8] -> bits[1:0]

// REG8/REGA packing for NUM[17:0]
static constexpr uint8_t REG8_NUM_MSB_MASK    = 0x3Fu; // NUM[17:12]
static constexpr uint8_t REGA_NUM_LSB_SHIFT   = 4u;    // NUM[3:0] -> bits[7:4]
static constexpr uint8_t REGA_NUM_LSB_MASK    = 0xF0u;

// REGB (output config) bits (BST/FILT/RFO/OD)
static constexpr uint8_t REGB_BST       = (1u << 7);
static constexpr uint8_t REGB_FILT_MASK = 0x60u; // FILT[1:0] -> bits[6:5]
static constexpr uint8_t REGB_RFO_MASK  = 0x18u; // RFO[1:0]  -> bits[4:3]
static constexpr uint8_t REGB_OD_MASK   = 0x07u; // OD[2:0]   -> bits[2:0]

// REGD bits (charge pump control)
static constexpr uint8_t REGD_CPCHI  = (1u << 7);
static constexpr uint8_t REGD_CPCLO  = (1u << 6);
static constexpr uint8_t REGD_CPMID  = (1u << 5);
static constexpr uint8_t REGD_CPINV  = (1u << 4);
static constexpr uint8_t REGD_CPWIDE = (1u << 3);
static constexpr uint8_t REGD_CPRST  = (1u << 2);
static constexpr uint8_t REGD_CPUP   = (1u << 1);
static constexpr uint8_t REGD_CPDN   = (1u << 0);

// Ranges
static constexpr uint8_t  RD_MIN  = 1;
static constexpr uint8_t  RD_MAX  = 31;
static constexpr uint16_t ND_MIN  = 32;
static constexpr uint16_t ND_MAX  = 1023;
static constexpr uint32_t NUM_MAX = MODULUS - 1;
static constexpr uint8_t  OD_MIN  = 1;
static constexpr uint8_t  OD_MAX  = 6;

/***********************************************************************
 * Human-readable enums
 **********************************************************************/
// REG3
enum class mode_t : uint8_t { fractional = 0, integer = 1 };
enum class dither_t : uint8_t { off = 0, on = 1 };

static inline uint8_t reg3_set_mode(uint8_t base, mode_t m)
{
    return (m == mode_t::integer) ? uint8_t(base | REG3_INTN) : uint8_t(base & ~REG3_INTN);
}
static inline uint8_t reg3_set_dither(uint8_t base, dither_t d)
{
    return (d == dither_t::on) ? uint8_t(base | REG3_DITHEN) : uint8_t(base & ~REG3_DITHEN);
}
static inline uint8_t pack_reg3(uint8_t base, mode_t m, dither_t d)
{
    return reg3_set_dither(reg3_set_mode(base, m), d);
}

// REG4
enum class ldov_t : uint8_t { v0 = 0, v1 = 1, v2 = 2, v3 = 3 };

static inline uint8_t reg4_set_ldov(uint8_t base, ldov_t ldov)
{
    return static_cast<uint8_t>((base & ~REG4_LDOV_MASK) | (static_cast<uint8_t>(ldov) & REG4_LDOV_MASK));
}
static inline uint8_t reg4_set_ldoen(uint8_t base, bool en)
{
    return en ? static_cast<uint8_t>(base | REG4_LDOEN)
              : static_cast<uint8_t>(base & ~REG4_LDOEN);
}
static inline uint8_t reg4_set_cple(uint8_t base, bool en)
{
    return en ? static_cast<uint8_t>(base | REG4_CPLE)
              : static_cast<uint8_t>(base & ~REG4_CPLE);
}
static inline uint8_t reg4_set_bd(uint8_t base, uint8_t bd)
{
    return static_cast<uint8_t>((base & ~REG4_BD_MASK) | ((bd << 4) & REG4_BD_MASK));
}
static inline uint8_t pack_reg4(uint8_t bd, bool ldoen, ldov_t ldov, bool cple)
{
    uint8_t v = 0;
    v = reg4_set_bd(v, bd);
    v = reg4_set_ldoen(v, ldoen);
    v = reg4_set_ldov(v, ldov);
    v = reg4_set_cple(v, cple);
    return v;
}

// REGB fields
enum class filt_t : uint8_t { f0 = 0, f1 = 1, f2 = 2, f3 = 3 };
enum class rfo_t  : uint8_t { lvl0 = 0, lvl1 = 1, lvl2 = 2, lvl3 = 3 };

static inline uint8_t filt_bits(filt_t f) { return static_cast<uint8_t>(f) & 0x03u; }
static inline uint8_t rfo_bits(rfo_t r)   { return static_cast<uint8_t>(r) & 0x03u; }

static inline filt_t get_filt(uint8_t regb)
{
    return static_cast<filt_t>((regb & REGB_FILT_MASK) >> 5);
}
static inline rfo_t get_rfo(uint8_t regb)
{
    return static_cast<rfo_t>((regb & REGB_RFO_MASK) >> 3);
}
static inline bool get_bst(uint8_t regb) { return (regb & REGB_BST) != 0; }
static inline uint8_t get_od(uint8_t regb) { return regb & REGB_OD_MASK; }

/***********************************************************************
 * Tiny pack/unpack helpers (avoid magic numbers)
 **********************************************************************/
static inline uint8_t pack_reg6(uint8_t rd, uint16_t nd)
{
    return static_cast<uint8_t>(((rd << REG6_RD_SHIFT) & REG6_RD_MASK) | ((nd >> 8) & REG6_ND_MSB_MASK));
}

static inline uint8_t nd_msb_from_reg6(uint8_t reg6) { return static_cast<uint8_t>(reg6 & REG6_ND_MSB_MASK); }
static inline uint8_t rd_from_reg6(uint8_t reg6) { return static_cast<uint8_t>((reg6 & REG6_RD_MASK) >> REG6_RD_SHIFT); }

static inline uint32_t pack_num(uint8_t reg8, uint8_t reg9, uint8_t rega)
{
    const uint32_t msb = (uint32_t(reg8) & REG8_NUM_MSB_MASK) << 12;
    const uint32_t mid = uint32_t(reg9) << 4;
    const uint32_t lsb = (uint32_t(rega) & REGA_NUM_LSB_MASK) >> REGA_NUM_LSB_SHIFT;
    return msb | mid | lsb;
}

static inline void num_to_regs(uint32_t num, uint8_t& reg8, uint8_t& reg9, uint8_t& rega_num_nibble)
{
    num &= NUM_MAX;
    reg8 = static_cast<uint8_t>((num >> 12) & REG8_NUM_MSB_MASK);                // NUM[17:12]
    reg9 = static_cast<uint8_t>((num >> 4) & 0xFFu);                             // NUM[11:4]
    rega_num_nibble = static_cast<uint8_t>((num & 0x0Fu) << REGA_NUM_LSB_SHIFT); // NUM[3:0] into [7:4]
}

static inline uint8_t regb_set_od(uint8_t base_regb, uint8_t od)
{
    return static_cast<uint8_t>((base_regb & ~REGB_OD_MASK) | (od & REGB_OD_MASK));
}

static inline uint8_t pack_regb(uint8_t od, bool bst, uint8_t filt, uint8_t rfo)
{
    uint8_t v = 0;
    if (bst) v |= REGB_BST;
    v |= static_cast<uint8_t>((filt << 5) & REGB_FILT_MASK);
    v |= static_cast<uint8_t>((rfo  << 3) & REGB_RFO_MASK);
    v |= static_cast<uint8_t>(od & REGB_OD_MASK);
    return v;
}

static inline uint8_t pack_regb(uint8_t od, bool bst, filt_t filt, rfo_t rfo)
{
    return pack_regb(od, bst, filt_bits(filt), rfo_bits(rfo));
}

static inline uint8_t regd_cp_normal_clamps_on()
{
    // CPCHI=1, CPCLO=1, all other CP control bits 0 (CPINV=0, CPRST=0, CPMID=0 ...)
    return static_cast<uint8_t>(REGD_CPCHI | REGD_CPCLO);
}

}}}}} // namespace uhd::usrp::dboard::db_kintex7sdr::ltc6948

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC6948_REGMAP_HPP
