#ifndef UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC5594_REGMAP_HPP
#define UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC5594_REGMAP_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr { namespace ltc5594 {

/***********************************************************************
 * LTC5594 SPI (16-bit)
 *  - First byte: MSB is R/W (1=read), lower 7 bits = address
 *  - Second byte: data (or dummy for reads)
 **********************************************************************/
static inline uint16_t make_addr_byte_rd(uint8_t addr7)
{
    return uint16_t(0x80u | (addr7 & 0x7Fu));
}
static inline uint16_t make_addr_byte_wr(uint8_t addr7)
{
    return uint16_t(addr7 & 0x7Fu);
}

static inline uint16_t make_word_wr(uint8_t addr7, uint8_t data)
{
    return uint16_t(make_addr_byte_wr(addr7) << 8) | data;
}

static inline uint16_t make_word_rd(uint8_t addr7)
{
    return uint16_t(make_addr_byte_rd(addr7) << 8); // data byte = 0
}

static inline uint8_t rx_data_byte(uint16_t rx_word)
{
    return uint8_t(rx_word & 0xFFu);
}

/***********************************************************************
 * Register address map (0x00..0x17)
 * Verified against datasheet Table 9 (Serial Port Register Contents).
 **********************************************************************/
enum reg_t : uint8_t {
    REG_IM3QY        = 0x00,
    REG_IM3QX        = 0x01,
    REG_IM3IY        = 0x02,
    REG_IM3IX        = 0x03,
    REG_IM2QX        = 0x04,
    REG_IM2IX        = 0x05,
    REG_HD3QY        = 0x06,
    REG_HD3QX        = 0x07,
    REG_HD3IY        = 0x08,
    REG_HD3IX        = 0x09,
    REG_HD2QY        = 0x0A,
    REG_HD2QX        = 0x0B,
    REG_HD2IY        = 0x0C,
    REG_HD2IX        = 0x0D,

    REG_DCOI         = 0x0E,
    REG_DCOQ         = 0x0F,

    REG_IP3IC        = 0x10,
    REG_GERR_IP3CC   = 0x11,

    REG_LVCM_CF1     = 0x12, // [7:5]=LVCM, [4:0]=CF1
    REG_BAND_LF1_CF2 = 0x13, // [7]=BAND, [6:5]=LF1, [4:0]=CF2

    REG_PHA_8_1      = 0x14, // [7:0] = PHA[8:1]
    REG_PHA0_MISC    = 0x15, // [7]   = PHA[0], other bits = AMP settings

    REG_BCTL         = 0x16,
    REG_CHIPID       = 0x17
};

static constexpr std::size_t NUM_REGS = 0x18;

// Backward-compatible aliases (older local names used in the project)
static constexpr uint8_t REG_DCO  = REG_LVCM_CF1;
static constexpr uint8_t REG_IPC  = REG_BAND_LF1_CF2;
static constexpr uint8_t REG_PHA  = REG_PHA_8_1;
static constexpr uint8_t REG_PHAI = REG_PHA0_MISC;

/***********************************************************************
 * REG_LVCM_CF1 (0x12)
 **********************************************************************/
static constexpr uint8_t REG12_LVCM_MASK  = 0xE0u; // [7:5]
static constexpr uint8_t REG12_CF1_MASK   = 0x1Fu; // [4:0]
static constexpr uint8_t REG12_LVCM_SHIFT = 5u;

static inline uint8_t pack_reg12(uint8_t lvc_m, uint8_t cf1)
{
    return static_cast<uint8_t>(((lvc_m & 0x07u) << REG12_LVCM_SHIFT) | (cf1 & REG12_CF1_MASK));
}

/***********************************************************************
 * REG_BAND_LF1_CF2 (0x13)
 **********************************************************************/
static constexpr uint8_t REG13_BAND       = 0x80u; // bit7
static constexpr uint8_t REG13_LF1_MASK   = 0x60u; // [6:5]
static constexpr uint8_t REG13_CF2_MASK   = 0x1Fu; // [4:0]
static constexpr uint8_t REG13_LF1_SHIFT  = 5u;

static inline uint8_t pack_reg13(bool band, uint8_t lf1, uint8_t cf2)
{
    const uint8_t b = band ? REG13_BAND : 0u;
    return static_cast<uint8_t>(b | ((lf1 & 0x03u) << REG13_LF1_SHIFT) | (cf2 & REG13_CF2_MASK));
}

/***********************************************************************
 * REG_GERR_IP3CC (0x11)
 **********************************************************************/
static constexpr uint8_t REG11_GERR_MASK   = 0xFCu; // [7:2]
static constexpr uint8_t REG11_GERR_SHIFT  = 2u;
static constexpr uint8_t REG11_IP3CC_MASK  = 0x03u; // [1:0]

static inline uint8_t pack_reg11_gerr(uint8_t old_reg11, uint8_t gerr_6b)
{
    return static_cast<uint8_t>((old_reg11 & REG11_IP3CC_MASK)
        | ((gerr_6b & 0x3Fu) << REG11_GERR_SHIFT));
}

static inline uint8_t unpack_reg11_gerr(uint8_t reg11)
{
    return static_cast<uint8_t>((reg11 & REG11_GERR_MASK) >> REG11_GERR_SHIFT);
}

/***********************************************************************
 * REG_PHA_8_1 (0x14) and REG_PHA0_MISC (0x15)
 *  - 0x14 stores PHA[8:1]
 *  - 0x15[7] stores PHA[0]; other bits in 0x15 are amplifier settings
 **********************************************************************/
static constexpr uint8_t REG15_PHA0_MASK = 0x80u;

static inline uint8_t pack_reg15_pha0(uint8_t old_reg15, uint8_t pha0)
{
    return static_cast<uint8_t>((old_reg15 & ~REG15_PHA0_MASK) | (pha0 ? REG15_PHA0_MASK : 0u));
}

static inline void pack_pha_9b(uint16_t pha_code_9b, uint8_t& reg14_out, uint8_t old_reg15, uint8_t& reg15_out)
{
    pha_code_9b &= 0x01FFu;
    reg14_out = static_cast<uint8_t>((pha_code_9b >> 1) & 0xFFu);
    reg15_out = pack_reg15_pha0(old_reg15, static_cast<uint8_t>(pha_code_9b & 0x1u));
}

/***********************************************************************
 * REG_BCTL (0x16) — block enables + soft reset
 **********************************************************************/
namespace bctl {
static constexpr uint8_t BIT_EDEM = (1u << 7);
static constexpr uint8_t BIT_EDC  = (1u << 6);
static constexpr uint8_t BIT_EADJ = (1u << 5);
static constexpr uint8_t BIT_EAMP = (1u << 4);
static constexpr uint8_t BIT_SRST = (1u << 3);

static constexpr uint8_t ENABLE_ALL = static_cast<uint8_t>(BIT_EDEM | BIT_EDC | BIT_EADJ | BIT_EAMP);
} // namespace bctl

static inline uint8_t pack_bctl(uint8_t enable_mask, bool srst)
{
    return static_cast<uint8_t>((enable_mask & 0xF0u) | (srst ? bctl::BIT_SRST : 0u));
}

/***********************************************************************
 * LO matching table helper (datasheet Table 2: Register Settings for
 * Single-Ended LO Matching)
 **********************************************************************/
struct lo_match_entry_t {
    uint32_t f_lo_min_hz;
    uint32_t f_lo_max_hz_excl; // exclusive upper bound
    bool band;
    uint8_t cf1;
    uint8_t lf1;
    uint8_t cf2;
};

// Board default LO bias setting (LVCM). Datasheet default is 0x02.
static constexpr uint8_t DEFAULT_LVCM = 0x02u;

// Datasheet default matching values
static constexpr bool    DEFAULT_BAND = true;
static constexpr uint8_t DEFAULT_CF1  = 8u;
static constexpr uint8_t DEFAULT_LF1  = 3u;
static constexpr uint8_t DEFAULT_CF2  = 3u;

static constexpr std::array<lo_match_entry_t, 16> LO_MATCH_TABLE_SINGLE_ENDED = {{
    //  f_lo_min   f_lo_max_excl  BAND  CF1 LF1 CF2
    { 300000000u,  339000000u,    false, 31u, 3u, 31u},
    { 339000000u,  398000000u,    false, 21u, 3u, 24u},
    { 398000000u,  419000000u,    false, 14u, 3u, 23u},
    { 419000000u,  556000000u,    false, 17u, 2u, 31u},
    { 556000000u,  625000000u,    false, 10u, 2u, 23u},
    { 625000000u,  801000000u,    false, 15u, 1u, 31u},
    { 801000000u,  831000000u,    false, 14u, 1u, 27u},
    { 831000000u, 1046000000u,    false,  8u, 1u, 21u},

    {1046000000u, 1242000000u,    true,  31u, 3u, 31u},
    {1242000000u, 1411000000u,    true,  21u, 3u, 28u},
    {1411000000u, 1696000000u,    true,  17u, 2u, 26u},
    {1696000000u, 2070000000u,    true,  15u, 1u, 31u},
    {2070000000u, 2470000000u,    true,   8u, 1u, 21u},

    {2470000000u, 2980000000u,    true,   2u, 1u, 10u},
    {2980000000u, 3500000000u,    true,   1u, 0u, 19u},
    {3500000000u, 9000000000u,    true,   0u, 0u,  0u}
}};

enum class lo_drive_mode_t {
    single_ended,
    differential
};

struct lo_match_result_t {
    bool valid{false};
    std::size_t table_index{0};
    uint8_t reg12{0};
    uint8_t reg13{0};
};

static inline lo_match_result_t resolve_lo_match_single_ended(double f_lo_hz)
{
    // Clamp to uint32 range for comparisons
    const double f = f_lo_hz;
    const uint32_t fhz = (f <= 0.0) ? 0u
        : (f >= double(std::numeric_limits<uint32_t>::max())
            ? std::numeric_limits<uint32_t>::max()
            : uint32_t(f));

    for (std::size_t i = 0; i < LO_MATCH_TABLE_SINGLE_ENDED.size(); i++) {
        const auto& e = LO_MATCH_TABLE_SINGLE_ENDED[i];
        if (fhz >= e.f_lo_min_hz && fhz < e.f_lo_max_hz_excl) {
            lo_match_result_t r;
            r.valid = true;
            r.table_index = i;
            r.reg12 = pack_reg12(DEFAULT_LVCM, e.cf1);
            r.reg13 = pack_reg13(e.band, e.lf1, e.cf2);
            return r;
        }
    }
    return lo_match_result_t{};
}

static inline lo_match_result_t resolve_lo_match(double f_lo_hz, lo_drive_mode_t mode)
{
    // Datasheet provides explicit LO matching settings for single-ended LO drive.
    // For differential LO drive, we currently reuse the same table as a practical
    // starting point. If lab characterization shows different optimal settings,
    // add a dedicated differential table and switch on "mode".
    (void)mode;
    return resolve_lo_match_single_ended(f_lo_hz);
}


}}}}} // namespace uhd::usrp::dboard::db_kintex7sdr::ltc5594

#endif // UHD_USRP_DBOARD_DB_KINTEX7SDR_LTC5594_REGMAP_HPP
