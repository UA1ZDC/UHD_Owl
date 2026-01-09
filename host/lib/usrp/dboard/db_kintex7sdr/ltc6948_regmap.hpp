// ==============================
// File: ltc6948_regmap.hpp
// ==============================
#ifndef DB_KINTEX7SDR_LTC6948_REGMAP_HPP
#define DB_KINTEX7SDR_LTC6948_REGMAP_HPP

#include <algorithm>
#include <cstdint>
#include <cmath>
#include <functional>

namespace uhd { namespace usrp { namespace dboard { namespace db_kintex7sdr {
namespace ltc6948 {

/***********************************************************************
 * LTC6948 SPI protocol (MODE0, MSB-first):
 *  1st byte: [7:1]=ADDR (0x00..0x0E), [0]=R/W (1=read, 0=write)
 *  2nd byte: DATA (8-bit)
 *
 * For read: send addr byte (R/W=1) then dummy; data is returned in 2nd byte.
 **********************************************************************/
using spi_xfer16_fn = std::function<uint16_t(uint16_t /*tx16*/)>;

static constexpr inline uint8_t addr_wr(uint8_t a) { return uint8_t((a << 1) | 0u); }
static constexpr inline uint8_t addr_rd(uint8_t a) { return uint8_t((a << 1) | 1u); }

// Registers we touch (Table 15)
enum : uint8_t {
    REG_00 = 0x00,
    REG_01 = 0x01,
    REG_02 = 0x02,
    REG_03 = 0x03,
    REG_05 = 0x05,
    REG_06 = 0x06,
    REG_07 = 0x07,
    REG_08 = 0x08,
};

// REG_00 bits: PDALL PDVCO PRST ENLO ENO ENFILT ENCP ENPFD
enum : uint8_t {
    R00_ENLO   = (1u << 4),
    R00_ENO    = (1u << 3),
    R00_ENFILT = (1u << 2),
    R00_ENCP   = (1u << 1),
    R00_ENPFD  = (1u << 0),
};

// REG_08 bits: ABP ... (we rely on defaults, but keep constant here if needed)
enum : uint8_t {
    R08_ABP = (1u << 7),
};

class ltc6948_iface
{
public:
    explicit ltc6948_iface(spi_xfer16_fn xfer16) : _xfer16(std::move(xfer16)) {}

    void write_reg(uint8_t addr, uint8_t data)
    {
        const uint16_t tx = (uint16_t(addr_wr(addr)) << 8) | uint16_t(data);
        (void)_xfer16(tx);
    }

    uint8_t read_reg(uint8_t addr)
    {
        const uint16_t tx = (uint16_t(addr_rd(addr)) << 8) | 0x00u;
        const uint16_t rx = _xfer16(tx);
        return uint8_t(rx & 0xFFu);
    }

    void power_up()
    {
        // Enable blocks needed for LO generation. Keep PDALL/PDVCO/PRST = 0.
        const uint8_t v = uint8_t(R00_ENLO | R00_ENO | R00_ENFILT | R00_ENCP | R00_ENPFD);
        write_reg(REG_00, v);
    }

    // Set RFOUT frequency using fractional-N:
    // fRFOUT = (fREF / R) * (N + NUM/2^18) / O
    //
    // Notes:
    // - Uses power-of-two output divider: O = 2^OD (OD in REG_05[7:5])
    // - Programs RD = R-1 into REG_01[4:0]
    // - Programs ND = N-1 into REG_02/REG_03
    // - Programs NUM into REG_03/REG_06/REG_07
    //
    // Returns the actual frequency (Hz) based on quantized NUM.
    double set_frequency(double fout_hz, double fref_hz)
    {
        // Conservative VCO range from LTC6948 family datasheet
        const double vco_min = 0.37e9;
        const double vco_max = 6.39e9;

        // Choose smallest output divider (O) that brings VCO into range.
        int od = 0;
        double vco = fout_hz;
        for (int cand_od = 0; cand_od <= 7; cand_od++) {
            const double cand_vco = fout_hz * double(1u << cand_od);
            if (cand_vco >= vco_min && cand_vco <= vco_max) {
                od = cand_od;
                vco = cand_vco;
                break;
            }
        }

        // PFD frequency limit (conservative); choose smallest R so fpfd <= max.
        const double fpfd_max = 100e6;
        int R = int(std::ceil(fref_hz / fpfd_max));
        R = std::max(1, std::min(32, R));
        double fpfd = fref_hz / double(R);

        // Compute N and NUM (18-bit)
        auto compute = [&](int R_in, int od_in) {
            const double fpfd2 = fref_hz / double(R_in);
            const double vco2 = fout_hz * double(1u << od_in);
            double n_real = vco2 / fpfd2;
            int N = int(std::floor(n_real));
            double frac = n_real - double(N);
            int NUM = int(std::lround(frac * double(1u << 18)));

            if (NUM >= (1u << 18)) { // carry
                NUM = 0;
                N += 1;
            }
            return std::tuple<int,int,double>(N, NUM, fpfd2);
        };

        int N = 0, NUM = 0;
        std::tie(N, NUM, fpfd) = compute(R, od);

        // Keep N in 10-bit range (1..1024-ish). If out, adjust R within [1..32].
        // ND register stores (N-1) in 10 bits -> N must be 1..1024.
        while ((N < 1 || N > 1024) && (R >= 1 && R <= 32)) {
            if (N > 1024 && R > 1) {
                R -= 1; // increase fpfd -> smaller N
            } else if (N < 1 && R < 32) {
                R += 1; // decrease fpfd -> bigger N
            } else {
                break;
            }
            std::tie(N, NUM, fpfd) = compute(R, od);
        }

        // Program dividers
        const int RD = std::max(0, std::min(31, R - 1));
        const int ND = std::max(0, std::min(1023, N - 1));
        const int OD = std::max(0, std::min(7, od));
        const int NUM18 = std::max(0, std::min((1 << 18) - 1, NUM));

        // RD -> REG_01[4:0]
        uint8_t r01 = read_reg(REG_01);
        r01 = uint8_t((r01 & 0xE0u) | uint8_t(RD & 0x1Fu));
        write_reg(REG_01, r01);

        // OD -> REG_05[7:5]
        uint8_t r05 = 0;
        r05 = uint8_t((OD & 0x7) << 5);
        write_reg(REG_05, r05);

        // ND -> REG_02 (ND[9:2]) and REG_03[7:6] (ND[1:0])
        const uint8_t nd_hi = uint8_t((ND >> 2) & 0xFF);
        uint8_t r03 = read_reg(REG_03);
        r03 = uint8_t((r03 & 0x3Fu) | uint8_t((ND & 0x3) << 6));
        write_reg(REG_02, nd_hi);
        write_reg(REG_03, r03);

        // NUM -> REG_03[5:0] (NUM[17:12]), REG_06 (NUM[11:4]), REG_07[7:4] (NUM[3:0])
        r03 = uint8_t((r03 & 0xC0u) | uint8_t((NUM18 >> 12) & 0x3Fu));
        write_reg(REG_03, r03);

        const uint8_t r06 = uint8_t((NUM18 >> 4) & 0xFFu);
        write_reg(REG_06, r06);

        uint8_t r07 = read_reg(REG_07);
        r07 = uint8_t((r07 & 0x0Fu) | uint8_t((NUM18 & 0x0Fu) << 4));
        write_reg(REG_07, r07);

        // Compute actual fout
        const double actual = (fref_hz / double(R)) * (double(N) + (double(NUM18) / double(1u << 18))) / double(1u << od);
        return actual;
    }

private:
    spi_xfer16_fn _xfer16;
};

} // namespace ltc6948
}}}}} // namespace uhd::usrp::dboard::db_kintex7sdr

#endif // DB_KINTEX7SDR_LTC6948_REGMAP_HPP
