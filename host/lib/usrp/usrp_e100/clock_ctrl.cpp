//
// Copyright 2010-2011 Ettus Research LLC
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include "clock_ctrl.hpp"
#include "ad9522_regs.hpp"
#include <uhd/utils/assert.hpp>
#include <boost/cstdint.hpp>
#include "usrp_e100_regs.hpp" //spi slave constants
#include <boost/assign/list_of.hpp>
#include <boost/foreach.hpp>
#include <boost/format.hpp>
#include <boost/operators.hpp>
#include <boost/math/common_factor_rt.hpp> //gcd
#include <algorithm>
#include <utility>
#include <iostream>

using namespace uhd;

/***********************************************************************
 * Constants
 **********************************************************************/
static const bool ENABLE_THE_TEST_OUT = false;
static const double REFERENCE_INPUT_RATE = 10e6;
static const double DEFAULT_OUTPUT_RATE = 64e6;

/***********************************************************************
 * Helpers
 **********************************************************************/
template <typename div_type, typename bypass_type> static void set_clock_divider(
    size_t divider, div_type &low, div_type &high, bypass_type &bypass
){
    high = divider/2 - 1;
    low = divider - high - 2;
    bypass = (divider == 1)? 1 : 0;
}

/***********************************************************************
 * Clock rate calculation stuff:
 *   Using the internal VCO between 1400 and 1800 MHz
 **********************************************************************/
struct clock_settings_type : boost::totally_ordered<clock_settings_type>{
    size_t ref_clock_doubler, r_counter, a_counter, b_counter, prescaler, vco_divider, chan_divider;
    size_t get_n_counter(void) const{return prescaler * b_counter + a_counter;}
    double get_ref_rate(void) const{return REFERENCE_INPUT_RATE * ref_clock_doubler;}
    double get_vco_rate(void) const{return get_ref_rate()/r_counter * get_n_counter();}
    double get_chan_rate(void) const{return get_vco_rate()/vco_divider;}
    double get_out_rate(void) const{return get_chan_rate()/chan_divider;}
    std::string to_pp_string(void) const{
        return str(boost::format(
            "  r_counter: %d\n"
            "  a_counter: %d\n"
            "  b_counter: %d\n"
            "  prescaler: %d\n"
            "  vco_divider: %d\n"
            "  chan_divider: %d\n"
            "  vco_rate: %fMHz\n"
            "  chan_rate: %fMHz\n"
            "  out_rate: %fMHz\n"
            )
            % r_counter
            % a_counter
            % b_counter
            % prescaler
            % vco_divider
            % chan_divider
            % (get_vco_rate()/1e6)
            % (get_chan_rate()/1e6)
            % (get_out_rate()/1e6)
        );
    }
};

bool operator<(const clock_settings_type &lhs, const clock_settings_type &rhs){
    if (lhs.get_out_rate() != rhs.get_out_rate()) //sort small to large out rates
        return lhs.get_out_rate() < rhs.get_out_rate();

    if (lhs.r_counter != rhs.r_counter) //sort small to large r dividers
        return lhs.r_counter < rhs.r_counter;

    if (lhs.get_vco_rate() != rhs.get_vco_rate()) //sort large to small vco rates
        return lhs.get_vco_rate() > rhs.get_vco_rate();

    return false; //whatever case
}

static std::vector<clock_settings_type> _get_clock_settings(void){
    std::vector<clock_settings_type> clock_settings;

    clock_settings_type cs;
    cs.ref_clock_doubler = 2; //always doubling
    cs.prescaler = 8; //set to 8 when input is under 2400 MHz

    for (cs.r_counter = 1; cs.r_counter <= 3; cs.r_counter++){
    for (cs.b_counter = 3; cs.b_counter <= 10; cs.b_counter++){
    for (cs.a_counter = 0; cs.a_counter <= 10; cs.a_counter++){
    for (cs.vco_divider = 2; cs.vco_divider <= 6; cs.vco_divider++){
    for (cs.chan_divider = 1; cs.chan_divider <= 32; cs.chan_divider++){
        if (cs.get_vco_rate() > 1800e6) continue;
        if (cs.get_vco_rate() < 1400e6) continue;
        if (cs.get_out_rate() < 32e6) continue; //lowest we allow for GPMC interface
        clock_settings.push_back(cs);
    }}}}}
}

/***********************************************************************
 * Constants
 **********************************************************************/
static const bool enable_test_clock = true;

/***********************************************************************
 * Clock Control Implementation
 **********************************************************************/
class usrp_e100_clock_ctrl_impl : public usrp_e100_clock_ctrl{
public:
    usrp_e100_clock_ctrl_impl(usrp_e100_iface::sptr iface){
        _iface = iface;
        _chan_rate = 0.0;
        _out_rate = 0.0;

        //init the clock gen registers
        //Note: out0 should already be clocking the FPGA or this isnt going to work
        _ad9522_regs.sdo_active = ad9522_regs_t::SDO_ACTIVE_SDO_SDIO;
        _ad9522_regs.enable_clock_doubler = 1; //enable ref clock doubler
        _ad9522_regs.enb_stat_eeprom_at_stat_pin = 0; //use status pin
        _ad9522_regs.status_pin_control = 0x2; //r divider
        _ad9522_regs.ld_pin_control = 0x00; //dld
        _ad9522_regs.refmon_pin_control = 0x12; //show ref2
        _ad9522_regs.lock_detect_counter = ad9522_regs_t::LOCK_DETECT_COUNTER_255CYC;

        this->use_internal_ref();

        this->set_fpga_clock_rate(DEFAULT_OUTPUT_RATE); //initialize to something

        this->enable_test_clock(ENABLE_THE_TEST_OUT);
        this->enable_rx_dboard_clock(false);
        this->enable_tx_dboard_clock(false);
    }

    ~usrp_e100_clock_ctrl_impl(void){
        this->enable_test_clock(ENABLE_THE_TEST_OUT);
        this->enable_rx_dboard_clock(false);
        this->enable_tx_dboard_clock(false);
    }

    /***********************************************************************
     * Clock rate control:
     *  - set clock rate w/ internal VCO
     *  - set clock rate w/ external VCXO
     **********************************************************************/
    void set_clock_settings_with_internal_vco(const clock_settings_type &cs){
        //set the rates to private variables so the implementation knows!
        _chan_rate = cs.get_chan_rate();
        _out_rate = cs.get_out_rate();

        _ad9522_regs.enable_clock_doubler = (cs.ref_clock_doubler == 2)? 1 : 0;

        _ad9522_regs.set_r_counter(cs.r_counter);
        _ad9522_regs.a_counter = cs.a_counter;
        _ad9522_regs.set_b_counter(cs.b_counter);
        UHD_ASSERT_THROW(cs.prescaler == 8); //assumes this below:
        _ad9522_regs.prescaler_p = ad9522_regs_t::PRESCALER_P_DIV8_9;

        _ad9522_regs.pll_power_down = ad9522_regs_t::PLL_POWER_DOWN_NORMAL;
        _ad9522_regs.cp_current = ad9522_regs_t::CP_CURRENT_1_2MA;

        _ad9522_regs.vco_calibration_now = 1; //calibrate it!
        _ad9522_regs.bypass_vco_divider = 0;
        switch(cs.vco_divider){
        case 1: _ad9522_regs.vco_divider = ad9522_regs_t::VCO_DIVIDER_DIV1; break;
        case 2: _ad9522_regs.vco_divider = ad9522_regs_t::VCO_DIVIDER_DIV2; break;
        case 3: _ad9522_regs.vco_divider = ad9522_regs_t::VCO_DIVIDER_DIV3; break;
        case 4: _ad9522_regs.vco_divider = ad9522_regs_t::VCO_DIVIDER_DIV4; break;
        case 5: _ad9522_regs.vco_divider = ad9522_regs_t::VCO_DIVIDER_DIV5; break;
        case 6: _ad9522_regs.vco_divider = ad9522_regs_t::VCO_DIVIDER_DIV6; break;
        }
        _ad9522_regs.select_vco_or_clock = ad9522_regs_t::SELECT_VCO_OR_CLOCK_VCO;

        //setup fpga master clock
        _ad9522_regs.out0_format = ad9522_regs_t::OUT0_FORMAT_LVDS;
        set_clock_divider(cs.chan_divider,
            _ad9522_regs.divider0_low_cycles,
            _ad9522_regs.divider0_high_cycles,
            _ad9522_regs.divider0_bypass
        );

        //setup codec clock
        _ad9522_regs.out3_format = ad9522_regs_t::OUT3_FORMAT_LVDS;
        set_clock_divider(cs.chan_divider,
            _ad9522_regs.divider1_low_cycles,
            _ad9522_regs.divider1_high_cycles,
            _ad9522_regs.divider1_bypass
        );

        this->send_all_regs();
    }

    void set_clock_settings_with_external_vcxo(double rate){
        //set the rates to private variables so the implementation knows!
        _chan_rate = rate;
        _out_rate = rate;

        _ad9522_regs.enable_clock_doubler = 1; //doubler always on
        const double ref_rate = REFERENCE_INPUT_RATE*2;

        //bypass prescaler such that N = B
        long gcd = boost::math::gcd(long(ref_rate), long(rate));
        _ad9522_regs.set_r_counter(int(ref_rate/gcd));
        _ad9522_regs.a_counter = 0;
        _ad9522_regs.set_b_counter(int(rate/gcd));
        _ad9522_regs.prescaler_p = ad9522_regs_t::PRESCALER_P_DIV1;

        //setup external vcxo
        _ad9522_regs.pll_power_down = ad9522_regs_t::PLL_POWER_DOWN_NORMAL;
        _ad9522_regs.cp_current = ad9522_regs_t::CP_CURRENT_1_2MA;
        _ad9522_regs.bypass_vco_divider = 1;
        _ad9522_regs.select_vco_or_clock = ad9522_regs_t::SELECT_VCO_OR_CLOCK_EXTERNAL;

        //setup fpga master clock
        _ad9522_regs.out0_format = ad9522_regs_t::OUT0_FORMAT_LVDS;
        _ad9522_regs.divider0_bypass = 1;

        //setup codec clock
        _ad9522_regs.out3_format = ad9522_regs_t::OUT3_FORMAT_LVDS;
        _ad9522_regs.divider1_bypass = 1;

        this->send_all_regs();
    }

    void set_fpga_clock_rate(double rate){
        if (_out_rate == rate) return;

        if (rate == 61.44e6){
            set_clock_settings_with_external_vcxo(rate);
        }
        else{
            BOOST_FOREACH(const clock_settings_type &cs, get_clock_settings()){
                //std::cout << cs.to_pp_string() << std::endl;
                if (rate != cs.get_out_rate()) continue;
                std::cout << "USRP-E100 clock control:" << std::endl << cs.to_pp_string() << std::endl;
                set_clock_settings_with_internal_vco(cs);
                return; //done here, exits loop
            }
            throw std::runtime_error(str(boost::format(
                "USRP-E100 clock control: could not find settings for clock rate %fMHz"
            ) % (rate/1e6)));
        }
    }

    double get_fpga_clock_rate(void){
        return this->_out_rate;
    }

    /***********************************************************************
     * Special test clock output
     **********************************************************************/
    void enable_test_clock(bool enb){
        //setup test clock (same divider as codec clock)
        _ad9522_regs.out4_format = ad9522_regs_t::OUT4_FORMAT_CMOS;
        _ad9522_regs.out4_cmos_configuration = (enb)?
            ad9522_regs_t::OUT4_CMOS_CONFIGURATION_A_ON :
            ad9522_regs_t::OUT4_CMOS_CONFIGURATION_OFF;
        this->send_reg(0x0F0);
        this->latch_regs();
    }

    /***********************************************************************
     * RX Dboard Clock Control (output 9, divider 3)
     **********************************************************************/
    void enable_rx_dboard_clock(bool enb){
        _ad9522_regs.out9_format = ad9522_regs_t::OUT9_FORMAT_CMOS;
        _ad9522_regs.out9_cmos_configuration = (enb)?
            ad9522_regs_t::OUT9_CMOS_CONFIGURATION_B_ON :
            ad9522_regs_t::OUT9_CMOS_CONFIGURATION_OFF;
        this->send_reg(0x0F9);
        this->latch_regs();
    }

    std::vector<double> get_rx_dboard_clock_rates(void){
        std::vector<double> rates;
        for(size_t div = 1; div <= 16+16; div++)
            rates.push_back(this->_chan_rate/div);
        return rates;
    }

    void set_rx_dboard_clock_rate(double rate){
        assert_has(get_rx_dboard_clock_rates(), rate, "rx dboard clock rate");
        size_t divider = size_t(this->_chan_rate/rate);
        //set the divider registers
        set_clock_divider(divider,
            _ad9522_regs.divider3_low_cycles,
            _ad9522_regs.divider3_high_cycles,
            _ad9522_regs.divider3_bypass
        );
        this->send_reg(0x199);
        this->send_reg(0x19a);
        this->latch_regs();
    }

    /***********************************************************************
     * TX Dboard Clock Control (output 6, divider 2)
     **********************************************************************/
    void enable_tx_dboard_clock(bool enb){
        _ad9522_regs.out6_format = ad9522_regs_t::OUT6_FORMAT_CMOS;
        _ad9522_regs.out6_cmos_configuration = (enb)?
            ad9522_regs_t::OUT6_CMOS_CONFIGURATION_B_ON :
            ad9522_regs_t::OUT6_CMOS_CONFIGURATION_OFF;
        this->send_reg(0x0F6);
        this->latch_regs();
    }

    std::vector<double> get_tx_dboard_clock_rates(void){
        return get_rx_dboard_clock_rates(); //same master clock, same dividers...
    }

    void set_tx_dboard_clock_rate(double rate){
        assert_has(get_tx_dboard_clock_rates(), rate, "tx dboard clock rate");
        size_t divider = size_t(this->_chan_rate/rate);
        //set the divider registers
        set_clock_divider(divider,
            _ad9522_regs.divider2_low_cycles,
            _ad9522_regs.divider2_high_cycles,
            _ad9522_regs.divider2_bypass
        );
        this->send_reg(0x196);
        this->send_reg(0x197);
        this->latch_regs();
    }
    
    /***********************************************************************
     * Clock reference control
     **********************************************************************/
    void use_internal_ref(void) {
        _ad9522_regs.enable_ref2 = 1;
        _ad9522_regs.enable_ref1 = 0;
        _ad9522_regs.select_ref = ad9522_regs_t::SELECT_REF_REF2;
        _ad9522_regs.enb_auto_ref_switchover = ad9522_regs_t::ENB_AUTO_REF_SWITCHOVER_MANUAL;
        this->send_reg(0x01C);
    }
    
    void use_external_ref(void) {
        _ad9522_regs.enable_ref2 = 0;
        _ad9522_regs.enable_ref1 = 1;
        _ad9522_regs.select_ref = ad9522_regs_t::SELECT_REF_REF1;
        _ad9522_regs.enb_auto_ref_switchover = ad9522_regs_t::ENB_AUTO_REF_SWITCHOVER_MANUAL;
        this->send_reg(0x01C);
    }
    
    void use_auto_ref(void) {
        _ad9522_regs.enable_ref2 = 1;
        _ad9522_regs.enable_ref1 = 1;
        _ad9522_regs.select_ref = ad9522_regs_t::SELECT_REF_REF1;
        _ad9522_regs.enb_auto_ref_switchover = ad9522_regs_t::ENB_AUTO_REF_SWITCHOVER_AUTO;
    }

private:
    usrp_e100_iface::sptr _iface;
    ad9522_regs_t _ad9522_regs;
    double _out_rate; //rate at the fpga and codec
    double _chan_rate; //rate before final dividers

    void latch_regs(void){
        _ad9522_regs.io_update = 1;
        this->send_reg(0x232);
    }

    void send_reg(boost::uint16_t addr){
        boost::uint32_t reg = _ad9522_regs.get_write_reg(addr);
        //std::cout << "clock control write reg: " << std::hex << reg << std::endl;
        _iface->transact_spi(
            UE_SPI_SS_AD9522,
            spi_config_t::EDGE_RISE,
            reg, 24, false /*no rb*/
        );
    }

    void send_all_regs(void){
        //setup a list of register ranges to write
        typedef std::pair<boost::uint16_t, boost::uint16_t> range_t;
        static const std::vector<range_t> ranges = boost::assign::list_of
            (range_t(0x000, 0x000)) (range_t(0x010, 0x01F))
            (range_t(0x0F0, 0x0FD)) (range_t(0x190, 0x19B))
            (range_t(0x1E0, 0x1E1)) (range_t(0x230, 0x230))
        ;

        //write initial register values and latch/update
        BOOST_FOREACH(const range_t &range, ranges){
            for(boost::uint16_t addr = range.first; addr <= range.second; addr++){
                this->send_reg(addr);
            }
        }
        this->latch_regs();
    }
};

/***********************************************************************
 * Clock Control Make
 **********************************************************************/
usrp_e100_clock_ctrl::sptr usrp_e100_clock_ctrl::make(usrp_e100_iface::sptr iface){
    return sptr(new usrp_e100_clock_ctrl_impl(iface));
}
