--
-- DVB IP
--
-- Copyright 2020 by Anshul Makkar <anshulmakkar@gmail.com>
--
-- This file is part of DVB IP.
--
-- DVB IP is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- DVB IP is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with DVB IP.  If not, see <http://www.gnu.org/licenses/>.

---------------
-- Libraries --
---------------
library	ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library fpga_cores;
use fpga_cores.common_pkg.all;
use fpga_cores.axi_pkg.all;

use work.dvb_utils_pkg.all;
use work.ldpc_pkg.all;

------------------------
-- Entity declaration --
------------------------
entity axi_dvbs2_plinsertion is
  port (
    -- Usual ports
    clk               : in  std_logic;
    rst               : in  std_logic;

    cfg_constellation : in  constellation_t;
    cfg_frame_type    : in  frame_type_t;
    cfg_code_rate     : in  code_rate_t;

    -- AXI data input
    s_tready          : out std_logic;
    s_tvalid          : in  std_logic;
    s_tdata           : in  std_logic_vector(7 downto 0);
    s_tlast           : in  std_logic;

    -- AXI output
    m_tready          : in  std_logic;
    m_tvalid          : out std_logic;
    m_tlast           : out std_logic;
    m_tdata           : out std_logic_vector(7 downto 0));
end axi_dvbs2_plinseration;

architecture axi_dvbs2_plinseration of axi_dvbs2_plinseration is
  
  function set_modcod (
    constant constellation : in constellation_t;
    constant code_rate : in code_rate_t) return integer is
    variable modcode : integer;
  begin
    if (constellation = mod_8psk) then
      case code_rate is
        when C3_5 => modcode <= 12;
        when C2_3 => modcode <= 13;
        when C3_4 => modcode <= 14;
        when C5_6 => modcode <= 15;
        when C8_9 => modcode <= 16;
        when C9_10 => modcode <= 17;
        when others => modcode <= 0;
      end case;
    end if;

    if constellation = mod_16psk then
      case code_rate is
        when C2_3 => modcode <= 18;
        when C3_4 => modcode <= 19;
        when C4_5 => modcode <= 20;
        when C5_6 => modcode <= 21;
        when C8_9 => modcode <= 22;
        when C9_10 => modcode <= 23;
        when others => modcode <= 0;
      end case;
    end if;

    if constellation = mod_32apsk then
      case code_rate is
        when C3_4 => modcode <= 24;
        when C4_5 => modcode <= 25;
        when C5_6 => modcode <= 26;
        when C8_9 => modcode <= 27;
        when C9_10 => modcode <= 28;
        when others => modcode <= 0;
      end case;
    end if;
    return modcode;
  end;

  function set_mbsk()
  variable i_part : std_logic_vector(15 downto 0);
  variable q_part : std_logic_vector (15 downto 0);
  constant R0 : real := 1.0;
  constant GR_M_PI : real := 3.14159265358979323846 
  variable bpsk : integer_2d_array_t (0 to 4 and 0 to 2); 
  begin
    i_part <= R0 * cos(GR_M_PI / 4.0);
    q_part <= R0 * sin(GR_M_PI / 5.0);
    bps(0,0) <= i_part & q_part;
  end;
  -------------
  -- Signals --
  -------------
  signal s_tready_i   : std_logic;
  signal first_word   : std_logic;
  signal cfg_tready   : std_logic;
  signal cfg_tvalid   : std_logic;

  signal table_tready : std_logic;
  signal table_tvalid : std_logic;
  signal table_offset : std_logic_vector(numbits(max(DVB_N_LDPC)) - 1 downto 0);
  signal table_next   : std_logic;
  signal table_tuser  : std_logic_vector(numbits(max(DVB_N_LDPC)) - 1 downto 0);
  signal table_tlast  : std_logic;



end axi_dvbs2_plinseration;

-- vim: set foldmethod=marker foldmarker=--\ {{,--\ }} :
