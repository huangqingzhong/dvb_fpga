--
-- DVB IP
--
-- Copyright 2019 by Suoto <andre820@gmail.com>
--
-- This file is part of the DVB IP.
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

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

use work.dvb_utils_pkg.all;

package ldpc_pkg is

  -- LDPC tables 7a and 7b
  -- Largest Q is 135, so we need 8 bits
  constant LDPC_Q_WIDTH : natural := 8;

  type ldpc_q_array_t is array (natural range <>) of unsigned(LDPC_Q_WIDTH - 1 downto 0);

  constant LDPC_Q_ROM : ldpc_q_array_t;

end ldpc_pkg;

package body ldpc_pkg is

  impure function get_ldpc_q_rom return ldpc_q_array_t is
    constant ROM_LENGTH : integer := 2**(CODE_RATE_WIDTH + FRAME_TYPE_WIDTH);
    variable result     : ldpc_q_array_t(ROM_LENGTH - 1 downto 0);
    variable addr       : natural := 0;
    variable code_rate  : code_rate_t;
    variable frame_type : frame_type_t;
    variable value      : natural := 0;
  begin

    for frame_type_index in frame_type_t'pos(frame_type_t'left) to frame_type_t'pos(frame_type_t'right) loop
      for code_rate_index in code_rate_t'pos(code_rate_t'left) to code_rate_t'pos(code_rate_t'right) loop
        code_rate := code_rate_t'val(code_rate_index);
        frame_type := frame_type_t'val(frame_type_index);

        if    frame_type = FECFRAME_SHORT  and code_rate = C1_4    then value := 36;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C1_3    then value := 30;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C2_5    then value := 27;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C1_2    then value := 25;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C3_5    then value := 18;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C2_3    then value := 15;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C3_4    then value := 12;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C4_5    then value := 10;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C5_6    then value := 8;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C8_9    then value := 5;
        elsif frame_type = FECFRAME_SHORT  and code_rate = C9_10   then value := 0;
        elsif frame_type = FECFRAME_SHORT  and code_rate = not_set then value := 0;

        elsif frame_type = FECFRAME_NORMAL and code_rate = C1_4    then value := 135;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C1_3    then value := 120;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C2_5    then value := 108;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C1_2    then value := 90;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C3_5    then value := 72;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C2_3    then value := 60;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C3_4    then value := 45;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C4_5    then value := 36;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C5_6    then value := 30;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C8_9    then value := 20;
        elsif frame_type = FECFRAME_NORMAL and code_rate = C9_10   then value := 18;
        elsif frame_type = FECFRAME_NORMAL and code_rate = not_set then value := 0;
        end if;

        addr := to_integer(unsigned(encode(frame_type) & encode(code_rate)));
        result(addr) := to_unsigned(value, LDPC_Q_WIDTH);

      end loop;
    end loop;

    return result;

  end;

  constant LDPC_Q_ROM : ldpc_q_array_t := get_ldpc_q_rom;

end package body;
