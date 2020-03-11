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

use work.common_pkg.all;
use work.dvb_utils_pkg.all;

package ldpc_pkg is

  -- LDPC tables 7a and 7b
  -- Largest Q is 135, so we need 8 bits
  constant LDPC_Q_WIDTH : natural := 8;
  --
  constant DVB_LDPC_GROUP_LENGTH : natural := 360;

  type ldpc_q_array_t is array (natural range <>) of unsigned(LDPC_Q_WIDTH - 1 downto 0);

  constant LDPC_Q_ROM : ldpc_q_array_t;

  function get_ldpc_q (
    constant frame_type : frame_type_t;
    constant code_rate : code_rate_t) return natural;

  impure function ldpc_table_to_rom (
    constant table      : integer_2d_array_t;
    constant data_width : natural
  ) return std_logic_vector_2d_t;

end ldpc_pkg;

package body ldpc_pkg is

  function get_ldpc_q (
    constant frame_type : frame_type_t;
    constant code_rate : code_rate_t) return natural is
  begin

    if    frame_type = FECFRAME_SHORT  and code_rate = C1_4    then return 36;
    elsif frame_type = FECFRAME_SHORT  and code_rate = C1_3    then return 30;
    elsif frame_type = FECFRAME_SHORT  and code_rate = C2_5    then return 27;
    elsif frame_type = FECFRAME_SHORT  and code_rate = C1_2    then return 25;
    elsif frame_type = FECFRAME_SHORT  and code_rate = C3_5    then return 18;
    elsif frame_type = FECFRAME_SHORT  and code_rate = C2_3    then return 15;
    elsif frame_type = FECFRAME_SHORT  and code_rate = C3_4    then return 12;
    elsif frame_type = FECFRAME_SHORT  and code_rate = C4_5    then return 10;
    elsif frame_type = FECFRAME_SHORT  and code_rate = C5_6    then return 8;
    elsif frame_type = FECFRAME_SHORT  and code_rate = C8_9    then return 5;
    -- elsif frame_type = FECFRAME_SHORT  and code_rate = C9_10   then return 0;
    -- elsif frame_type = FECFRAME_SHORT  and code_rate = not_set then return 0;

    elsif frame_type = FECFRAME_NORMAL and code_rate = C1_4    then return 135;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C1_3    then return 120;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C2_5    then return 108;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C1_2    then return 90;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C3_5    then return 72;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C2_3    then return 60;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C3_4    then return 45;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C4_5    then return 36;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C5_6    then return 30;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C8_9    then return 20;
    elsif frame_type = FECFRAME_NORMAL and code_rate = C9_10   then return 18;
    -- elsif frame_type = FECFRAME_NORMAL and code_rate = not_set then return 0;
    end if;

    return 0;

  end;

  impure function get_ldpc_q_rom return ldpc_q_array_t is
    constant ROM_LENGTH : integer := 2**(CODE_RATE_WIDTH + FRAME_TYPE_WIDTH);
    variable result     : ldpc_q_array_t(ROM_LENGTH - 1 downto 0);
    variable addr       : natural := 0;
    variable code_rate  : code_rate_t;
    variable frame_type : frame_type_t;
  begin

    for frame_type_index in frame_type_t'pos(frame_type_t'left) to frame_type_t'pos(frame_type_t'right) loop
      for code_rate_index in code_rate_t'pos(code_rate_t'left) to code_rate_t'pos(code_rate_t'right) loop
        code_rate := code_rate_t'val(code_rate_index);
        frame_type := frame_type_t'val(frame_type_index);

        addr := to_integer(unsigned(std_logic_vector'(encode(frame_type) & encode(code_rate))));
        result(addr) := to_unsigned(get_ldpc_q(frame_type, code_rate), LDPC_Q_WIDTH);

      end loop;
    end loop;

    return result;

  end;

  constant LDPC_Q_ROM : ldpc_q_array_t := get_ldpc_q_rom;

  impure function ldpc_table_to_rom (
    constant table      : integer_2d_array_t;
    constant data_width : natural
  ) return std_logic_vector_2d_t is
    constant table_depth : natural := table'length;

    function get_number_of_entries return integer_array_t is
      variable items : integer_array_t(table_depth - 1 downto 0);
    begin
      for i in 0 to table_depth - 1 loop
        items(i) := table(i)(0);
      end loop;
      return items;
    end;

    constant entry_num       : integer_array_t := get_number_of_entries;
    constant rom_depth       : natural := sum(entry_num);
    variable result          : std_logic_vector_2d_t(rom_depth - 1 downto 0)(data_width - 1 downto 0);

    constant addr_width      : natural := data_width - 1;

    variable addr            : natural;
    variable increment       : std_logic := '0';

    variable rom_addr        : natural := 0;
    variable rom_data        : std_logic_vector(data_width - 1 downto 0);
    variable columns         : natural;
  begin

    -- report cr & "entry_num=" & to_string(sum(entry_num)) & ", " & to_string(table_depth)
    -- severity error;

    for row in 0 to table_depth - 1 loop
      columns := entry_num(row);

      for column in 1 to columns loop
        addr      := table(row)(column);

        if column = columns then
          increment := '1';
        else
          increment := '0';
        end if;

        -- report sformat("row %d/%d, column %d/%d => %d, %s", fo(row), fo(table_depth - 1), fo(column), fo(columns), fo(rom_addr), fo(increment));

        -- Make sure addr_width is enough to represent every address
        assert to_unsigned(addr, addr_width) = addr
          report "DATA_WIDTH of " & to_string(data_width) & " leaves only " & to_string(addr_width) & " bits for address, which  is too small to represent " & to_string(addr)
          severity Failure;

        rom_data := increment & std_logic_vector(to_unsigned(addr, addr_width));

        result(rom_addr) := rom_data;
        rom_addr         := rom_addr + 1;
      end loop;
    end loop;
    
    return result;
  end;

end package body;
