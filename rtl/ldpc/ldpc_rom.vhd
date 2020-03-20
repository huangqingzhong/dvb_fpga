--
-- DVB IP
--
-- Copyright 2020 by Suoto <andre820@gmail.com>
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

---------------
-- Libraries --
---------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library fpga_cores;
use fpga_cores.common_pkg.all;

use work.dvb_utils_pkg.all;
use work.ldpc_pkg.all;
use work.ldpc_tables_pkg.all;

------------------------
-- Entity declaration --
------------------------
entity ldpc_rom is
  generic (
    LENGTH_WIDTH : natural := 16; -- TODO: Need to see a better way to infer this
    ADDR_WIDTH   : natural := 8;
    DATA_WIDTH   : natural := 16;
    RAM_TYPE     : string  := "auto";
    OUTPUT_DELAY : natural := 0
  );
  port (
    -- Usual ports
    clk        : in  std_logic;

    --
    frame_type : in frame_type_t;
    code_rate  : in code_rate_t;

    q          : out unsigned(LDPC_Q_WIDTH - 1 downto 0);
    length     : out unsigned(LENGTH_WIDTH - 1 downto 0);

    --
    addr       : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);
    dout       : out std_logic_vector(DATA_WIDTH - 1 downto 0);
    last       : out std_logic);
end ldpc_rom;

architecture ldpc_rom of ldpc_rom is

  ---------------
  -- Constants --
  ---------------
  -- Fixed for now. Add 1 because the generic DATA_WIDTH defines the port width but the
  -- ROM also has a bit to identify the last bit of word (needed because all table
  -- contents are stacked).
  -- constant LDPC_TABLE     : std_logic_vector_2d_t := ldpc_table_to_rom(DVB_16200_S2_C8_T2_B6 , DATA_WIDTH + 1);

  -- constant ROM_ADDR_WIDTH : natural := numbits(LDPC_TABLE'length);

  -------------
  -- Signals --
  -------------
  signal dout_i : std_logic_vector(DATA_WIDTH downto 0);
  signal q_in   : natural range 0 to 2**LDPC_Q_WIDTH - 1;
  signal q_out  : std_logic_vector(LDPC_Q_WIDTH - 1 downto 0);

begin

  -------------------
  -- Port mappings --
  -------------------
  -- table_u : entity fpga_cores.rom_inference
  --   generic map (
  --     DATA         => LDPC_TABLE,
  --     RAM_TYPE     => RAM_TYPE,
  --     OUTPUT_DELAY => 0)
  --   port map (
  --     -- Usual ports
  --     clk  => clk,

  --     -- Block specifics
  --     addr => addr(ROM_ADDR_WIDTH - 1 downto 0),
  --     dout => dout_i);

  q_delay_u : entity fpga_cores.sr_delay
  generic map (
    DELAY_CYCLES  => OUTPUT_DELAY,
    DATA_WIDTH    => q'length,
    EXTRACT_SHREG => False)
  port map (
    clk     => clk,
    clken   => '1',

    din     => std_logic_vector(to_unsigned(q_in, LDPC_Q_WIDTH)),
    dout    => q_out);

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  q_in   <= get_ldpc_q(frame_type, code_rate);
  q      <= unsigned(q_out);

  -- length <= to_unsigned(LDPC_TABLE'length, length'length);

  -- There will be a mux here, mimic the effect of chosing an invalid config
  dout <= dout_i(DATA_WIDTH - 1 downto 0) when frame_type /= not_set and code_rate /= not_set else
          (others => 'U');

  last <= dout_i(DATA_WIDTH) when frame_type /= not_set and code_rate /= not_set else 'U';

  ---------------
  -- Processes --
  ---------------
  process(clk)
  begin
    if clk'event and clk = '1' then
      assert (frame_type = FECFRAME_SHORT and code_rate = C4_5) or
             (frame_type = not_set and code_rate = not_set)
        report "Unsupported parameters: " & frame_type_t'image(frame_type) & ", " & code_rate_t'image(code_rate)
        severity Warning;
    end if;
  end process;

end ldpc_rom;
