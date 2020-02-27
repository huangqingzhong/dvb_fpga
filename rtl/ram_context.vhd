--
-- DVB FPGA
--
-- Copyright 2019 by Suoto <andre820@gmail.com>
--
-- This file is part of DVB FPGA.
--
-- DVB FPGA is free software: you can redistribute it and/or modify
-- it under the terms of the GNU General Public License as published by
-- the Free Software Foundation, either version 3 of the License, or
-- (at your option) any later version.
--
-- DVB FPGA is distributed in the hope that it will be useful,
-- but WITHOUT ANY WARRANTY; without even the implied warranty of
-- MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
-- GNU General Public License for more details.
--
-- You should have received a copy of the GNU General Public License
-- along with DVB FPGA.  If not, see <http://www.gnu.org/licenses/>.

---------------
-- Libraries --
---------------
library	ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

------------------------
-- Entity declaration --
------------------------
entity ram_context is
  generic (
    ADDR_WIDTH          : positive := 16;
    DATA_WIDTH          : positive := 16;
    RAM_INFERENCE_STYLE : string   := "auto");
  port (
    clk         : in  std_logic;

    en          : in  std_logic;
    addr        : in  std_logic_vector(ADDR_WIDTH - 1 downto 0);
    context_out : out std_logic_vector(DATA_WIDTH - 1 downto 0);
    context_in  : in  std_logic_vector(DATA_WIDTH - 1 downto 0));
end ram_context;

architecture ram_context of ram_context is

  -----------
  -- Types --
  -----------
  type addr_array_t is array (natural range <>) of std_logic_vector(ADDR_WIDTH - 1 downto 0);
  type data_array_t is array (natural range <>) of std_logic_vector(DATA_WIDTH - 1 downto 0);

  -------------
  -- Signals --
  -------------
  signal ram_wren   : std_logic;
  signal ram_rddata : std_logic_vector(DATA_WIDTH - 1 downto 0);

  signal data_sr    : data_array_t(2 downto 0);
  signal addr_sr    : addr_array_t(2 downto 0);

begin

  -------------------
  -- Port mappings --
  -------------------
  ram_u : entity work.ram_inference
    generic map (
      ADDR_WIDTH          => ADDR_WIDTH,
      DATA_WIDTH          => DATA_WIDTH,
      RAM_INFERENCE_STYLE => RAM_INFERENCE_STYLE,
      EXTRA_OUTPUT_DELAY  => 0)
    port map (
      -- Port A
      clk_a     => clk,
      clken_a   => '1',
      wren_a    => ram_wren,
      addr_a    => addr_sr(1),
      wrdata_a  => context_in,
      rddata_a  => open,

      -- Port B
      clk_b     => clk,
      clken_b   => '1',
      addr_b    => addr,
      rddata_b  => ram_rddata);

  ram_wren_u : entity work.sr_delay
    generic map (
      DELAY_CYCLES  => 1,
      DATA_WIDTH    => 1,
      EXTRACT_SHREG => True)
    port map (
      clk     => clk,
      clken   => '1',

      din(0)  => en,
      dout(0) => ram_wren);

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  -- Loop data until
  context_out <= context_in when addr = addr_sr(0) else
                 data_sr(0) when addr = addr_sr(1) else
                 ram_rddata;

  addr_sr(0) <= addr;
  data_sr(0) <= context_in;

  ---------------
  -- Processes --
  ---------------
  process(clk)
  begin
    if rising_edge(clk) then
      addr_sr(addr_sr'length - 1 downto 1) <= addr_sr(addr_sr'length - 2 downto 0);
      data_sr(data_sr'length - 1 downto 1) <= data_sr(data_sr'length - 2 downto 0);
    end if;
  end process;

end ram_context;
