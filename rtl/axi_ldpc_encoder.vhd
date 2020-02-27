--
-- DVB IP
--
-- Copyright 2019 by Suoto <andre820@gmail.com>
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

library str_format;
use str_format.str_format_pkg.all;

use work.common_pkg.all;
use work.dvb_utils_pkg.all;
use work.ldpc_pkg.all;
-- use work.ldpc_tables_pkg.all;

------------------------
-- Entity declaration --
------------------------
entity axi_ldpc_encoder is
  port (
    -- Usual ports
    clk               : in  std_logic;
    rst               : in  std_logic;

    cfg_constellation : in  constellation_t;
    cfg_frame_type    : in  frame_type_t;
    cfg_code_rate     : in  code_rate_t;

    -- AXI input
    s_tvalid          : in  std_logic;
    s_tdata           : in  std_logic;
    s_tlast           : in  std_logic;
    s_tready          : out std_logic;

    -- AXI output
    m_tready          : in  std_logic;
    m_tvalid          : out std_logic;
    m_tlast           : out std_logic;
    m_tdata           : out std_logic);
end axi_ldpc_encoder;

architecture axi_ldpc_encoder of axi_ldpc_encoder is

  ---------------
  -- Constants --
  ---------------
  constant ROM_DATA_WIDTH   : natural := numbits(max(DVB_N_LDPC));
  constant ROM_ADDR_WIDTH   : natural := 16;
  constant ROM_LENGTH_WIDTH : natural := 16;

  constant FRAME_RAM_DATA_WIDTH : natural := 16;
  constant FRAME_RAM_ADDR_WIDTH : natural
    := numbits((max(DVB_N_LDPC) + FRAME_RAM_DATA_WIDTH - 1) / FRAME_RAM_DATA_WIDTH);

  -------------
  -- Signals --
  -------------
  signal constellation    : constellation_t;
  signal frame_type       : frame_type_t;
  signal code_rate        : code_rate_t;

  signal s_axi_dv         : std_logic;

  -- Interface with the LDPC table ROM
  signal rom_addr         : unsigned(ROM_ADDR_WIDTH - 1 downto 0);
  signal rom_data         : std_logic_vector(ROM_DATA_WIDTH - 1 downto 0);
  signal rom_last         : std_logic;
  signal rom_q            : unsigned(LDPC_Q_WIDTH - 1 downto 0);
  signal rom_table_length : unsigned(ROM_LENGTH_WIDTH - 1 downto 0);

  signal rom_data_sync    : std_logic_vector(ROM_DATA_WIDTH - 1 downto 0);
  signal rom_last_sync    : std_logic;

  -- Interface with the frame RAM
  signal frame_ram_en     : std_logic;
  signal frame_addr       : std_logic_vector(FRAME_RAM_ADDR_WIDTH - 1 downto 0);
  signal frame_ram_wrdata : std_logic_vector(FRAME_RAM_DATA_WIDTH - 1 downto 0);
  signal frame_ram_rddata : std_logic_vector(FRAME_RAM_DATA_WIDTH  - 1 downto 0);
  signal bit_index        : unsigned(numbits(ROM_DATA_WIDTH) - 1 downto 0);

  signal first_group      : std_logic;
  signal group_bit_cnt    : unsigned(numbits(DVB_LDPC_GROUP_LENGTH) - 1 downto 0);
  signal group_cnt        : unsigned(numbits(max(DVB_N_LDPC) / DVB_LDPC_GROUP_LENGTH) - 1 downto 0);
  signal frame_addr_sync  : std_logic_vector(FRAME_RAM_ADDR_WIDTH - 1 downto 0);
  signal bit_index_sync   : unsigned(numbits(ROM_DATA_WIDTH) - 1 downto 0);

  signal s_tready_i       : std_logic;
  signal m_tvalid_i       : std_logic;


  signal dbg_bit_cnt      : natural := 0;

begin

  -------------------
  -- Port mappings --
  -------------------
  ldpc_rom_u : entity work.ldpc_rom
    generic map (
      LENGTH_WIDTH        => ROM_LENGTH_WIDTH,
      ADDR_WIDTH          => ROM_ADDR_WIDTH,
      DATA_WIDTH          => ROM_DATA_WIDTH,
      RAM_INFERENCE_STYLE => "auto",
      EXTRA_OUTPUT_DELAY  => 0)
    port map (
      -- Usual ports
      clk        => clk,

      --
      frame_type => frame_type,
      code_rate  => code_rate,

      q          => rom_q,
      length     => rom_table_length,

      --
      addr       => std_logic_vector(rom_addr),
      dout       => rom_data,
      last       => rom_last);

  rom_data_delay_u : entity work.sr_delay
    generic map (
      DELAY_CYCLES  => 1,
      DATA_WIDTH    => rom_data'length,
      EXTRACT_SHREG => True)
    port map (
      clk   => clk,
      clken => '1',
      din   => rom_data,
      dout  => rom_data_sync);

  rom_last_delay_u : entity work.sr_delay
    generic map (
      DELAY_CYCLES  => 1,
      DATA_WIDTH    => 1,
      EXTRACT_SHREG => True)
    port map (
      clk     => clk,
      clken   => '1',
      din(0)  => rom_last,
      dout(0) => rom_last_sync);

  frame_ram_u : entity work.ram_context
    generic map (
      ADDR_WIDTH          => FRAME_RAM_ADDR_WIDTH,
      DATA_WIDTH          => FRAME_RAM_DATA_WIDTH,
      RAM_INFERENCE_STYLE => "bram")
    port map (
      clk         => clk,
      en          => frame_ram_en,
      addr        => std_logic_vector(frame_addr),
      context_out => frame_ram_rddata,
      context_in  => frame_ram_wrdata);

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  -- Values for the current word
  frame_addr <= rom_data(ROM_DATA_WIDTH - 1 downto numbits(FRAME_RAM_DATA_WIDTH));
  bit_index  <= unsigned(rom_data(numbits(FRAME_RAM_DATA_WIDTH) - 1 downto 0));

  -- Values synchronized with data from ram_context
  frame_addr_sync <= rom_data_sync(ROM_DATA_WIDTH - 1 downto numbits(FRAME_RAM_DATA_WIDTH));
  bit_index_sync  <= unsigned(rom_data_sync(numbits(FRAME_RAM_DATA_WIDTH) - 1 downto 0));

  first_group     <= '1' when group_cnt = 0 else '0';

  -- AXI slave specifics
  s_axi_dv   <= '1' when s_tready_i = '1' and s_tvalid = '1' else '0';

  s_tready_i <= m_tready when rom_addr = 0 else
                rom_last;

  m_tvalid_i <= s_tvalid;
  m_tdata    <= s_tdata;
  m_tlast    <= s_tlast;

  -- Assign internals
  s_tready   <= s_tready_i;
  m_tvalid   <= m_tvalid_i;

  ---------------
  -- Processes --
  ---------------
  write_side_p : process(clk, rst)
  begin
    if rst = '1' then
      rom_addr      <= (others => '0');
      group_cnt     <= (others => '0');
      group_bit_cnt <= (others => '0');
    elsif clk'event and clk = '1' then

      if s_axi_dv = '1' or rom_addr > 0 then
        if rom_addr < rom_table_length - 1 then
          rom_addr  <= rom_addr + 1;
        else
          rom_addr  <= (others => '0');
        end if;
      end if;

      if s_axi_dv = '1' then
        dbg_bit_cnt <= dbg_bit_cnt + 1;
        if group_bit_cnt < DVB_LDPC_GROUP_LENGTH - 1 then
          group_bit_cnt <= group_bit_cnt + 1;
        else
          group_bit_cnt <= (others => '0');
          group_cnt     <= group_cnt + 1;
        end if;
      end if;

    end if;
  end process;

  -- The config ports are valid at the first word of the frame, but we must not rely on
  -- the user keeping it unchanged. Hide this on a block to leave the core code a bit
  -- cleaner
  config_sample_block : block
    signal constellation_ff : constellation_t;
    signal frame_type_ff    : frame_type_t;
    signal code_rate_ff     : code_rate_t;
    signal first_word       : std_logic;
  begin

    constellation <= cfg_constellation when first_word = '1' else constellation_ff;
    frame_type    <= cfg_frame_type when first_word = '1' else frame_type_ff;
    code_rate     <= cfg_code_rate when first_word = '1' else code_rate_ff;

    process(clk, rst)
    begin
      if rst = '1' then
        first_word  <= '1';
      elsif rising_edge(clk) then
        if s_axi_dv = '1' then
          first_word <= s_tlast;

          -- Sample the BCH code used on the first word
          if first_word = '1' then
            constellation_ff <= cfg_constellation;
            frame_type_ff    <= cfg_frame_type;
            code_rate_ff     <= cfg_code_rate;
          end if;

        end if;

      end if;
    end process;
  end block config_sample_block;

end axi_ldpc_encoder;
