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

------------------------
-- Entity declaration --
------------------------
entity axi_ldpc_encoder is
  generic (
    DATA_WIDTH : positive := 8
  );
  port (
    -- Usual ports
    clk               : in  std_logic;
    rst               : in  std_logic;

    cfg_constellation : in  constellation_t;
    cfg_frame_type    : in  frame_type_t;
    cfg_code_rate     : in  code_rate_t;

    -- AXI input
    s_tvalid          : in  std_logic;
    s_tdata           : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
    s_tlast           : in  std_logic;
    s_tready          : out std_logic;

    -- AXI output
    m_tready          : in  std_logic;
    m_tvalid          : out std_logic;
    m_tlast           : out std_logic;
    m_tdata           : out std_logic_vector(DATA_WIDTH - 1 downto 0));
end axi_ldpc_encoder;

architecture axi_ldpc_encoder of axi_ldpc_encoder is

  ---------------
  -- Constants --
  ---------------
  -- 
  constant MAX_UNCODED_FRAME_SIZE : integer := 58_320 / DATA_WIDTH;
  constant MAX_CODED_FRAME_SIZE   : integer := 64_800 / DATA_WIDTH;

  constant FRAME_PTR_DEPTH        : integer := 2;

  -------------
  -- Signals --
  -------------
  signal constellation  : constellation_t;
  signal frame_type     : frame_type_t;
  signal code_rate      : code_rate_t;

  signal s_axi_dv       : std_logic;
  signal s_tready_i     : std_logic;

  signal diff_frame_ptr : unsigned(numbits(FRAME_PTR_DEPTH) - 1 downto 0);
  signal wr_frame_ptr   : unsigned(numbits(FRAME_PTR_DEPTH) - 1 downto 0);
  signal rd_frame_ptr   : unsigned(numbits(FRAME_PTR_DEPTH) - 1 downto 0);

  signal ram_wr_ptr     : unsigned(numbits(MAX_UNCODED_FRAME_SIZE) downto 0);

  signal ram_rd_ptr     : unsigned(numbits(MAX_UNCODED_FRAME_SIZE) downto 0);
  signal ram_rd_data    : std_logic_vector(DATA_WIDTH - 1 downto 0);

begin

  -------------------
  -- Port mappings --
  -------------------
  frame_ram_u : entity work.ram_inference
    generic map (
      ADDR_WIDTH          => numbits(MAX_UNCODED_FRAME_SIZE) + 1,
      DATA_WIDTH          => DATA_WIDTH,
      RAM_INFERENCE_STYLE => "auto",
      EXTRA_OUTPUT_DELAY  => 0)
    port map (
      -- Port A
      clk_a     => clk,
      clken_a   => '1',
      wren_a    => s_axi_dv,
      addr_a    => std_logic_vector(ram_wr_ptr),
      wrdata_a  => s_tdata,
      rddata_a  => open,

      -- Port B
      clk_b     => clk,
      clken_b   => '1',
      addr_b    => std_logic_vector(ram_rd_ptr),
      rddata_b  => ram_rd_data);

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  s_axi_dv     <= '1' when s_tready_i = '1' and s_tvalid = '1' else '0';

  diff_frame_ptr <= wr_frame_ptr - rd_frame_ptr when wr_frame_ptr > rd_frame_ptr else
                    wr_frame_ptr + FRAME_PTR_DEPTH - rd_frame_ptr;

  s_tready_i <= '1' when diff_frame_ptr < FRAME_PTR_DEPTH - 1 else '0';

  -- Assign internals
  s_tready <= s_tready_i;


  ---------------
  -- Processes --
  ---------------
  write_side_p : process(clk, rst)
  begin
    if rst = '1' then
      wr_frame_ptr <= (others => '0');
      ram_wr_ptr   <= (others => '0');
    elsif clk'event and clk = '1' then

      if s_axi_dv = '1' then
        ram_wr_ptr <= ram_wr_ptr + 1;
        if s_tlast = '1' then
          wr_frame_ptr <= wr_frame_ptr + 1;
        end if;
      end if;

    end if;
  end process;

  read_side_p : process(clk, rst)
  begin
    if rst = '1' then
      rd_frame_ptr <= (others => '0');
      ram_rd_ptr   <= (others => '0');
    elsif clk'event and clk = '1' then

      if diff_frame_ptr /= 0 then
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
