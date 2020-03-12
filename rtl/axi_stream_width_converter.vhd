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

---------------------------------
-- Block name and description --
--------------------------------

---------------
-- Libraries --
---------------
library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

------------------------
-- Entity declaration --
------------------------
entity axi_stream_width_converter is
  generic (
    INPUT_DATA_WIDTH  : natural := 16;
    OUTPUT_DATA_WIDTH : natural := 16;
    AXI_TID_WIDTH     : natural := 8);
  port (
    -- Usual ports
    clk      : in  std_logic;
    rst      : in  std_logic;
    -- AXI stream input
    s_tready : out std_logic;
    s_tdata  : in  std_logic_vector(INPUT_DATA_WIDTH - 1 downto 0);
    s_tkeep  : in  std_logic_vector((INPUT_DATA_WIDTH + 7) / 8 - 1 downto 0);
    s_tid    : in  std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
    s_tvalid : in  std_logic;
    s_tlast  : in  std_logic;
    -- AXI stream output
    m_tready : in  std_logic;
    m_tdata  : out std_logic_vector(INPUT_DATA_WIDTH - 1 downto 0);
    m_tkeep  : out std_logic_vector((INPUT_DATA_WIDTH + 7) / 8 - 1 downto 0);
    m_tid    : out std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
    m_tvalid : out std_logic;
    m_tlast  : out std_logic);
end axi_stream_width_converter;

architecture axi_stream_width_converter of axi_stream_width_converter is

  ---------------
  -- Constants --
  ---------------
  constant INPUT_BYTE_WIDTH  : natural := (INPUT_DATA_WIDTH + 7) / 8;
  constant OUTPUT_BYTE_WIDTH : natural := (OUTPUT_DATA_WIDTH + 7) / 8;

  -----------
  -- Types --
  -----------

  -------------
  -- Signals --
  -------------
  signal s_data_valid : std_logic;
  signal s_tready_i   : std_logic;
  signal m_tvalid_i   : std_logic;

begin

  g_pass_through : if INPUT_DATA_WIDTH = OUTPUT_DATA_WIDTH generate
    signal first_word : std_logic;
    signal s_tid_reg  : std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
  begin

    s_tready_i <= m_tready;
    m_tdata    <= s_tdata;
    m_tkeep    <= s_tkeep;
    m_tvalid_i <= s_tvalid;
    m_tlast    <= s_tlast;
    m_tid      <= s_tid when first_word else s_tid_reg;

    process(clk, rst)
    begin
      if rst = '1' then
        first_word <= '1';
      elsif rising_edge(clk) then

        if s_data_valid = '1' then
          first_word <= s_tlast;

          if first_word = '1' then
            s_tid_reg <= s_tid;
          end if;
        end if;

      end if;
    end process;
  end generate g_pass_through;

  -------------------
  -- Port mappings --
  -------------------

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  s_data_valid     <= s_tready_i and s_tvalid;

  s_tready <= s_tready_i;
  m_tvalid <= m_tvalid_i;

  ---------------
  -- Processes --
  ---------------
  process(clk, rst)
  begin
    if rst = '1' then
      null;
    elsif clk'event and clk = '1' then
      null;
    end if;
  end process;

end axi_stream_width_converter;

