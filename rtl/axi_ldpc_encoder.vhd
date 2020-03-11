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

use work.common_pkg.all;
use work.dvb_utils_pkg.all;
use work.ldpc_pkg.all;

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

    -- AXI LDPC table input
    s_ldpc_offset     : in  std_logic_vector(numbits(max(DVB_N_LDPC)) - 1 downto 0);
    s_ldpc_tuser      : in  std_logic_vector(numbits(max(DVB_N_LDPC)) - 1 downto 0);
    s_ldpc_tvalid     : in  std_logic;
    s_ldpc_tlast      : in  std_logic;
    s_ldpc_tready     : out std_logic := '1';

    -- AXI data input
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
  signal constellation        : constellation_t;
  signal frame_type           : frame_type_t;
  signal code_rate            : code_rate_t;

  signal s_axi_dv             : std_logic;
  signal s_ldpc_dv            : std_logic;
  signal frame_ram_ready      : std_logic;
  signal data_completed       : std_logic := '0';
  signal frame_bits_remaining : unsigned(numbits(max(DVB_N_LDPC)) - 1 downto 0);

  -- AXI data synchronized to the frame RAM output data
  signal axi_tdata            : std_logic;

  -- Interface with the frame RAM
  signal frame_ram_en         : std_logic;
  signal frame_addr_in        : unsigned(FRAME_RAM_ADDR_WIDTH - 1 downto 0);

  -- Frame RAM output
  signal frame_addr_out       : std_logic_vector(FRAME_RAM_ADDR_WIDTH - 1 downto 0);
  -- bit_index is sync with frame_addr_out and rame_ram_rddata
  signal bit_index            : std_logic_vector(numbits(FRAME_RAM_DATA_WIDTH) - 1 downto 0);
  signal frame_ram_rddata     : std_logic_vector(FRAME_RAM_DATA_WIDTH  - 1 downto 0);

  -- Frame RAM data loop
  signal frame_ram_wrdata     : std_logic_vector(FRAME_RAM_DATA_WIDTH - 1 downto 0);

  signal first_tdata          : std_logic;

  signal extract_frame_data   : std_logic := '0';
  signal encoded_tdata        : std_logic_vector(FRAME_RAM_DATA_WIDTH - 1 downto 0);

  signal dbg_addr_0           : std_logic;

  signal s_tready_i           : std_logic;
  signal s_ldpc_tready_i      : std_logic;
  signal m_tvalid_i           : std_logic;
  signal bit_index_int        : natural range 0 to ROM_DATA_WIDTH - 1;

begin

  -------------------
  -- Port mappings --
  -------------------
  frame_ram_u : entity work.pipeline_context_ram
    generic map (
      ADDR_WIDTH          => FRAME_RAM_ADDR_WIDTH,
      DATA_WIDTH          => FRAME_RAM_DATA_WIDTH,
      RAM_INFERENCE_STYLE => "bram")
    port map (
      clk         => clk,
      -- Checkout request interface
      en_in       => frame_ram_en,
      addr_in     => std_logic_vector(frame_addr_in),
      -- Data checkout output
      en_out      => frame_ram_ready,
      addr_out    => frame_addr_out,
      context_out => frame_ram_rddata,
      -- Updated data input
      context_in  => frame_ram_wrdata);

  bit_offset_delay_u : entity work.sr_delay
    generic map (
      DELAY_CYCLES  => 2,
      DATA_WIDTH    => numbits(FRAME_RAM_DATA_WIDTH),
      EXTRACT_SHREG => True)
    port map (
      clk   => clk,
      clken => '1',
      din   => s_ldpc_offset(numbits(FRAME_RAM_DATA_WIDTH) - 1 downto 0),
      dout  => bit_index);

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  -- Values for the current word

  dbg_addr_0 <= frame_ram_ready when unsigned(frame_addr_out) = 0 else '0';

  -- Values synchronized with data from pipeline_context_ram
  bit_index_int  <= to_integer(unsigned(bit_index));

  -- AXI slave specifics
  s_axi_dv  <= '1' when s_tready_i = '1' and s_tvalid = '1' else '0';
  s_ldpc_dv <= '1' when s_ldpc_tready_i = '1' and s_ldpc_tvalid = '1' else '0';

  m_tvalid_i <= '0'; --s_tvalid;
  m_tdata    <= s_tdata;
  m_tlast    <= s_tlast;

  -- Assign internals
  s_tready      <= '0' when rst = '1' or extract_frame_data = '1' else s_tready_i;
  s_ldpc_tready <= '0' when rst = '1' or extract_frame_data = '1' else s_ldpc_tready_i;
  m_tvalid      <= m_tvalid_i;

  s_ldpc_tready_i <= m_tready;

  ---------------
  -- Processes --
  ---------------
  write_side_p : process(clk, rst)
    variable xored_data : std_logic_vector(FRAME_RAM_DATA_WIDTH - 1 downto 0);
  begin
    if rst = '1' then
      s_tready_i           <= '1';
      encoded_tdata        <= (others => 'U');
      frame_bits_remaining <= (others => '0');
    elsif rising_edge(clk) then

      -- Always return the context, will change only when needed
      frame_ram_wrdata <= frame_ram_rddata;

      -- Frame RAM addressing depends if we're calculating the codes or extracting them
      if extract_frame_data = '0' then
        -- When on normal operation, extract RAM addr
        frame_addr_in <= unsigned(s_ldpc_offset(ROM_DATA_WIDTH - 1 downto numbits(FRAME_RAM_DATA_WIDTH)));
        frame_ram_en  <= s_ldpc_dv;
      else
        -- When extracting frame data, we need to complete the given frame. Since the
        -- frame size is not always an integer multiple of the frame length, we also need
        -- to check if data bit cnt has wrapped (MSB is 1).
        if frame_bits_remaining = 0 or frame_bits_remaining(frame_bits_remaining'length - 1) = '1' then
          frame_addr_in      <= (others => '0');
          frame_ram_en       <= '0';
          extract_frame_data <= '0';
        else
          frame_addr_in      <= frame_addr_in + 1;
        end if;
      end if;

      -- When calculating the final XOR'ed value, the first bit will depend on the
      -- address. We can safely assign here and avoid extra logic levels on the FF control
      if unsigned(frame_addr_out) = 0 then
        xored_data(0) := frame_ram_rddata(0);
      else
        xored_data(0) := encoded_tdata(FRAME_RAM_DATA_WIDTH - 1) xor frame_ram_rddata(0);
      end if;

      -- Handle data coming out of the frame RAM, either by using the input data of by
      -- calculating the actual final XOR'ed value
      if frame_ram_ready = '1' then
        if extract_frame_data = '0' then
          frame_ram_wrdata(bit_index_int) <= axi_tdata xor frame_ram_rddata(bit_index_int);
        else
          -- Need to clear the RAM for the next frame
          frame_ram_wrdata <= (others => '0');

          -- Calculate the final XOR between output bits
          for i in 1 to FRAME_RAM_DATA_WIDTH - 1 loop
            xored_data(i) := frame_ram_rddata(i) xor xored_data(i - 1);
          end loop;

          -- Assign data out and decrement the bits consumed
          frame_bits_remaining     <= frame_bits_remaining - FRAME_RAM_DATA_WIDTH;
          encoded_tdata    <= xored_data;

        end if;
      end if;

      -- AXI LDPC table control
      if s_ldpc_dv = '1' then
        s_tready_i <= s_ldpc_tlast and not extract_frame_data;

        if s_ldpc_tlast = '1' and data_completed = '1' then
          -- We'll complete the frame with enough data to complete either the short or
          -- normal frames (16,200 or 64,800 bits respectively)
          data_completed     <= '0';
          extract_frame_data <= '1';
          frame_addr_in      <= (others => '0');
          frame_ram_en       <= '1';
        end if;
      end if;

      -- AXI frame data control
      if s_axi_dv = '1' then
        s_tready_i   <= '0';

        -- Set the expected frame length. Data will passthrough and LDPC codes will be
        -- appended to complete the appropriate n_ldpc length (either 16,200 or 64,800).
        -- Timing-wise, the best way to do this is by setting the counter to - N + 2;
        -- whenever it gets to 1 it will have counted N items.
        if first_tdata = '1' then
          if cfg_frame_type = FECFRAME_SHORT then
            frame_bits_remaining <= to_unsigned(16_199, frame_bits_remaining'length);
          elsif cfg_frame_type = FECFRAME_NORMAL then
            frame_bits_remaining <= to_unsigned(64_799, frame_bits_remaining'length);
          else
            report "Need a valid frame size" severity Error;
          end if;
        else
          frame_bits_remaining <= frame_bits_remaining - 1;
        end if;

        if s_tlast = '1' then
          data_completed <= '1';
        end if;

      end if;

    end if;
  end process;

  -- The config ports are valid at the first word of the frame, but we must not rely on
  -- the user keeping it unchanged. Hide this on a block to leave the core code a bit
  -- cleaner
  config_sample_block : block -- {{
    signal constellation_ff : constellation_t;
    signal frame_type_ff    : frame_type_t;
    signal code_rate_ff     : code_rate_t;
    signal s_tdata_reg      : std_logic;
  begin

    process(clk, rst)
    begin
      if rst = '1' then
        first_tdata  <= '1';
        axi_tdata    <= 'U'; -- We don't want a mux with rst here
      elsif rising_edge(clk) then
        axi_tdata     <= s_tdata_reg;
        constellation <= constellation_ff;
        frame_type    <= frame_type_ff;
        code_rate     <= code_rate_ff;


        if s_axi_dv = '1' then
          first_tdata <= s_tlast;
          s_tdata_reg <= s_tdata;

          -- Sample the BCH code used on the first word
          if first_tdata = '1' then
            constellation_ff <= cfg_constellation;
            frame_type_ff    <= cfg_frame_type;
            code_rate_ff     <= cfg_code_rate;
          end if;

        end if;

      end if;
    end process;
  end block config_sample_block; -- }}

end axi_ldpc_encoder;

-- vim: set foldmethod=marker foldmarker=--\ {{,--\ }} :
