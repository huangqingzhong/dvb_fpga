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

library fpga_cores;
use fpga_cores.common_pkg.all;

use work.dvb_utils_pkg.all;
use work.ldpc_pkg.all;

------------------------
-- Entity declaration --
------------------------
entity axi_ldpc_encoder is
  generic ( DATA_WIDTH : natural := 8 );
  port (
    -- Usual ports
    clk               : in  std_logic;
    rst               : in  std_logic;

    cfg_constellation : in  constellation_t;
    cfg_frame_type    : in  frame_type_t;
    cfg_code_rate     : in  code_rate_t;

    -- AXI LDPC table input
    s_ldpc_tvalid     : in  std_logic;
    s_ldpc_tready     : out std_logic := '1';
    s_ldpc_offset     : in  std_logic_vector(numbits(max(DVB_N_LDPC)) - 1 downto 0);
    s_ldpc_tuser      : in  std_logic_vector(numbits(max(DVB_N_LDPC)) - 1 downto 0);
    s_ldpc_tlast      : in  std_logic;

    -- AXI data input
    s_tvalid          : in  std_logic;
    s_tready          : out std_logic;
    s_tdata           : in  std_logic_vector(DATA_WIDTH - 1 downto 0);
    s_tlast           : in  std_logic;

    -- AXI output
    m_tvalid          : out std_logic;
    m_tready          : in  std_logic;
    m_tlast           : out std_logic;
    m_tdata           : out std_logic_vector(DATA_WIDTH - 1 downto 0));
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
  signal axi_in_tready        : std_logic;
  signal axi_in_tdata         : std_logic;
  signal axi_in_tvalid        : std_logic;
  signal axi_in_tlast         : std_logic;

  signal axi_in_dv            : std_logic;
  signal axi_in_first_word    : std_logic;
  signal axi_in_has_data      : std_logic;
  signal axi_in_tready_p      : std_logic;

  signal axi_in_constellation : constellation_t;
  signal axi_in_frame_type    : frame_type_t;
  signal axi_in_code_rate     : code_rate_t;

  signal axi_in_completed     : std_logic := '0';
  -- AXI data synchronized to the frame RAM output data
  signal axi_in_tdata_sampled : std_logic;

  signal ldpc_dv              : std_logic;
  signal s_ldpc_tready_i      : std_logic;
  signal ldpc_offset          : std_logic_vector(numbits(max(DVB_N_LDPC)) - 1 downto 0);

  signal frame_ram_ready      : std_logic;
  signal frame_bits_remaining : unsigned(numbits(max(DVB_N_LDPC)) - 1 downto 0);

  -- Interface with the frame RAM
  signal frame_ram_en         : std_logic;
  signal frame_addr_in        : unsigned(FRAME_RAM_ADDR_WIDTH - 1 downto 0);

  -- Frame RAM output
  signal frame_addr_out       : std_logic_vector(FRAME_RAM_ADDR_WIDTH - 1 downto 0);
  -- frame_bit_index is sync with frame_addr_out and rame_ram_rddata
  signal frame_bit_index      : std_logic_vector(numbits(FRAME_RAM_DATA_WIDTH) - 1 downto 0);
  signal frame_ram_rddata     : std_logic_vector(FRAME_RAM_DATA_WIDTH  - 1 downto 0);

  -- Frame RAM data loop
  signal frame_ram_wrdata     : std_logic_vector(FRAME_RAM_DATA_WIDTH - 1 downto 0);

  signal frame_ram_last       : std_logic := '0';
  signal extract_frame_data   : std_logic := '0';

  signal code_proc_ready      : std_logic;
  signal code_first_word      : std_logic;

  signal encoded_data_wr_en   : std_logic;
  signal encoded_data_wr_full : std_logic;
  signal encoded_data_wr_data : std_logic_vector(FRAME_RAM_DATA_WIDTH - 1 downto 0);
  signal encoded_data_wr_last : std_logic;

  signal axi_out_tready       : std_logic;
  signal axi_out_tdata        : std_logic_vector(FRAME_RAM_DATA_WIDTH - 1 downto 0);
  signal axi_out_tvalid       : std_logic;
  signal axi_out_tlast        : std_logic;

  signal m_tvalid_i           : std_logic;
  signal frame_bit_index_i    : natural range 0 to ROM_DATA_WIDTH - 1;


begin

  -------------------
  -- Port mappings --
  -------------------
  -- Convert from FRAME_RAM_DATA_WIDTH to the specified data width
  input_conversion_block : block
    constant ID_WIDTH : integer := FRAME_TYPE_WIDTH + CONSTELLATION_WIDTH + CODE_RATE_WIDTH;
    signal s_tid      : std_logic_vector(ID_WIDTH - 1 downto 0);
    signal m_tid      : std_logic_vector(ID_WIDTH - 1 downto 0);
  begin

    s_tid <= encode(cfg_code_rate) &
             encode(cfg_constellation) &
             encode(cfg_frame_type);

    axi_in_code_rate     <= decode(m_tid(m_tid'length - 1 downto FRAME_TYPE_WIDTH + CONSTELLATION_WIDTH));
    axi_in_constellation <= decode(m_tid(FRAME_TYPE_WIDTH + CONSTELLATION_WIDTH - 1 downto FRAME_TYPE_WIDTH));
    axi_in_frame_type    <= decode(m_tid(FRAME_TYPE_WIDTH - 1 downto 0));

    input_width_conversion_u : entity fpga_cores.axi_stream_width_converter
      generic map (
        INPUT_DATA_WIDTH  => DATA_WIDTH,
        OUTPUT_DATA_WIDTH => 1,
        AXI_TID_WIDTH     => ID_WIDTH)
      port map (
        -- Usual ports
        clk        => clk,
        rst        => rst,
        -- AXI stream input
        s_tready   => s_tready,
        s_tdata    => mirror_bits(s_tdata), -- width converter is little endian, we need big endian
        s_tkeep    => (others => '0'),
        s_tid      => s_tid,
        s_tvalid   => s_tvalid,
        s_tlast    => s_tlast,
        -- AXI stream output
        m_tready   => axi_in_tready,
        m_tdata(0) => axi_in_tdata,
        m_tkeep    => open,
        m_tid      => m_tid,
        m_tvalid   => axi_in_tvalid,
        m_tlast    => axi_in_tlast);
    end block;

  frame_ram_u : entity fpga_cores.pipeline_context_ram
    generic map (
      ADDR_WIDTH => FRAME_RAM_ADDR_WIDTH,
      DATA_WIDTH => FRAME_RAM_DATA_WIDTH,
      RAM_TYPE   => "bram")
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

  -- Can't stop reading from the frame RAM instantly, allow some slack
  frame_ram_adapter_u : entity fpga_cores.axi_stream_master_adapter
    generic map (
      MAX_SKEW_CYCLES => 3,
      TDATA_WIDTH     => encoded_data_wr_data'length)
    port map (
      -- Usual ports
      clk      => clk,
      reset    => rst,
      -- wanna-be AXI interface
      wr_en    => encoded_data_wr_en,
      wr_full  => encoded_data_wr_full,
      wr_data  => encoded_data_wr_data,
      wr_last  => encoded_data_wr_last,
      -- AXI master
      m_tvalid => axi_out_tvalid,
      m_tready => axi_out_tready,
      m_tdata  => axi_out_tdata,
      m_tlast  => axi_out_tlast);

  -- Convert from FRAME_RAM_DATA_WIDTH to the specified data width
  output_width_conversion_u : entity fpga_cores.axi_stream_width_converter
    generic map (
      INPUT_DATA_WIDTH  => FRAME_RAM_DATA_WIDTH,
      OUTPUT_DATA_WIDTH => DATA_WIDTH,
      AXI_TID_WIDTH     => 0)
    port map (
      -- Usual ports
      clk        => clk,
      rst        => rst,
      -- AXI stream input
      s_tready   => axi_out_tready,
      s_tdata    => axi_out_tdata,
      s_tkeep    => (others => '1'),
      s_tid      => (others => '0'),
      s_tvalid   => axi_out_tvalid,
      s_tlast    => axi_out_tlast,
      -- AXI stream output
      m_tready   => m_tready,
      m_tdata    => m_tdata,
      m_tkeep    => open,
      m_tid      => open,
      m_tvalid   => m_tvalid_i,
      m_tlast    => m_tlast);

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  -- Values synchronized with data from pipeline_context_ram
  frame_bit_index_i <= to_integer(unsigned(frame_bit_index));
  s_ldpc_tready_i   <= '0' when rst = '1' or extract_frame_data = '1' else code_proc_ready;


  -- AXI slave specifics
  axi_in_dv     <= '1' when axi_in_tready = '1' and axi_in_tvalid = '1' else '0';
  ldpc_dv       <= '1' when s_ldpc_tready_i = '1' and s_ldpc_tvalid = '1' else '0';
  axi_in_tready <= '0' when rst = '1' or frame_ram_last = '1' else axi_in_tready_p;

  -- Assign internals
  s_ldpc_tready <= s_ldpc_tready_i;
  m_tvalid      <= m_tvalid_i;

  ---------------
  -- Processes --
  ---------------
  write_side_p : process(clk, rst)
    variable xored_data    : std_logic_vector(FRAME_RAM_DATA_WIDTH - 1 downto 0);
    variable has_ldpc_data : boolean := False;
    variable has_axi_data  : boolean := False;
  begin
    if rst = '1' then
      axi_in_tready_p      <= '1';
      code_proc_ready      <= '1';
      frame_bits_remaining <= (others => '0');

      encoded_data_wr_data <= (others => 'U');
      encoded_data_wr_en   <= '0';
    elsif rising_edge(clk) then

      frame_ram_en       <= '0';
      encoded_data_wr_en <= '0';
      extract_frame_data <= frame_ram_last;

      -- Always return the context, will change only when needed
      frame_ram_wrdata <= to_01(frame_ram_rddata);

      -- Frame RAM addressing depends if we're calculating the codes or extracting them
      if (extract_frame_data = '1' or frame_ram_last = '1') then
        -- Respect AXI master adapter
        if encoded_data_wr_full = '0' then

          -- When extracting frame data, we need to complete the given frame. Since the
          -- frame size is not always an integer multiple of the frame length, we also need
          -- to check if data bit cnt has wrapped (MSB is 1).
          if frame_bits_remaining = 0 or frame_bits_remaining(frame_bits_remaining'length - 1) = '1' then
            frame_addr_in      <= (others => '0');
            -- extract_frame_data <= '0';
            frame_ram_last     <= '0';
          else
            frame_ram_en       <= '1';
            frame_addr_in      <= frame_addr_in + 1;
          end if;
        end if;
      end if;

      -- When calculating the final XOR'ed value, the first bit will depend on the
      -- address. We can safely assign here and avoid extra logic levels on the FF control
      if unsigned(frame_addr_out) = 0 then
        xored_data(0) := frame_ram_rddata(0);
      else
        xored_data(0) := encoded_data_wr_data(FRAME_RAM_DATA_WIDTH - 1) xor frame_ram_rddata(0);
      end if;

      -- Handle data coming out of the frame RAM, either by using the input data of by
      -- calculating the actual final XOR'ed value
      if frame_ram_ready = '1' and extract_frame_data = '0' then
        frame_ram_wrdata(frame_bit_index_i) <= axi_in_tdata_sampled xor to_01(frame_ram_rddata(frame_bit_index_i));
      elsif frame_ram_ready = '1' and extract_frame_data = '1' then
        -- Need to clear the RAM for the next frame
        frame_ram_wrdata <= (others => '0');

        -- Calculate the final XOR between output bits
        for i in 1 to FRAME_RAM_DATA_WIDTH - 1 loop
          xored_data(i) := frame_ram_rddata(i) xor xored_data(i - 1);
        end loop;

        -- Assign data out and decrement the bits consumed
        frame_bits_remaining <= frame_bits_remaining - FRAME_RAM_DATA_WIDTH;
        encoded_data_wr_data <= xored_data;
        encoded_data_wr_en   <= '1';
      elsif frame_ram_ready = '0' and extract_frame_data = '1' then
        frame_ram_wrdata <= (others => '0');
      end if;

      -- AXI LDPC table control
      if ldpc_dv = '1' then
        ldpc_offset     <= s_ldpc_offset;
        frame_addr_in   <= unsigned(s_ldpc_offset(ROM_DATA_WIDTH - 1 downto numbits(FRAME_RAM_DATA_WIDTH)));
        has_ldpc_data   := True;
        code_proc_ready <= '0';

        if s_ldpc_tlast = '1' and extract_frame_data = '0' then
          axi_in_tready_p <= '1';
        end if;

        if s_ldpc_tlast = '1' and axi_in_completed = '1' then
          -- We'll complete the frame with enough data to complete either the short or
          -- normal frames (16,200 or 64,800 bits respectively)
          axi_in_completed <= '0';
          frame_ram_last   <= '1';
          frame_addr_in    <= (others => '0');
        end if;
      end if;

      -- AXI frame data control
      if axi_in_dv = '1' then
        has_axi_data    := True;
        axi_in_tready_p <= '0';

        -- Set the expected frame length. Data will passthrough and LDPC codes will be
        -- appended to complete the appropriate n_ldpc length (either 16,200 or 64,800).
        -- Timing-wise, the best way to do this is by setting the counter to - N + 2;
        -- whenever it gets to 1 it will have counted N items.
        if axi_in_first_word = '1' then
          if axi_in_frame_type = FECFRAME_SHORT then
            frame_bits_remaining <= to_unsigned(16_199, frame_bits_remaining'length);
          elsif axi_in_frame_type = FECFRAME_NORMAL then
            frame_bits_remaining <= to_unsigned(64_799, frame_bits_remaining'length);
          else
            report "Don't know how to handle frame type=" & quote(frame_type_t'image(axi_in_frame_type))
              severity Error;
          end if;
        else
          frame_bits_remaining <= frame_bits_remaining - 1;
        end if;

        if axi_in_tlast = '1' then
          axi_in_completed <= '1';
        end if;

      end if;

      if has_axi_data and has_ldpc_data then
        frame_ram_en    <= '1';
        frame_bit_index <= ldpc_offset(numbits(FRAME_RAM_DATA_WIDTH) - 1 downto 0);

      end if;

      if has_axi_data then
        has_ldpc_data   := False;
        code_proc_ready <= '1';
      end if;

      if has_ldpc_data then
        if ldpc_dv = '1' and s_ldpc_tlast = '0' then
          has_axi_data    := False;
          axi_in_tready_p <= '1';
        end if;
      end if;

      -- if has_ldpc_data and has_axi_data then
      --   has_axi_data    := False;
      --   has_ldpc_data   := False;
      --   axi_in_tready_p <= '1';
      --   code_proc_ready <= '1';

    end if;
  end process;

  -- The config ports are valid at the first word of the frame, but we must not rely on
  -- the user keeping it unchanged. Hide this on a block to leave the core code a bit
  -- cleaner
  config_sample_block  : block -- {{
  begin

    process(clk, rst)
    begin
      if rst = '1' then
        axi_in_first_word    <= '1';
        code_first_word      <= '1';
        axi_in_tdata_sampled <= 'U'; -- We don't want a mux with rst here
      elsif rising_edge(clk) then

        if axi_in_tready = '1' then
          axi_in_has_data <= '0';
        end if;

        if axi_in_dv = '1' then
          axi_in_has_data      <= '1';
          axi_in_first_word    <= axi_in_tlast;
          axi_in_tdata_sampled <= axi_in_tdata;
        end if;

        if ldpc_dv = '1' then
          code_first_word <= s_ldpc_tlast;
        end if;

      end if;
    end process;
  end block config_sample_block; -- }}

end axi_ldpc_encoder;

-- vim: set foldmethod=marker foldmarker=--\ {{,--\ }} :
