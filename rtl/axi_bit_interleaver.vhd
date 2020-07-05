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

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

library str_format;
use str_format.str_format_pkg.all;

------------------------
-- Entity declaration --
------------------------
entity axi_bit_interleaver is
  generic (DATA_WIDTH : positive := 8);
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
end axi_bit_interleaver;

architecture axi_bit_interleaver of axi_bit_interleaver is

  ---------------
  -- Constants --
  ---------------
  constant RAM_PTR_WIDTH : integer := 2;
  constant MAX_ROWS      : integer := 21_600 / DATA_WIDTH;
  constant MAX_COLUMNS   : integer := 5;

  type cfg_t is record
    last_row    : unsigned(numbits(MAX_ROWS) - 1 downto 0);
    last_column : unsigned(numbits(MAX_COLUMNS) - 1 downto 0);
    remainder   : unsigned(numbits(DATA_WIDTH) - 1 downto 0);
  end record;

  type addr_array_t is array (natural range <>)
    of unsigned(numbits(MAX_ROWS) + RAM_PTR_WIDTH - 1 downto 0);
  type data_array_t is array (natural range <>) of std_logic_vector(DATA_WIDTH - 1 downto 0);

  -------------
  -- Signals --
  -------------
  signal s_tready_i                : std_logic;
  signal handler_ready             : std_logic;

  signal s_axi_dv                  : std_logic;
  signal s_tlast_reg               : std_logic;

  -- RAM base pointers to handle back to back frames by writing and reading from different
  -- regions of the RAM. This will introduce 1 frame of latency though
  signal ram_ptr_diff              : unsigned(RAM_PTR_WIDTH - 1 downto 0);
  signal wr_ram_ptr                : unsigned(RAM_PTR_WIDTH - 1 downto 0);
  signal rd_ram_ptr                : unsigned(RAM_PTR_WIDTH - 1 downto 0);

  -- Write side config
  signal cfg_wr_constellation      : constellation_t;
  signal cfg_wr_frame_type         : frame_type_t;
  signal cfg_wr_code_rate          : code_rate_t;
  signal cfg_wr_cnt                : cfg_t;
  signal cfg_fifo_wren             : std_logic;
  signal cfg_fifo_full             : std_logic;
  -- Write side counters
  signal bit_cnt                   : unsigned(2*numbits(DATA_WIDTH) - 1 downto 0);
  signal wr_row_cnt                : unsigned(numbits(MAX_ROWS) - 1 downto 0);
  signal wr_column_cnt             : unsigned(numbits(MAX_COLUMNS) - 1 downto 0);
  -- signal wr_column_cnt_reg1        : unsigned(numbits(MAX_COLUMNS) - 1 downto 0);
  signal wr_remainder              : unsigned(numbits(DATA_WIDTH) - 1 downto 0);

  signal wr_data_mux               : data_array_t(2*DATA_WIDTH - 1 downto 0);
  signal last_word_mux             : data_array_t(DATA_WIDTH - 1 downto 0);

  signal wr_addr_init              : std_logic := '0';

  -- RAM write interface
  type ram_wr_t is record
    addr : unsigned(numbits(MAX_ROWS) + RAM_PTR_WIDTH - 1 downto 0);
    data : std_logic_vector(DATA_WIDTH - 1 downto 0);
    en   : std_logic;
  end record;

  type ram_wr_array_t is array (MAX_COLUMNS - 1 downto 0) of ram_wr_t;

  signal ram_wr       : ram_wr_array_t;

  signal dbg_tdata_sr : std_logic_vector(3*DATA_WIDTH - 1 downto 0);

  -- Read side config
  signal cfg_fifo_rd_constellation : constellation_t;
  signal cfg_fifo_rd_frame_rate    : frame_type_t;
  signal cfg_fifo_rd_code_rate     : code_rate_t;
  signal cfg_fifo_rden             : std_logic;
  signal cfg_fifo_empty            : std_logic;

  signal cfg_rd_constellation      : constellation_t;
  signal cfg_rd_frame_type         : frame_type_t;
  signal cfg_rd_code_rate          : code_rate_t;

  signal cfg_rd_cnt                : cfg_t;
  signal rd_row_cnt                : unsigned(numbits(MAX_ROWS) - 1 downto 0);

  signal rd_column_cnt             : unsigned(numbits(MAX_COLUMNS) - 1 downto 0);
  signal rd_column_cnt_reg         : unsigned(numbits(MAX_COLUMNS) - 1 downto 0);

  -- RAM read interface
  signal ram_rd_addr               : unsigned(numbits(MAX_ROWS) + RAM_PTR_WIDTH - 1 downto 0);
  signal ram_rd_data               : data_array_t(0 to MAX_COLUMNS - 1);

  signal rd_data_sr                : std_logic_vector(MAX_COLUMNS*DATA_WIDTH - 1 downto 0);

  signal interleaved_3c_012        : std_logic_vector(3*DATA_WIDTH - 1 downto 0);
  signal interleaved_3c_210        : std_logic_vector(3*DATA_WIDTH - 1 downto 0);
  signal interleaved_4c_0123       : std_logic_vector(4*DATA_WIDTH - 1 downto 0);
  signal interleaved_4c_3210       : std_logic_vector(4*DATA_WIDTH - 1 downto 0);
  signal interleaved_4c_3201       : std_logic_vector(4*DATA_WIDTH - 1 downto 0);
  signal interleaved_5c            : std_logic_vector(5*DATA_WIDTH - 1 downto 0);

  signal m_wr_en                   : std_logic := '0';
  signal m_wr_en_reg               : std_logic := '0';
  signal m_wr_full                 : std_logic;
  signal m_wr_data                 : std_logic_vector(DATA_WIDTH - 1 downto 0);
  signal m_wr_last                 : std_logic := '0';
  signal m_wr_last_reg             : std_logic := '0';

  signal wr_first_word             : std_logic; -- To sample config
  signal rd_first_word             : std_logic; -- To sample config

begin

  -------------------
  -- Port mappings --
  -------------------
  -- Generate 1 RAM for each column, each one gets written sequentially
  generate_rams : for column in 0 to MAX_COLUMNS - 1 generate
    -- Needed got GHDL simulation to work
    signal addr_a : std_logic_vector(numbits(MAX_ROWS) downto 0);
    signal addr_b : std_logic_vector(numbits(MAX_ROWS) downto 0);
  begin

    addr_a <= std_logic_vector(ram_wr(column).addr(numbits(MAX_ROWS) downto 0));
    addr_b <= std_logic_vector(ram_rd_addr(numbits(MAX_ROWS) downto 0));

    ram : entity fpga_cores.ram_inference
      generic map (
        ADDR_WIDTH   => numbits(MAX_ROWS) + 1,
        DATA_WIDTH   => DATA_WIDTH,
        RAM_TYPE     => "auto",
        OUTPUT_DELAY => 1)
      port map (
        -- Port A
        clk_a     => clk,
        clken_a   => '1',
        wren_a    => ram_wr(column).en,
        addr_a    => addr_a,
        wrdata_a  => ram_wr(column).data,
        rddata_a  => open,

        -- Port B
        clk_b     => clk,
        clken_b   => '1',
        addr_b    => addr_b,
        rddata_b  => ram_rd_data(column));
  end generate generate_rams;

  -- Interleaved data takes 1 cycle after the address has changed, add support for
  -- a couple of cycles to stop the pipeline
  axi_master_adapter_u : entity fpga_cores.axi_stream_master_adapter
    generic map (
      MAX_SKEW_CYCLES => 3,
      TDATA_WIDTH     => DATA_WIDTH)
    port map (
      -- Usual ports
      clk      => clk,
      reset    => rst,
      -- Wannabe AXI interface
      wr_en    => m_wr_en_reg,
      wr_full  => m_wr_full,
      wr_data  => m_wr_data,
      wr_last  => m_wr_last_reg,
      -- AXI master
      m_tvalid => m_tvalid,
      m_tready => m_tready,
      m_tdata  => m_tdata,
      m_tlast  => m_tlast);

  -- First word samples config, whenever it's deasserted we can write the config
  wr_en_gen: entity fpga_cores.edge_detector
    generic map (
      SYNCHRONIZE_INPUT => False,
      OUTPUT_DELAY      => 0)
    port map (
      -- Usual ports
      clk     => clk,
      clken   => '1',
      --
      din     => wr_first_word,
      -- Edges detected
      rising  => open,
      falling => cfg_fifo_wren,
      toggle  => open);

  rd_en_gen: entity fpga_cores.edge_detector
    generic map (
      SYNCHRONIZE_INPUT => False,
      OUTPUT_DELAY      => 0)
    port map (
      -- Usual ports
      clk     => clk,
      clken   => '1',
      --
      din     => rd_first_word,
      -- Edges detected
      rising  => open,
      falling => cfg_fifo_rden,
      toggle  => open);

  -- Write and read side might be on different timings, use a small FIFO to pass config
  -- across
  cfg_fifo_u: entity work.config_fifo
    generic map ( FIFO_DEPTH => 2 )
    port map (
      -- Usual ports
      clk     => clk,
      rst     => rst,

      -- Write side
      wr_en           => cfg_fifo_wren,
      full            => cfg_fifo_full,
      constellation_i => cfg_wr_constellation,
      frame_type_i    => cfg_wr_frame_type,
      code_rate_i     => cfg_wr_code_rate,

      rd_en           => cfg_fifo_rden,
      empty           => cfg_fifo_empty,
      constellation_o => cfg_fifo_rd_constellation,
      frame_type_o    => cfg_fifo_rd_frame_rate,
      code_rate_o     => cfg_fifo_rd_code_rate);

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  m_wr_data <= mirror_bits(rd_data_sr(DATA_WIDTH - 1 downto 0));

  -- Assign the interleaved data statically
  iter_rows : for row in 0 to DATA_WIDTH - 1 generate
    iter_3_columns : for column in 0 to 2 generate
      interleaved_3c_012(3*DATA_WIDTH - (3 * row + column) - 1) <= ram_rd_data(column)(DATA_WIDTH - row - 1);
      interleaved_3c_210(3*DATA_WIDTH - (3 * row + column) - 1) <= ram_rd_data(2 - column)(DATA_WIDTH - row - 1);
    end generate;

    interleaved_4c_0123(4*DATA_WIDTH - (4 * row + 0) - 1) <= ram_rd_data(0)(DATA_WIDTH - row - 1);
    interleaved_4c_0123(4*DATA_WIDTH - (4 * row + 1) - 1) <= ram_rd_data(1)(DATA_WIDTH - row - 1);
    interleaved_4c_0123(4*DATA_WIDTH - (4 * row + 2) - 1) <= ram_rd_data(2)(DATA_WIDTH - row - 1);
    interleaved_4c_0123(4*DATA_WIDTH - (4 * row + 3) - 1) <= ram_rd_data(3)(DATA_WIDTH - row - 1);

    interleaved_4c_3210(4*DATA_WIDTH - (4 * row + 0) - 1) <= ram_rd_data(3)(DATA_WIDTH - row - 1);
    interleaved_4c_3210(4*DATA_WIDTH - (4 * row + 1) - 1) <= ram_rd_data(2)(DATA_WIDTH - row - 1);
    interleaved_4c_3210(4*DATA_WIDTH - (4 * row + 2) - 1) <= ram_rd_data(1)(DATA_WIDTH - row - 1);
    interleaved_4c_3210(4*DATA_WIDTH - (4 * row + 3) - 1) <= ram_rd_data(0)(DATA_WIDTH - row - 1);


    interleaved_4c_3201(4*DATA_WIDTH - (4 * row + 0) - 1) <= ram_rd_data(3)(DATA_WIDTH - row - 1);
    interleaved_4c_3201(4*DATA_WIDTH - (4 * row + 1) - 1) <= ram_rd_data(2)(DATA_WIDTH - row - 1);
    interleaved_4c_3201(4*DATA_WIDTH - (4 * row + 2) - 1) <= ram_rd_data(0)(DATA_WIDTH - row - 1);
    interleaved_4c_3201(4*DATA_WIDTH - (4 * row + 3) - 1) <= ram_rd_data(1)(DATA_WIDTH - row - 1);

    iter_5_columns : for column in 0 to 4 generate
      interleaved_5c(5*DATA_WIDTH - (5 * row + column) - 1) <= ram_rd_data(column)(DATA_WIDTH - row - 1);
    end generate iter_5_columns;
  end generate iter_rows;

  ram_ptr_diff    <= wr_ram_ptr - rd_ram_ptr when wr_ram_ptr > rd_ram_ptr else
                     2**RAM_PTR_WIDTH + wr_ram_ptr - rd_ram_ptr;

  -- s_tready_i      <= handler_ready when ram_ptr_diff < 2 else '0';
  s_tready_i      <= handler_ready and not cfg_fifo_full;

  s_axi_dv        <= '1' when s_tready_i = '1' and s_tvalid = '1' else '0';

  -- Pack addr/data/en just for visualizing in the waveform
  -- g_dbg_ram_wr : for i in 0 to MAX_COLUMNS - 1 generate
  --   ram_wr(i) <= (addr => ram_wr_addr(i), data => ram_wr_data(i), en => ram_wr_en(i));
  -- end generate;

  -- Assign internals
  s_tready        <= s_tready_i;

  --------------------------------
  -- Handle write side pointers --
  --------------------------------
  write_side_p : process(clk, rst)
    variable bit_cnt_v       : unsigned(bit_cnt'range) := (others => '0');
    variable wr_column_cnt_i : natural range 0 to MAX_COLUMNS - 1;
    variable tdata_sr        : std_logic_vector(dbg_tdata_sr'range);
  begin
    if rst = '1' then
      wr_column_cnt <= (others => '0');
      wr_row_cnt    <= (others => '0');
      wr_remainder  <= (others => '0');
      wr_ram_ptr    <= (others => '0');

      bit_cnt       <= (others => '0');

      handler_ready <= '1';
      wr_addr_init  <= '1';

      -- FIXME: Revisit this in the future; for some reason GHDL Synth and/or Yosys fail
      -- if we don't add this (see https://github.com/ghdl/ghdl/issues/1136)
      -- ram_wr_data  <= (others => (others => 'U'));

      for i in ram_wr'range loop
        ram_wr(i) <= (addr => (others => 'U'),
                          data => (others => 'U'),
                          en => '0');
      end loop;

      dbg_tdata_sr <= (others => 'U');
      tdata_sr     := (others => 'U');

    elsif rising_edge(clk) then

      handler_ready <= '1';

      -- Only to reduce footprint of converting to integer when slicing vectors
      wr_column_cnt_i := to_integer(wr_column_cnt);

      if s_axi_dv = '1' then
        tdata_sr      := tdata_sr(tdata_sr'length - DATA_WIDTH - 1 downto 0) & s_tdata;
        bit_cnt_v     := bit_cnt_v + DATA_WIDTH;
        s_tlast_reg   <= s_tlast;
        if s_tlast = '1' then
          handler_ready <= '0';
        end if;
      end if;

      -- Increment RAM addr every time it gets written
      for column in 0 to MAX_COLUMNS - 1 loop
        ram_wr(column).en <= '0';
        if ram_wr(column).en = '1' then
          ram_wr(column).addr <= ram_wr(column).addr + 1;
        end if;
      end loop;

      if s_tlast_reg = '1' then

        debug(
          sformat(
            "ram_wr(%d).data <= tdata_sr(%d downto 0) & (%d downto 0 => '0');",
            fo(wr_column_cnt_i),
            fo(minimum(to_integer(bit_cnt_v), DATA_WIDTH) - 1),
            fo(to_integer(DATA_WIDTH - minimum(bit_cnt_v, DATA_WIDTH) - 1))
          )
        );


        s_tlast_reg                  <= '0';
        ram_wr(wr_column_cnt_i).en   <= '1';
        ram_wr(wr_column_cnt_i).data <= tdata_sr(minimum(to_integer(bit_cnt_v), DATA_WIDTH) - 1 downto 0) & (DATA_WIDTH - minimum(to_integer(bit_cnt_v), DATA_WIDTH) - 1 downto 0 => '0');
        tdata_sr                     := (others => 'U');
        bit_cnt_v                    := (others => '0');
        wr_column_cnt                <= (others => '0');
        wr_row_cnt                   <= (others => '0');
        wr_ram_ptr                   <= wr_ram_ptr + 1;
        wr_addr_init                 <= '1';
      elsif bit_cnt_v >= DATA_WIDTH then

        ram_wr(wr_column_cnt_i).en   <= '1';
        ram_wr(wr_column_cnt_i).data <= tdata_sr(to_integer(bit_cnt_v) - 1 downto to_integer(bit_cnt_v) - DATA_WIDTH);
        wr_addr_init                 <= '0';

        -- Initialize each RAM's initial write address at every first row
        if wr_addr_init = '1' then
          ram_wr(wr_column_cnt_i).addr <= wr_ram_ptr & (numbits(MAX_ROWS) - 1 downto 0 => '0');
        end if;

        wr_row : if wr_row_cnt /= cfg_wr_cnt.last_row then
          wr_row_cnt <= wr_row_cnt + 1;
          bit_cnt_v  := bit_cnt_v - DATA_WIDTH;
        else
          wr_addr_init <= '1';
          wr_row_cnt   <= (others => '0');
          wr_remainder <= wr_remainder + cfg_wr_cnt.remainder;

          bit_cnt_v := bit_cnt_v - cfg_wr_cnt.remainder;

          -- Chain counters
          if wr_column_cnt = cfg_wr_cnt.last_column then
            wr_column_cnt <= (others => '0');
            wr_ram_ptr    <= wr_ram_ptr + 1;
          else
            wr_column_cnt <= wr_column_cnt + 1;
          end if;

          -- if s_tlast_reg = '1' then
          --   error(sformat("bit_cnt_v=%d", fo(bit_cnt_v)));
          --   bit_cnt_v     := (others => '0');
          -- end if;

        end if wr_row;

        if bit_cnt_v >= DATA_WIDTH then
          handler_ready <= '0';
        end if;

      end if;

      bit_cnt      <= bit_cnt_v;
      dbg_tdata_sr <= tdata_sr;

    end if;
  end process write_side_p;

  -------------------------------
  -- Handle read side pointers --
  -------------------------------
  read_side_p : process(clk, rst)

    -------------------------------------------------------------------------------------
    -- Read side has a slightly different set of counter limits
    function get_rd_cnt_max_values (
      constant constellation : in constellation_t;
      constant frame_type    : in frame_type_t) return cfg_t is
      variable rows          : natural;
      variable columns       : natural;
      variable remainder     : natural;
    begin

      if frame_type = fecframe_normal then
        if constellation = mod_8psk then
          rows := 21_600;
        elsif constellation = mod_16apsk then
          rows := 16_200;
        elsif constellation = mod_32apsk then
          rows := 12_960;
        end if;
      elsif frame_type = fecframe_short then
        if constellation = mod_8psk then
          rows := 5_400;
        elsif constellation = mod_16apsk then
          rows := 4_050;
        elsif constellation = mod_32apsk then
          rows := 3_240;
        end if;
      end if;

      if constellation = mod_8psk then
        columns := 3;
      elsif constellation = mod_16apsk then
        columns := 4;
      elsif constellation = mod_32apsk then
        columns := 5;
      end if;

      remainder := rows mod DATA_WIDTH;
      rows := rows / DATA_WIDTH;

      if remainder /= 0 then
        rows := rows + 1;
      end if;

      return (last_row    => to_unsigned(rows, numbits(MAX_ROWS)) - 1,
              last_column => to_unsigned(columns, numbits(MAX_COLUMNS)) - 1,
              remainder   => to_unsigned(remainder, numbits(DATA_WIDTH)));

    end function get_rd_cnt_max_values;
    -------------------------------------------------------------------------------------

  begin
    if rst = '1' then
      m_wr_en       <= '0';
      m_wr_last     <= '0';

      rd_row_cnt    <= (others => '0');
      rd_column_cnt <= (others => '0');
      rd_ram_ptr    <= (others => '0');

      ram_rd_addr   <= (others => '0');

      rd_first_word <= '1';

    elsif clk'event and clk = '1' then

      m_wr_en   <= '0';
      m_wr_last <= '0';

      -- Data comes out of the RAMs 1 cycle after the address has changed, need to keep
      -- track of the actual value being handled
      rd_column_cnt_reg <= rd_column_cnt;
      m_wr_en_reg       <= m_wr_en;
      m_wr_last_reg     <= m_wr_last;

      if rd_first_word = '1' then
        cfg_rd_cnt <= get_rd_cnt_max_values(cfg_fifo_rd_constellation, cfg_fifo_rd_frame_rate);

        -- Sample data form the config FIFO whenever we read from it
        cfg_rd_constellation <= cfg_fifo_rd_constellation;
        cfg_rd_frame_type    <= cfg_fifo_rd_frame_rate;
        cfg_rd_code_rate     <= cfg_fifo_rd_code_rate;

      end if;

      -- If pointers are different and the AXI adapter has space, keep writing
      if m_wr_full = '0' and rd_ram_ptr /= wr_ram_ptr then

        rd_first_word <= '0';
        m_wr_en       <= '1';

        -- Read pointers control logic
        if rd_column_cnt /= cfg_rd_cnt.last_column then
          rd_column_cnt <= rd_column_cnt + 1;
        else
          rd_column_cnt <= (others => '0');

          if rd_row_cnt /= cfg_rd_cnt.last_row then
            rd_row_cnt  <= rd_row_cnt + 1;
            ram_rd_addr <= ram_rd_addr + 1;
          else
            rd_row_cnt    <= (others => '0');
            rd_ram_ptr    <= rd_ram_ptr + 1;
            m_wr_last     <= '1';
            ram_rd_addr   <= (rd_ram_ptr + 1) & (numbits(MAX_ROWS) - 1 downto 0 => '0');
            rd_first_word <= '1';
          end if;
        end if;

        if cfg_rd_cnt.remainder /= 0 and rd_row_cnt + 1 > cfg_rd_cnt.last_row then
          rd_column_cnt <= (others => '0');
          rd_row_cnt    <= (others => '0');
          rd_ram_ptr    <= rd_ram_ptr + 1;
          m_wr_last     <= '1';
          ram_rd_addr   <= (rd_ram_ptr + 1) & (numbits(MAX_ROWS) - 1 downto 0 => '0');
          rd_first_word <= '1';
        end if;

      end if;

      if m_wr_en = '1' then
        if rd_column_cnt_reg = 0 then
          -- Assign to undefined so we can track in simulation the parts that we not
          -- assigned
          rd_data_sr <= (others => 'U');
          -- We'll swap byte ordering (e.g ABCD becomes DCBA) because it's easier to
          -- assign the write data from the shift register's LSB
          if cfg_rd_constellation = mod_8psk then
            if cfg_rd_code_rate = C3_5 then
              rd_data_sr(interleaved_3c_210'range) <= mirror_bits(interleaved_3c_210);
            else
              rd_data_sr(interleaved_3c_012'range) <= mirror_bits(interleaved_3c_012);
            end if;

          elsif cfg_rd_constellation = mod_16apsk then
            -- DVB-S2 doesn't specify a different interleaving sequence for code rate 3/5
            -- and 16 APSK, only 8 PSK but DVB-S2(X) does
            if cfg_rd_code_rate = C3_5 then
              if cfg_rd_frame_type = fecframe_normal then
                rd_data_sr(interleaved_4c_3210'range) <= mirror_bits(interleaved_4c_3210);
              else
                rd_data_sr(interleaved_4c_3201'range) <= mirror_bits(interleaved_4c_3201);
              end if;
            else
              rd_data_sr(interleaved_4c_0123'range) <= mirror_bits(interleaved_4c_0123);
            end if;

          elsif cfg_rd_constellation = mod_32apsk then
            rd_data_sr <= mirror_bits(interleaved_5c);
          end if;

        else
          -- We'll write the LSB, shift data right
          rd_data_sr <= (DATA_WIDTH - 1 downto 0 => 'U')
            & rd_data_sr(rd_data_sr'length - 1 downto DATA_WIDTH);

        end if;

      end if;

    end if;
  end process read_side_p;

  cfg_sample_block : block
  begin

    process(clk, rst)
    begin
      if rst = '1' then
        wr_first_word   <= '1';
      elsif rising_edge(clk) then
        -- Data handling will use the delayed AXI data -- can register config safely
        if s_axi_dv = '1' then
          wr_first_word  <= s_tlast;
        end if;
      end if;
    end process;

    -- TODO: We're loading the config instantly to get the interleaving done. There should
    -- be a cycle at least to allow using BRAMs
    process(s_axi_dv, wr_first_word, cfg_constellation, cfg_frame_type, cfg_code_rate)
      -------------------------------------------------------------------------------------
      function get_wr_cnt_max_values (
        constant constellation : in constellation_t;
        constant frame_type    : in frame_type_t) return cfg_t is
        variable rows          : natural;
        variable columns       : natural;
        variable remainder     : natural;
      begin

        if frame_type = fecframe_normal then
          if constellation = mod_8psk then
            rows := 21_600;
          elsif constellation = mod_16apsk then
            rows := 16_200;
          elsif constellation = mod_32apsk then
            rows := 12_960;
          end if;
        elsif frame_type = fecframe_short then
          if constellation = mod_8psk then
            rows := 5_400;
          elsif constellation = mod_16apsk then
            rows := 4_050;
          elsif constellation = mod_32apsk then
            rows := 3_240;
          end if;
        end if;

        if constellation = mod_8psk then
          columns := 3;
        elsif constellation = mod_16apsk then
          columns := 4;
        elsif constellation = mod_32apsk then
          columns := 5;
        end if;

        return (last_row    => to_unsigned(rows / DATA_WIDTH, numbits(MAX_ROWS)) - 0,
                last_column => to_unsigned(columns, numbits(MAX_COLUMNS)) - 1,
                remainder   => to_unsigned(rows mod DATA_WIDTH, numbits(DATA_WIDTH)));

      end function get_wr_cnt_max_values;
    begin
      if s_axi_dv = '1' and wr_first_word = '1' then
        cfg_wr_cnt           <= get_wr_cnt_max_values(cfg_constellation, cfg_frame_type);
        cfg_wr_constellation <= cfg_constellation;
        cfg_wr_frame_type    <= cfg_frame_type;
        cfg_wr_code_rate     <= cfg_code_rate;
      end if;
    end process;

    process(dbg_tdata_sr)
    begin
        for i in 0 to wr_data_mux'length - 1 loop

          wr_data_mux(i) <= dbg_tdata_sr(DATA_WIDTH + i - 1 downto i);
        end loop;
    end process;

    process(clk)
      variable data : std_logic_vector(DATA_WIDTH - 1 downto 0);
    begin
      if rising_edge(clk) then

        if s_axi_dv = '1' then
          -- wr_data_mux(0) <= s_tdata;

          -- Unwrap muxes explicitly, Vivado doesn't seem to understand if we do this on
          -- the fly. Note that index 0 is data itself and is not assigned within this loop
          for i in 0 to last_word_mux'length - 1 loop

            data := dbg_tdata_sr(DATA_WIDTH + i - 1 downto i);
            -- the last word will use the MSB of the actual word
            last_word_mux(i) <= data(DATA_WIDTH - 1 downto DATA_WIDTH - i) &
                                (DATA_WIDTH - i - 1 downto 0 => '0');
          end loop;

          -- for i in 0 to DATA_WIDTH - 1 loop
          --   last_word_mux(i) <= s_tdata(DATA_WIDTH - 1 downto DATA_WIDTH - i) &
          --                       (DATA_WIDTH - i - 1 downto 0 => '0');
          -- end loop;

        end if;
      end if;
    end process;


  end block;

end axi_bit_interleaver;
