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

-- vunit: run_all_in_same_sim

use std.textio.all;

library ieee;
use ieee.std_logic_1164.all;
use ieee.numeric_std.all;

library vunit_lib;
context vunit_lib.vunit_context;
context vunit_lib.com_context;

library osvvm;
use osvvm.RandomPkg.all;

library str_format;
use str_format.str_format_pkg.all;

library fpga_cores;
use fpga_cores.common_pkg.all;
use fpga_cores.file_utils_pkg.all;
use fpga_cores.testbench_utils_pkg.all;

use work.dvb_sim_utils_pkg.all;
use work.dvb_utils_pkg.all;
use work.ldpc_pkg.all;
use work.ldpc_tables_pkg.all;

library fpga_cores;

entity axi_ldpc_encoder_tb is
  generic (
    RUNNER_CFG            : string;
    TEST_CFG              : string;
    NUMBER_OF_TEST_FRAMES : integer := 8);
end axi_ldpc_encoder_tb;

architecture axi_ldpc_encoder_tb of axi_ldpc_encoder_tb is

  ---------------
  -- Constants --
  ---------------
  constant configs               : config_array_t := get_test_cfg(TEST_CFG);

  constant DATA_WIDTH            : integer := 8;

  constant FILE_READER_NAME      : string := "file_reader";
  constant FILE_CHECKER_NAME     : string := "file_checker";
  constant CLK_PERIOD            : time := 5 ns;
  constant ERROR_CNT_WIDTH       : integer := 8;

  -- constant table       : integer_2d_array_t := DVB_64800_S2_B6;
  -- constant LDPC_Q      : natural := 60;
  -- constant ldpc_length : natural := 64_800 - 43_200; --16200 - 12600;

  constant table       : integer_2d_array_t := DVB_16200_S2_C8_T2_B6;
  constant LDPC_Q      : natural := 10;
  constant ldpc_length : natural := 16200 - 12600;



  -------------
  -- Signals --
  -------------
  -- Usual ports
  signal clk                : std_logic := '1';
  signal rst                : std_logic;

  signal cfg_constellation  : constellation_t;
  signal cfg_frame_type     : frame_type_t;
  signal cfg_code_rate      : code_rate_t;

  signal tvalid_probability : real range 0.0 to 1.0 := 1.0;
  signal tready_probability : real range 0.0 to 1.0 := 1.0;

  -- AXI input
  signal m_tready           : std_logic := '1';
  signal m_tvalid           : std_logic;
  signal m_tdata            : std_logic_vector(DATA_WIDTH - 1 downto 0);
  signal m_tlast            : std_logic;
  signal m_data_valid       : boolean;

  -- AXI output
  signal s_tvalid           : std_logic;
  signal s_tdata            : std_logic_vector(DATA_WIDTH - 1 downto 0);
  signal s_tlast            : std_logic;
  signal s_tready           : std_logic := '1';
  signal s_data_valid       : boolean;

  signal expected_tdata     : std_logic_vector(DATA_WIDTH - 1 downto 0);
  signal expected_tlast     : std_logic;
  signal tdata_error_cnt    : std_logic_vector(ERROR_CNT_WIDTH - 1 downto 0);
  signal tlast_error_cnt    : std_logic_vector(ERROR_CNT_WIDTH - 1 downto 0);
  signal error_cnt          : std_logic_vector(ERROR_CNT_WIDTH - 1 downto 0);

  signal s_ldpc_offset : std_logic_vector(numbits(max(DVB_N_LDPC)) - 1 downto 0);
  signal s_ldpc_tuser  : std_logic_vector(numbits(max(DVB_N_LDPC)) - 1 downto 0);
  signal s_ldpc_tvalid : std_logic;
  signal s_ldpc_tlast  : std_logic;
  signal s_ldpc_tready : std_logic := '1';

begin

  -------------------
  -- Port mappings --
  -------------------
  dut : entity work.axi_ldpc_encoder
    generic map ( DATA_WIDTH => DATA_WIDTH )
    port map (
      -- Usual ports
      clk               => clk,
      rst               => rst,

      cfg_constellation => cfg_constellation,
      cfg_frame_type    => cfg_frame_type,
      cfg_code_rate     => cfg_code_rate,

      s_ldpc_offset     => s_ldpc_offset,
      s_ldpc_tuser      => s_ldpc_tuser,
      s_ldpc_tvalid     => s_ldpc_tvalid,
      s_ldpc_tlast      => s_ldpc_tlast,
      s_ldpc_tready     => s_ldpc_tready,

      -- AXI input
      s_tvalid          => m_tvalid,
      s_tdata           => m_tdata,
      s_tlast           => m_tlast,
      s_tready          => m_tready,

      -- AXI output
      m_tready          => s_tready,
      m_tvalid          => s_tvalid,
      m_tlast           => s_tlast,
      m_tdata           => s_tdata);


  -- AXI file read
  axi_file_reader_u : entity fpga_cores.axi_file_reader
    generic map (
      READER_NAME => FILE_READER_NAME,
      DATA_WIDTH  => DATA_WIDTH)
    port map (
      -- Usual ports
      clk                => clk,
      rst                => rst,
      -- Config and status
      completed          => open,
      tvalid_probability => tvalid_probability,

      -- Data output
      m_tready           => m_tready,
      m_tdata            => m_tdata,
      m_tvalid           => m_tvalid,
      m_tlast            => m_tlast);

  axi_file_compare_u : entity fpga_cores.axi_file_compare
    generic map (
      READER_NAME     => FILE_CHECKER_NAME,
      ERROR_CNT_WIDTH => ERROR_CNT_WIDTH,
      REPORT_SEVERITY => Warning,
      DATA_WIDTH      => DATA_WIDTH)
    port map (
      -- Usual ports
      clk                => clk,
      rst                => rst,
      -- Config and status
      tdata_error_cnt    => tdata_error_cnt,
      tlast_error_cnt    => tlast_error_cnt,
      error_cnt          => error_cnt,
      tready_probability => tready_probability,
      -- Debug stuff
      expected_tdata     => expected_tdata,
      expected_tlast     => expected_tlast,
      -- Data input
      s_tready           => s_tready,
      s_tdata            => s_tdata,
      s_tvalid           => s_tvalid,
      s_tlast            => s_tlast);

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  clk <= not clk after CLK_PERIOD/2;

  test_runner_watchdog(runner, 250 us);

  m_data_valid <= m_tvalid = '1' and m_tready = '1';
  s_data_valid <= s_tvalid = '1' and s_tready = '1';

  ---------------
  -- Processes --
  ---------------
  main : process -- {{
    constant self         : actor_t := new_actor("main");
    constant input_cfg_p  : actor_t := find("input_cfg_p");
    variable file_checker : file_reader_t := new_file_reader(FILE_CHECKER_NAME);
    ------------------------------------------------------------------------------------
    procedure walk(constant steps : natural) is
    begin
      if steps /= 0 then
        for step in 0 to steps - 1 loop
          wait until rising_edge(clk);
        end loop;
      end if;
    end procedure walk;

    ------------------------------------------------------------------------------------
    procedure run_test (
      constant config           : config_t;
      constant number_of_frames : in positive) is
      variable file_reader_msg  : msg_t;
      variable ldpc_table_msg   : msg_t;
    begin

      info("Running test with:");
      info(" - constellation  : " & constellation_t'image(config.constellation));
      info(" - frame_type     : " & frame_type_t'image(config.frame_type));
      info(" - code_rate      : " & code_rate_t'image(config.code_rate));
      info(" - input_file     : " & config.files.input);
      info(" - reference_file : " & config.files.reference);

      for i in 0 to number_of_frames - 1 loop
        -- File reader message
        file_reader_msg := new_msg;
        file_reader_msg.sender := self;

        push(file_reader_msg, config.files.input);
        push(file_reader_msg, config.constellation);
        push(file_reader_msg, config.frame_type);
        push(file_reader_msg, config.code_rate);

        send(net, input_cfg_p, file_reader_msg);

        -- Table write message
        ldpc_table_msg := new_msg;
        ldpc_table_msg.sender := self;

        push(ldpc_table_msg, config.constellation);
        push(ldpc_table_msg, config.frame_type);
        push(ldpc_table_msg, config.code_rate);

        send(net, find("ldpc_table_p"), ldpc_table_msg);

        enqueue_file(
          net,
          file_checker,
          config.files.reference,
          "1:8"
        );

      end loop;

    end procedure run_test;

    ------------------------------------------------------------------------------------
    procedure wait_for_transfers ( constant count : in natural) is
      variable msg : msg_t;
    begin
      -- Will get one response for each frame from the file checker and one for the input
      -- config. The order shouldn't matter
      receive(net, self, msg);
      -- Failure(sformat("Got reply from '%s'", name(msg.sender)));

      wait_all_read(net, file_checker);
    end procedure wait_for_transfers;
    ------------------------------------------------------------------------------------

  begin

    test_runner_setup(runner, RUNNER_CFG);
    show(display_handler, debug);

    while test_suite loop
      rst <= '1';
      walk(4);
      rst <= '0';
      walk(4);

      tvalid_probability <= 1.0;
      tready_probability <= 1.0;

      -- set_timeout(runner, configs'length * NUMBER_OF_TEST_FRAMES * 500 us);

      if run("back_to_back") then
        tvalid_probability <= 1.0;
        tready_probability <= 1.0;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => 1);
        end loop;

        wait_for_transfers(configs'length);

        walk(128);


      elsif run("slow_master") then
        tvalid_probability <= 0.5;
        tready_probability <= 1.0;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => NUMBER_OF_TEST_FRAMES);
        end loop;
        wait_for_transfers(configs'length);

      elsif run("slow_slave") then
        tvalid_probability <= 1.0;
        tready_probability <= 0.5;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => NUMBER_OF_TEST_FRAMES);
        end loop;
        wait_for_transfers(configs'length);

      elsif run("both_slow") then
        tvalid_probability <= 0.75;
        tready_probability <= 0.75;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => NUMBER_OF_TEST_FRAMES);
        end loop;
        wait_for_transfers(configs'length);

      end if;

      walk(1);

      check_false(has_message(input_cfg_p));

      -- check_equal(error_cnt, 0);

      walk(32);

    end loop;

    test_runner_cleanup(runner);
    wait;
  end process; -- }}

  input_cfg_p : process -- {{
    constant self        : actor_t := new_actor("input_cfg_p");
    constant main        : actor_t := find("main");
    variable cfg_msg     : msg_t;
    variable file_reader : file_reader_t := new_file_reader(FILE_READER_NAME);
  begin

    receive(net, self, cfg_msg);

    -- Configure the file reader
    enqueue_file(net, file_reader, pop(cfg_msg), "1:8");

    wait until rising_edge(clk);

    -- Keep the config stuff active for a single cycle to make sure blocks use the correct
    -- values
    cfg_constellation <= pop(cfg_msg);
    cfg_frame_type    <= pop(cfg_msg);
    cfg_code_rate     <= pop(cfg_msg);
    wait until m_data_valid and m_tlast = '0' and rising_edge(clk);
    cfg_constellation <= not_set;
    cfg_frame_type    <= not_set;
    cfg_code_rate     <= not_set;

    wait until m_data_valid and m_tlast = '1';

    -- When this is received, the file reader has finished reading the file
    wait_file_read(net, file_reader);

    -- If there's no more messages, notify the main process that we're done here
    if not has_message(self) then
      cfg_msg := new_msg;
      push(cfg_msg, True);
      cfg_msg.sender := self;
      send(net, main, cfg_msg);
    end if;
    -- check_equal(error_cnt, 0);
  end process; -- }}

  ldpc_table_p : process
    constant self : actor_t := new_actor("ldpc_table_p");
    constant main : actor_t := find("main");
    variable msg  : msg_t;

    ------------------------------------------------------------------
    procedure write_cell (
      constant offset : natural;
      constant tuser  : natural;
      constant last   : boolean := False) is
    begin
      s_ldpc_offset <= std_logic_vector(to_unsigned(offset, s_ldpc_offset'length));
      s_ldpc_tuser  <= std_logic_vector(to_unsigned(tuser, s_ldpc_tuser'length));
      s_ldpc_tvalid <= '1';
      if last then
        s_ldpc_tlast <= '1';
      end if;

      wait until s_ldpc_tvalid = '1' and s_ldpc_tready = '1' and rising_edge(clk);

      s_ldpc_offset <= (others => 'U');
      s_ldpc_tuser  <= (others => 'U');
      s_ldpc_tvalid <= '0';
      s_ldpc_tlast  <= '0';
    end;

    ------------------------------------------------------------------
    procedure write_config ( constant msg : msg_t ) is
      constant constellation : constellation_t    := pop(msg);
      constant frame_type    : frame_type_t       := pop(msg);
      constant code_rate     : code_rate_t        := pop(msg);
      -- -- TODO: Table should be retrieved from above parameters
      -- constant table         : integer_2d_array_t := DVB_64800_S2_B6;
      constant lines         : natural            := table'length;
      variable bit_index     : natural            := 0;

      ------------------------------------------------------------------
      procedure write_line(
        constant q         : natural;
        constant L         : fpga_cores.common_pkg.integer_array_t) is
        constant rows      : natural := L(0);
      begin
        -- info(sformat("Line has %d rows", fo(rows)));
        for group_cnt in 0 to 359 loop
          for cell in 1 to rows loop
              write_cell((L(cell) + (bit_index mod 360) * q) mod ldpc_length, bit_index, last => cell = rows);
  --           ptr := (table(line_no)(row) + (bit_index mod 360) * LDPC_Q) mod ldpc_length;
            -- write_cell(L(cell) + (bit_index mod 360) * q, group_cnt, last => cell = rows);
          end loop;
          bit_index := bit_index + 1;
        end loop;
      end;

    begin
      for table_line in table'range loop
        -- info(sformat("Writing line %d", fo(table_line)));
        write_line(q => LDPC_Q, L => table(table_line));
      end loop;
    end;


  begin
    s_ldpc_tlast  <= '0';
    s_ldpc_tvalid <= '0';

    wait until rst = '0';

    -- while True loop
      receive(net, self, msg);
      write_config(msg);
    -- end loop;

  end process;

  -- ----------------------------------------------------------------------------------------------------------
  -- dbg_proc_linear : process
  --   constant logger : logger_t := get_logger("dbg_proc_linear");
  --   variable mem    : std_logic_vector(ldpc_length - 1 downto 0);

  --   procedure accumulate_ldpc (
  --     constant table     : in integer_2d_array_t;
  --     variable data      : out std_logic_vector(ldpc_length - 1 downto 0)) is
  --     variable rows      : natural := 0;
  --     variable ptr       : natural := 0;
  --     variable bit_index : natural := 0;
  --   begin
  --     data := (others => '0');

  --     for line_no in table'range loop
  --       rows := table(line_no)(0);

  --       for group_cnt in 0 to 359 loop
  --         wait until rising_edge(clk) and m_tvalid = '1' and m_tready = '1';

  --         for row in 1 to table(line_no)'length - 1 loop
  --           ptr := (table(line_no)(row) + (bit_index mod 360) * LDPC_Q) mod ldpc_length;

  --           if (bit_index mod 360) < 4 then
  --             info(
  --               logger,
  --               sformat(
  --                 "bit #%d: %s => line_no=%d, row=%d, ptr = %d",
  --                 fo(bit_index),
  --                 fo(m_tdata(0)),
  --                 fo(line_no),
  --                 fo(row),
  --                 fo(ptr)));
  --           end if;

  --           data(ptr) := m_tdata(0) xor data(ptr);

  --         end loop;

  --         if (bit_index mod 360) < 4 then
  --           info(logger, "                                                              ");
  --         end if;

  --         if m_tlast = '1' then
  --           info(
  --             logger,
  --             sformat(
  --               "Exiting at line_no=%d / %d, bit %d",
  --               fo(line_no),
  --               fo(table'length),
  --               fo(bit_index)));
  --           return;
  --         end if;

  --         bit_index := bit_index + 1;
  --       end loop;

  --     end loop;

  --   end ;

  -- begin
  --   mem := (others => '0');

  --   wait until rst = '0';

  --   while True loop
  --     accumulate_ldpc(table, mem);

  --     for i in mem'range loop
  --       if mem(i) /= '1' and mem(i) /= '0' then
  --         error(logger, sformat("Bit %d value is %s", fo(i), fo(mem(i))));
  --       end if;
  --     end loop;

  --     info(logger, "Before post XOR:");
  --     for byte in 0 to 7 loop
  --       info(
  --         logger,
  --         sformat(
  --           "%d  | %r  | %b",
  --           fo(byte),
  --           fo(mirror_bits(mem(8*(byte + 1) - 1 downto 8*byte))),
  --           fo(mirror_bits(mem(8*(byte + 1) - 1 downto 8*byte)))));
  --     end loop;

  --     for i in 1 to mem'length - 1 loop
  --       mem(i) := mem(i) xor mem(i - 1);
  --     end loop;

  --     info(logger, "Post XOR:");
  --     for byte in 0 to 7 loop
  --       info(
  --         logger,
  --         sformat(
  --           "%d  | %r  | %b",
  --           fo(byte),
  --           fo(mirror_bits(mem(8*(byte + 1) - 1 downto 8*byte))),
  --           fo(mirror_bits(mem(8*(byte + 1) - 1 downto 8*byte)))));
  --     end loop;



  --   end loop;

  --   -- wait until rising_edge(clk);
  --   -- if rst = '0' then
  --   --   check_equal(error_cnt, 0, sformat("Expected 0 errors but got %d", fo(error_cnt)));
  --   -- end if;
  -- end process;


  ----------------------------------------------------------------------------------------------------------
  -- dbg_proc_array : process
  --   constant logger : logger_t := get_logger("dbg_proc_array");
  --   variable mem    : std_logic_vector_2d_t(ldpc_length/16 - 1 downto 0)(15 downto 0);

  --   procedure accumulate_ldpc (
  --     constant table     : in integer_2d_array_t;
  --     -- variable data      : out std_logic_vector(ldpc_length - 1 downto 0)) is
  --     variable data      : out std_logic_vector_2d_t(ldpc_length/16 - 1 downto 0)(15 downto 0)) is
  --     variable rows      : natural := 0;
  --     variable ptr       : natural := 0;
  --     variable ptr_addr  : natural := 0;
  --     variable ptr_bit   : natural := 0;
  --     variable bit_index : natural := 0;
  --   begin
  --     data := (others => (others => '0'));

  --     for line_no in table'range loop
  --       rows := table(line_no)(0);

  --       for group_cnt in 0 to 359 loop
  --         wait until rising_edge(clk) and m_tvalid = '1' and m_tready = '1';

  --         for row in 1 to table(line_no)'length - 1 loop
  --           ptr := (table(line_no)(row) + (bit_index mod 360) * LDPC_Q) mod ldpc_length;

  --           ptr_addr := ptr / 16;
  --           ptr_bit  := ptr mod 16;

  --           if (bit_index mod 360) < 8 then
  --             info(
  --               logger,
  --               sformat(
  --                 "bit #%d: %s => line_no=%d, row=%d, ptr = %d (%d, %d)",
  --                 fo(bit_index),
  --                 fo(m_tdata),
  --                 fo(line_no),
  --                 fo(row),
  --                 fo(ptr),
  --                 fo(ptr_addr),
  --                 fo(ptr_bit)
  --               ));
  --           end if;

  --           data(ptr_addr)(ptr_bit) := m_tdata xor data(ptr_addr)(ptr_bit);

  --         end loop;

  --         if (bit_index mod 360) < 8 then
  --           info(logger, "                                                              ");
  --         end if;

  --         if m_tlast = '1' then
  --           info(logger, sformat("Exiting at line_no=%d / %d, bit %d", fo(line_no), fo(table'length), fo(bit_index)));
  --           return;
  --         end if;

  --         bit_index := bit_index + 1;
  --       end loop;

  --     end loop;

  --   end;

  --   --------------------------------------------------------------------------------

  --   impure function post_xor (
  --     constant data : std_logic_vector_2d_t(ldpc_length/16 - 1 downto 0)(15 downto 0))
  --     return std_logic_vector_2d_t is
  --     variable result : std_logic_vector_2d_t(ldpc_length/16 - 1 downto 0)(15 downto 0);
  --     variable addr   : natural;
  --     variable offset : natural;
  --   begin

  --     result := data;

  --     for i in 1 to ldpc_length - 1 loop
  --       addr := i / 16;
  --       offset := i mod 16;

  --       result(addr)(offset) := result(addr)(offset) xor result((i - 1) / 16)((i - 1) mod 16);

  --       -- if addr = 0 then
  --       --   debug(
  --       --     logger,
  --       --     sformat(
  --       --       "result(%d)(%d) := result(%d)(%d) xor result(%d)(%d) ==> " &
  --       --       "result(%d)(%d) := %r xor %r => %r",
  --       --       fo(addr), fo(offset),
  --       --       fo(addr), fo(offset),
  --       --       fo((i - 1) / 16), fo((i - 1) mod 16),
  --       --       fo(addr), fo(offset),
  --       --       fo(result(addr)(offset)), fo(result((i - 1) / 16)((i - 1) mod 16)),
  --       --       fo(result(addr)(offset))
  --       --     )
  --       --   );
  --       -- end if;

  --     end loop;
  --     return result;
  --   end;

  -- begin
  --   mem := (others => (others => '0'));

  --   wait until rst = '0';

  --   while True loop
  --     accumulate_ldpc(table, mem);

  --     info(logger, "Before post XOR:");
  --     for word in 0 to 7 loop
  --       info(
  --         logger,
  --         sformat(
  --           "%3d | %r  | %b || mirrored: %r | %b",
  --           fo(word),
  --           fo(mem(word)),
  --           fo(mem(word)),
  --           fo(mirror_bits(mem(word))),
  --           fo(mirror_bits(mem(word)))
  --         )
  --       );
  --     end loop;

  --     mem := post_xor(mem);

  --     info(logger, "Post XOR:");
  --     for word in 0 to 7 loop
  --       info(
  --         logger,
  --         sformat(
  --           "%3d | %r  | %b || mirrored: %r | %b",
  --           fo(word),
  --           fo(mem(word)),
  --           fo(mem(word)),
  --           fo(mirror_bits(mem(word))),
  --           fo(mirror_bits(mem(word)))
  --         ));
  --     end loop;

  --   end loop;

  --   -- wait until rising_edge(clk);
  --   -- if rst = '0' then
  --   --   check_equal(error_cnt, 0, sformat("Expected 0 errors but got %d", fo(error_cnt)));
  --   -- end if;
  -- end process;

end axi_ldpc_encoder_tb;

-- vim: set foldmethod=marker foldmarker=--\ {{,--\ }} :
