-- vim: set foldmethod=marker foldmarker=--\ {{,--\ }} :
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
use fpga_cores.axi_pkg.all;

library fpga_cores_sim;
use fpga_cores_sim.file_utils_pkg.all;
use fpga_cores_sim.testbench_utils_pkg.all;
use fpga_cores_sim.axi_stream_bfm_pkg.all;

use work.dvb_sim_utils_pkg.all;
use work.dvb_utils_pkg.all;
use work.ldpc_pkg.all;
use work.ldpc_tables_pkg.all;

library modelsim_lib;
use modelsim_lib.util.all;

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
  constant AXI_TABLE_BFM         : string := "axi_table";
  constant CLK_PERIOD            : time := 5 ns;
  constant ERROR_CNT_WIDTH       : integer := 8;

  -- constant table       : integer_2d_array_t := DVB_64800_S2_B6;
  -- constant LDPC_Q      : natural := 60;
  -- constant ldpc_length : natural := 64_800 - 43_200; --16200 - 12600;

  constant cfg_table       : integer_2d_array_t := DVB_16200_S2_C8_T2_B6;
  constant cfg_LDPC_Q      : natural := 10;
  constant cfg_ldpc_length : natural := 16200 - 12600;

  -------------
  -- Signals --
  -------------
  -- Usual ports
  signal clk                : std_logic := '1';
  signal rst                : std_logic;

  signal m_frame_cnt        : natural := 0;
  signal m_word_cnt         : natural := 0;
  signal m_bit_cnt          : natural := 0;

  signal s_frame_cnt        : natural := 0;
  signal s_word_cnt         : natural := 0;
  signal s_bit_cnt          : natural := 0;

  signal tdata_error_cnt    : std_logic_vector(ERROR_CNT_WIDTH - 1 downto 0);
  signal tlast_error_cnt    : std_logic_vector(ERROR_CNT_WIDTH - 1 downto 0);
  signal error_cnt          : std_logic_vector(ERROR_CNT_WIDTH - 1 downto 0);

  signal cfg_constellation  : constellation_t;
  signal cfg_frame_type     : frame_type_t;
  signal cfg_code_rate      : code_rate_t;

  signal data_probability   : real range 0.0 to 1.0 := 1.0;
  signal table_probability  : real range 0.0 to 1.0 := 1.0;
  signal tready_probability : real range 0.0 to 1.0 := 1.0;

  signal axi_ldpc           : axi_stream_bus_t(tdata(numbits(max(DVB_N_LDPC)) - 1 downto 0),
                                               tuser(numbits(max(DVB_N_LDPC)) downto 0));

  -- AXI input
  signal axi_master         : axi_stream_data_bus_t(tdata(DATA_WIDTH - 1 downto 0));
  -- AXI output
  signal axi_slave          : axi_stream_data_bus_t(tdata(DATA_WIDTH - 1 downto 0));
  signal m_data_valid       : boolean;
  signal s_data_valid       : boolean;

  signal expected_tdata     : std_logic_vector(DATA_WIDTH - 1 downto 0);
  signal expected_tlast     : std_logic;

  type ram_t is array (0 to 4095) of std_logic_vector(15 downto 0);
  signal dut_ram : ram_t;


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

      s_ldpc_offset     => axi_ldpc.tdata,
      s_ldpc_next       => axi_ldpc.tuser(axi_ldpc.tuser'length - 1),
      s_ldpc_tuser      => axi_ldpc.tuser(axi_ldpc.tuser'length - 2 downto 0),
      s_ldpc_tvalid     => axi_ldpc.tvalid,
      s_ldpc_tlast      => axi_ldpc.tlast,
      s_ldpc_tready     => axi_ldpc.tready,

      -- AXI input
      s_tvalid          => axi_master.tvalid,
      s_tdata           => axi_master.tdata,
      s_tlast           => axi_master.tlast,
      s_tready          => axi_master.tready,

      -- AXI output
      m_tready          => axi_slave.tready,
      m_tvalid          => axi_slave.tvalid,
      m_tlast           => axi_slave.tlast,
      m_tdata           => axi_slave.tdata);

  axi_table_u : entity fpga_cores_sim.axi_stream_bfm
    generic map (
      NAME        => AXI_TABLE_BFM,
      TDATA_WIDTH => axi_ldpc.tdata'length,
      TUSER_WIDTH => axi_ldpc.tuser'length,
      TID_WIDTH   => 0)
    port map (
      -- Usual ports
      clk      => clk,
      rst      => rst,
      -- AXI stream output
      m_tready => axi_ldpc.tready,
      m_tdata  => axi_ldpc.tdata,
      m_tuser  => axi_ldpc.tuser,
      m_tkeep  => open,
      m_tid    => open,
      m_tvalid => axi_ldpc.tvalid,
      m_tlast  => axi_ldpc.tlast);

  -- AXI file read
  axi_file_reader_u : entity fpga_cores_sim.axi_file_reader
    generic map (
      READER_NAME => FILE_READER_NAME,
      DATA_WIDTH  => DATA_WIDTH)
    port map (
      -- Usual ports
      clk                => clk,
      rst                => rst,
      -- Config and status
      completed          => open,
      tvalid_probability => data_probability,

      -- Data output
      m_tready           => axi_master.tready,
      m_tdata            => axi_master.tdata,
      m_tvalid           => axi_master.tvalid,
      m_tlast            => axi_master.tlast);

  axi_file_compare_u : entity fpga_cores_sim.axi_file_compare
    generic map (
      READER_NAME     => FILE_CHECKER_NAME,
      ERROR_CNT_WIDTH => ERROR_CNT_WIDTH,
      REPORT_SEVERITY => Error,
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
      s_tready           => axi_slave.tready,
      s_tdata            => axi_slave.tdata,
      s_tvalid           => axi_slave.tvalid,
      s_tlast            => axi_slave.tlast);

  ------------------------------
  -- Asynchronous assignments --
  ------------------------------
  clk <= not clk after CLK_PERIOD/2;

  test_runner_watchdog(runner, 25000 us);

  m_data_valid <= axi_master.tvalid = '1' and axi_master.tready = '1';
  s_data_valid <= axi_slave.tvalid = '1' and axi_slave.tready = '1';

  ---------------
  -- Processes --
  ---------------
  main : process -- {{
    constant self         : actor_t := new_actor("main");
    constant input_cfg_p  : actor_t := find("input_cfg_p");
    variable file_checker : file_reader_t := new_file_reader(FILE_CHECKER_NAME);

    variable table_bfm    : axi_stream_bfm_t := create_bfm(AXI_TABLE_BFM);

    subtype data_tuple_constr_t
      is data_tuple_t(tdata(axi_ldpc.tdata'range), tuser(axi_ldpc.tuser'range));

    subtype data_tuple_array_constr_t
      is data_tuple_array_t(open)(tdata(axi_ldpc.tdata'range), tuser(axi_ldpc.tuser'range));

    -- It's not trivial to determine the length of the AXI frame, so we'll use a linked list
    package data_array_pkg is new fpga_cores_sim.linked_list_pkg generic map (type_t => data_tuple_constr_t);

    procedure walk(constant steps : natural) is -- {{ ----------------------------------
    begin
      if steps /= 0 then
        for step in 0 to steps - 1 loop
          wait until rising_edge(clk);
        end loop;
      end if;
    end procedure walk; -- }} ----------------------------------------------------------

    procedure write_ldpc_table ( -- {{ --------------------------------------------------
      constant config : config_t
    ) is

      variable dbg_enough    : boolean := False;

      impure function get_axi_table_data return data_tuple_array_constr_t is -- {{ -----
        constant table : ldpc_table_t := get_ldpc_table(config.frame_type, config.code_rate);
        variable list  : data_array_pkg.linked_list_t;

        impure function populate_list return integer is -- {{ --------------------------
          variable bit_index : natural  := 0;
          variable rows      : natural;
          variable offset    : natural;
        begin
          -- info(sformat("Line has %d rows", fo(rows)));
          for i in table.data'range loop
            rows := table.data(i)(0);

            for group_cnt in 0 to 359 loop
              -- write(msg, sformat("[items=%5d] Group: %4d || ", fo(list.size), fo(group_cnt)));
              for cell in 1 to rows loop
                offset := (table.data(i)(cell) + (bit_index mod 360) * table.q) mod table.length;
                -- write(msg, sformat("%4d ", fo(offset)));

                list.push_back(
                  (
                    tdata => std_logic_vector(to_unsigned(offset, axi_ldpc.tdata'length)),
                    tuser => from_boolean(cell = rows) &
                             std_logic_vector(to_unsigned(bit_index, axi_ldpc.tuser'length - 1))
                  )
                );

              end loop;

              -- if not dbg_enough then
              --   info(msg.all);
              --   deallocate(msg);
              --   msg := null;
              -- end if;

              bit_index := bit_index + 1;

            end loop;

            -- assert dbg_enough or msg = null;
          end loop;

          return list.size;
        end function; -- }} ------------------------------------------------------------

        -- Populate the list to get its actual length, then read from the
        -- linked list to fill it
        constant words  : natural := populate_list;
        variable result : data_tuple_array_constr_t(0 to words - 1);

      begin

        warning(sformat("list has %d items", fo(list.size)));

        for i in 0 to words - 1 loop
          result(i) := list.pop_front;
        end loop;

        assert list.empty;

        return result;

      end; -- }} -----------------------------------------------------------------------

      constant data : data_tuple_array_constr_t := get_axi_table_data;

    begin

      info("Sending frame to AXI BFM write");

      axi_bfm_write(net,
        bfm         => table_bfm,
        data        => data,
        probability => table_probability,
        blocking    => False);
    end procedure; -- }} ---------------------------------------------------------------

    procedure run_test ( -- {{ ---------------------------------------------------------
      constant config           : config_t;
      constant number_of_frames : in positive) is
      variable file_reader_msg  : msg_t;
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

        write_ldpc_table(config);

        enqueue_file(
          net,
          file_checker,
          config.files.reference,
          "1:8"
        );

      end loop;

    end procedure run_test; -- }} ------------------------------------------------------

    procedure wait_for_transfers is -- {{ ----------------------------------------------
      variable msg : msg_t;
    begin
      receive(net, self, msg);
      wait_all_read(net, file_checker);

      join(net, table_bfm);

      wait until rising_edge(clk) and axi_slave.tvalid = '0' for 1 ms;

      walk(1);

    end procedure wait_for_transfers; -- }} --------------------------------------------

  begin

    test_runner_setup(runner, RUNNER_CFG);
    show(display_handler, debug);

    while test_suite loop
      rst                <= '1';
      data_probability   <= 1.0;
      table_probability  <= 1.0;
      tready_probability <= 1.0;

      walk(32);

      rst <= '0';

      walk(32);

      set_timeout(runner, 1 ms);

      if run("back_to_back") then
        data_probability   <= 1.0;
        table_probability  <= 1.0;
        tready_probability <= 1.0;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => 1);
        end loop;

      -- wait_for_transfers;

      -- walk(256);

      -- for i in configs'range loop
      --   run_test(configs(i), number_of_frames => 1);
      -- end loop;


      elsif run("data=0.5,table=1.0,slave=1.0") then
        data_probability   <= 0.5;
        table_probability  <= 1.0;
        tready_probability <= 1.0;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => NUMBER_OF_TEST_FRAMES);
        end loop;

      elsif run("data=1.0,table=1.0,slave=0.5") then
        data_probability   <= 1.0;
        table_probability  <= 1.0;
        tready_probability <= 0.5;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => NUMBER_OF_TEST_FRAMES);
        end loop;

      elsif run("data=0.75,table=1.0,slave=0.75") then
        data_probability   <= 0.75;
        table_probability  <= 1.0;
        tready_probability <= 0.75;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => NUMBER_OF_TEST_FRAMES);
        end loop;

      elsif run("data=1.0,table=0.5,slave=1.0") then
        data_probability   <= 1.0;
        table_probability  <= 0.5;
        tready_probability <= 1.0;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => NUMBER_OF_TEST_FRAMES);
        end loop;

      elsif run("data=1.0,table=0.75,slave=0.75") then
        data_probability   <= 1.0;
        table_probability  <= 0.75;
        tready_probability <= 0.75;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => NUMBER_OF_TEST_FRAMES);
        end loop;

      elsif run("data=0.8,table=0.8,slave=0.8") then
        data_probability   <= 0.8;
        table_probability  <= 0.8;
        tready_probability <= 0.8;

        for i in configs'range loop
          run_test(configs(i), number_of_frames => NUMBER_OF_TEST_FRAMES);
        end loop;

      end if;

      wait_for_transfers;

      check_false(has_message(input_cfg_p));

      check_equal(axi_slave.tvalid, '0', "axi_slave.tvalid should be '0'");
      check_equal(error_cnt, 0);

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
    wait until m_data_valid and axi_master.tlast = '0' and rising_edge(clk);
    cfg_constellation <= not_set;
    cfg_frame_type    <= not_set;
    cfg_code_rate     <= not_set;

    wait until m_data_valid and axi_master.tlast = '1';

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

  cnt_p : process -- {{ ----------------------------------------------------------------
  begin
    wait until rst = '0';
    while True loop
      wait until rising_edge(clk);

      if s_data_valid then
        s_word_cnt <= s_word_cnt + 1;
        s_bit_cnt  <= s_bit_cnt + DATA_WIDTH;

        if axi_slave.tlast = '1' then
          s_word_cnt  <= 0;
          s_bit_cnt   <= 0;
          s_frame_cnt <= s_frame_cnt + 1;
        end if;
      end if;

      if m_data_valid then
        m_word_cnt <= m_word_cnt + 1;
        m_bit_cnt  <= m_bit_cnt + DATA_WIDTH;

        if axi_master.tlast = '1' then
          m_word_cnt  <= 0;
          m_bit_cnt   <= 0;
          m_frame_cnt <= m_frame_cnt + 1;
        end if;
      end if;
    end loop;
  end process; -- }} -------------------------------------------------------------------

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
  --         wait until rising_edge(clk) and axi_master.tvalid = '1' and axi_master.tready = '1';

  --         for row in 1 to table(line_no)'length - 1 loop
  --           ptr := (table(line_no)(row) + (bit_index mod 360) * LDPC_Q) mod ldpc_length;

  --           if (bit_index mod 360) < 4 then
  --             info(
  --               logger,
  --               sformat(
  --                 "bit #%d: %s => line_no=%d, row=%d, ptr = %d",
  --                 fo(bit_index),
  --                 fo(axi_master.tdata(0)),
  --                 fo(line_no),
  --                 fo(row),
  --                 fo(ptr)));
  --           end if;

  --           data(ptr) := axi_master.tdata(0) xor data(ptr);

  --         end loop;

  --         if (bit_index mod 360) < 4 then
  --           info(logger, "                                                              ");
  --         end if;

  --         if axi_master.tlast = '1' then
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


    dbg_proc_array : process -- {{ -------------------------------------------------------
    constant logger : logger_t := get_logger("dbg_proc_array");

    variable mem    : std_logic_vector_2d_t(cfg_ldpc_length/16 - 1 downto 0)(15 downto 0);

    constant offset_checker_p : actor_t := find("offset_checker_p");

    procedure notify (
      constant offset : in natural;
      constant data   : in std_logic) is
      variable msg    : msg_t := new_msg;
    begin
      push(msg, offset);
      push(msg, data);
      send(net, offset_checker_p, msg);
    end procedure;


    procedure accumulate_ldpc ( -- {{ --------------------------------------------------
      constant table            : in integer_2d_array_t;
      variable codes            : out std_logic_vector_2d_t(cfg_ldpc_length/16 - 1 downto 0)(15 downto 0)) is
      variable rows             : natural := 0;
      variable offset           : natural := 0;
      variable offset_addr      : natural := 0;
      variable offset_bit       : natural := 0;
      variable input_bit_number : natural := 0;
      variable data_index       : natural := 0;
      variable tdata            : std_logic_vector(DATA_WIDTH - 1 downto 0);
      variable data_bit         : std_logic;
    begin
      codes := (others => (others => '0'));

      for line_no in table'range loop
        rows := table(line_no)(0);

        for group_cnt in 0 to 359 loop

          if data_index = 0 then
            wait until rising_edge(clk) and axi_master.tvalid = '1' and axi_master.tready = '1';
            tdata := axi_master.tdata;
          end if;

          data_bit := tdata(DATA_WIDTH - 1 - data_index);

          if data_index = DATA_WIDTH - 1 then
            data_index := 0;
          else
            data_index := data_index + 1;
          end if;

          for row in 1 to rows loop
            offset := (table(line_no)(row) + (input_bit_number mod 360) * cfg_LDPC_Q) mod cfg_ldpc_length;

            offset_addr := offset / 16;
            offset_bit  := offset mod 16;

            codes(offset_addr)(offset_bit) := data_bit xor codes(offset_addr)(offset_bit);

            notify(offset, codes(offset_addr)(offset_bit));

            -- if offset = 1078 then
            -- if axi_master.tlast = '1' then
            -- if input_bit_number < 32 or axi_master.tlast = '1' then
            --   debug(
            --     logger,
            --     sformat(
            --       "[%2d] codes(%4d)(%2d) = %r  || axi_master.tdata = %r (offset = %5d, data_bit = %r)",
            --       fo(data_index),
            --       fo(offset_addr),
            --       fo(offset_bit),
            --       fo(codes(offset_addr)),
            --       fo(tdata),
            --       fo(offset),
            --       fo(data_bit)
            --     )
            --   );
            -- end if;

          end loop;

          -- info(logger, "");-- {{-- }}

          if axi_master.tlast = '1' then
            info(
              logger,
              sformat(
                "Exiting at line_no=%d / %d, bit %d. Last offset was %d",
                fo(line_no),
                fo(table'length - 1),
                fo(input_bit_number),
                fo(offset)
              )
            );

            return;
          end if;

          input_bit_number := input_bit_number + 1;
        end loop;

      end loop;

    end procedure; -- }} ---------------------------------------------------------------

    impure function post_xor ( -- {{ ---------------------------------------------------
      constant data : std_logic_vector_2d_t(cfg_ldpc_length/16 - 1 downto 0)(15 downto 0))
      return std_logic_vector_2d_t is
      variable result : std_logic_vector_2d_t(cfg_ldpc_length/16 - 1 downto 0)(15 downto 0);
      variable addr   : natural;
      variable offset : natural;
    begin

      result := data;

      for i in 1 to cfg_ldpc_length - 1 loop
        addr := i / 16;
        offset := i mod 16;

        result(addr)(offset) := result(addr)(offset) xor result((i - 1) / 16)((i - 1) mod 16);

        -- if addr = 0 then
        --   debug(
        --     logger,
        --     sformat(
        --       "result(%d)(%d) := result(%d)(%d) xor result(%d)(%d) ==> " &
        --       "result(%d)(%d) := %r xor %r => %r",
        --       fo(addr), fo(offset),
        --       fo(addr), fo(offset),
        --       fo((i - 1) / 16), fo((i - 1) mod 16),
        --       fo(addr), fo(offset),
        --       fo(result(addr)(offset)), fo(result((i - 1) / 16)((i - 1) mod 16)),
        --       fo(result(addr)(offset))
        --     )
        --   );
        -- end if;

      end loop;
      return result;
    end function; -- }} ----------------------------------------------------------------

    procedure compare_ram is -- {{ -----------------------------------------------------
      variable errors : natural := 0;
    begin
      -- for i in 0 to mem'length - 1 loop
      for i in 0 to 7 loop
        if to_01(dut_ram(i)) /= to_01(mem(i)) then
          warning(
            logger,
            sformat("[%4d] dut_ram = %r, mem = %r", fo(i), fo(dut_ram(i)), fo(mem(i))
          )
        );
        errors := errors + 1;
        end if;
      end loop;

      if errors = 0 then
        info(logger, "DUT RAM and TB RAM match");
      else
          warning(
            logger,
            sformat("DUT RAM and TB RAM have %d (%d \%) differences", fo(errors), fo(100*errors / mem'length)
          )
        );
      end if;

    end procedure; -- }} ---------------------------------------------------------------

  begin
    mem := (others => (others => '0'));

    wait until rst = '0';

    init_signal_spy("/axi_ldpc_encoder_tb/dut/frame_ram_u/ram_u/ram", "/axi_ldpc_encoder_tb/dut_ram", 1);

    while True loop
      mem := (others => (others => '0'));
      accumulate_ldpc(cfg_table, mem);

      compare_ram;

      info(logger, "Before post XOR:");
      for word in 0 to 7 loop
      -- for word in 16#40# to 16#50# loop
        info(
          logger,
          sformat(
            "%3d | %r  | %b || mirrored: %r | %b",
            fo(word),
            fo(mem(word)),
            fo(mem(word)),
            fo(mirror_bits(mem(word))),
            fo(mirror_bits(mem(word)))
          )
        );
      end loop;

      mem := post_xor(mem);

      info(logger, "Post XOR:");
      for word in 0 to 7 loop
      -- for word in 16#40# to 16#50# loop
        info(
          logger,
          sformat(
            "%3d | %r  | %b || mirrored: %r | %b",
            fo(word),
            fo(mem(word)),
            fo(mem(word)),
            fo(mirror_bits(mem(word))),
            fo(mirror_bits(mem(word)))
          ));
      end loop;

    end loop;

    -- wait until rising_edge(clk);
    -- if rst = '0' then
    --   check_equal(error_cnt, 0, sformat("Expected 0 errors but got %d", fo(error_cnt)));
    -- end if;
  end process; -- }} -------------------------------------------------------------------

  -- This will only work on ModelSim
  frame_ram_monitor : block
    constant logger    : logger_t := get_logger("frame_ram_monitor");
    signal ram_we      : std_logic;
    signal ram_wr_addr : std_logic_vector(11 downto 0);
    signal ram_wr_data : std_logic_vector(15 downto 0);
  begin

    signal_spy_p : process
    begin
      wait until rst = '0';
      init_signal_spy("/axi_ldpc_encoder_tb/dut/frame_ram_u/ram_u/wren_a", "ram_we", 1);
      init_signal_spy("/axi_ldpc_encoder_tb/dut/frame_ram_u/ram_u/addr_a", "ram_wr_addr", 1);
      init_signal_spy("/axi_ldpc_encoder_tb/dut/frame_ram_u/ram_u/wrdata_a", "ram_wr_data", 1);
      wait;
    end process;

    check_p : process
      constant self     : actor_t := new_actor("offset_checker_p");
      variable msg      : msg_t;
      variable offset   : natural;
      variable exp_addr : std_logic_vector(11 downto 0);
      variable exp_data : std_logic;
      variable errors   : natural := 0;
    begin
      wait until rst = '0';

      while True loop
        receive(net, self, msg);

        wait until rising_edge(clk) and ram_we = '1';

        offset   := pop(msg);
        exp_data := pop(msg);

        if unsigned(ram_wr_addr) /= ( offset / 16 ) or ram_wr_data( offset mod 16 ) /= exp_data then
          errors := errors + 1;
          error(
            logger,
            sformat(
              "[%3d] Offset = %4d (%3d, %2d) || ram_wr_addr = %4d || ram_wr_data = %r (%r) || exp_data = %r",
              fo(errors),
              fo(offset),
              fo(offset / 16),
              fo(offset mod 16),
              fo(ram_wr_addr),
              fo(ram_wr_data),
              fo(ram_wr_data(offset mod 16)),
              fo(exp_data)
            )
          );
        -- else
        --   debug(logger, sformat("Cool, addr=%r, data=%r, matched", fo(ram_wr_addr), fo(ram_wr_data)));

        end if;

      end loop;
    end process;

  end block;

end axi_ldpc_encoder_tb;
