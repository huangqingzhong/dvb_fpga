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

use work.common_pkg.all;
use work.testbench_utils_pkg.all;

entity axi_stream_width_converter_tb is
  generic (
    RUNNER_CFG        : string;
    INPUT_DATA_WIDTH  : natural := 16;
    OUTPUT_DATA_WIDTH : natural := 16);
end axi_stream_width_converter_tb;

architecture axi_stream_width_converter_tb of axi_stream_width_converter_tb is

  ---------------
  -- Constants --
  ---------------
  constant CLK_PERIOD        : time := 5 ns;
  constant ERROR_CNT_WIDTH   : natural := 8;
  constant INPUT_BYTE_WIDTH  : natural := (INPUT_DATA_WIDTH + 7) / 8;
  constant OUTPUT_BYTE_WIDTH : natural := (OUTPUT_DATA_WIDTH + 7) / 8;
  constant AXI_TID_WIDTH     : natural := 8;

  type frame_t is record
    id   : std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
    data : std_logic_vector_2d_t;
  end record;

  procedure push(msg : msg_t; frame : frame_t ) is
  begin
    info(sformat("Pushing ID %r, data is %d x %d", fo(frame.id), fo(frame.data'length), fo(frame.data(0)'length)));
    push(msg, frame.id);
    push(msg, frame.data);
  end;

  impure function pop(msg : msg_t) return frame_t is
    constant id   : std_logic_vector      := pop(msg);
    -- constant data : std_logic_vector_2d_t := pop(msg);
  begin
    info(sformat("Popped ID %r", fo(id)));
    return frame_t'(id => id, data => pop(msg));
    -- data);
  end;

  -------------
  -- Signals --
  -------------
  -- Usual ports
  signal clk                : std_logic := '1';
  signal rst                : std_logic;

  signal tvalid_probability : real range 0.0 to 1.0 := 1.0;
  signal tready_probability : real range 0.0 to 1.0 := 1.0;

  -- AXI input
  signal m_tready           : std_logic := '1';
  signal m_tvalid           : std_logic;
  signal m_tdata            : std_logic_vector(INPUT_DATA_WIDTH - 1 downto 0);
  signal m_tkeep            : std_logic_vector(INPUT_BYTE_WIDTH - 1 downto 0);
  signal m_tid              : std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
  signal m_tlast            : std_logic;
  signal m_data_valid       : boolean;

  signal s_tready           : std_logic := '1';
  signal s_tvalid           : std_logic;
  signal s_tdata            : std_logic_vector(OUTPUT_DATA_WIDTH - 1 downto 0);
  signal s_tkeep            : std_logic_vector(OUTPUT_BYTE_WIDTH - 1 downto 0);
  signal s_tid              : std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
  signal s_tlast            : std_logic;
  signal s_data_valid       : boolean;

begin

  -------------------
  -- Port mappings --
  -------------------
  dut : entity work.axi_stream_width_converter
    generic map (
      INPUT_DATA_WIDTH  => INPUT_DATA_WIDTH,
      OUTPUT_DATA_WIDTH => OUTPUT_DATA_WIDTH)
    port map (
      -- Usual ports
      clk      => clk,
      rst      => rst,
      -- AXI stream input
      s_tready => m_tready,
      s_tdata  => m_tdata,
      s_tkeep  => m_tkeep,
      s_tid    => m_tid,
      s_tvalid => m_tvalid,
      s_tlast  => m_tlast,
      -- AXI stream output
      m_tready => s_tready,
      m_tdata  => s_tdata,
      m_tkeep  => s_tkeep,
      m_tid    => s_tid,
      m_tvalid => s_tvalid,
      m_tlast  => s_tlast);

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
    constant self : actor_t := new_actor("main");
    variable rand : RandomPType;

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
    impure function random ( constant length : natural ) return byte_array_t is
      variable result : std_logic_vector_2d_t(0 to length - 1)(7 downto 0);
    begin
      for i in 0 to length - 1 loop
        result(i) := rand.RandSlv(8);
      end loop;
      return result;
    end;

    ------------------------------------------------------------------------------------
    impure function counter ( constant length : natural ) return std_logic_vector_2d_t is
      variable result : std_logic_vector_2d_t(0 to length - 1)(7 downto 0);
    begin
      for i in 0 to length - 1 loop
        result(i) := std_logic_vector(to_unsigned(i, 8));
      end loop;
      return result;
    end;

    ------------------------------------------------------------------
    procedure write (
      constant data : std_logic_vector(INPUT_DATA_WIDTH - 1 downto 0);
      constant mask : std_logic_vector(INPUT_BYTE_WIDTH - 1 downto 0);
      variable id   : std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
      constant last : boolean := False) is
    begin
      debug(sformat("Writing: %r %r %s", fo(data), fo(mask), fo(last)));

      m_tdata   <= data;
      m_tkeep   <= mask;
      m_tid     <= id;
      m_tvalid  <= '1';
      if last then
        m_tlast <= '1';
      end if;

      wait until m_tvalid = '1' and m_tready = '1' and rising_edge(clk);

      m_tdata  <= (others => 'U');
      m_tkeep  <= (others => 'U');
      m_tid    <= (others => 'U');
      m_tvalid <= '0';
      m_tlast  <= '0';
    end;

    ------------------------------------------------------------------------------------
    procedure write_frame ( constant frame : frame_t ) is
      variable word : std_logic_vector(INPUT_DATA_WIDTH - 1 downto 0);
      variable mask : std_logic_vector(INPUT_BYTE_WIDTH - 1 downto 0);
      variable byte : natural;
      variable id   : std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
    begin

      id := frame.id;

      for i in 0 to frame.data'length - 1 loop
        byte := i mod INPUT_BYTE_WIDTH;

        word(8*(byte + 1) - 1 downto 8*byte) := frame.data(i);

        if ((i + 1) mod INPUT_BYTE_WIDTH) = 0 then
          if i /= frame.data'length - 1 then
            write(word, (others => '0'), id, False);
          else
            write(word, (others => '1'), id, True);
          end if;

          word := (others => 'U');
          id := (others => 'U');
        end if;
      end loop;

      if byte = 1 then
        return;
      end if;

      mask                := (others => '0');
      mask(byte downto 0) := (others => '1');

      write(word, mask, id, True);

    end;
    ------------------------------------------------------------------------------------
    procedure send_frame ( constant frame : frame_t ) is
      variable msg : msg_t := new_msg(sender => self);
    begin
      info("Sending frame");
      push(msg, frame);
      send(net, find("checker_p"), msg);
    end;

    ------------------------------------------------------------------------------------
    procedure test_frame ( constant id   : std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
                           constant data : byte_array_t ) is
    begin
      send_frame((id, data));
      write_frame((id, data));
    end;
    ------------------------------------------------------------------------------------

  begin

    m_tvalid <= '0';
    m_tlast  <= '0';

    rand.InitSeed("some_seed");

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

        test_frame(id => rand.RandSlv(AXI_TID_WIDTH), data => counter(17));

      end if;

      walk(32);

    end loop;

    test_runner_cleanup(runner);
    wait;
  end process; -- }}

  checker_p : process -- {{
    constant self      : actor_t := new_actor("checker_p");
    constant logger    : logger_t := get_logger("checker_p");
    constant main      : actor_t := find("main");
    variable msg       : msg_t;
    variable frame_cnt : natural := 0;

    ------------------------------------------------------------------------------------
    procedure check_frame ( constant frame : frame_t ) is
      variable exp_tdata : std_logic_vector(INPUT_DATA_WIDTH - 1 downto 0);
      variable exp_tkeep : std_logic_vector(INPUT_BYTE_WIDTH - 1 downto 0);
      variable byte      : natural;
      variable word_cnt  : natural := 0;
      variable failed    : boolean := False;

      ------------------------------------------------------------------------------------
      procedure check_word (
        constant data : std_logic_vector(INPUT_DATA_WIDTH - 1 downto 0);
        constant mask : std_logic_vector(INPUT_BYTE_WIDTH - 1 downto 0);
        constant id   : std_logic_vector(AXI_TID_WIDTH - 1 downto 0);
        constant last : boolean := False) is
      begin
          wait until s_tvalid = '1' and s_tready = '1' and rising_edge(clk);

          if data /= s_tdata then
            warning(
              logger,
              sformat(
                "TDATA ERROR @ frame %d, word %d: Got %r, expected %r",
                fo(frame_cnt),
                fo(word_cnt),
                fo(s_tdata),
                fo(data)
              )
            );
            failed := True;
          end if;

          if id /= s_tid then
            warning(
              logger,
              sformat(
                "TID ERROR @ frame %d, word %d: Got %r, expected %r",
                fo(frame_cnt),
                fo(word_cnt),
                fo(s_tid),
                fo(id)
              )
            );
            failed := True;
          end if;

          if mask /= s_tkeep then
            warning(
              logger,
              sformat(
                "TKEEP ERROR @ frame %d, word %d: Got %r, expected %r",
                fo(frame_cnt),
                fo(word_cnt),
                fo(s_tkeep),
                fo(mask)
              )
            );
            failed := True;
          end if;

          if (last and s_tlast /= '1') or (not last and s_tlast /= '0') then
            warning(
              logger,
              sformat(
                "TLAST ERROR @ frame %d, word %d: Got %s, expected %s",
                fo(frame_cnt),
                fo(word_cnt),
                fo(s_tlast),
                fo(last)
              )
            );
            failed := True;
          end if;

      end;

    begin
      for i in 0 to frame.data'length - 1 loop
        byte := i mod INPUT_BYTE_WIDTH;

        exp_tdata(8*(byte + 1) - 1 downto 8*byte) := frame.data(i);

        if ((i + 1) mod INPUT_BYTE_WIDTH) = 0 then
          if i /= frame.data'length - 1 then
            check_word(exp_tdata, (others => '0'), frame.id, False);
          else
            check_word(exp_tdata, (others => '1'), frame.id, True);
          end if;

          exp_tdata := (others => 'U');
          word_cnt  := word_cnt + 1;
        end if;
      end loop;

      if byte = 1 then
        return;
      end if;

      exp_tkeep                := (others => '0');
      exp_tkeep(byte downto 0) := (others => '1');

      check_word(exp_tdata, exp_tkeep, frame.id, True);
    end;

  begin
    receive(net, self, msg);
    info(logger, "Received frame");
    check_frame(pop(msg));
    frame_cnt := frame_cnt + 1;
    wait;
  end process; -- }}

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


end axi_stream_width_converter_tb;

-- vim: set foldmethod=marker foldmarker=--\ {{,--\ }} :
