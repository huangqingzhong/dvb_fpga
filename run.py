#!/usr/bin/env python3
#
# DVB FPGA
#
# Copyright 2019 by Suoto <andre820@gmail.com>
#
# This file is part of DVB FPGA.
#
# DVB FPGA is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# DVB FPGA is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with DVB FPGA.  If not, see <http://www.gnu.org/licenses/>.
"VUnit test runner for DVB FPGA"

# pylint: disable=bad-continuation

import logging
import os
import os.path as p
import random
import subprocess as subp
import sys
import time
from enum import Enum
from multiprocessing import Pool
from typing import NamedTuple

from vunit import VUnit  # type: ignore

_logger = logging.getLogger(__name__)

ROOT = p.abspath(p.dirname(__file__))


class ConstellationType(Enum):
    """
    Constellation types as defined in the DVB-S2 spec. Enum names should match
    the C/C++ defines, values are a nice string representation for the test
    names.
    """

    MOD_8PSK = "8PSK"
    MOD_16APSK = "16APSK"
    MOD_32APSK = "32APSK"


class FrameLength(Enum):
    """
    Frame types as defined in the DVB-S2 spec. Enum names should match
    the C/C++ defines, values are a nice string representation for the test
    names.
    """

    FECFRAME_NORMAL = "normal"
    FECFRAME_SHORT = "short"


class CodeRate(Enum):
    """
    Code rates as defined in the DVB-S2 spec. Enum names should match the C/C++
    defines, values are a nice string representation for the test names.
    """

    C1_4 = "1/4"
    C1_3 = "1/3"
    C2_5 = "2/5"
    C1_2 = "1/2"
    C3_5 = "3/5"
    C2_3 = "2/3"
    C3_4 = "3/4"
    C4_5 = "4/5"
    C5_6 = "5/6"
    C8_9 = "8/9"
    C9_10 = "9/10"


class TestDefinition(
    NamedTuple(
        "TestDefinition",
        [
            ("name", str),
            ("test_files_path", str),
            ("code_rate", CodeRate),
            ("frame_length", FrameLength),
            ("constellation", ConstellationType),
        ],
    )
):
    """
    Placeholder for a test config
    """

    def __init__(self, *args, **kwargs):
        super(TestDefinition, self).__init__()
        self.timestamp = p.join(self.test_files_path, "timestamp")

    def getTestConfig(self, input_file_path, reference_file_path):
        for path in (
            p.join(self.test_files_path, input_file_path),
            p.join(self.test_files_path, reference_file_path),
        ):
            assert p.exists(path), f"No such file '{path}'"

        return ",".join(
            [
                self.constellation.name,
                self.frame_length.name,
                self.code_rate.name,
                p.join(self.test_files_path, input_file_path),
                p.join(self.test_files_path, reference_file_path),
            ]
        )


def _getAllConfigs(
    code_rates=CodeRate, frame_lengths=FrameLength, constellations=ConstellationType
):

    for code_rate in code_rates:
        for frame_length in frame_lengths:
            for constellation in constellations:
                if (
                    frame_length is FrameLength.FECFRAME_SHORT
                    and code_rate is CodeRate.C9_10
                ):
                    continue

                test_files_path = p.join(
                    ROOT,
                    "gnuradio_data",
                    f"{frame_length.name}_{constellation.name}_{code_rate.name}".upper(),
                )

                name = ",".join(
                    [
                        f"FrameLength={frame_length.value}",
                        f"ConstellationType={constellation.value}",
                        f"CodeRate={code_rate.value}",
                    ]
                )

                yield TestDefinition(
                    name, test_files_path, code_rate, frame_length, constellation
                )


TEST_CONFIGS = set(_getAllConfigs())


def _runGnuRadio(config):
    """
    Runs gnuradio_data/dvbs2_tx.py script via shell. Reason for not importing
    and running locally is to allow GNI Radio's Python environment to be
    independent of VUnit's Python env.
    """
    print("Generating data for %s" % config.name)

    command = [
        p.join(ROOT, "gnuradio_data", "dvbs2_tx.py"),
        "--frame-type",
        config.frame_length.name,
        "--constellation",
        config.constellation.name,
        "--code-rate",
        config.code_rate.name,
    ]

    if not p.exists(config.test_files_path):
        os.makedirs(config.test_files_path)

    try:
        subp.check_call(command, cwd=config.test_files_path)
        open(config.timestamp, "w").write(time.asctime())
    except subp.CalledProcessError:
        _logger.exception("Failed to run %s", command)
        raise


def _generateGnuRadioData():
    """
    Generates GNU Radio data for configs whose test files can't found
    """
    configs = []

    for config in TEST_CONFIGS:
        if not p.exists(config.timestamp):
            configs += [config]

    # Generate needed data on a process pool to speed up things
    pool = Pool()
    pool.map(_runGnuRadio, configs)
    pool.close()
    pool.join()


def main():
    "Main entry point for DVB FPGA test runner"

    _generateGnuRadioData()

    cli = VUnit.from_argv()
    cli.add_osvvm()
    cli.add_com()
    cli.enable_location_preprocessing()

    library = cli.add_library("lib")
    library.add_source_files(p.join(ROOT, "rtl", "*.vhd"))
    library.add_source_files(p.join(ROOT, "rtl", "ldpc", "*.vhd"))
    library.add_source_files(p.join(ROOT, "rtl", "bch_generated", "*.vhd"))
    library.add_source_files(p.join(ROOT, "testbench", "*.vhd"))

    cli.add_library("str_format").add_source_files(
        p.join(ROOT, "third_party", "hdl_string_format", "src", "*.vhd")
    )

    cli.add_library("fpga_cores").add_source_files(
        p.join(ROOT, "third_party", "fpga_cores", "src", "*.vhd")
    )
    cli.add_library("fpga_cores_sim").add_source_files(
        p.join(ROOT, "third_party", "fpga_cores", "sim", "*.vhd")
    )

    addAllConfigsTest(
        entity=cli.library("lib").entity("axi_bch_encoder_tb"),
        configs=TEST_CONFIGS,
        input_file_basename="bch_encoder_input.bin",
        reference_file_basename="ldpc_encoder_input.bin",
    )

    addAllConfigsTest(
        entity=cli.library("lib").entity("axi_baseband_scrambler_tb"),
        configs=TEST_CONFIGS,
        input_file_basename="bb_scrambler_input.bin",
        reference_file_basename="bch_encoder_input.bin",
    )

    # Uncomment this to test configs individually
    #  # BCH encoding doesn't depend on the constellation type, choose any
    #  for config in _getAllConfigs(constellations=(ConstellationType.MOD_8PSK,)):
    #      cli.library("lib").entity("axi_bch_encoder_tb").add_config(
    #          name=config.name,
    #          generics=dict(
    #              test_cfg=config.getTestConfig(
    #                  input_file_path="bch_encoder_input.bin",
    #                  reference_file_path="ldpc_encoder_input.bin",
    #              ),
    #              NUMBER_OF_TEST_FRAMES=8,
    #          ),
    #      )

    for config in _getAllConfigs(
        constellations=(ConstellationType.MOD_8PSK,),
        code_rates={CodeRate.C4_5,},
        frame_lengths={FrameLength.FECFRAME_SHORT,},
    ):
        cli.library("lib").entity("axi_ldpc_encoder_tb").add_config(
            name=config.name,
            generics=dict(
                test_cfg=config.getTestConfig(
                    input_file_path="ldpc_encoder_input.bin",
                    reference_file_path="bit_interleaver_input.bin",
                ),
                NUMBER_OF_TEST_FRAMES=1,
            ),
        )

    #  for config in _getAllConfigs(
    #      constellations=(ConstellationType.MOD_8PSK,),
    #      code_rates={CodeRate.C2_3,},
    #      frame_lengths={FrameLength.FECFRAME_NORMAL,},
    #  ):
    #      cli.library("lib").entity("axi_ldpc_encoder_tb").add_config(
    #          name=config.name,
    #          generics=dict(
    #              test_cfg=config.getTestConfig(
    #                  input_file_path="bch_encoder_input.bin",
    #                  reference_file_path="ldpc_encoder_input.bin",
    #              ),
    #              NUMBER_OF_TEST_FRAMES=1,
    #          ),
    #      )

    for data_width in (1, 8):
        all_configs = []
        for config in _getAllConfigs():
            all_configs += [
                config.getTestConfig(
                    input_file_path="bit_interleaver_input.bin",
                    reference_file_path="bit_interleaver_output.bin",
                )
            ]

            # Uncomment this to test configs individually
            #  cli.library("lib").entity("axi_bit_interleaver_tb").add_config(
            #      name=f"data_width={data_width},{config.name}",
            #      generics=dict(
            #          DATA_WIDTH=data_width,
            #          test_cfg=config.getTestConfig(
            #              input_file_path="bit_interleaver_input.bin",
            #              reference_file_path="bit_interleaver_output.bin",
            #          ),
            #          NUMBER_OF_TEST_FRAMES=8,
            #      ),
            #  )

        cli.library("lib").entity("axi_bit_interleaver_tb").add_config(
            name=f"data_width={data_width},all_parameters",
            generics=dict(
                DATA_WIDTH=data_width,
                test_cfg="|".join(all_configs),
                NUMBER_OF_TEST_FRAMES=2,
            ),
        )

    cli.set_compile_option("modelsim.vcom_flags", ["-explicit"])

    # Not all options are supported by all GHDL backends
    #  cli.set_compile_option("ghdl.flags", ["-frelaxed-rules"])
    cli.set_sim_option("ghdl.elab_flags", ["-frelaxed-rules"])
    cli.set_compile_option("ghdl.flags", ["-frelaxed-rules", "-O2", "-g"])

    # Make components not bound (error 3473) an error
    cli.set_sim_option("modelsim.vsim_flags", ["-error", "3473", '-voptargs="+acc=n"'])

    cli.set_sim_option("disable_ieee_warnings", True)
    cli.set_sim_option("modelsim.init_file.gui", p.join(ROOT, "wave.do"))
    cli.main()


def addAllConfigsTest(entity, configs, input_file_basename, reference_file_basename):
    """
    Adds a test config with all combinations of configurations (assuming both
    input and reference files ca be found)
    """
    params = []

    for config in configs:
        params += [config.getTestConfig(input_file_basename, reference_file_basename)]

    random.shuffle(params)

    assert params, "Could not find any config files!"

    entity.add_config(
        name="test_all_configs",
        generics=dict(test_cfg="|".join(params), NUMBER_OF_TEST_FRAMES=1),
    )


if __name__ == "__main__":
    sys.exit(main())
