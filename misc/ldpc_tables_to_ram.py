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
"""
Simple script to generate a VHDL package with the LDPC tables. Used tables from
https://github.com/aicodix/tables
"""

import logging
import math
import os.path as p
import sys
from os import makedirs

logging.basicConfig(stream=sys.stdout, level=logging.DEBUG)
_logger = logging.getLogger(__name__)

HEADER = """\
--
-- DVB IP
--
-- Copyright 2020 by Suoto <andre820@gmail.com>
--
-- This file is part of the DVB IP.
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

"""


class Unsigned:
    def __init__(self, value, width):
        self._value = value
        self._width = width

    def render(self):
        return f"to_unsigned({self._value}, {self._width})"

    def __repr__(self):
        return self.render()


class LdpcTable:
    def __init__(self, path):
        self._path = path
        self._table = tuple(self._read())

        self._widths = self._getColumnWidths()

    def _read(self):
        _logger.info("Reading %s", self._path)
        for line in (x.strip() for x in open(self._path).readlines()):
            if not line:
                continue
            yield tuple(int(x) for x in line.split("\t"))

    def _getColumnWidths(self):
        max_values = {}

        for line in self._table:
            max_values[0] = max(len(line), max_values.get(0, 0))
            for cnum, col in enumerate(line):
                max_values[cnum + 1] = max(col, max_values.get(cnum + 1, 0))

        widths = {}
        for col, value in max_values.items():
            widths[col] = math.ceil(math.log2(value))

        return widths

    def _renderRamLine(self, line):
        items = [Unsigned(len(line), self._widths[0])]
        for i in range(1, len(self._widths)):
            try:
                item = repr(Unsigned(line[i - 1], self._widths[i]))
            except IndexError:
                item = f"({self._widths[i] - 1} downto 0 => 'U')"
            items.append(item)

        return " & ".join([str(x) for x in items])

    @property
    def name(self):
        return p.basename(self._path).split(".")[0]

    def _getWidthArray(self):
        return (
            f"constant {self.name.upper()}_COLUMN_WIDTHS : integer_array_t := ("
            + ", ".join(
                ["%d => %d" % (key, value) for key, value in self._widths.items()]
            )
            + ");"
        )

    def render(self):
        _logger.debug("widths=%s", self._widths)

        length = len(self._table)
        width = sum(self._widths.values())
        brams_18k = max((width + 17) // 18, length * width / 1024 / 18)
        brams_36k = max((width + 35) // 36, length * width / 1024 / 36)

        lines = [
            f"  -- From {self._path}, table is {length}x{width} ({length*width/8.} bytes)",
            f"  -- Resource estimation: {brams_18k} x 18 kB BRAMs or {brams_36k} x 36 kB BRAMs",
            f"  type {self.name}_t is array ({length - 1} downto 0) of std_logic_vector({width - 1} downto 0);",
            "",
            f"  {self._getWidthArray()}",
            "",
            f"  constant {self.name.upper()} : {self.name}_t := (",
        ]

        for i, line in enumerate(self._table):
            if i < length - 1:
                lines.append(
                    f"    {i} => std_logic_vector({self._renderRamLine(line)}),"
                )
            else:
                lines.append(
                    f"    {i} => std_logic_vector({self._renderRamLine(line)})"
                )

        lines.append("  );")

        _logger.info(
            "RAM has %.2f kb (%d x %d), or %d BRAMs",
            length * width / 1024.0,
            length,
            width,
            brams_18k,
        )

        return "\n".join(lines)


def main():
    root = p.abspath(p.dirname(__file__))

    lines = str(HEADER)

    lines += "\n".join(
        [
            "library ieee;",
            "use ieee.std_logic_1164.all;",
            "use ieee.numeric_std.all;",
            "",
            "use work.common_pkg.all;",
            "",
            "package ldpc_tables_pkg is",
            "",
            "",
        ]
    )

    paths = sys.argv[1:]
    paths.sort()

    for path in paths:
        ldpc_table = LdpcTable(path)
        lines += ldpc_table.render()
        lines += "\n\n"

    lines += "\n".join(
        [
            "",
            "end package ldpc_tables_pkg;",
            "",
            "package body ldpc_tables_pkg is",
            "end package body ldpc_tables_pkg;",
        ]
    )

    target_file = p.abspath(
        p.join(root, "..", "rtl", "generated", "ldpc_tables_pkg.vhd")
    )
    if not p.exists(p.dirname(target_file)):
        makedirs(p.dirname(target_file))
    open(target_file, "w").write(lines)


main()
