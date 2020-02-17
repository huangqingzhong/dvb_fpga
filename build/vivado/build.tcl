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

create_project dvbs2_tx build/vivado/dvbs2_tx -part xczu4cg-sfvc784-1LV-i

add_files /home/asouto/dev/dvb_fpga/third_party/hdl_string_format/src/str_format_pkg.vhd
set_property library str_format [ get_files /home/asouto/dev/dvb_fpga/third_party/hdl_string_format/src/str_format_pkg.vhd ]

add_files [ glob rtl/bch_generated/*.vhd ]
add_files [ glob rtl/*.vhd ]

launch_runs synth_1
wait_on_run synth_1

launch_runs impl_1
wait_on_run impl_1
