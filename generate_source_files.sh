#! /bin/bash

#
# Copyright (C) 2020-2026 Embedded AMS B.V. - All Rights Reserved
#
# This file is part of Embedded Proto.
#
# Embedded Proto is open source software: you can redistribute it and/or 
# modify it under the terms of the GNU General Public License as published 
# by the Free Software Foundation, version 3 of the license.
#
# Embedded Proto  is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with Embedded Proto. If not, see <https://www.gnu.org/licenses/>.
#
# For commercial and closed source application please visit:
# <https://EmbeddedProto.com/license/>.
#
# Embedded AMS B.V.
# Info:
#   info at EmbeddedProto dot com
#
# Postal address:
#   Johan Huizingalaan 763a
#   1066 VH, Amsterdam
#   the Netherlands
#

# Generate the embedded source code.
mkdir -p ./frdm-keo2z40m/generated
mkdir -p ./desktop/generated

# The embeddedproto command lives in the virtual environment of the submodule. It runs the protobuf compiler which
# came with the package, so it generates the Python code for the desktop script as well.
./EmbeddedProto/venv/bin/embeddedproto -I./proto --eams_out=./frdm-keo2z40m/generated --python_out=./desktop/generated ./proto/uart_messages.proto

