/*
 * Copyright 2023 Ettus Research, a National Instruments Brand
 *
 * SPDX-License-Identifier: LGPL-3.0-or-later
 * 
 * This file was automatically generated by the RFNoC image builder tool.
 * Re-running that tool will overwrite this file!
 */

/dts-v1/;
/plugin/;

% for dtsi in config.dts_includes:
#include "${dtsi}"

% endfor