// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Synchronous single-port SRAM model

`include "prim_assert.sv"
`define DUMMYBOY


module prim_ram_1p import prim_ram_1p_pkg::*; #(
  parameter  int Width           = 32, // bit
  parameter  int Depth           = 128,
  parameter  int DataBitsPerMask = 1, // Number of data bits per bit of write mask
  parameter      MemInitFile     = "", // VMEM file to initialize the memory with
  parameter  int Otp             = 0,
  parameter  bit PrintSimCfg     = 1'b1,
  parameter      FPGAMemMacro    = "auto", // Select memory macro for FPGA
  localparam int Aw              = $clog2(Depth)  // derived parameter
) (
  input  logic             clk_i,
  input  logic             rst_ni,

  input  logic             req_i,
  input  logic             write_i,
  input  logic [Aw-1:0]    addr_i,
  input  logic [Width-1:0] wdata_i,
  input  logic [Width-1:0] wmask_i,
  output logic [Width-1:0] rdata_o, // Read data. Data is returned one cycle after req_i is high.
  input ram_1p_cfg_t       cfg_i
);

 logic  unused;
 assign unused = ^cfg_i;

 tc_sram #(
    .NumWords(Depth),
    .DataWidth(Width),
    .ByteWidth(1),
    .NumPorts(1),
    // Note: ifdef can be removed when all tc_sram instantiations will be compliant
`ifdef TARGET_XILINX
    .FPGAImplKey(FPGAMemMacro),
`endif
    .SimInit("zeros")
 ) ram_primitive (
    .clk_i,
    .rst_ni,
    .req_i,
    .addr_i,
    .wdata_i,
    .rdata_o,
    .we_i(write_i),
    .be_i(wmask_i)
 );

endmodule
