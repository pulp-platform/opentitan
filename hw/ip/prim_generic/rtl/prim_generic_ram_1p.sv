// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Synchronous single-port SRAM model

`include "prim_assert.sv"

module prim_ram_1p import prim_ram_1p_pkg::*; #(
  parameter  int Width           = 32, // bit
  parameter  int Depth           = 128,
  parameter  int DataBitsPerMask = 1, // Number of data bits per bit of write mask
  parameter      MemInitFile     = "", // VMEM file to initialize the memory with
  parameter  int Otp             = 0,
  parameter  bit PrintSimCfg     = 1'b1,
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

  logic unused_cfg;
  assign unused_cfg = ^cfg_i;

  localparam int MaskWidth = (Width % 8 == 0) ? Width / 8 : (Width / 8) + 1;
  logic [MaskWidth-1:0] wmask;

  for (genvar k = 0; k < MaskWidth; k++) begin : gen_wmask
    assign wmask[k] = wmask_i[k*8] ? 1'b1 : 1'b0;
  end

  tc_sram #(
     .NumWords(Depth),
     .DataWidth(Width),
     .NumPorts(1),
     .SimInit("zeros")
  ) ram_primitive (
     .clk_i,
     .rst_ni,
     .req_i,
     .addr_i,
     .wdata_i,
     .rdata_o,
     .we_i(write_i),
     .be_i(wmask)
   );

endmodule
