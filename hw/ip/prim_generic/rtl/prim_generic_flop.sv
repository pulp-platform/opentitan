// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

`include "prim_assert.sv"

module prim_flop #(
  parameter int               Width      = 1,
  parameter logic [Width-1:0] ResetValue = 0
) (
  input                    clk_i,
  input                    rst_ni,
  input        [Width-1:0] d_i,
  output logic [Width-1:0] q_o
);

  // always_ff @(posedge clk_i or negedge rst_ni) begin
  //   if (!rst_ni) begin
  //     q_o <= ResetValue;
  //   end else begin
  //     q_o <= d_i;
  //   end
  // end

  // Replacing with tech specific cell reference
  for (genvar i=0; i<Width; i++) begin
    if (ResetValue[i] == 1'b0) begin
      tc_flop_async_low_reset i_flop_reset (
        .CK (clk_i ),
        .D  (d_i[i]),
        .RN (rst_ni),
        .op (q_o[i])
      );
    end else begin
      tc_flop_async_low_set i_flop_set (
        .CK (clk_i ),
        .D  (d_i[i]),
        .RN (rst_ni),
        .op (q_o[i])
      );
    end
  end

endmodule
