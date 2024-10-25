// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

`include "prim_assert.sv"

module prim_buf #(
  parameter int Width = 1
) (
  input        [Width-1:0] in_i,
  output logic [Width-1:0] out_o
);

  // logic [Width-1:0] inv;
  // assign inv = ~in_i;
  // assign out_o = ~inv;

  // Replacing with tech specific cell reference
  for (genvar i=0; i<Width; i++) begin : p_buf_gen
    tc_buf i_buf (
      .inp (in_i[i] ),
      .op  (out_o[i])
      );
  end

endmodule
