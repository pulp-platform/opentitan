// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

`include "prim_assert.sv"

module prim_xnor2 #(
  parameter int Width = 1
) (
  input        [Width-1:0] in0_i,
  input        [Width-1:0] in1_i,
  output logic [Width-1:0] out_o
);

  // assign out_o = !(in0_i ^ in1_i);

  // Replacing with tech specific cell reference
  for (genvar i=0; i<Width; i++) begin : p_xnor2_gen
    tc_xnor2 i_xnor2 (
      .inp_a (in0_i[i]),
      .inp_b (in1_i[i]),
      .op_c  (out_o[i])
      );
  end

endmodule
