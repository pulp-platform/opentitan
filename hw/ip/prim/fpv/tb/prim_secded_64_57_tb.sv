// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// SECDED FPV testbench generated by util/design/secded_gen.py

module prim_secded_64_57_tb (
  input               clk_i,
  input               rst_ni,
  input        [56:0] data_i,
  output logic [56:0] data_o,
  output logic [63:0] encoded_o,
  output logic [6:0] syndrome_o,
  output logic [1:0]  err_o,
  input        [63:0] error_inject_i
);

  prim_ot_secded_64_57_enc prim_secded_64_57_enc (
    .data_i,
    .data_o(encoded_o)
  );

  prim_ot_secded_64_57_dec prim_secded_64_57_dec (
    .data_i(encoded_o ^ error_inject_i),
    .data_o,
    .syndrome_o,
    .err_o
  );

endmodule : prim_secded_64_57_tb
