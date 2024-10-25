// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0

`include "prim_assert.sv"

module prim_ot_flop_en #(
  parameter int               Width      = 1,
  parameter bit               EnSecBuf   = 0,
  parameter logic [Width-1:0] ResetValue = 0
) (
  input                    clk_i,
  input                    rst_ni,
  input                    en_i,
  input        [Width-1:0] d_i,
  output logic [Width-1:0] q_o
);

  logic en;
  if (EnSecBuf) begin : gen_en_sec_buf
    prim_sec_anchor_buf #(
      .Width(1)
    ) u_en_buf (
      .in_i(en_i),
      .out_o(en)
    );
  end else begin : gen_en_no_sec_buf
    assign en = en_i;
  end

  // always_ff @(posedge clk_i or negedge rst_ni) begin
  //   if (!rst_ni) begin
  //     q_o <= ResetValue;
  //   end else if (en) begin
  //     q_o <= d_i;
  //   end
  // end

  // Replacing with tech specific cell reference
  for (genvar i=0; i<Width; i++) begin
    if (ResetValue[i] == 1'b0) begin
      tc_flop_async_low_reset i_flop_reset (
        .clk_i   (clk_i ),
        .d_i     (d_i[i]),
        .rst_ni  (rst_ni),
        .q_o     (q_o[i])
      );
    end else begin
      tc_flop_async_low_set i_flop_set (
        .clk_i   (clk_i ),
        .d_i     (d_i[i]),
        .set_ni  (rst_ni),
        .q_o     (q_o[i])
      );
    end
  end

endmodule
