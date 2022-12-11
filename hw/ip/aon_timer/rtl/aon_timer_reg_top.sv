// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Top module auto-generated by `reggen`

`include "prim_assert.sv"

module aon_timer_reg_top (
  input clk_i,
  input rst_ni,
  input clk_aon_i,
  input rst_aon_ni,
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,
  // To HW
  output aon_timer_reg_pkg::aon_timer_reg2hw_t reg2hw, // Write
  input  aon_timer_reg_pkg::aon_timer_hw2reg_t hw2reg, // Read

  // Integrity check errors
  output logic intg_err_o,

  // Config
  input devmode_i // If 1, explicit error return for unmapped register access
);

  import aon_timer_reg_pkg::* ;

  localparam int AW = 6;
  localparam int DW = 32;
  localparam int DBW = DW/8;                    // Byte Width

  // register signals
  logic           reg_we;
  logic           reg_re;
  logic [AW-1:0]  reg_addr;
  logic [DW-1:0]  reg_wdata;
  logic [DBW-1:0] reg_be;
  logic [DW-1:0]  reg_rdata;
  logic           reg_error;

  logic          addrmiss, wr_err;

  logic [DW-1:0] reg_rdata_next;
  logic reg_busy;

  tlul_pkg::tl_h2d_t tl_reg_h2d;
  tlul_pkg::tl_d2h_t tl_reg_d2h;


  // incoming payload check
  logic intg_err;
  tlul_cmd_intg_chk u_chk (
    .tl_i(tl_i),
    .err_o(intg_err)
  );

  // also check for spurious write enables
  logic reg_we_err;
  logic [11:0] reg_we_check;
  prim_reg_we_check #(
    .OneHotWidth(12)
  ) u_prim_reg_we_check (
    .clk_i(clk_i),
    .rst_ni(rst_ni),
    .oh_i  (reg_we_check),
    .en_i  (reg_we && !addrmiss),
    .err_o (reg_we_err)
  );

  logic err_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      err_q <= '0;
    end else if (intg_err || reg_we_err) begin
      err_q <= 1'b1;
    end
  end

  // integrity error output is permanent and should be used for alert generation
  // register errors are transactional
  assign intg_err_o = err_q | intg_err | reg_we_err;

  // outgoing integrity generation
  tlul_pkg::tl_d2h_t tl_o_pre;
  tlul_rsp_intg_gen #(
    .EnableRspIntgGen(1),
    .EnableDataIntgGen(1)
  ) u_rsp_intg_gen (
    .tl_i(tl_o_pre),
    .tl_o(tl_o)
  );

  assign tl_reg_h2d = tl_i;
  assign tl_o_pre   = tl_reg_d2h;

  tlul_adapter_reg #(
    .RegAw(AW),
    .RegDw(DW),
    .EnableDataIntgGen(0)
  ) u_reg_if (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),

    .tl_i (tl_reg_h2d),
    .tl_o (tl_reg_d2h),

    .en_ifetch_i(prim_mubi_pkg::MuBi4False),
    .intg_error_o(),

    .we_o    (reg_we),
    .re_o    (reg_re),
    .addr_o  (reg_addr),
    .wdata_o (reg_wdata),
    .be_o    (reg_be),
    .busy_i  (reg_busy),
    .rdata_i (reg_rdata),
    .error_i (reg_error)
  );

  // cdc oversampling signals

  assign reg_rdata = reg_rdata_next ;
  assign reg_error = (devmode_i & addrmiss) | wr_err | intg_err;

  // Define SW related signals
  // Format: <reg>_<field>_{wd|we|qs}
  //        or <reg>_{wd|we|qs} if field == 1 or 0
  logic alert_test_we;
  logic alert_test_wd;
  logic wkup_ctrl_we;
  logic [12:0] wkup_ctrl_qs;
  logic wkup_ctrl_busy;
  logic wkup_thold_we;
  logic [31:0] wkup_thold_qs;
  logic wkup_thold_busy;
  logic wkup_count_we;
  logic [31:0] wkup_count_qs;
  logic wkup_count_busy;
  logic wdog_regwen_we;
  logic wdog_regwen_qs;
  logic wdog_regwen_wd;
  logic wdog_ctrl_we;
  logic [1:0] wdog_ctrl_qs;
  logic wdog_ctrl_busy;
  logic wdog_bark_thold_we;
  logic [31:0] wdog_bark_thold_qs;
  logic wdog_bark_thold_busy;
  logic wdog_bite_thold_we;
  logic [31:0] wdog_bite_thold_qs;
  logic wdog_bite_thold_busy;
  logic wdog_count_we;
  logic [31:0] wdog_count_qs;
  logic wdog_count_busy;
  logic intr_state_we;
  logic intr_state_wkup_timer_expired_qs;
  logic intr_state_wkup_timer_expired_wd;
  logic intr_state_wdog_timer_bark_qs;
  logic intr_state_wdog_timer_bark_wd;
  logic intr_test_we;
  logic intr_test_wkup_timer_expired_wd;
  logic intr_test_wdog_timer_bark_wd;
  logic wkup_cause_we;
  logic [0:0] wkup_cause_qs;
  logic wkup_cause_busy;
  // Define register CDC handling.
  // CDC handling is done on a per-reg instead of per-field boundary.

  logic  aon_wkup_ctrl_enable_qs_int;
  logic [11:0]  aon_wkup_ctrl_prescaler_qs_int;
  logic [12:0] aon_wkup_ctrl_qs;
  logic [12:0] aon_wkup_ctrl_wdata;
  logic aon_wkup_ctrl_we;
  logic unused_aon_wkup_ctrl_wdata;

  always_comb begin
    aon_wkup_ctrl_qs = 13'h0;
    aon_wkup_ctrl_qs[0] = aon_wkup_ctrl_enable_qs_int;
    aon_wkup_ctrl_qs[12:1] = aon_wkup_ctrl_prescaler_qs_int;
  end

  prim_reg_cdc #(
    .DataWidth(13),
    .ResetVal(13'h0),
    .BitMask(13'h1fff),
    .DstWrReq(0)
  ) u_wkup_ctrl_cdc (
    .clk_src_i    (clk_i),
    .rst_src_ni   (rst_ni),
    .clk_dst_i    (clk_aon_i),
    .rst_dst_ni   (rst_aon_ni),
    .src_regwen_i ('0),
    .src_we_i     (wkup_ctrl_we),
    .src_re_i     ('0),
    .src_wd_i     (reg_wdata[12:0]),
    .src_busy_o   (wkup_ctrl_busy),
    .src_qs_o     (wkup_ctrl_qs), // for software read back
    .dst_update_i ('0),
    .dst_ds_i     ('0),
    .dst_qs_i     (aon_wkup_ctrl_qs),
    .dst_we_o     (aon_wkup_ctrl_we),
    .dst_re_o     (),
    .dst_regwen_o (),
    .dst_wd_o     (aon_wkup_ctrl_wdata)
  );
  assign unused_aon_wkup_ctrl_wdata =
      ^aon_wkup_ctrl_wdata;

  logic [31:0]  aon_wkup_thold_qs_int;
  logic [31:0] aon_wkup_thold_qs;
  logic [31:0] aon_wkup_thold_wdata;
  logic aon_wkup_thold_we;
  logic unused_aon_wkup_thold_wdata;

  always_comb begin
    aon_wkup_thold_qs = 32'h0;
    aon_wkup_thold_qs = aon_wkup_thold_qs_int;
  end

  prim_reg_cdc #(
    .DataWidth(32),
    .ResetVal(32'h0),
    .BitMask(32'hffffffff),
    .DstWrReq(0)
  ) u_wkup_thold_cdc (
    .clk_src_i    (clk_i),
    .rst_src_ni   (rst_ni),
    .clk_dst_i    (clk_aon_i),
    .rst_dst_ni   (rst_aon_ni),
    .src_regwen_i ('0),
    .src_we_i     (wkup_thold_we),
    .src_re_i     ('0),
    .src_wd_i     (reg_wdata[31:0]),
    .src_busy_o   (wkup_thold_busy),
    .src_qs_o     (wkup_thold_qs), // for software read back
    .dst_update_i ('0),
    .dst_ds_i     ('0),
    .dst_qs_i     (aon_wkup_thold_qs),
    .dst_we_o     (aon_wkup_thold_we),
    .dst_re_o     (),
    .dst_regwen_o (),
    .dst_wd_o     (aon_wkup_thold_wdata)
  );
  assign unused_aon_wkup_thold_wdata =
      ^aon_wkup_thold_wdata;

  logic [31:0]  aon_wkup_count_ds_int;
  logic [31:0]  aon_wkup_count_qs_int;
  logic [31:0] aon_wkup_count_ds;
  logic aon_wkup_count_qe;
  logic [31:0] aon_wkup_count_qs;
  logic [31:0] aon_wkup_count_wdata;
  logic aon_wkup_count_we;
  logic unused_aon_wkup_count_wdata;

  always_comb begin
    aon_wkup_count_qs = 32'h0;
    aon_wkup_count_ds = 32'h0;
    aon_wkup_count_ds = aon_wkup_count_ds_int;
    aon_wkup_count_qs = aon_wkup_count_qs_int;
  end

  prim_reg_cdc #(
    .DataWidth(32),
    .ResetVal(32'h0),
    .BitMask(32'hffffffff),
    .DstWrReq(1)
  ) u_wkup_count_cdc (
    .clk_src_i    (clk_i),
    .rst_src_ni   (rst_ni),
    .clk_dst_i    (clk_aon_i),
    .rst_dst_ni   (rst_aon_ni),
    .src_regwen_i ('0),
    .src_we_i     (wkup_count_we),
    .src_re_i     ('0),
    .src_wd_i     (reg_wdata[31:0]),
    .src_busy_o   (wkup_count_busy),
    .src_qs_o     (wkup_count_qs), // for software read back
    .dst_update_i (aon_wkup_count_qe),
    .dst_ds_i     (aon_wkup_count_ds),
    .dst_qs_i     (aon_wkup_count_qs),
    .dst_we_o     (aon_wkup_count_we),
    .dst_re_o     (),
    .dst_regwen_o (),
    .dst_wd_o     (aon_wkup_count_wdata)
  );
  assign unused_aon_wkup_count_wdata =
      ^aon_wkup_count_wdata;

  logic  aon_wdog_ctrl_enable_qs_int;
  logic  aon_wdog_ctrl_pause_in_sleep_qs_int;
  logic [1:0] aon_wdog_ctrl_qs;
  logic [1:0] aon_wdog_ctrl_wdata;
  logic aon_wdog_ctrl_we;
  logic unused_aon_wdog_ctrl_wdata;
  logic aon_wdog_ctrl_regwen;

  always_comb begin
    aon_wdog_ctrl_qs = 2'h0;
    aon_wdog_ctrl_qs[0] = aon_wdog_ctrl_enable_qs_int;
    aon_wdog_ctrl_qs[1] = aon_wdog_ctrl_pause_in_sleep_qs_int;
  end

  prim_reg_cdc #(
    .DataWidth(2),
    .ResetVal(2'h0),
    .BitMask(2'h3),
    .DstWrReq(0)
  ) u_wdog_ctrl_cdc (
    .clk_src_i    (clk_i),
    .rst_src_ni   (rst_ni),
    .clk_dst_i    (clk_aon_i),
    .rst_dst_ni   (rst_aon_ni),
    .src_regwen_i (wdog_regwen_qs),
    .src_we_i     (wdog_ctrl_we),
    .src_re_i     ('0),
    .src_wd_i     (reg_wdata[1:0]),
    .src_busy_o   (wdog_ctrl_busy),
    .src_qs_o     (wdog_ctrl_qs), // for software read back
    .dst_update_i ('0),
    .dst_ds_i     ('0),
    .dst_qs_i     (aon_wdog_ctrl_qs),
    .dst_we_o     (aon_wdog_ctrl_we),
    .dst_re_o     (),
    .dst_regwen_o (aon_wdog_ctrl_regwen),
    .dst_wd_o     (aon_wdog_ctrl_wdata)
  );
  assign unused_aon_wdog_ctrl_wdata =
      ^aon_wdog_ctrl_wdata;

  logic [31:0]  aon_wdog_bark_thold_qs_int;
  logic [31:0] aon_wdog_bark_thold_qs;
  logic [31:0] aon_wdog_bark_thold_wdata;
  logic aon_wdog_bark_thold_we;
  logic unused_aon_wdog_bark_thold_wdata;
  logic aon_wdog_bark_thold_regwen;

  always_comb begin
    aon_wdog_bark_thold_qs = 32'h0;
    aon_wdog_bark_thold_qs = aon_wdog_bark_thold_qs_int;
  end

  prim_reg_cdc #(
    .DataWidth(32),
    .ResetVal(32'h0),
    .BitMask(32'hffffffff),
    .DstWrReq(0)
  ) u_wdog_bark_thold_cdc (
    .clk_src_i    (clk_i),
    .rst_src_ni   (rst_ni),
    .clk_dst_i    (clk_aon_i),
    .rst_dst_ni   (rst_aon_ni),
    .src_regwen_i (wdog_regwen_qs),
    .src_we_i     (wdog_bark_thold_we),
    .src_re_i     ('0),
    .src_wd_i     (reg_wdata[31:0]),
    .src_busy_o   (wdog_bark_thold_busy),
    .src_qs_o     (wdog_bark_thold_qs), // for software read back
    .dst_update_i ('0),
    .dst_ds_i     ('0),
    .dst_qs_i     (aon_wdog_bark_thold_qs),
    .dst_we_o     (aon_wdog_bark_thold_we),
    .dst_re_o     (),
    .dst_regwen_o (aon_wdog_bark_thold_regwen),
    .dst_wd_o     (aon_wdog_bark_thold_wdata)
  );
  assign unused_aon_wdog_bark_thold_wdata =
      ^aon_wdog_bark_thold_wdata;

  logic [31:0]  aon_wdog_bite_thold_qs_int;
  logic [31:0] aon_wdog_bite_thold_qs;
  logic [31:0] aon_wdog_bite_thold_wdata;
  logic aon_wdog_bite_thold_we;
  logic unused_aon_wdog_bite_thold_wdata;
  logic aon_wdog_bite_thold_regwen;

  always_comb begin
    aon_wdog_bite_thold_qs = 32'h0;
    aon_wdog_bite_thold_qs = aon_wdog_bite_thold_qs_int;
  end

  prim_reg_cdc #(
    .DataWidth(32),
    .ResetVal(32'h0),
    .BitMask(32'hffffffff),
    .DstWrReq(0)
  ) u_wdog_bite_thold_cdc (
    .clk_src_i    (clk_i),
    .rst_src_ni   (rst_ni),
    .clk_dst_i    (clk_aon_i),
    .rst_dst_ni   (rst_aon_ni),
    .src_regwen_i (wdog_regwen_qs),
    .src_we_i     (wdog_bite_thold_we),
    .src_re_i     ('0),
    .src_wd_i     (reg_wdata[31:0]),
    .src_busy_o   (wdog_bite_thold_busy),
    .src_qs_o     (wdog_bite_thold_qs), // for software read back
    .dst_update_i ('0),
    .dst_ds_i     ('0),
    .dst_qs_i     (aon_wdog_bite_thold_qs),
    .dst_we_o     (aon_wdog_bite_thold_we),
    .dst_re_o     (),
    .dst_regwen_o (aon_wdog_bite_thold_regwen),
    .dst_wd_o     (aon_wdog_bite_thold_wdata)
  );
  assign unused_aon_wdog_bite_thold_wdata =
      ^aon_wdog_bite_thold_wdata;

  logic [31:0]  aon_wdog_count_ds_int;
  logic [31:0]  aon_wdog_count_qs_int;
  logic [31:0] aon_wdog_count_ds;
  logic aon_wdog_count_qe;
  logic [31:0] aon_wdog_count_qs;
  logic [31:0] aon_wdog_count_wdata;
  logic aon_wdog_count_we;
  logic unused_aon_wdog_count_wdata;

  always_comb begin
    aon_wdog_count_qs = 32'h0;
    aon_wdog_count_ds = 32'h0;
    aon_wdog_count_ds = aon_wdog_count_ds_int;
    aon_wdog_count_qs = aon_wdog_count_qs_int;
  end

  prim_reg_cdc #(
    .DataWidth(32),
    .ResetVal(32'h0),
    .BitMask(32'hffffffff),
    .DstWrReq(1)
  ) u_wdog_count_cdc (
    .clk_src_i    (clk_i),
    .rst_src_ni   (rst_ni),
    .clk_dst_i    (clk_aon_i),
    .rst_dst_ni   (rst_aon_ni),
    .src_regwen_i ('0),
    .src_we_i     (wdog_count_we),
    .src_re_i     ('0),
    .src_wd_i     (reg_wdata[31:0]),
    .src_busy_o   (wdog_count_busy),
    .src_qs_o     (wdog_count_qs), // for software read back
    .dst_update_i (aon_wdog_count_qe),
    .dst_ds_i     (aon_wdog_count_ds),
    .dst_qs_i     (aon_wdog_count_qs),
    .dst_we_o     (aon_wdog_count_we),
    .dst_re_o     (),
    .dst_regwen_o (),
    .dst_wd_o     (aon_wdog_count_wdata)
  );
  assign unused_aon_wdog_count_wdata =
      ^aon_wdog_count_wdata;

  logic  aon_wkup_cause_ds_int;
  logic  aon_wkup_cause_qs_int;
  logic [0:0] aon_wkup_cause_ds;
  logic aon_wkup_cause_qe;
  logic [0:0] aon_wkup_cause_qs;
  logic [0:0] aon_wkup_cause_wdata;
  logic aon_wkup_cause_we;
  logic unused_aon_wkup_cause_wdata;

  always_comb begin
    aon_wkup_cause_qs = 1'h0;
    aon_wkup_cause_ds = 1'h0;
    aon_wkup_cause_ds = aon_wkup_cause_ds_int;
    aon_wkup_cause_qs = aon_wkup_cause_qs_int;
  end

  prim_reg_cdc #(
    .DataWidth(1),
    .ResetVal(1'h0),
    .BitMask(1'h1),
    .DstWrReq(1)
  ) u_wkup_cause_cdc (
    .clk_src_i    (clk_i),
    .rst_src_ni   (rst_ni),
    .clk_dst_i    (clk_aon_i),
    .rst_dst_ni   (rst_aon_ni),
    .src_regwen_i ('0),
    .src_we_i     (wkup_cause_we),
    .src_re_i     ('0),
    .src_wd_i     (reg_wdata[0:0]),
    .src_busy_o   (wkup_cause_busy),
    .src_qs_o     (wkup_cause_qs), // for software read back
    .dst_update_i (aon_wkup_cause_qe),
    .dst_ds_i     (aon_wkup_cause_ds),
    .dst_qs_i     (aon_wkup_cause_qs),
    .dst_we_o     (aon_wkup_cause_we),
    .dst_re_o     (),
    .dst_regwen_o (),
    .dst_wd_o     (aon_wkup_cause_wdata)
  );
  assign unused_aon_wkup_cause_wdata =
      ^aon_wkup_cause_wdata;

  // Register instances
  // R[alert_test]: V(True)
  logic alert_test_qe;
  logic [0:0] alert_test_flds_we;
  assign alert_test_qe = &alert_test_flds_we;
  prim_ot_subreg_ext #(
    .DW    (1)
  ) u_alert_test (
    .re     (1'b0),
    .we     (alert_test_we),
    .wd     (alert_test_wd),
    .d      ('0),
    .qre    (),
    .qe     (alert_test_flds_we[0]),
    .q      (reg2hw.alert_test.q),
    .ds     (),
    .qs     ()
  );
  assign reg2hw.alert_test.qe = alert_test_qe;


  // R[wkup_ctrl]: V(False)
  //   F[enable]: 0:0
  prim_ot_subreg #(
    .DW      (1),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (1'h0)
  ) u_wkup_ctrl_enable (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wkup_ctrl_we),
    .wd     (aon_wkup_ctrl_wdata[0]),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.wkup_ctrl.enable.q),
    .ds     (),

    // to register interface (read)
    .qs     (aon_wkup_ctrl_enable_qs_int)
  );

  //   F[prescaler]: 12:1
  prim_ot_subreg #(
    .DW      (12),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (12'h0)
  ) u_wkup_ctrl_prescaler (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wkup_ctrl_we),
    .wd     (aon_wkup_ctrl_wdata[12:1]),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.wkup_ctrl.prescaler.q),
    .ds     (),

    // to register interface (read)
    .qs     (aon_wkup_ctrl_prescaler_qs_int)
  );


  // R[wkup_thold]: V(False)
  prim_ot_subreg #(
    .DW      (32),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0)
  ) u_wkup_thold (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wkup_thold_we),
    .wd     (aon_wkup_thold_wdata[31:0]),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.wkup_thold.q),
    .ds     (),

    // to register interface (read)
    .qs     (aon_wkup_thold_qs_int)
  );


  // R[wkup_count]: V(False)
  logic [0:0] wkup_count_flds_we;
  assign aon_wkup_count_qe = |wkup_count_flds_we;
  prim_ot_subreg #(
    .DW      (32),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0)
  ) u_wkup_count (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wkup_count_we),
    .wd     (aon_wkup_count_wdata[31:0]),

    // from internal hardware
    .de     (hw2reg.wkup_count.de),
    .d      (hw2reg.wkup_count.d),

    // to internal hardware
    .qe     (wkup_count_flds_we[0]),
    .q      (reg2hw.wkup_count.q),
    .ds     (aon_wkup_count_ds_int),

    // to register interface (read)
    .qs     (aon_wkup_count_qs_int)
  );


  // R[wdog_regwen]: V(False)
  prim_ot_subreg #(
    .DW      (1),
    .SwAccess(prim_ot_subreg_pkg::SwAccessW0C),
    .RESVAL  (1'h1)
  ) u_wdog_regwen (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (wdog_regwen_we),
    .wd     (wdog_regwen_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (),
    .ds     (),

    // to register interface (read)
    .qs     (wdog_regwen_qs)
  );


  // R[wdog_ctrl]: V(False)
  // Create REGWEN-gated WE signal
  logic aon_wdog_ctrl_gated_we;
  assign aon_wdog_ctrl_gated_we = aon_wdog_ctrl_we & aon_wdog_ctrl_regwen;
  //   F[enable]: 0:0
  prim_ot_subreg #(
    .DW      (1),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (1'h0)
  ) u_wdog_ctrl_enable (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wdog_ctrl_gated_we),
    .wd     (aon_wdog_ctrl_wdata[0]),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.wdog_ctrl.enable.q),
    .ds     (),

    // to register interface (read)
    .qs     (aon_wdog_ctrl_enable_qs_int)
  );

  //   F[pause_in_sleep]: 1:1
  prim_ot_subreg #(
    .DW      (1),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (1'h0)
  ) u_wdog_ctrl_pause_in_sleep (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wdog_ctrl_gated_we),
    .wd     (aon_wdog_ctrl_wdata[1]),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.wdog_ctrl.pause_in_sleep.q),
    .ds     (),

    // to register interface (read)
    .qs     (aon_wdog_ctrl_pause_in_sleep_qs_int)
  );


  // R[wdog_bark_thold]: V(False)
  // Create REGWEN-gated WE signal
  logic aon_wdog_bark_thold_gated_we;
  assign aon_wdog_bark_thold_gated_we = aon_wdog_bark_thold_we & aon_wdog_bark_thold_regwen;
  prim_ot_subreg #(
    .DW      (32),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0)
  ) u_wdog_bark_thold (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wdog_bark_thold_gated_we),
    .wd     (aon_wdog_bark_thold_wdata[31:0]),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.wdog_bark_thold.q),
    .ds     (),

    // to register interface (read)
    .qs     (aon_wdog_bark_thold_qs_int)
  );


  // R[wdog_bite_thold]: V(False)
  // Create REGWEN-gated WE signal
  logic aon_wdog_bite_thold_gated_we;
  assign aon_wdog_bite_thold_gated_we = aon_wdog_bite_thold_we & aon_wdog_bite_thold_regwen;
  prim_ot_subreg #(
    .DW      (32),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0)
  ) u_wdog_bite_thold (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wdog_bite_thold_gated_we),
    .wd     (aon_wdog_bite_thold_wdata[31:0]),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.wdog_bite_thold.q),
    .ds     (),

    // to register interface (read)
    .qs     (aon_wdog_bite_thold_qs_int)
  );


  // R[wdog_count]: V(False)
  logic [0:0] wdog_count_flds_we;
  assign aon_wdog_count_qe = |wdog_count_flds_we;
  prim_ot_subreg #(
    .DW      (32),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0)
  ) u_wdog_count (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wdog_count_we),
    .wd     (aon_wdog_count_wdata[31:0]),

    // from internal hardware
    .de     (hw2reg.wdog_count.de),
    .d      (hw2reg.wdog_count.d),

    // to internal hardware
    .qe     (wdog_count_flds_we[0]),
    .q      (reg2hw.wdog_count.q),
    .ds     (aon_wdog_count_ds_int),

    // to register interface (read)
    .qs     (aon_wdog_count_qs_int)
  );


  // R[intr_state]: V(False)
  //   F[wkup_timer_expired]: 0:0
  prim_ot_subreg #(
    .DW      (1),
    .SwAccess(prim_ot_subreg_pkg::SwAccessW1C),
    .RESVAL  (1'h0)
  ) u_intr_state_wkup_timer_expired (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (intr_state_we),
    .wd     (intr_state_wkup_timer_expired_wd),

    // from internal hardware
    .de     (hw2reg.intr_state.wkup_timer_expired.de),
    .d      (hw2reg.intr_state.wkup_timer_expired.d),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_state.wkup_timer_expired.q),
    .ds     (),

    // to register interface (read)
    .qs     (intr_state_wkup_timer_expired_qs)
  );

  //   F[wdog_timer_bark]: 1:1
  prim_ot_subreg #(
    .DW      (1),
    .SwAccess(prim_ot_subreg_pkg::SwAccessW1C),
    .RESVAL  (1'h0)
  ) u_intr_state_wdog_timer_bark (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (intr_state_we),
    .wd     (intr_state_wdog_timer_bark_wd),

    // from internal hardware
    .de     (hw2reg.intr_state.wdog_timer_bark.de),
    .d      (hw2reg.intr_state.wdog_timer_bark.d),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.intr_state.wdog_timer_bark.q),
    .ds     (),

    // to register interface (read)
    .qs     (intr_state_wdog_timer_bark_qs)
  );


  // R[intr_test]: V(True)
  logic intr_test_qe;
  logic [1:0] intr_test_flds_we;
  assign intr_test_qe = &intr_test_flds_we;
  //   F[wkup_timer_expired]: 0:0
  prim_ot_subreg_ext #(
    .DW    (1)
  ) u_intr_test_wkup_timer_expired (
    .re     (1'b0),
    .we     (intr_test_we),
    .wd     (intr_test_wkup_timer_expired_wd),
    .d      ('0),
    .qre    (),
    .qe     (intr_test_flds_we[0]),
    .q      (reg2hw.intr_test.wkup_timer_expired.q),
    .ds     (),
    .qs     ()
  );
  assign reg2hw.intr_test.wkup_timer_expired.qe = intr_test_qe;

  //   F[wdog_timer_bark]: 1:1
  prim_ot_subreg_ext #(
    .DW    (1)
  ) u_intr_test_wdog_timer_bark (
    .re     (1'b0),
    .we     (intr_test_we),
    .wd     (intr_test_wdog_timer_bark_wd),
    .d      ('0),
    .qre    (),
    .qe     (intr_test_flds_we[1]),
    .q      (reg2hw.intr_test.wdog_timer_bark.q),
    .ds     (),
    .qs     ()
  );
  assign reg2hw.intr_test.wdog_timer_bark.qe = intr_test_qe;


  // R[wkup_cause]: V(False)
  logic [0:0] wkup_cause_flds_we;
  assign aon_wkup_cause_qe = |wkup_cause_flds_we;
  prim_ot_subreg #(
    .DW      (1),
    .SwAccess(prim_ot_subreg_pkg::SwAccessW0C),
    .RESVAL  (1'h0)
  ) u_wkup_cause (
    .clk_i   (clk_aon_i),
    .rst_ni  (rst_aon_ni),

    // from register interface
    .we     (aon_wkup_cause_we),
    .wd     (aon_wkup_cause_wdata[0]),

    // from internal hardware
    .de     (hw2reg.wkup_cause.de),
    .d      (hw2reg.wkup_cause.d),

    // to internal hardware
    .qe     (wkup_cause_flds_we[0]),
    .q      (reg2hw.wkup_cause.q),
    .ds     (aon_wkup_cause_ds_int),

    // to register interface (read)
    .qs     (aon_wkup_cause_qs_int)
  );



  logic [11:0] addr_hit;
  always_comb begin
    addr_hit = '0;
    addr_hit[ 0] = (reg_addr == AON_TIMER_ALERT_TEST_OFFSET);
    addr_hit[ 1] = (reg_addr == AON_TIMER_WKUP_CTRL_OFFSET);
    addr_hit[ 2] = (reg_addr == AON_TIMER_WKUP_THOLD_OFFSET);
    addr_hit[ 3] = (reg_addr == AON_TIMER_WKUP_COUNT_OFFSET);
    addr_hit[ 4] = (reg_addr == AON_TIMER_WDOG_REGWEN_OFFSET);
    addr_hit[ 5] = (reg_addr == AON_TIMER_WDOG_CTRL_OFFSET);
    addr_hit[ 6] = (reg_addr == AON_TIMER_WDOG_BARK_THOLD_OFFSET);
    addr_hit[ 7] = (reg_addr == AON_TIMER_WDOG_BITE_THOLD_OFFSET);
    addr_hit[ 8] = (reg_addr == AON_TIMER_WDOG_COUNT_OFFSET);
    addr_hit[ 9] = (reg_addr == AON_TIMER_INTR_STATE_OFFSET);
    addr_hit[10] = (reg_addr == AON_TIMER_INTR_TEST_OFFSET);
    addr_hit[11] = (reg_addr == AON_TIMER_WKUP_CAUSE_OFFSET);
  end

  assign addrmiss = (reg_re || reg_we) ? ~|addr_hit : 1'b0 ;

  // Check sub-word write is permitted
  always_comb begin
    wr_err = (reg_we &
              ((addr_hit[ 0] & (|(AON_TIMER_PERMIT[ 0] & ~reg_be))) |
               (addr_hit[ 1] & (|(AON_TIMER_PERMIT[ 1] & ~reg_be))) |
               (addr_hit[ 2] & (|(AON_TIMER_PERMIT[ 2] & ~reg_be))) |
               (addr_hit[ 3] & (|(AON_TIMER_PERMIT[ 3] & ~reg_be))) |
               (addr_hit[ 4] & (|(AON_TIMER_PERMIT[ 4] & ~reg_be))) |
               (addr_hit[ 5] & (|(AON_TIMER_PERMIT[ 5] & ~reg_be))) |
               (addr_hit[ 6] & (|(AON_TIMER_PERMIT[ 6] & ~reg_be))) |
               (addr_hit[ 7] & (|(AON_TIMER_PERMIT[ 7] & ~reg_be))) |
               (addr_hit[ 8] & (|(AON_TIMER_PERMIT[ 8] & ~reg_be))) |
               (addr_hit[ 9] & (|(AON_TIMER_PERMIT[ 9] & ~reg_be))) |
               (addr_hit[10] & (|(AON_TIMER_PERMIT[10] & ~reg_be))) |
               (addr_hit[11] & (|(AON_TIMER_PERMIT[11] & ~reg_be)))));
  end

  // Generate write-enables
  assign alert_test_we = addr_hit[0] & reg_we & !reg_error;

  assign alert_test_wd = reg_wdata[0];
  assign wkup_ctrl_we = addr_hit[1] & reg_we & !reg_error;


  assign wkup_thold_we = addr_hit[2] & reg_we & !reg_error;

  assign wkup_count_we = addr_hit[3] & reg_we & !reg_error;

  assign wdog_regwen_we = addr_hit[4] & reg_we & !reg_error;

  assign wdog_regwen_wd = reg_wdata[0];
  assign wdog_ctrl_we = addr_hit[5] & reg_we & !reg_error;


  assign wdog_bark_thold_we = addr_hit[6] & reg_we & !reg_error;

  assign wdog_bite_thold_we = addr_hit[7] & reg_we & !reg_error;

  assign wdog_count_we = addr_hit[8] & reg_we & !reg_error;

  assign intr_state_we = addr_hit[9] & reg_we & !reg_error;

  assign intr_state_wkup_timer_expired_wd = reg_wdata[0];

  assign intr_state_wdog_timer_bark_wd = reg_wdata[1];
  assign intr_test_we = addr_hit[10] & reg_we & !reg_error;

  assign intr_test_wkup_timer_expired_wd = reg_wdata[0];

  assign intr_test_wdog_timer_bark_wd = reg_wdata[1];
  assign wkup_cause_we = addr_hit[11] & reg_we & !reg_error;


  // Assign write-enables to checker logic vector.
  always_comb begin
    reg_we_check = '0;
    reg_we_check[0] = alert_test_we;
    reg_we_check[1] = wkup_ctrl_we;
    reg_we_check[2] = wkup_thold_we;
    reg_we_check[3] = wkup_count_we;
    reg_we_check[4] = wdog_regwen_we;
    reg_we_check[5] = wdog_ctrl_we;
    reg_we_check[6] = wdog_bark_thold_we;
    reg_we_check[7] = wdog_bite_thold_we;
    reg_we_check[8] = wdog_count_we;
    reg_we_check[9] = intr_state_we;
    reg_we_check[10] = intr_test_we;
    reg_we_check[11] = wkup_cause_we;
  end

  // Read data return
  always_comb begin
    reg_rdata_next = '0;
    unique case (1'b1)
      addr_hit[0]: begin
        reg_rdata_next[0] = '0;
      end

      addr_hit[1]: begin
        reg_rdata_next = DW'(wkup_ctrl_qs);
      end
      addr_hit[2]: begin
        reg_rdata_next = DW'(wkup_thold_qs);
      end
      addr_hit[3]: begin
        reg_rdata_next = DW'(wkup_count_qs);
      end
      addr_hit[4]: begin
        reg_rdata_next[0] = wdog_regwen_qs;
      end

      addr_hit[5]: begin
        reg_rdata_next = DW'(wdog_ctrl_qs);
      end
      addr_hit[6]: begin
        reg_rdata_next = DW'(wdog_bark_thold_qs);
      end
      addr_hit[7]: begin
        reg_rdata_next = DW'(wdog_bite_thold_qs);
      end
      addr_hit[8]: begin
        reg_rdata_next = DW'(wdog_count_qs);
      end
      addr_hit[9]: begin
        reg_rdata_next[0] = intr_state_wkup_timer_expired_qs;
        reg_rdata_next[1] = intr_state_wdog_timer_bark_qs;
      end

      addr_hit[10]: begin
        reg_rdata_next[0] = '0;
        reg_rdata_next[1] = '0;
      end

      addr_hit[11]: begin
        reg_rdata_next = DW'(wkup_cause_qs);
      end
      default: begin
        reg_rdata_next = '1;
      end
    endcase
  end

  // shadow busy
  logic shadow_busy;
  assign shadow_busy = 1'b0;

  // register busy
  logic reg_busy_sel;
  assign reg_busy = reg_busy_sel | shadow_busy;
  always_comb begin
    reg_busy_sel = '0;
    unique case (1'b1)
      addr_hit[1]: begin
        reg_busy_sel = wkup_ctrl_busy;
      end
      addr_hit[2]: begin
        reg_busy_sel = wkup_thold_busy;
      end
      addr_hit[3]: begin
        reg_busy_sel = wkup_count_busy;
      end
      addr_hit[5]: begin
        reg_busy_sel = wdog_ctrl_busy;
      end
      addr_hit[6]: begin
        reg_busy_sel = wdog_bark_thold_busy;
      end
      addr_hit[7]: begin
        reg_busy_sel = wdog_bite_thold_busy;
      end
      addr_hit[8]: begin
        reg_busy_sel = wdog_count_busy;
      end
      addr_hit[11]: begin
        reg_busy_sel = wkup_cause_busy;
      end
      default: begin
        reg_busy_sel  = '0;
      end
    endcase
  end


  // Unused signal tieoff

  // wdata / byte enable are not always fully used
  // add a blanket unused statement to handle lint waivers
  logic unused_wdata;
  logic unused_be;
  assign unused_wdata = ^reg_wdata;
  assign unused_be = ^reg_be;

  // Assertions for Register Interface
  `ASSERT_PULSE(wePulse, reg_we, clk_i, !rst_ni)
  `ASSERT_PULSE(rePulse, reg_re, clk_i, !rst_ni)

  `ASSERT(reAfterRv, $rose(reg_re || reg_we) |=> tl_o_pre.d_valid, clk_i, !rst_ni)

  `ASSERT(en2addrHit, (reg_we || reg_re) |-> $onehot0(addr_hit), clk_i, !rst_ni)

  // this is formulated as an assumption such that the FPV testbenches do disprove this
  // property by mistake
  //`ASSUME(reqParity, tl_reg_h2d.a_valid |-> tl_reg_h2d.a_user.chk_en == tlul_pkg::CheckDis)

endmodule
