// Copyright lowRISC contributors.
// Licensed under the Apache License, Version 2.0, see LICENSE for details.
// SPDX-License-Identifier: Apache-2.0
//
// Register Top module auto-generated by `reggen`

`include "prim_assert.sv"

module debug_mode_regs_reg_top (
  input clk_i,
  input rst_ni,
  input  tlul_pkg::tl_h2d_t tl_i,
  output tlul_pkg::tl_d2h_t tl_o,
  // To HW
  output debug_mode_regs_reg_pkg::debug_mode_regs_reg2hw_t reg2hw, // Write
  input  debug_mode_regs_reg_pkg::debug_mode_regs_hw2reg_t hw2reg, // Read

  // Integrity check errors
  output logic intg_err_o,

  // Config
  input devmode_i // If 1, explicit error return for unmapped register access
);

  import debug_mode_regs_reg_pkg::* ;

  localparam int AW = 5;
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
  logic [5:0] reg_we_check;
  prim_reg_we_check #(
    .OneHotWidth(6)
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
  logic payload_1_we;
  logic [31:0] payload_1_qs;
  logic [31:0] payload_1_wd;
  logic payload_2_we;
  logic [31:0] payload_2_qs;
  logic [31:0] payload_2_wd;
  logic payload_3_we;
  logic [31:0] payload_3_qs;
  logic [31:0] payload_3_wd;
  logic address_we;
  logic [31:0] address_qs;
  logic [31:0] address_wd;
  logic start_we;
  logic start_start_qs;
  logic start_start_wd;
  logic [30:0] start_field1_qs;
  logic [30:0] start_field1_wd;
  logic debug_mode_we;
  logic debug_mode_debug_mode_qs;
  logic debug_mode_debug_mode_wd;
  logic [30:0] debug_mode_field1_qs;
  logic [30:0] debug_mode_field1_wd;

  // Register instances
  // R[payload_1]: V(False)
  prim_ot_subreg #(
    .DW      (32),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0)
  ) u_payload_1 (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (payload_1_we),
    .wd     (payload_1_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.payload_1.q),
    .ds     (),

    // to register interface (read)
    .qs     (payload_1_qs)
  );


  // R[payload_2]: V(False)
  prim_ot_subreg #(
    .DW      (32),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0)
  ) u_payload_2 (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (payload_2_we),
    .wd     (payload_2_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.payload_2.q),
    .ds     (),

    // to register interface (read)
    .qs     (payload_2_qs)
  );


  // R[payload_3]: V(False)
  prim_ot_subreg #(
    .DW      (32),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0)
  ) u_payload_3 (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (payload_3_we),
    .wd     (payload_3_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.payload_3.q),
    .ds     (),

    // to register interface (read)
    .qs     (payload_3_qs)
  );


  // R[address]: V(False)
  prim_ot_subreg #(
    .DW      (32),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (32'h0)
  ) u_address (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (address_we),
    .wd     (address_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.address.q),
    .ds     (),

    // to register interface (read)
    .qs     (address_qs)
  );


  // R[start]: V(False)
  //   F[start]: 0:0
  prim_ot_subreg #(
    .DW      (1),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (1'h0)
  ) u_start_start (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (start_we),
    .wd     (start_start_wd),

    // from internal hardware
    .de     (hw2reg.start.start.de),
    .d      (hw2reg.start.start.d),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.start.start.q),
    .ds     (),

    // to register interface (read)
    .qs     (start_start_qs)
  );

  //   F[field1]: 31:1
  prim_ot_subreg #(
    .DW      (31),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (31'h0)
  ) u_start_field1 (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (start_we),
    .wd     (start_field1_wd),

    // from internal hardware
    .de     (hw2reg.start.field1.de),
    .d      (hw2reg.start.field1.d),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.start.field1.q),
    .ds     (),

    // to register interface (read)
    .qs     (start_field1_qs)
  );


  // R[debug_mode]: V(False)
  //   F[debug_mode]: 0:0
  prim_ot_subreg #(
    .DW      (1),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (1'h1)
  ) u_debug_mode_debug_mode (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (debug_mode_we),
    .wd     (debug_mode_debug_mode_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.debug_mode.debug_mode.q),
    .ds     (),

    // to register interface (read)
    .qs     (debug_mode_debug_mode_qs)
  );

  //   F[field1]: 31:1
  prim_ot_subreg #(
    .DW      (31),
    .SwAccess(prim_ot_subreg_pkg::SwAccessRW),
    .RESVAL  (31'h0)
  ) u_debug_mode_field1 (
    .clk_i   (clk_i),
    .rst_ni  (rst_ni),

    // from register interface
    .we     (debug_mode_we),
    .wd     (debug_mode_field1_wd),

    // from internal hardware
    .de     (1'b0),
    .d      ('0),

    // to internal hardware
    .qe     (),
    .q      (reg2hw.debug_mode.field1.q),
    .ds     (),

    // to register interface (read)
    .qs     (debug_mode_field1_qs)
  );



  logic [5:0] addr_hit;
  always_comb begin
    addr_hit = '0;
    addr_hit[0] = (reg_addr == DEBUG_MODE_REGS_PAYLOAD_1_OFFSET);
    addr_hit[1] = (reg_addr == DEBUG_MODE_REGS_PAYLOAD_2_OFFSET);
    addr_hit[2] = (reg_addr == DEBUG_MODE_REGS_PAYLOAD_3_OFFSET);
    addr_hit[3] = (reg_addr == DEBUG_MODE_REGS_ADDRESS_OFFSET);
    addr_hit[4] = (reg_addr == DEBUG_MODE_REGS_START_OFFSET);
    addr_hit[5] = (reg_addr == DEBUG_MODE_REGS_DEBUG_MODE_OFFSET);
  end

  assign addrmiss = (reg_re || reg_we) ? ~|addr_hit : 1'b0 ;

  // Check sub-word write is permitted
  always_comb begin
    wr_err = (reg_we &
              ((addr_hit[0] & (|(DEBUG_MODE_REGS_PERMIT[0] & ~reg_be))) |
               (addr_hit[1] & (|(DEBUG_MODE_REGS_PERMIT[1] & ~reg_be))) |
               (addr_hit[2] & (|(DEBUG_MODE_REGS_PERMIT[2] & ~reg_be))) |
               (addr_hit[3] & (|(DEBUG_MODE_REGS_PERMIT[3] & ~reg_be))) |
               (addr_hit[4] & (|(DEBUG_MODE_REGS_PERMIT[4] & ~reg_be))) |
               (addr_hit[5] & (|(DEBUG_MODE_REGS_PERMIT[5] & ~reg_be)))));
  end

  // Generate write-enables
  assign payload_1_we = addr_hit[0] & reg_we & !reg_error;

  assign payload_1_wd = reg_wdata[31:0];
  assign payload_2_we = addr_hit[1] & reg_we & !reg_error;

  assign payload_2_wd = reg_wdata[31:0];
  assign payload_3_we = addr_hit[2] & reg_we & !reg_error;

  assign payload_3_wd = reg_wdata[31:0];
  assign address_we = addr_hit[3] & reg_we & !reg_error;

  assign address_wd = reg_wdata[31:0];
  assign start_we = addr_hit[4] & reg_we & !reg_error;

  assign start_start_wd = reg_wdata[0];

  assign start_field1_wd = reg_wdata[31:1];
  assign debug_mode_we = addr_hit[5] & reg_we & !reg_error;

  assign debug_mode_debug_mode_wd = reg_wdata[0];

  assign debug_mode_field1_wd = reg_wdata[31:1];

  // Assign write-enables to checker logic vector.
  always_comb begin
    reg_we_check = '0;
    reg_we_check[0] = payload_1_we;
    reg_we_check[1] = payload_2_we;
    reg_we_check[2] = payload_3_we;
    reg_we_check[3] = address_we;
    reg_we_check[4] = start_we;
    reg_we_check[5] = debug_mode_we;
  end

  // Read data return
  always_comb begin
    reg_rdata_next = '0;
    unique case (1'b1)
      addr_hit[0]: begin
        reg_rdata_next[31:0] = payload_1_qs;
      end

      addr_hit[1]: begin
        reg_rdata_next[31:0] = payload_2_qs;
      end

      addr_hit[2]: begin
        reg_rdata_next[31:0] = payload_3_qs;
      end

      addr_hit[3]: begin
        reg_rdata_next[31:0] = address_qs;
      end

      addr_hit[4]: begin
        reg_rdata_next[0] = start_start_qs;
        reg_rdata_next[31:1] = start_field1_qs;
      end

      addr_hit[5]: begin
        reg_rdata_next[0] = debug_mode_debug_mode_qs;
        reg_rdata_next[31:1] = debug_mode_field1_qs;
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
  assign reg_busy = shadow_busy;

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
