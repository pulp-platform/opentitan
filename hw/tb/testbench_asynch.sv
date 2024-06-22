// Copyright 2022 ETH Zurich and University of Bologna.
// Copyright and related rights are licensed under the Solderpad Hardware
// License, Version 0.51 (the "License"); you may not use this file except in
// compliance with the License.  You may obtain a copy of the License at
// http://solderpad.org/licenses/SHL-0.51. Unless required by applicable law
// or agreed to in writing, software, hardware and materials distributed under
// this License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

`include "axi_assign.svh"
`include "axi_typedef.svh"

module testbench_asynch ();

   import lc_ctrl_pkg::*;
   import jtag_pkg::*;
   import jtag_test::*;
   import dm::*;
   import tlul2axi_pkg::*;
   import top_earlgrey_pkg::*;
   import secure_subsystem_synth_pkg::*;
   import "DPI-C" function read_elf(input string filename);
   import "DPI-C" function byte get_section(output longint address, output longint len);
   import "DPI-C" context function byte read_section(input longint address, inout byte buffer[]);


 ////////////////////////////  Defines ////////////////////////////
 
   localparam AxiWideBeWidth    = 4;
   localparam AxiWideByteOffset = $clog2(AxiWideBeWidth);

   
   localparam int unsigned AxiAddrWidth          = SynthAxiAddrWidth;
   localparam int unsigned AxiDataWidth          = SynthAxiDataWidth;
   localparam int unsigned AxiUserWidth          = SynthAxiUserWidth;
   localparam int unsigned AxiOutIdWidth         = SynthAxiOutIdWidth;

   localparam int unsigned AxiOtAddrWidth        = SynthOtAxiAddrWidth;
   localparam int unsigned AxiOtDataWidth        = SynthOtAxiDataWidth;
   localparam int unsigned AxiOtUserWidth        = SynthOtAxiUserWidth;
   localparam int unsigned AxiOtOutIdWidth       = SynthOtAxiOutIdWidth;
 
   localparam int unsigned AsyncAxiOutAwWidth    = SynthAsyncAxiOutAwWidth;
   localparam int unsigned AsyncAxiOutWWidth     = SynthAsyncAxiOutWWidth;
   localparam int unsigned AsyncAxiOutBWidth     = SynthAsyncAxiOutBWidth;
   localparam int unsigned AsyncAxiOutArWidth    = SynthAsyncAxiOutArWidth;
   localparam int unsigned AsyncAxiOutRWidth     = SynthAsyncAxiOutRWidth;

   localparam type         axi_out_aw_chan_t     = synth_axi_out_aw_chan_t;
   localparam type         axi_out_w_chan_t      = synth_axi_out_w_chan_t;
   localparam type         axi_out_b_chan_t      = synth_axi_out_b_chan_t;
   localparam type         axi_out_ar_chan_t     = synth_axi_out_ar_chan_t;
   localparam type         axi_out_r_chan_t      = synth_axi_out_r_chan_t;
   localparam type         axi_out_req_t         = synth_axi_out_req_t;
   localparam type         axi_out_resp_t        = synth_axi_out_resp_t;

   localparam type         axi_ot_out_aw_chan_t  = synth_ot_axi_out_aw_chan_t;
   localparam type         axi_ot_out_w_chan_t   = synth_ot_axi_out_w_chan_t;
   localparam type         axi_ot_out_b_chan_t   = synth_ot_axi_out_b_chan_t;
   localparam type         axi_ot_out_ar_chan_t  = synth_ot_axi_out_ar_chan_t;
   localparam type         axi_ot_out_r_chan_t   = synth_ot_axi_out_r_chan_t;
   localparam type         axi_ot_out_req_t      = synth_ot_axi_out_req_t;
   localparam type         axi_ot_out_resp_t     = synth_ot_axi_out_resp_t;

   localparam int  unsigned LogDepth             = SynthLogDepth;
   localparam int  unsigned CdcSyncStages        = SynthCdcSyncStages;
   
   localparam bit  RAND_RESP = 0; 
   localparam int  AX_MIN_WAIT_CYCLES = 0;   
   localparam int  AX_MAX_WAIT_CYCLES = 1;   
   localparam int  R_MIN_WAIT_CYCLES = 0;   
   localparam int  R_MAX_WAIT_CYCLES = 1;   
   localparam int  RESP_MIN_WAIT_CYCLES = 0;
   localparam int  RESP_MAX_WAIT_CYCLES = 1;
   localparam int  NUM_BEATS = 100;

   parameter  TOHOST              = 32'h1C000000;
   logic [31:0] binary_entry;
   bit [31:0]  exit_code;

   localparam int unsigned JTAG_CLOCK_PERIOD = 40ns;
   localparam int unsigned RTC_CLOCK_PERIOD = 20ns;

   localparam time TA   = 5ns;
   localparam time TT   = 10ns;
//JTAG_CLOCK_PERIOD/5;
   /*
   localparam time TA   = JTAG_CLOCK_PERIOD*0.01;
   localparam time TT   = JTAG_CLOCK_PERIOD*0.7;
*/
   int          secd_sections [bit [31:0]];
   logic [31:0] secd_memory[bit [31:0]];
   string       sram;
   logic [1:0]  boot_mode;

   typedef bit [31:0] word_bt;

   logic [1:0]  bootmode;

   logic aon_clk = 1'b0;
   logic io_clk = 1'b0;
   logic usb_clk = 1'b0;
   logic rst_sys_n;
   logic es_rng_fips;
   logic SCK, CSNeg;
   logic [3:0] SPIdata_i, SPIdata_o, SPIdata_oe_o;
   
   wire  I0, I1, I2, I3, WPNeg, RESETNeg;
   wire  PWROK_S, IOPWROK_S, BIAS_S, RETC_S;
   wire  ibex_uart_rx, ibex_uart_tx;


   logic [AsyncAxiOutAwWidth-1:0] async_axi_out_aw_data_o;
   logic             [LogDepth:0] async_axi_out_aw_wptr_o;
   logic             [LogDepth:0] async_axi_out_aw_rptr_i;
   logic [ AsyncAxiOutWWidth-1:0] async_axi_out_w_data_o;
   logic             [LogDepth:0] async_axi_out_w_wptr_o;
   logic             [LogDepth:0] async_axi_out_w_rptr_i;
   logic [ AsyncAxiOutBWidth-1:0] async_axi_out_b_data_i;
   logic             [LogDepth:0] async_axi_out_b_wptr_i;
   logic             [LogDepth:0] async_axi_out_b_rptr_o;
   logic [AsyncAxiOutArWidth-1:0] async_axi_out_ar_data_o;
   logic             [LogDepth:0] async_axi_out_ar_wptr_o;
   logic             [LogDepth:0] async_axi_out_ar_rptr_i;
   logic [ AsyncAxiOutRWidth-1:0] async_axi_out_r_data_i;
   logic             [LogDepth:0] async_axi_out_r_wptr_i;
   logic             [LogDepth:0] async_axi_out_r_rptr_o;
  
   uart_bus #(.BAUD_RATE(1250000), .PARITY_EN(0)) i_uart0_bus (.rx(ibex_uart_tx), .tx(ibex_uart_rx), .rx_en(1'b1)); //1470588
   
   typedef axi_test::axi_rand_slave #(  
     .AW( AxiAddrWidth  ),
     .DW( AxiDataWidth  ),
     .IW( AxiOutIdWidth ),
     .UW( AxiUserWidth  ),
     .TA(TA),
     .TT(TT),
     .RAND_RESP(RAND_RESP),
     .AX_MIN_WAIT_CYCLES(AX_MIN_WAIT_CYCLES),
     .AX_MAX_WAIT_CYCLES(AX_MAX_WAIT_CYCLES),
     .R_MIN_WAIT_CYCLES(R_MIN_WAIT_CYCLES),
     .R_MAX_WAIT_CYCLES(R_MAX_WAIT_CYCLES),
     .RESP_MIN_WAIT_CYCLES(RESP_MIN_WAIT_CYCLES),
     .RESP_MAX_WAIT_CYCLES(RESP_MAX_WAIT_CYCLES)
   ) axi_ran_slave;
      
   AXI_BUS #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth  ),
    .AXI_DATA_WIDTH ( AxiDataWidth  ),
    .AXI_ID_WIDTH   ( AxiOutIdWidth ),
    .AXI_USER_WIDTH ( AxiUserWidth  )
   ) axi_slave();

   AXI_BUS_DV #(
    .AXI_ADDR_WIDTH ( AxiAddrWidth  ),
    .AXI_DATA_WIDTH ( AxiDataWidth  ),
    .AXI_ID_WIDTH   ( AxiOutIdWidth ),
    .AXI_USER_WIDTH ( AxiUserWidth  )
   ) axi (clk_sys);
   
   typedef jtag_test::riscv_dbg #(
      .IrLength       (5                   ),
      .TA             (TA                   ),
      .TT             (TT                   )
   ) riscv_dbg_t;

   JTAG_DV jtag_mst (jtag_clk);

   jtag_pkg::jtag_req_t jtag_i;
   jtag_pkg::jtag_rsp_t jtag_o;

   axi_out_req_t   ot_axi_req;
   axi_out_resp_t  ot_axi_rsp;

   entropy_src_pkg::entropy_src_rng_req_t es_rng_req;
   entropy_src_pkg::entropy_src_rng_rsp_t es_rng_rsp;

   riscv_dbg_t::jtag_driver_t jtag_driver = new(jtag_mst);
   riscv_dbg_t riscv_dbg = new(jtag_driver);

   axi_ran_slave axi_rand_slave = new(axi);

   `AXI_ASSIGN (axi, axi_slave)
   `AXI_ASSIGN_FROM_REQ (axi_slave, ot_axi_req)
   `AXI_ASSIGN_TO_RESP  (ot_axi_rsp, axi_slave)

   assign jtag_i.tck        = jtag_clk;
   assign jtag_i.trst_n     = jtag_mst.trst_n;
   assign jtag_i.tms        = jtag_mst.tms;
   assign jtag_i.tdi        = jtag_mst.tdi;
   assign jtag_mst.tdo      = jtag_o.tdo;

   assign RESETNeg = 1'b1;
   assign WPNeg    = 1'b0;

   assign ibex_uart_rx = '0;

   clk_rst_gen #(
    .ClkPeriod    ( JTAG_CLOCK_PERIOD ),
    .RstClkCycles ( 1000 )
   ) i_clk_jtag (
    .clk_o  ( jtag_clk ),
    .rst_no ( rst_o )
   );

   clk_rst_gen #(
    .ClkPeriod    ( RTC_CLOCK_PERIOD ),
    .RstClkCycles ( 100 )
   ) i_clk_sys (
    .clk_o  ( clk_sys ),
    .rst_no ( rst_sys_n )
   );

localparam dm::sbcs_t JtagInitSbcs = dm::sbcs_t'{
                                      sbautoincrement: 1'b1,
                                      sbreadondata: 1'b1,
                                      sbaccess: 3'h2,
                                      default: '0
                                    };
`ifdef VIPS
   pad_alsaqr i_I0 ( .OEN(~SPIdata_oe_o[0]), .I(SPIdata_o[0]), .O(), .PUEN(1'b1), .PAD(I0),
                     .DRV(2'b00), .SLW(1'b0), .SMT(1'b0), .PWROK(PWROK_S),
                     .IOPWROK(IOPWROK_S), .BIAS(BIAS_S), .RETC(RETC_S)   );
   pad_alsaqr i_I1 ( .OEN(~SPIdata_oe_o[1]), .I(), .O(SPIdata_i[1]), .PUEN(1'b1), .PAD(I1),
                     .DRV(2'b00), .SLW(1'b0), .SMT(1'b0), .PWROK(PWROK_S), .IOPWROK(IOPWROK_S),
                     .BIAS(BIAS_S), .RETC(RETC_S)   );
   s25fs256s #(
    .TimingModel   ( "S25FS256SAGMFI000_F_30pF" ),
    .mem_file_name ( "./sw/tests/opentitan/flash_hmac_smoketest/bazel-out/flash_hmac_smoketest_signed8.vmem" ),
    .UserPreload   ( 1 )
   ) i_spi_flash_csn0 (
    .SI       ( I0 ),
    .SO       ( I1 ),
    .SCK,
    .CSNeg,
    .WPNeg    (    ),
    .RESETNeg (    )
   );
`endif //  `ifdef VIPS

   axi_cdc_dst #(
     .LogDepth   ( LogDepth         ),
     .SyncStages ( CdcSyncStages    ),
     .aw_chan_t  ( axi_out_aw_chan_t ),
     .w_chan_t   ( axi_out_w_chan_t  ),
     .b_chan_t   ( axi_out_b_chan_t  ),
     .ar_chan_t  ( axi_out_ar_chan_t ),
     .r_chan_t   ( axi_out_r_chan_t  ),
     .axi_req_t  ( axi_out_req_t     ),
     .axi_resp_t ( axi_out_resp_t    )
   ) i_cdc_in (
     .async_data_slave_aw_data_i( async_axi_out_aw_data_o ),
     .async_data_slave_aw_wptr_i( async_axi_out_aw_wptr_o ),
     .async_data_slave_aw_rptr_o( async_axi_out_aw_rptr_i ),
     .async_data_slave_w_data_i ( async_axi_out_w_data_o  ),
     .async_data_slave_w_wptr_i ( async_axi_out_w_wptr_o  ),
     .async_data_slave_w_rptr_o ( async_axi_out_w_rptr_i  ),
     .async_data_slave_b_data_o ( async_axi_out_b_data_i  ),
     .async_data_slave_b_wptr_o ( async_axi_out_b_wptr_i  ),
     .async_data_slave_b_rptr_i ( async_axi_out_b_rptr_o  ),
     .async_data_slave_ar_data_i( async_axi_out_ar_data_o ),
     .async_data_slave_ar_wptr_i( async_axi_out_ar_wptr_o ),
     .async_data_slave_ar_rptr_o( async_axi_out_ar_rptr_i ),
     .async_data_slave_r_data_o ( async_axi_out_r_data_i  ),
     .async_data_slave_r_wptr_o ( async_axi_out_r_wptr_i  ),
     .async_data_slave_r_rptr_i ( async_axi_out_r_rptr_o  ),
     .dst_clk_i                 ( clk_sys   ),
     .dst_rst_ni                ( rst_sys_n ),
     .dst_req_o                 ( ot_axi_req ),
     .dst_resp_i                ( ot_axi_rsp )
   );

/////////////////////////////// DUT ///////////////////////////////

`ifndef NETLIST
   secure_subsystem_synth_wrap dut (
`else
   security_island dut (
`endif
       .clk_i            ( clk_sys       ),
       .clk_ref_i        ( clk_sys       ),
       .rst_ni           ( rst_sys_n     ),
       .pwr_on_rst_ni    ( rst_sys_n     ),
       .fetch_en_i       ( '0            ),
       .bootmode_i       ( bootmode      ),
       .test_enable_i    ( '0            ),
       .irq_ibex_i       ( '0            ),
    // JTAG port
       .jtag_tck_i       ( jtag_i.tck    ),
       .jtag_tms_i       ( jtag_i.tms    ),
       .jtag_trst_n_i    ( jtag_i.trst_n ),
       .jtag_tdi_i       ( jtag_i.tdi    ),
       .jtag_tdo_o       ( jtag_o.tdo    ),
       .jtag_tdo_oe_o    (               ),
    // Asynch axi port
       .async_axi_out_aw_data_o,
       .async_axi_out_aw_wptr_o,
       .async_axi_out_aw_rptr_i,
       .async_axi_out_w_data_o,
       .async_axi_out_w_wptr_o,
       .async_axi_out_w_rptr_i,
       .async_axi_out_b_data_i,
       .async_axi_out_b_wptr_i,
       .async_axi_out_b_rptr_o,
       .async_axi_out_ar_data_o,
       .async_axi_out_ar_wptr_o,
       .async_axi_out_ar_rptr_i,
       .async_axi_out_r_data_i,
       .async_axi_out_r_wptr_i,
       .async_axi_out_r_rptr_o,      
    // Uart
       .ibex_uart_rx_i   ( ibex_uart_rx  ),
       .ibex_uart_tx_o   ( ibex_uart_tx  ),
    // SPI host
 `ifdef VIPS
       .spi_host_SCK_o   ( SCK           ),
       .spi_host_SCK_en_o(               ),
       .spi_host_CSB_o   ( CSNeg         ),
       .spi_host_CSB_en_o(               ),
       .spi_host_SD_o    ( SPIdata_o     ),
       .spi_host_SD_i    ( SPIdata_i     ),
       .spi_host_SD_en_o ( SPIdata_oe_o  ),            
 `else
       .spi_host_SCK_o   (               ),
       .spi_host_SCK_en_o(               ),
       .spi_host_CSB_o   (               ),
       .spi_host_CSB_en_o(               ),
       .spi_host_SD_o    (               ),
       .spi_host_SD_i    ( '0            ),
       .spi_host_SD_en_o (               ),
 `endif
       .axi_isolated_o   (               ),
       .axi_isolate_i    ( '0            ),
       .gpio_0_i         ( '0            ), 
       .gpio_1_i         ( '0            ),
       .gpio_0_o         (               ), 
       .gpio_1_o         (               ),
       .gpio_0_oe_o      (               ), 
       .gpio_1_oe_o      (               )
   );
 
///////////////////////// Processes ///////////////////////////////

  initial begin  : axi_slave_process

    @(posedge rst_sys_n);
    axi_rand_slave.reset();
    repeat (4)  @(posedge clk_sys);
     axi_rand_slave.run();

  end

  initial begin
    @(negedge rst_sys_n);
    riscv_dbg.reset_master();
  end

  initial  begin : bootmodes

    if(!$value$plusargs("BOOTMODE=%d", boot_mode)) begin
       boot_mode=0;
       $display("BOOTMODE: %d", boot_mode);
    end
    if(!$value$plusargs("SRAM=%s", sram)) begin
       sram="";
       $display("Loading to SRAM: %s", sram);
    end
     
    case(boot_mode)
        0:begin
          bootmode = 2'b00;
          riscv_dbg.reset_master();
          if (sram != "") begin
               repeat(10000)
                 @(posedge clk_sys);
               jtag_init();
               jtag_elf_load(sram, binary_entry);
               jtag_elf_run(32'h e0000080); //preload the flashif
          `ifdef JTAG_SEC_BOOT
               repeat(250000)
                 @(posedge clk_sys);
               jtag_elf_run(32'h d0008080); //secure boot
          `endif
               jtag_wait_for_eoc(TOHOST);
          end
        end
        1:begin
          bootmode = 2'b01;
          riscv_dbg.reset_master();
          jtag_wait_for_eoc(TOHOST);
        end
        default:begin
          $fatal("Unsupported bootmode");
        end
    endcase // case (boot_mode)
  end // block: bootmodes

///////////////////////////// Tasks ///////////////////////////////

  task automatic jtag_read_reg;
    input logic [31:0] addr;
    output logic [31:0] rdata;

    automatic dm::sbcs_t sbcs = '{
      sbautoincrement: 1'b1,
      sbreadondata   : 1'b1,
      default        : 1'b0
    };

    sbcs.sbreadonaddr = 1;
    riscv_dbg.write_dmi(dm::SBCS, sbcs);
    do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
    while (sbcs.sbbusy);
    riscv_dbg.write_dmi(dm::SBAddress0, addr);
    do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
    while (sbcs.sbbusy);
    riscv_dbg.read_dmi_exp_backoff(dm::SBData0, rdata);
    // Wait until SBA is free to read another 32 bits
    do riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
    while (sbcs.sbbusy);
  endtask

  task automatic jtag_write_reg(input logic [31:0] start_addr, input logic [31:0] value);
    logic [31:0]      rdata;

    $display("[JTAG] Start writing at %x ", start_addr);
    jtag_write(dm::SBCS, JtagInitSbcs, 1, 1);
    // Write address
    jtag_write(dm::SBAddress0, start_addr);
    // Write data
    jtag_write(dm::SBData0, value[31:0]);

    //Check correctess
    jtag_read_reg(start_addr, rdata);
    if(rdata!=value) begin
      $fatal(1,"rdata at %x: %x" , start_addr, rdata);
    end else begin
      $display("W/R sanity check at %x ok! : %x", start_addr, rdata);
    end
  endtask

  task automatic jtag_write(
    input dm::dm_csr_e addr,
    input word_bt data,
    input bit wait_cmd = 0,
    input bit wait_sba = 0
  );
    riscv_dbg.write_dmi(addr, data);
    if (wait_cmd) begin
      dm::abstractcs_t acs;
      do begin
        riscv_dbg.read_dmi_exp_backoff(dm::AbstractCS, acs);
        if (acs.cmderr) $fatal(1, "[JTAG] Abstract command error!");
      end while (acs.busy);
    end
    if (wait_sba) begin
      dm::sbcs_t sbcs;
      do begin
        riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
        if (sbcs.sberror | sbcs.sbbusyerror) $fatal(1, "[JTAG] System bus error!");
      end while (sbcs.sbbusy);
    end
  endtask

  // Initialize the debug module
  task automatic jtag_init();
    dm::dtm_op_status_e op;
    automatic int dmi_wait_cycles = 10;
    logic[31:0] idcode;
    dm::dmcontrol_t dmcontrol = '{dmactive: 1, default: '0};
    // Check ID code
    repeat(5000) @(posedge clk_sys);
    riscv_dbg.get_idcode(idcode);
    $display("ID: 0x%h",idcode);
    jtag_write(dm::DMControl, dmcontrol);
    riscv_dbg.read_dmi_exp_backoff(dm::DMControl, dmcontrol);
    //while (~dmcontrol.dmactive);
    // Activate, wait for system bus
    jtag_write(dm::SBCS, JtagInitSbcs, 0, 1);
    $display("[JTAG] Initialization success");
  endtask

  task automatic jtag_poll_bit0(
    input word_bt addr,
    output word_bt data,
    input int unsigned idle_cycles
  );
    automatic dm::sbcs_t sbcs = dm::sbcs_t'{sbreadonaddr: 1'b1, sbaccess: 2, default: '0};
    jtag_write(dm::SBCS, sbcs, 0, 1);
    do begin
      jtag_write(dm::SBAddress0, addr);
      riscv_dbg.wait_idle(idle_cycles);
      riscv_dbg.read_dmi_exp_backoff(dm::SBData0, data);
    end while (~data[0]);
  endtask

  // Load a binary
  task automatic jtag_elf_load(input string binary, output word_bt binary_entry);
    dm::dmstatus_t status;
    // Halt hart i
    jtag_write(dm::DMControl, dm::dmcontrol_t'{haltreq: 1, dmactive: 1, default: '0});
    do riscv_dbg.read_dmi_exp_backoff(dm::DMStatus, status);
    while (~status.allhalted);
    jtag_elf_preload(binary, binary_entry);
  endtask

  // Run a binary
  task automatic jtag_elf_run(input word_bt binary_entry);
    dm::sbcs_t sbcs;
    do begin
      riscv_dbg.read_dmi_exp_backoff(dm::SBCS, sbcs);
      if (sbcs.sberror | sbcs.sbbusyerror) $fatal(1, "[JTAG] System bus error!");
    end while (sbcs.sbbusy);
    // Repoint execution
    $display("boot at : %h", binary_entry);
    jtag_write(dm::Data0, binary_entry);
    jtag_write(dm::Command, {8'h0,1'b0,3'h2,1'b0,1'b0,1'b1,1'b1,4'h0,dm::CSR_DPC});
    // Resume hart 0
    jtag_write(dm::DMControl, dm::dmcontrol_t'{resumereq: 1, dmactive: 1, default: '0});
  endtask

  // Load a binary
  task automatic jtag_elf_preload(input string binary, output word_bt entry);
    longint sec_addr, sec_len;
    entry = 0;
    $display("[JTAG] Preloading ELF binary: %s", binary);
    if (read_elf(binary))
      $fatal(1, "[JTAG] Failed to load ELF!");
    while (get_section(sec_addr, sec_len)) begin
      byte bf[] = new [sec_len];
      $display("[JTAG] Preloading section at 0x%h (%0d bytes)", sec_addr, sec_len);
      if (read_section(sec_addr, bf)) $fatal(1, "[JTAG] Failed to read ELF section!");
      jtag_write(dm::SBCS, JtagInitSbcs, 1, 1);
      // Write address as 64-bit double
      jtag_write(dm::SBAddress0, sec_addr);
      for (longint i = 0; i <= sec_len ; i += 4) begin
        bit checkpoint = (i != 0 && i % 512 == 0);
        if (checkpoint)
          $display("[JTAG] - %0d/%0d bytes (%0d%%)", i, sec_len, i*100/(sec_len>1 ? sec_len-1 : 1));
        jtag_write(dm::SBData0, {bf[i+3], bf[i+2], bf[i+1], bf[i]}, checkpoint, checkpoint);
      end
    end
    $display("[JTAG] Preload complete");
  endtask

  // Wait for termination signal and get return code
  task automatic jtag_wait_for_eoc(input word_bt tohost);
    jtag_poll_bit0(tohost, exit_code, 10);
    exit_code >>= 1;
    if (exit_code) $error("[JTAG] FAILED: return code %0d", exit_code);
    else $display("[JTAG] SUCCESS");
    $finish;
  endtask

endmodule
