// Copyright 2020 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Author: Florian Zaruba <zarubaf@iis.ee.ethz.ch>

`include "common_cells/assertions.svh"
`include "common_cells/registers.svh"
`include "snitch/typedef.svh"
`include "reqrsp_interface/typedef.svh"
`include "tcdm_interface/typedef.svh"
`include "dca_interface/typedef.svh"

/// Snitch Core Complex (CC)
/// Contains the Snitch Integer Core + FPU + Private Accelerators
module snitch_cc
  import snitch_cc_pkg::*;
#(
  /// Address width of the buses
  parameter int unsigned AddrWidth          = 0,
  /// Data width of the buses.
  parameter int unsigned DataWidth          = 0,
  /// Address width of the TCDM bus.
  parameter int unsigned TcdmAddrWidth      = 0,
  /// User width of the TCDM bus.
  parameter int unsigned TcdmUserWidth      = 0,
  /// Data width of the AXI DMA buses.
  parameter int unsigned DMADataWidth       = 0,
  /// Id width of the AXI DMA bus.
  parameter int unsigned DMAIdWidth         = 0,
  /// User width of the AXI DMA bus.
  parameter int unsigned DMAUserWidth       = 0,
  parameter int unsigned DMANumAxInFlight   = 0,
  parameter int unsigned DMAReqFifoDepth    = 0,
  parameter int unsigned DMANumChannels     = 0,
  parameter bit          DMAEnableCompute   = 1'b0,
  parameter idma_pkg::compute_enable_t DMAComputeOps    = '1,
  parameter idma_pkg::compute_tuning_t DMAComputeTuning = '1,
  parameter type         axi_ar_chan_t      = logic,
  parameter type         axi_aw_chan_t      = logic,
  parameter type         axi_req_t          = logic,
  parameter type         axi_rsp_t          = logic,
  parameter type         hive_req_t         = logic,
  parameter type         hive_rsp_t         = logic,
  parameter type         dma_events_t       = logic,
  // XIF parameters
  parameter bit          EnableXif          = 1,
  parameter int unsigned XifIdWidth         = 4,
  // XIF port types
  parameter type         x_issue_req_t      = logic,
  parameter type         x_issue_resp_t     = logic,
  parameter type         x_register_t       = logic,
  parameter type         x_commit_t         = logic,
  parameter type         x_result_t         = logic,
  parameter fpnew_pkg::fpu_implementation_t FPUImplementation = '0,
  /// Boot address of core.
  parameter logic [31:0] BootAddr           = 32'h0000_1000,
  /// Core ISA configuration.
  parameter snitch_pkg::isa_cfg_t IsaCfg    = '0,
  /// Enable private IPU.
  parameter bit          PrivateIpu         = 0,
  /// Has virtual memory support.
  parameter bit          VMSupport          = 1,
  parameter int unsigned NumIntOutstandingLoads = 0,
  parameter int unsigned NumIntOutstandingMem = 0,
  parameter int unsigned NumFPOutstandingLoads = 0,
  parameter int unsigned NumFPOutstandingMem = 0,
  parameter int unsigned NumDTLBEntries = 0,
  parameter int unsigned NumITLBEntries = 0,
  parameter int unsigned NumSequencerInstr = 0,
  parameter int unsigned NumSequencerLoops = 0,
  parameter int unsigned NumSsrs = 0,
  parameter int unsigned SsrMuxRspDepth = 0,
  parameter snitch_ssr_pkg::ssr_cfg_t [cc_pkg::iomsb(NumSsrs):0] SsrCfgs = '0,
  parameter logic [cc_pkg::iomsb(NumSsrs):0][4:0] SsrRegs = '0,
  /// Spatz parameters
  parameter int unsigned NumSpatzOutstandingLoads = 4,
  parameter bit          SpatzDoubleBw      = 0,
  /// Add isochronous clock-domain crossings e.g., make it possible to operate
  /// the core in a slower clock domain.
  parameter bit          IsoCrossing        = 0,
  /// Timing Parameters
  /// Insert Pipeline registers into off-loading path (request)
  parameter bit          RegisterOffloadReq = 0,
  /// Insert Pipeline registers into off-loading path (response)
  parameter bit          RegisterOffloadRsp = 0,
  /// Insert Pipeline registers into data memory path (request)
  parameter bit          RegisterCoreReq    = 0,
  /// Insert Pipeline registers into data memory path (response)
  parameter bit          RegisterCoreRsp    = 0,
  /// Insert Pipeline register into the FPU data path (request)
  parameter bit          RegisterFPUReq     = 0,
  /// Insert Pipeline registers after sequencer
  parameter bit          RegisterSequencer  = 0,
  /// Insert Pipeline registers immediately before FPU datapath
  parameter bit          RegisterFPUIn      = 0,
  /// Insert Pipeline registers immediately after FPU datapath
  parameter bit          RegisterFPUOut     = 0,
  /// Cut DCA request to FPU
  parameter bit          RegisterDcaReq     = 0,
  /// Cut DCA response from FPU
  parameter bit          RegisterDcaRsp     = 0,
  parameter snitch_pma_pkg::snitch_pma_t SnitchPMACfg = '{default: 0},
  /// Consistency Address Queue (CAQ) parameters.
  parameter int unsigned CaqDepth     = 0,
  parameter int unsigned CaqTagWidth  = 0,
  /// Enable debug support.
  parameter bit          DebugSupport = 1,
  /// Optional fixed TCDM alias.
  parameter bit          TCDMAliasEnable = 1'b0,
  parameter logic [AddrWidth-1:0] TCDMAliasStart  = '0,
  /// Width of the collective operation field
  parameter int unsigned CollectiveWidth    = 1,
  /// Enable direct compute access (DCA).
  parameter bit          EnableDca          = 0,
  /// Enable Spatz vector-to-AXI joiner (v0.1)
  /// When 0, Spatz VLSU ports go straight to TCDM (current behavior).
  /// When 1, each VLSU port is routed by address, either to TCDM ports or AXI contiguous access are joined
  /// to one 64B beat. DMA is disabled in this mode (`Xdma` + `EnableSpatzAxi` is illegal)
  parameter bit          EnableSpatzAxi     = 1'b1,
  /// Derived parameter *Do not override*
  localparam int unsigned NumTcdmPorts = snitch_cc_pkg::get_tcdm_ports(
    IsaCfg,
    NumSsrs,
    spatz_pkg::N_FU,
    SpatzDoubleBw
  ),
  localparam type addr_t = logic [AddrWidth-1:0],
  localparam type lsu_req_t = `LSU_REQ_STRUCT(DataWidth, AddrWidth, snitch_pkg::UserWidth),
  localparam type lsu_rsp_t = `LSU_RSP_STRUCT(DataWidth),
  localparam type tcdm_req_t = `TCDM_REQ_STRUCT(DataWidth, TcdmAddrWidth, TcdmUserWidth),
  localparam type tcdm_rsp_t = `TCDM_RSP_STRUCT(DataWidth),
  localparam int unsigned DcaDataWidth = datapath_width(IsaCfg, DataWidth),
  localparam type dca_req_t = `DCA_REQ_STRUCT(DcaDataWidth),
  localparam type dca_rsp_t = `DCA_RSP_STRUCT(DcaDataWidth)
) (
  input  logic                              clk_i,
  input  logic                              clk_d2_i,
  input  logic                              rst_ni,
  input  logic                              rst_int_ss_ni,
  input  logic                              rst_fp_ss_ni,
  input  snitch_pkg::hart_id_t              hart_id_i,
  input  snitch_pkg::interrupts_t           irq_i,
  output hive_req_t                         hive_req_o,
  input  hive_rsp_t                         hive_rsp_i,
  // SoC data port
  output lsu_req_t                          soc_req_o,
  input  lsu_rsp_t                          soc_rsp_i,
  // TCDM ports
  output tcdm_req_t [NumTcdmPorts-1:0]      tcdm_req_o,
  input  tcdm_rsp_t [NumTcdmPorts-1:0]      tcdm_rsp_i,
  // X-interface
  output x_issue_req_t                      x_issue_req_o,
  input  x_issue_resp_t                     x_issue_resp_i,
  output logic                              x_issue_valid_o,
  input  logic                              x_issue_ready_i,
  output x_register_t                       x_register_o,
  output logic                              x_register_valid_o,
  input  logic                              x_register_ready_i,
  output x_commit_t                         x_commit_o,
  output logic                              x_commit_valid_o,
  input  x_result_t                         x_result_i,
  input  logic                              x_result_valid_i,
  output logic                              x_result_ready_o,
  // DMA ports
  output axi_req_t    [DMANumChannels-1:0]  axi_dma_req_o,
  input  axi_rsp_t    [DMANumChannels-1:0]  axi_dma_res_i,
  output logic        [DMANumChannels-1:0]  axi_dma_busy_o, // NOTE: unused
  output dma_events_t [DMANumChannels-1:0]  axi_dma_events_o,
  // Core event strobes
  output snitch_pkg::core_events_t          core_events_o,
  input  addr_t                             tcdm_addr_base_i,
  // Cluster HW barrier
  output logic                              barrier_o,
  input  logic                              barrier_i,
  // Direct Compute Access (DCA) interface
  input  dca_req_t                          dca_req_i,
  output dca_rsp_t                          dca_rsp_o
);

  localparam bit NativeFpSupport = snitch_pkg::calculate_fp_enable(IsaCfg) && !IsaCfg.RVV;
  localparam bit Xpulpv2 = snitch_pkg::calculate_xpulpv2(IsaCfg);
  localparam int unsigned NumSpatzMemPorts = snitch_cc_pkg::num_spatz_mem_ports(
    spatz_pkg::N_FU,
    SpatzDoubleBw
  );
  typedef logic [DataWidth-1:0] data_t;

  // Define lsu_req_chan_t and lsu_rsp_chan_t
  `LSU_TYPEDEF_REQRSP_CHAN_ALL(lsu, DataWidth, AddrWidth, snitch_pkg::UserWidth)

  // Define tcdm_lsu_req_t and tcdm_lsu_rsp_t
  `LSU_TYPEDEF_ALL(tcdm_lsu, DataWidth, TcdmAddrWidth, TcdmUserWidth)

  // Define tcdm_req_chan_t and tcdm_rsp_chan_t
  // `TCDM_TYPEDEF_REQRSP_CHAN_ALL(tcdm, DataWidth, TcdmAddrWidth, TcdmUserWidth)

  // Define dca_req_chan_t and dca_rsp_chan_t
  `DCA_TYPEDEF_REQRSP_CHAN_ALL(dca, DcaDataWidth)

  // Define acc_req_t, acc_rsp_t, acc_req_chan_t and acc_rsp_chan_t
  `SNITCH_ACC_TYPEDEF_ALL(DataWidth, AddrWidth)

  // Accelerator offload interface
  acc_req_t snitch_acc_req;
  acc_rsp_t snitch_acc_rsp;
  acc_req_t snitch_acc_req_q;
  acc_rsp_t snitch_acc_rsp_q;
  acc_req_t [snitch_pkg::NUM_ACC-1:0] snitch_acc_req_demuxed;
  acc_rsp_t [snitch_pkg::NUM_ACC-1:0] snitch_acc_rsp_demuxed;

  // COPIFT interface
  logic [31:0] i2f_rdata;
  logic        i2f_rvalid;
  logic        i2f_rready;
  logic [31:0] f2i_wdata;
  logic        f2i_wvalid;
  logic        f2i_wready;
  logic        en_copift;

  // FPU control/status signals
  fpnew_pkg::roundmode_e fpu_rnd_mode, spatz_fpu_rnd_mode, fpss_fpu_rnd_mode;
  fpnew_pkg::fmt_mode_t  fpu_fmt_mode, spatz_fpu_fmt_mode, fpss_fpu_fmt_mode;
  fpnew_pkg::status_t    fpu_status, spatz_fpu_status, fpss_fpu_status;

  // Consistency Address Queue (CAQ) interface
  logic caq_pvalid, caq_pvalid_q;

  // Events
  snitch_pkg::core_events_t snitch_events;
  snitch_pkg::core_events_t fpss_events;

  // Snitch LSU interface
  lsu_req_t snitch_lsu_req_d, snitch_lsu_req_q;
  lsu_rsp_t snitch_lsu_rsp_d, snitch_lsu_rsp_q;

  // CPU-side XIF
  x_issue_req_t  x_issue_req;
  x_issue_resp_t x_issue_resp;
  logic          x_issue_valid;
  logic          x_issue_ready;
  x_register_t   x_register;
  logic          x_register_valid;
  logic          x_register_ready;
  x_commit_t     x_commit;
  logic          x_commit_valid;
  x_result_t     x_result;
  logic          x_result_valid;
  logic          x_result_ready;

  // Registered XIF result channel
  x_result_t x_result_q;
  logic      x_result_valid_q;
  logic      x_result_ready_q;

  // Coprocessor-side XIF
  x_issue_req_t  [NumCopro-1:0] cop_issue_req;
  logic          [NumCopro-1:0] cop_issue_valid;
  x_issue_resp_t [NumCopro-1:0] cop_issue_resp;
  logic          [NumCopro-1:0] cop_issue_ready;
  x_register_t   [NumCopro-1:0] cop_register;
  logic          [NumCopro-1:0] cop_register_valid;
  logic          [NumCopro-1:0] cop_register_ready;
  x_commit_t     [NumCopro-1:0] cop_commit;
  logic          [NumCopro-1:0] cop_commit_valid;
  x_result_t     [NumCopro-1:0] cop_result;
  logic          [NumCopro-1:0] cop_result_valid;
  logic          [NumCopro-1:0] cop_result_ready;

  // FPSS LSU interface
  lsu_req_t fpss_lsu_req;
  lsu_rsp_t fpss_lsu_rsp;

  // Registered DCA interface
  dca_req_t dca_req_q;
  dca_rsp_t dca_rsp_q;

  // Demuxed DCA interface
  dca_req_t [NumDcaDemuxPorts-1:0] dca_demux_req;
  dca_rsp_t [NumDcaDemuxPorts-1:0] dca_demux_rsp;

  // SSR interface
  logic  [2:0][4:0] ssr_raddr;
  data_t [2:0]      ssr_rdata;
  logic  [2:0]      ssr_rvalid;
  logic  [2:0]      ssr_rready;
  logic  [2:0]      ssr_rdone;
  logic  [0:0][4:0] ssr_waddr;
  data_t [0:0]      ssr_wdata;
  logic  [0:0]      ssr_wvalid;
  logic  [0:0]      ssr_wready;
  logic  [0:0]      ssr_wdone;
  logic             ssr_streamctl_done;
  logic             ssr_streamctl_valid;
  logic             ssr_streamctl_ready;

  // SSR TCDM interface
  tcdm_req_t [cc_pkg::iomsb(NumSsrs):0]   ssr_tcdm_req;
  tcdm_rsp_t [cc_pkg::iomsb(NumSsrs):0]   ssr_tcdm_rsp;
  tcdm_req_t                              ssr_tcdm_req_0;
  tcdm_rsp_t                              ssr_tcdm_rsp_0;
  tcdm_req_t [cc_pkg::iomsb(NumSsrs-1):0] ssr_tcdm_req_extra;
  tcdm_rsp_t [cc_pkg::iomsb(NumSsrs-1):0] ssr_tcdm_rsp_extra;

  // LSU/SSR0 muxed TCDM interface
  tcdm_req_t muxed_tcdm_req;
  tcdm_rsp_t muxed_tcdm_rsp;

  // Spatz FLSU interface
  lsu_req_t spatz_flsu_req;
  lsu_rsp_t spatz_flsu_rsp;

  // Spatz TCDM interface (narrow channels superseded by `spatz_full_*` above,
  // kept for reference):
  // tcdm_req_chan_t [NumSpatzMemPorts-1:0] spatz_tcdm_req_chan;
  // logic           [NumSpatzMemPorts-1:0] spatz_tcdm_req_valid;
  // logic           [NumSpatzMemPorts-1:0] spatz_tcdm_req_ready;
  // tcdm_rsp_chan_t [NumSpatzMemPorts-1:0] spatz_tcdm_rsp_chan;
  // logic           [NumSpatzMemPorts-1:0] spatz_tcdm_rsp_valid;
  tcdm_req_t      [NumSpatzMemPorts-1:0] spatz_tcdm_req;
  tcdm_rsp_t      [NumSpatzMemPorts-1:0] spatz_tcdm_rsp;

  // Spatz vector-to-AXI steering (v1 review, only used when `EnableSpatzAxi`).
  // Full-width VLSU channels recover the high address bits (Spatz computes
  // `rs1+offset` at full precision but the old TCDM-width channel truncated
  // them); no Spatz RTL change. TCDM path slices low bits back off.
  // Macro parameters:
  // - `DataWidth`: lane payload unchanged (`ELEN == 64 == DataWidth` if `RVD`).
  // - `AddrWidth`: == physical width by construction (`snitch_cluster.sv:1127`
  //   binds `.AddrWidth (PhysicalAddrWidth)`); matches `tcdm_addr_base_i` and
  //   the AXI ports, so no conversion at either consumer. Re-verify that
  //   binding if widths ever change.
  // - `TcdmUserWidth`: sideband untouched; keeps the TCDM slice a field copy.
  `TCDM_TYPEDEF_REQRSP_CHAN_ALL(spatz_full, DataWidth, AddrWidth, TcdmUserWidth)
  // Wide coalesced beat between join and converter (64B data, full address).
  `TCDM_TYPEDEF_ALL(spatz_wide, DMADataWidth, AddrWidth, TcdmUserWidth)
  spatz_full_req_chan_t [NumSpatzMemPorts-1:0] spatz_full_req_chan;
  logic                 [NumSpatzMemPorts-1:0] spatz_full_req_valid;
  logic                 [NumSpatzMemPorts-1:0] spatz_full_req_ready;
  spatz_full_rsp_chan_t [NumSpatzMemPorts-1:0] spatz_full_rsp_chan;
  logic                 [NumSpatzMemPorts-1:0] spatz_full_rsp_valid;
  // TCDM-hit flag: all VLSU ports in an access are assumed to target either
  // all AXI or all TCDM. Decoded from the transfer address against the TCDM base/mask rule.
  logic spatz_is_tcdm;
  // Group-level steering for depth-1 groups.
  logic spatz_group_tcdm, spatz_group_axi;
  // Outstanding-group tracking in `EnableSpatzAxi` mode: path of the accepted
  // group (0: TCDM, 1: AXI) plus per-port response-pending bits.
  logic spatz_path_q;
  logic [NumSpatzMemPorts-1:0] spatz_pend_q;
  // Memory fence support to sync between Spatz and Snitch
  logic [1:0] spatz_mem_finished;
  logic [1:0] spatz_mem_str_finished;
  // snitch and spatz may be in different clock domain (IsoCrossing), raw is
  // sampled by spatz
  logic spatz_busy_raw_d, spatz_busy_raw_q;
  // Counter for outstanding VLSU ops
  logic [2:0] spatz_vlsu_cnt_q, spatz_vlsu_cnt_d;
  logic       spatz_busy_synced;
  logic       spatz_issue_d2_q;
  logic       spatz_busy_d2;
  // AXI-side joiner request/response (drives the DMA AXI ports when enabled).
  axi_req_t spatz_join_req;
  axi_rsp_t spatz_join_res;

  // Muxed LSU request
  lsu_req_t muxed_lsu_req;
  lsu_rsp_t muxed_lsu_rsp;

  // LSU request towards TCDM
  lsu_req_t lsu_tcdm_req;
  lsu_rsp_t lsu_tcdm_rsp;

  // LSU request resized to match TCDM bus widths
  tcdm_lsu_req_t core_lsu_req_resized;
  tcdm_lsu_rsp_t core_lsu_rsp_resized;

  // LSU request converted to TCDM protocol
  tcdm_req_t core_tcdm_req;
  tcdm_rsp_t core_tcdm_rsp;

  // Trace interfaces
  // pragma translate_off
  snitch_pkg::snitch_trace_t        snitch_trace;
  snitch_pkg::fpss_trace_t          fpss_trace;
  snitch_pkg::fpu_sequencer_trace_t fpu_sequencer_trace;
  snitch_pkg::dca_trace_t           dca_trace;
  // pragma translate_on

  ////////////
  // Snitch //
  ////////////

  snitch #(
    .BootAddr              (BootAddr),
    .IsaCfg                (IsaCfg),
    .NativeFpSupport       (NativeFpSupport),
    .AddrWidth             (AddrWidth),
    .DataWidth             (DataWidth),
    .VMSupport             (VMSupport),
    .DebugSupport          (DebugSupport),
    .EnableXif             (EnableXif | IsaCfg.RVV),
    .XifIdWidth            (XifIdWidth),
    .NumIntOutstandingLoads(NumIntOutstandingLoads),
    .NumIntOutstandingMem  (NumIntOutstandingMem),
    .NumDTLBEntries        (NumDTLBEntries),
    .NumITLBEntries        (NumITLBEntries),
    .SnitchPMACfg          (SnitchPMACfg),
    .CaqDepth              (CaqDepth),
    .CaqTagWidth           (CaqTagWidth)
  ) i_snitch (
    .clk_i             (clk_d2_i),
    .rst_i             (~rst_ni),
    // pragma translate_off
    .trace_o           (snitch_trace),
    // pragma translate_on
    .hart_id_i,
    .irq_i,
    .flush_i_valid_o   (hive_req_o.flush_i_valid),
    .flush_i_ready_i   (hive_rsp_i.flush_i_ready),
    .inst_req_o        (hive_req_o.instr_req),
    .inst_rsp_i        (hive_rsp_i.instr_rsp),
    .acc_req_o         (snitch_acc_req),
    .acc_rsp_i         (snitch_acc_rsp),
    .x_issue_req_o     (x_issue_req),
    .x_issue_resp_i    (x_issue_resp),
    .x_issue_valid_o   (x_issue_valid),
    .x_issue_ready_i   (x_issue_ready),
    .x_register_o      (x_register),
    .x_register_valid_o(x_register_valid),
    .x_register_ready_i(x_register_ready),
    .x_commit_o        (x_commit),
    .x_commit_valid_o  (x_commit_valid),
    .x_result_i        (x_result_q),
    .x_result_valid_i  (x_result_valid_q),
    .x_result_ready_o  (x_result_ready_q),
    .i2f_rdata_o       (i2f_rdata),
    .i2f_rvalid_o      (i2f_rvalid),
    .i2f_rready_i      (i2f_rready),
    .f2i_wdata_i       (f2i_wdata),
    .f2i_wvalid_i      (f2i_wvalid),
    .f2i_wready_o      (f2i_wready),
    .caq_pvalid_i      (caq_pvalid_q),
    .lsu_req_o         (snitch_lsu_req_d),
    .lsu_rsp_i         (snitch_lsu_rsp_d),
    .ptw_req_o         (hive_req_o.ptw_req),
    .ptw_rsp_i         (hive_rsp_i.ptw_rsp),
    .fpu_rnd_mode_o    (fpu_rnd_mode),
    .fpu_fmt_mode_o    (fpu_fmt_mode),
    .fpu_status_i      (fpu_status),
    .core_events_o     (snitch_events),
    .barrier_o         (barrier_o),
    .barrier_i         (barrier_i),
    .spatz_busy_i      (spatz_busy_d2),
    .en_copift_o       (en_copift)
  );

  // Cut Snitch's LSU interface
  reqrsp_iso #(
    .req_chan_t(lsu_req_chan_t),
    .rsp_chan_t(lsu_rsp_chan_t),
    .BypassReq(!RegisterCoreReq),
    .BypassRsp(!IsoCrossing && !RegisterCoreRsp)
  ) i_data_cut (
    .src_clk_i (clk_d2_i),
    .src_rst_ni(rst_ni),
    .src_req_i (snitch_lsu_req_d),
    .src_rsp_o (snitch_lsu_rsp_d),
    .dst_clk_i (clk_i),
    .dst_rst_ni(rst_ni),
    .dst_req_o (snitch_lsu_req_q),
    .dst_rsp_i (snitch_lsu_rsp_q)
  );

  // Cut Snitch's accelerator interface
  reqrsp_cut #(
    .req_chan_t(acc_req_chan_t),
    .rsp_chan_t(acc_rsp_chan_t),
    .BypassReq (!RegisterOffloadReq),
    .BypassRsp (!RegisterOffloadRsp)
  ) i_acc_cut (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .slv_req_i(snitch_acc_req),
    .slv_rsp_o(snitch_acc_rsp),
    .mst_req_o(snitch_acc_req_q),
    .mst_rsp_i(snitch_acc_rsp_q)
  );

  // Cut CAQ response for proper handshake with divided clock
  cc_isochronous_spill_register #(
    .data_t(logic),
    .Bypass(!IsoCrossing)
  ) i_spill_register_caq_pvalid (
    .src_clk_i  (clk_i),
    .src_rst_ni (rst_ni),
    .src_valid_i(caq_pvalid),
    .src_ready_o(),
    .src_data_i ('0),
    .dst_clk_i  (clk_d2_i),
    .dst_rst_ni (rst_ni),
    .dst_valid_o(caq_pvalid_q),
    .dst_ready_i(1'b1),
    .dst_data_o ()
  );

  // Cut XIF result channel
  // ----------------------
  // Break the XIF result combinational loop: a coprocessor's x_issue_ready_o
  // can depend combinationally on x_result_ready_i (e.g. Spatz), which feeds
  // back through the demux result arbiter and Snitch's retire logic. A
  // non-bypass spill_register on the CPU-side result channel decouples the
  // ready signal and cuts the loop for any coprocessor.
  cc_spill_register #(
    .data_t(x_result_t),
    .Bypass(1'b0)
  ) i_xif_result_cut (
    .clk_i  (clk_i),
    .rst_ni (rst_ni),
    .clr_i  (1'b0),
    .valid_i(x_result_valid),
    .ready_o(x_result_ready),
    .data_i (x_result),
    .valid_o(x_result_valid_q),
    .ready_i(x_result_ready_q),
    .data_o (x_result_q)
  );

  // Demux FPU control/status signals to Spatz and FPSS
  assign fpss_fpu_rnd_mode = fpu_rnd_mode;
  assign fpss_fpu_fmt_mode = fpu_fmt_mode;
  assign spatz_fpu_rnd_mode = fpu_rnd_mode;
  assign spatz_fpu_fmt_mode = fpu_fmt_mode;
  assign fpu_status = fpss_fpu_status | spatz_fpu_status;

  // Demux accelerator interface to all accelerators.
  // NOTE: `Ordered` is explicitly disabled here to preserve this dispatch
  // path's pre-existing (long-standing, not a regression) behavior, where
  // responses from different accelerators are not guaranteed to be returned
  // in request order.
  reqrsp_demux #(
    .NrPorts   (snitch_pkg::NUM_ACC),
    .Ordered   (1'b0),
    .req_chan_t(acc_req_chan_t),
    .rsp_chan_t(acc_rsp_chan_t)
  ) i_acc_demux (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .slv_req_i(snitch_acc_req_q),
    .slv_rsp_o(snitch_acc_rsp_q),
    .mst_req_o(snitch_acc_req_demuxed),
    .mst_rsp_i(snitch_acc_rsp_demuxed),
    .select_i (snitch_acc_req_q.q.addr[$clog2(snitch_pkg::NUM_ACC)-1:0])
  );

  // Demux XIF to all coprocessors
  cvxif_demux #(
    .NumCopro      (NumCopro),
    .x_issue_req_t (x_issue_req_t),
    .x_issue_resp_t(x_issue_resp_t),
    .x_register_t  (x_register_t),
    .x_commit_t    (x_commit_t),
    .x_result_t    (x_result_t)
  ) i_cvxif_demux (
    .clk_i,
    .rst_ni,
    .cpu_issue_req_i       (x_issue_req),
    .cpu_issue_resp_o      (x_issue_resp),
    .cpu_issue_valid_i     (x_issue_valid),
    .cpu_issue_ready_o     (x_issue_ready),
    .cpu_register_i        (x_register),
    .cpu_register_valid_i  (x_register_valid),
    .cpu_register_ready_o  (x_register_ready),
    .cpu_commit_i          (x_commit),
    .cpu_commit_valid_i    (x_commit_valid),
    .cpu_result_o          (x_result),
    .cpu_result_valid_o    (x_result_valid),
    .cpu_result_ready_i    (x_result_ready),
    .copro_issue_req_o     (cop_issue_req),
    .copro_issue_valid_o   (cop_issue_valid),
    .copro_issue_resp_i    (cop_issue_resp),
    .copro_issue_ready_i   (cop_issue_ready),
    .copro_register_o      (cop_register),
    .copro_register_valid_o(cop_register_valid),
    .copro_register_ready_i(cop_register_ready),
    .copro_commit_o        (cop_commit),
    .copro_commit_valid_o  (cop_commit_valid),
    .copro_result_i        (cop_result),
    .copro_result_valid_i  (cop_result_valid),
    .copro_result_ready_o  (cop_result_ready)
  );

  // Connect external coprocessor port
  assign x_issue_req_o                     = cop_issue_req[ExternalCopro];
  assign x_issue_valid_o                   = cop_issue_valid[ExternalCopro];
  assign cop_issue_resp[ExternalCopro]     = x_issue_resp_i;
  assign cop_issue_ready[ExternalCopro]    = x_issue_ready_i;
  assign x_register_o                      = cop_register[ExternalCopro];
  assign x_register_valid_o                = cop_register_valid[ExternalCopro];
  assign cop_register_ready[ExternalCopro] = x_register_ready_i;
  assign x_commit_o                        = cop_commit[ExternalCopro];
  assign x_commit_valid_o                  = cop_commit_valid[ExternalCopro];
  assign cop_result[ExternalCopro]         = x_result_i;
  assign cop_result_valid[ExternalCopro]   = x_result_valid_i;
  assign x_result_ready_o                  = cop_result_ready[ExternalCopro];

  /////////
  // DMA //
  /////////

  // DMA is disabled when the Spatz vector-to-AXI joiner is enabled
  // if (IsaCfg.Xdma) begin : gen_dma
  if (IsaCfg.Xdma && !EnableSpatzAxi) begin : gen_dma
    idma_inst64_top #(
      .AxiAddrWidth   (AddrWidth),
      .AxiDataWidth   (DMADataWidth),
      .AxiIdWidth     (DMAIdWidth),
      .AxiUserWidth   (DMAUserWidth),
      .NumAxInFlight  (DMANumAxInFlight),
      .DMAReqFifoDepth(DMAReqFifoDepth),
      .NumChannels    (DMANumChannels),
      .EnableTcdmObi  (1'b0),
      .EnableCompute  (DMAEnableCompute),
      .ComputeOps     (DMAComputeOps),
      .ComputeTuning  (DMAComputeTuning),
      .DMATracing     (1),
      .axi_ar_chan_t  (axi_ar_chan_t),
      .axi_aw_chan_t  (axi_aw_chan_t),
      .axi_req_t      (axi_req_t),
      .axi_res_t      (axi_rsp_t),
      .acc_req_t      (acc_req_chan_t),
      .acc_res_t      (acc_rsp_chan_t),
      .dma_events_t   (dma_events_t)
    ) i_idma_inst64_top (
      .clk_i,
      .rst_ni,
      .axi_req_o      (axi_dma_req_o),
      .axi_res_i      (axi_dma_res_i),
      .busy_o         (axi_dma_busy_o),
      .acc_req_i      (snitch_acc_req_demuxed[snitch_pkg::DMA_SS].q),
      .acc_req_valid_i(snitch_acc_req_demuxed[snitch_pkg::DMA_SS].q_valid),
      .acc_req_ready_o(snitch_acc_rsp_demuxed[snitch_pkg::DMA_SS].q_ready),
      .acc_res_o      (snitch_acc_rsp_demuxed[snitch_pkg::DMA_SS].p),
      .acc_res_valid_o(snitch_acc_rsp_demuxed[snitch_pkg::DMA_SS].p_valid),
      .acc_res_ready_i(snitch_acc_req_demuxed[snitch_pkg::DMA_SS].p_ready),
      .hart_id_i      (hart_id_i),
      .events_o       (axi_dma_events_o),
      .obi_req_o      (),
      .obi_res_i      ('0),
      .addr_map_i     ('0)
    );
  end else begin : gen_no_dma
    // `axi_dma_req_o` tie-off lives here originally (kept for reference); with
    // `EnableSpatzAxi` the joiner in `gen_spatz_steer_axi` owns channel 0, so
    // the array tie-off below skips it to keep a single driver.
    // assign axi_dma_req_o = '0;
    if (!EnableSpatzAxi) begin : gen_no_dma_tie_req
      assign axi_dma_req_o = '0;
    end
    assign axi_dma_busy_o = '0;
    assign snitch_acc_rsp_demuxed[snitch_pkg::DMA_SS] = '0;
    assign axi_dma_events_o = '0;
  end

  /////////
  // IPU //
  /////////

  if (PrivateIpu) begin : gen_ipu
    snitch_ipu #(
      .IdWidth  (5),
      .Xpulpv2  (Xpulpv2),
      .acc_rsp_t(acc_rsp_t),
      .acc_req_t(acc_req_t)
    ) i_snitch_ipu (
      .clk_i,
      .rst_ni,
      .acc_req_i(snitch_acc_req_demuxed[snitch_pkg::IPU]),
      .acc_rsp_o(snitch_acc_rsp_demuxed[snitch_pkg::IPU])
    );
    assign hive_req_o.acc_req = '0;
  end else begin : gen_no_ipu
    assign hive_req_o.acc_req = snitch_acc_req_demuxed[snitch_pkg::IPU];
    assign snitch_acc_rsp_demuxed[snitch_pkg::IPU] = hive_rsp_i.acc_rsp;
  end

  /////////
  // DCA //
  /////////

  // Cut DCA interface
  reqrsp_cut #(
    .req_chan_t(dca_req_chan_t),
    .rsp_chan_t(dca_rsp_chan_t),
    .BypassReq (!EnableDca || !RegisterDcaReq),
    .BypassRsp (!EnableDca || !RegisterDcaRsp)
  ) i_dca_cut (
    .clk_i    (clk_i),
    .rst_ni   (rst_ni),
    .slv_req_i(dca_req_i),
    .slv_rsp_o(dca_rsp_o),
    .mst_req_o(dca_req_q),
    .mst_rsp_i(dca_rsp_q)
  );

  // Demux the DCA interface to either the FPSS or Spatz. Only one is ever active (see
  // `NativeFpSupport`), so the unselected side is always tied off by that subsystem's own
  // "absent" branch.
  reqrsp_demux #(
    .NrPorts   (NumDcaDemuxPorts),
    .req_chan_t(dca_req_chan_t),
    .rsp_chan_t(dca_rsp_chan_t)
  ) i_dca_demux (
    .clk_i,
    .rst_ni,
    .slv_req_i(dca_req_q),
    .slv_rsp_o(dca_rsp_q),
    .mst_req_o(dca_demux_req),
    .mst_rsp_i(dca_demux_rsp),
    .select_i (IsaCfg.RVV)
  );

  //////////////////
  // FP subsystem //
  //////////////////

  if (NativeFpSupport) begin : gen_fpu
    snitch_fp_ss #(
      .AddrWidth            (AddrWidth),
      .DataWidth            (DataWidth),
      .NumFPOutstandingLoads(NumFPOutstandingLoads),
      .NumFPOutstandingMem  (NumFPOutstandingMem),
      .NumFPUSequencerInstr (NumSequencerInstr),
      .NumFPUSequencerLoops (NumSequencerLoops),
      .FpuImplementation    (FPUImplementation),
      .IsaCfg               (IsaCfg),
      .NumSsrs              (NumSsrs),
      .SsrRegs              (SsrRegs),
      .RegisterSequencer    (RegisterSequencer),
      .RegisterFpuReq       (RegisterFPUIn),
      .RegisterFpuRsp       (RegisterFPUOut),
      .EnableDca            (EnableDca)
    ) i_snitch_fp_ss (
      .clk_i,
      .rst_i                  (~rst_ni | (~rst_fp_ss_ni)),
      // pragma translate_off
      .trace_o                (fpss_trace),
      .sequencer_trace_o      (fpu_sequencer_trace),
      .dca_trace_o            (dca_trace),
      // pragma translate_on
      .hart_id_i              (hart_id_i),
      .acc_req_i              (snitch_acc_req_demuxed[snitch_pkg::FP_SS]),
      .acc_rsp_o              (snitch_acc_rsp_demuxed[snitch_pkg::FP_SS]),
      .i2f_rdata_i            (i2f_rdata),
      .i2f_rvalid_i           (i2f_rvalid),
      .i2f_rready_o           (i2f_rready),
      .f2i_wdata_o            (f2i_wdata),
      .f2i_wvalid_o           (f2i_wvalid),
      .f2i_wready_i           (f2i_wready),
      .caq_pvalid_o           (caq_pvalid),
      .data_req_o             (fpss_lsu_req),
      .data_rsp_i             (fpss_lsu_rsp),
      .fpu_rnd_mode_i         (fpss_fpu_rnd_mode),
      .fpu_fmt_mode_i         (fpss_fpu_fmt_mode),
      .fpu_status_o           (fpss_fpu_status),
      .ssr_raddr_o            (ssr_raddr),
      .ssr_rdata_i            (ssr_rdata),
      .ssr_rvalid_o           (ssr_rvalid),
      .ssr_rready_i           (ssr_rready),
      .ssr_rdone_o            (ssr_rdone),
      .ssr_waddr_o            (ssr_waddr),
      .ssr_wdata_o            (ssr_wdata),
      .ssr_wvalid_o           (ssr_wvalid),
      .ssr_wready_i           (ssr_wready),
      .ssr_wdone_o            (ssr_wdone),
      .streamctl_done_i       (ssr_streamctl_done),
      .streamctl_valid_i      (ssr_streamctl_valid),
      .streamctl_ready_o      (ssr_streamctl_ready),
      .core_events_o          (fpss_events),
      .en_copift_i            (en_copift),
      .dca_req_i              (dca_demux_req[DcaFpss]),
      .dca_rsp_o              (dca_demux_rsp[DcaFpss])
    );
  end else begin : gen_no_fpu
    assign fpss_trace = '0;
    assign fpu_sequencer_trace = '0;
    assign dca_trace = '0;
    assign snitch_acc_rsp_demuxed[snitch_pkg::FP_SS] = '0;
    assign i2f_rready = '0;
    assign f2i_wdata = '0;
    assign f2i_wvalid = '0;
    assign caq_pvalid = '0;
    assign fpss_lsu_req = '0;
    assign fpss_fpu_status = '0;
    assign ssr_raddr = '0;
    assign ssr_rvalid = '0;
    assign ssr_rdone = '0;
    assign ssr_waddr = '0;
    assign ssr_wdata = '0;
    assign ssr_wvalid = '0;
    assign ssr_wdone = '0;
    assign ssr_streamctl_ready = '0;
    assign fpss_events = '0;
    assign dca_demux_rsp[DcaFpss] = '0;
  end

  ///////////
  // Spatz //
  ///////////

  if (IsaCfg.RVV) begin : gen_spatz
    spatz #(
      .NrMemPorts         (NumSpatzMemPorts),
      .NumOutstandingLoads(NumSpatzOutstandingLoads),
      .FPUImplementation  (FPUImplementation),
      .AddrWidth          (AddrWidth),
      .EnableDca          (EnableDca),
      .RegisterRsp        (RegisterOffloadRsp),
      .dreq_t             (lsu_req_t),
      .drsp_t             (lsu_rsp_t),
      // Full-width channels: recover high address bits for TCDM/AXI decode
      // (was `tcdm_req_chan_t`, kept below for reference).
      // .spatz_mem_req_t    (tcdm_req_chan_t),
      // .spatz_mem_rsp_t    (tcdm_rsp_chan_t),
      .spatz_mem_req_t    (spatz_full_req_chan_t),
      .spatz_mem_rsp_t    (spatz_full_rsp_chan_t),
      // X-IF types (used; Spatz must be compiled with `define X_INTERFACE).
      .x_issue_req_t      (x_issue_req_t),
      .x_issue_resp_t     (x_issue_resp_t),
      .x_register_t       (x_register_t),
      .x_commit_t         (x_commit_t),
      .x_result_t         (x_result_t)
    ) i_spatz (
      .clk_i                   (clk_i),
      .rst_ni                  (rst_ni),
      .testmode_i              (1'b0),
      .hart_id_i               (hart_id_i),
      .x_issue_valid_i         (cop_issue_valid[SpatzCopro]),
      .x_issue_ready_o         (cop_issue_ready[SpatzCopro]),
      .x_issue_req_i           (cop_issue_req[SpatzCopro]),
      .x_issue_resp_o          (cop_issue_resp[SpatzCopro]),
      .x_register_valid_i      (cop_register_valid[SpatzCopro]),
      .x_register_ready_o      (cop_register_ready[SpatzCopro]),
      .x_register_i            (cop_register[SpatzCopro]),
      .x_commit_valid_i        (cop_commit_valid[SpatzCopro]),
      .x_commit_i              (cop_commit[SpatzCopro]),
      .x_result_valid_o        (cop_result_valid[SpatzCopro]),
      .x_result_ready_i        (cop_result_ready[SpatzCopro]),
      .x_result_o              (cop_result[SpatzCopro]),
      // Full-width channels (were `spatz_tcdm_*`, kept for reference).
      // .spatz_mem_req_o         (spatz_tcdm_req_chan),
      // .spatz_mem_req_valid_o   (spatz_tcdm_req_valid),
      // .spatz_mem_req_ready_i   (spatz_tcdm_req_ready),
      // .spatz_mem_rsp_i         (spatz_tcdm_rsp_chan),
      // .spatz_mem_rsp_valid_i   (spatz_tcdm_rsp_valid),
      .spatz_mem_req_o         (spatz_full_req_chan),
      .spatz_mem_req_valid_o   (spatz_full_req_valid),
      .spatz_mem_req_ready_i   (spatz_full_req_ready),
      .spatz_mem_rsp_i         (spatz_full_rsp_chan),
      .spatz_mem_rsp_valid_i   (spatz_full_rsp_valid),
      .spatz_mem_finished_o    (spatz_mem_finished),
      .spatz_mem_str_finished_o(spatz_mem_str_finished),
      .fp_lsu_mem_req_o        (spatz_flsu_req),
      .fp_lsu_mem_rsp_i        (spatz_flsu_rsp),
      .fpu_rnd_mode_i          (spatz_fpu_rnd_mode),
      .fpu_fmt_mode_i          (spatz_fpu_fmt_mode),
      .fpu_status_o            (spatz_fpu_status),
      .dca_req_i               (dca_demux_req[DcaSpatz]),
      .dca_rsp_o               (dca_demux_rsp[DcaSpatz])
    );

    // Convert Spatz requests: slice the full-width channels to TCDM width.
    // Original narrow conversion (kept for reference, superseded by slicing):
    // for (genvar p = 0; p < NumSpatzMemPorts; p++) begin: gen_spatz_tcdm_assignment
    //   assign spatz_tcdm_req[p] = '{
    //       q: spatz_tcdm_req_chan[p],
    //       q_valid: spatz_tcdm_req_valid[p]
    //     };
    //   assign spatz_tcdm_req_ready[p] = spatz_tcdm_rsp[p].q_ready;
    //   assign spatz_tcdm_rsp_chan[p] = spatz_tcdm_rsp[p].p;
    //   assign spatz_tcdm_rsp_valid[p] = spatz_tcdm_rsp[p].p_valid;
    // end
    // `q_valid`/`q_ready` gating and the response path are steered per group
    // in `gen_spatz_steer` below; TCDM-bound groups keep the pack path above.
    for (genvar p = 0; p < NumSpatzMemPorts; p++) begin : gen_spatz_slice
      // NOTE: spatz_full_req use PhysicalAddrWidth, spatz_tcdm use TcdmAddrWidth
      // similar to lsu_width_converter
      assign spatz_tcdm_req[p].q.addr  = spatz_full_req_chan[p].addr[TcdmAddrWidth-1:0];
      assign spatz_tcdm_req[p].q.write = spatz_full_req_chan[p].write;
      assign spatz_tcdm_req[p].q.amo   = spatz_full_req_chan[p].amo;
      assign spatz_tcdm_req[p].q.data  = spatz_full_req_chan[p].data;
      assign spatz_tcdm_req[p].q.strb  = spatz_full_req_chan[p].strb;
      assign spatz_tcdm_req[p].q.user  = spatz_full_req_chan[p].user;
    end

    // Spatz TCDM/AXI steering (v1 review).
    // Disabled: transparent slice-through, identical to the original code above.
    if (!EnableSpatzAxi) begin : gen_spatz_steer_bypass
      for (genvar p = 0; p < NumSpatzMemPorts; p++) begin : gen_port
        assign spatz_tcdm_req[p].q_valid = spatz_full_req_valid[p];
        assign spatz_full_req_ready[p]   = spatz_tcdm_rsp[p].q_ready;
        assign spatz_full_rsp_chan[p]    = spatz_tcdm_rsp[p].p.data;
        assign spatz_full_rsp_valid[p]   = spatz_tcdm_rsp[p].p_valid;
      end
      assign spatz_is_tcdm    = 1'b0;
      assign spatz_group_tcdm = 1'b0;
      assign spatz_group_axi  = 1'b0;
      assign spatz_path_q     = 1'b0;
      assign spatz_join_req   = '0;
      assign spatz_join_res   = '0;
      assign spatz_busy_raw_q = 1'b0;
    end else begin : gen_spatz_axi
      // TCDM address rule, same base/mask form as the LSU demux above.
      // Assumption: every VLSU port either accesses all AXI or all TCDM.
      // Destination is decoded from the first presenting (valid) lane.
      localparam logic [AddrWidth-1:0] TcdmBaseMask = (addr_t'({AddrWidth{1'b1}}) << TcdmAddrWidth);
      function automatic logic check_is_tcdm(logic [AddrWidth-1:0] addr);
        return ((addr & TcdmBaseMask) == (tcdm_addr_base_i & TcdmBaseMask))
            || (TCDMAliasEnable && ((addr & TcdmBaseMask) == (TCDMAliasStart & TcdmBaseMask)));
      endfunction

      always_comb begin
        spatz_is_tcdm = 1'b0;
        for (int p = 0; p < NumSpatzMemPorts; p++) begin
          if (spatz_full_req_valid[p]) begin
            spatz_is_tcdm = check_is_tcdm(spatz_full_req_chan[p].addr);
            break;
          end
        end
      end

      // Assert that all valid ports in a group agree on the destination (all AXI or all TCDM).
      for (genvar p = 0; p < NumSpatzMemPorts; p++) begin : gen_assert_same_target
        `ASSERT(SpatzPortMatchesGroupTarget_A,
          spatz_full_req_valid[p] |-> (check_is_tcdm(spatz_full_req_chan[p].addr) == spatz_is_tcdm)
        )
      end

      logic any_valid;
      logic any_idle;
      assign any_valid        = |spatz_full_req_valid;
      assign any_idle         = (spatz_pend_q == '0);
      // Currently any_idle (from spatz_pend) serialize all requests, forcing effective NumOutstandingLoads=1.
      // To overcome this later on (with pipelining), we will need to replace these with per-port counter

      logic [NumSpatzMemPorts-1:0] spatz_pend_d; // pending requests
      logic spatz_path_d;
      logic any_accept;
      assign any_accept = |(spatz_full_req_valid & spatz_full_req_ready);
      always_comb begin
        spatz_pend_d = spatz_pend_q;
        spatz_path_d = spatz_path_q;

        if (any_accept) spatz_path_d = !spatz_is_tcdm;
        for (int p = 0; p < NumSpatzMemPorts; p++) begin
          if (spatz_full_req_valid[p] && spatz_full_req_ready[p]) begin
            spatz_pend_d[p] = 1'b1;
          end
          if (spatz_full_rsp_valid[p]) spatz_pend_d[p] = 1'b0;
        end
      end
      `FF(spatz_pend_q, spatz_pend_d, '0, clk_i, rst_ni)
      `FF(spatz_path_q, spatz_path_d, '0, clk_i, rst_ni)
      assign spatz_group_tcdm = any_valid &&  spatz_is_tcdm && any_idle;
      assign spatz_group_axi  = any_valid && !spatz_is_tcdm && any_idle;

      // Joiner slave side presents the full channels of AXI-bound groups.
      // Joiner slave request: full-width channel plus valid (response reuses
      // `tcdm_rsp_t`, which carries data only and already matches).
      localparam type spatz_join_req_t = `TCDM_REQ_STRUCT(DataWidth, AddrWidth, TcdmUserWidth);
      localparam type spatz_join_rsp_t = `TCDM_RSP_STRUCT(DataWidth);
      spatz_join_req_t [NumSpatzMemPorts-1:0] spatz_join_slv_req;
      spatz_join_rsp_t [NumSpatzMemPorts-1:0] spatz_join_slv_rsp;
      for (genvar p = 0; p < NumSpatzMemPorts; p++) begin : gen_join_slv
        assign spatz_join_slv_req[p].q       = spatz_full_req_chan[p];
        assign spatz_join_slv_req[p].q_valid = spatz_full_req_valid[p] && spatz_group_axi;
      end

      // Wide coalesced beat between join and converter.
      spatz_wide_req_t spatz_wide_req;
      spatz_wide_rsp_t spatz_wide_rsp;

      tcdm_narrow_join #(
        .NrPorts         (NumSpatzMemPorts),
        .NarrowDataWidth (DataWidth),
        .WideDataWidth   (DMADataWidth),
        .TcdmAddrWidth   (AddrWidth),
        .narrow_req_t    (spatz_join_req_t),
        .narrow_rsp_t    (spatz_join_rsp_t),
        .wide_req_t      (spatz_wide_req_t),
        .wide_rsp_t      (spatz_wide_rsp_t)
      ) i_spatz_narrow_join (
        .clk_i,
        .rst_ni,
        .narrow_req_i (spatz_join_slv_req),
        .narrow_rsp_o (spatz_join_slv_rsp),
        .wide_req_o   (spatz_wide_req),
        .wide_rsp_i   (spatz_wide_rsp)
      );

      tcdm_to_axi #(
        .DataWidth    (DMADataWidth),
        .TcdmAddrWidth(AddrWidth),
        .AxiAddrWidth (AddrWidth),
        .tcdm_req_t   (spatz_wide_req_t),
        .tcdm_rsp_t   (spatz_wide_rsp_t),
        .axi_req_t    (axi_req_t),
        .axi_rsp_t    (axi_rsp_t)
      ) i_tcdm_to_axi (
        .clk_i,
        .rst_ni,
        .tcdm_req_i (spatz_wide_req),
        .tcdm_rsp_o (spatz_wide_rsp),
        .axi_req_o  (spatz_join_req),
        .axi_rsp_i  (spatz_join_res)
      );

      // reuse iDMA wires
      assign axi_dma_req_o[0] = spatz_join_req;
      assign spatz_join_res = axi_dma_res_i[0];

      // Steer requests to TCDM or AXI based on the address map
      // only valid when at least 1 req is valid
      for (genvar p = 0; p < NumSpatzMemPorts; p++) begin : gen_req_steer
        assign spatz_tcdm_req[p].q_valid = spatz_full_req_valid[p] && spatz_group_tcdm;
        assign spatz_full_req_ready[p] = spatz_group_tcdm ? spatz_tcdm_rsp[p].q_ready :
                                  (spatz_group_axi ? spatz_join_slv_rsp[p].q_ready : 1'b0);
      end

      // Response steering by recorded path; pend bits clear on `p_valid`.
      for (genvar p = 0; p < NumSpatzMemPorts; p++) begin : gen_rsp_steer
        assign spatz_full_rsp_chan[p] = spatz_path_q ? spatz_join_slv_rsp[p].p.data :
          spatz_tcdm_rsp[p].p.data;
        assign spatz_full_rsp_valid[p] = spatz_path_q ? spatz_join_slv_rsp[p].p_valid :
          spatz_tcdm_rsp[p].p_valid;
      end

      // Depth-1 (`spatz_pend_q`) allows at most one outstanding group, so both
      // legs must never answer the same lane together.
      for (genvar p = 0; p < NumSpatzMemPorts; p++) begin : gen_assert_rsp_mutex
        `ASSERT(SpatzRspMutex,
          !(spatz_tcdm_rsp[p].p_valid && spatz_join_slv_rsp[p].p_valid)
        )
      end

      // restrict FENCE to only work with VLSU ops because its sole purpose is
      // replacing snrt_dma_wait_all()
      logic vlsu_mem_accept;
      assign vlsu_mem_accept = cop_issue_valid[SpatzCopro] & cop_issue_ready[SpatzCopro]
          && (cop_issue_req[SpatzCopro].instr inside
        {spatz_riscv_instr::VLE8_V, spatz_riscv_instr::VLE16_V,
        spatz_riscv_instr::VLE32_V, spatz_riscv_instr::VLE64_V,
        spatz_riscv_instr::VLSE8_V, spatz_riscv_instr::VLSE16_V,
        spatz_riscv_instr::VLSE32_V, spatz_riscv_instr::VLSE64_V,
        spatz_riscv_instr::VLOXEI8_V, spatz_riscv_instr::VLOXEI16_V,
        spatz_riscv_instr::VLOXEI32_V, spatz_riscv_instr::VLOXEI64_V,
        spatz_riscv_instr::VLUXEI8_V, spatz_riscv_instr::VLUXEI16_V,
        spatz_riscv_instr::VLUXEI32_V, spatz_riscv_instr::VLUXEI64_V,
        spatz_riscv_instr::VSE8_V, spatz_riscv_instr::VSE16_V,
        spatz_riscv_instr::VSE32_V, spatz_riscv_instr::VSE64_V,
        spatz_riscv_instr::VSSE8_V, spatz_riscv_instr::VSSE16_V,
        spatz_riscv_instr::VSSE32_V, spatz_riscv_instr::VSSE64_V,
        spatz_riscv_instr::VSOXEI8_V, spatz_riscv_instr::VSOXEI16_V,
        spatz_riscv_instr::VSOXEI32_V, spatz_riscv_instr::VSOXEI64_V,
        spatz_riscv_instr::VSUXEI8_V, spatz_riscv_instr::VSUXEI16_V,
        spatz_riscv_instr::VSUXEI32_V, spatz_riscv_instr::VSUXEI64_V});

      always_comb begin
        spatz_vlsu_cnt_d = spatz_vlsu_cnt_q;
        if (vlsu_mem_accept && !spatz_mem_finished[1] && spatz_vlsu_cnt_q != '1)
          spatz_vlsu_cnt_d = spatz_vlsu_cnt_q + 1'b1;
        else if (!vlsu_mem_accept && spatz_mem_finished[1] && spatz_vlsu_cnt_q != '0)
          spatz_vlsu_cnt_d = spatz_vlsu_cnt_q - 1'b1;
      end
      `FF(spatz_vlsu_cnt_q, spatz_vlsu_cnt_d, '0, clk_i, rst_ni)

      // Busy = counted-but-uncommitted vector ops + live request/pending
      // traffic (store-response tail W2, past finish).
      assign spatz_busy_raw_d = (spatz_vlsu_cnt_q != '0)
          || (|spatz_full_req_valid) || (|spatz_pend_q);
      `FF(spatz_busy_raw_q, spatz_busy_raw_d, '0, clk_i, rst_ni)
    end : gen_spatz_axi
  end else begin : gen_no_spatz
    assign cop_issue_ready[SpatzCopro] = '0;
    assign cop_issue_resp[SpatzCopro] = '0;
    assign cop_register_ready[SpatzCopro] = '0;
    assign cop_result_valid[SpatzCopro] = '0;
    assign cop_result[SpatzCopro] = '0;
    assign spatz_tcdm_req = '0;
    assign spatz_full_req_chan = '0;
    assign spatz_full_req_valid = '0;
    assign spatz_full_req_ready = '0;
    assign spatz_full_rsp_chan = '0;
    assign spatz_full_rsp_valid = '0;
    assign spatz_flsu_req = '0;
    assign spatz_fpu_status = '0;
    assign dca_demux_rsp[DcaSpatz] = '0;
    assign spatz_mem_finished = '0;
    assign spatz_mem_str_finished = '0;
    assign spatz_busy_raw_q = 1'b0;
    assign spatz_vlsu_cnt_q = '0;
  end

  if (IsoCrossing) begin : gen_spatz_busy_sync
    tc_sync #(
      .Stages     (2),
      .ResetValue (1'b0)
    ) i_spatz_busy_sync (
      .clk_i   (clk_d2_i),
      .rst_ni  (rst_ni),
      .serial_i(spatz_busy_raw_q),
      .serial_o(spatz_busy_synced)
    );

    // signal sampled at clk_d2_i to make sure busy signal is not missed for
    // too long that snitch already executed FENCE
    always_ff @(posedge clk_d2_i or negedge rst_ni) begin
      if (!rst_ni) begin
        spatz_issue_d2_q <= 1'b0;
      end else if (x_issue_valid) begin
        spatz_issue_d2_q <= 1'b1;
      end else if (!spatz_busy_synced) begin
        spatz_issue_d2_q <= 1'b0;
      end
    end
  end else begin : gen_no_iso_crossing
    assign spatz_busy_synced = spatz_busy_raw_q;
    assign spatz_issue_d2_q = 1'b0;
  end

  assign spatz_busy_d2 = spatz_issue_d2_q || spatz_busy_synced;

  /////////////////////////////////////////////
  // Mux Snitch, FPSS and Spatz LSU requests //
  /////////////////////////////////////////////

  reqrsp_mux #(
    .NrPorts    (3),
    .req_chan_t (lsu_req_chan_t),
    .rsp_chan_t (lsu_rsp_chan_t),
    // TODO(zarubaf): Wire-up to top-level.
    .RspDepth   (8),
    .RegisterReq({1'b0, RegisterFPUReq, RegisterFPUReq})
  ) i_reqrsp_mux (
    .clk_i,
    .rst_ni,
    .slv_req_i  ({snitch_lsu_req_q, fpss_lsu_req, spatz_flsu_req}),
    .slv_rsp_o  ({snitch_lsu_rsp_q, fpss_lsu_rsp, spatz_flsu_rsp}),
    .mst_req_o  (muxed_lsu_req),
    .mst_rsp_i  (muxed_lsu_rsp),
    .rsp_route_i('0),
    .idx_o      ()
  );

  //////////////////////////////
  // Demux LSU -> SoC or TCDM //
  //////////////////////////////

  typedef struct packed {
    int unsigned idx;
    logic [AddrWidth-1:0] base;
    logic [AddrWidth-1:0] mask;
  } reqrsp_rule_t;

  // Define the addrmap for the demux select logic.
  reqrsp_rule_t [TCDMAliasEnable:0] addr_map;
  assign addr_map[0] = '{
    idx: DreqSelectTcdm,
    base: tcdm_addr_base_i,
    mask: ({AddrWidth{1'b1}} << TcdmAddrWidth)
  };
  if (TCDMAliasEnable) begin : gen_tcdm_alias_rule
    assign addr_map[1] = '{
      idx: DreqSelectTcdm,
      base: TCDMAliasStart,
      mask: ({AddrWidth{1'b1}} << TcdmAddrWidth)
    };
  end

  // Collective communication operations are performed within the interconnect at the SoC
  // level. However, requests destined to the TCDM never arrive at the SoC interconnect,
  // as they are routed internally within the cluster. In order for collectives destined to
  // the TCDM to work, we need to handle them differently, and always forward them to the
  // SoC interconnect, which will reroute them back to the TCDM from outside the cluster.
  // The collective mask, in the user field, is used to detect collective operations.
  addr_t collective_mask;
  logic  is_collective;
  assign collective_mask = addr_t'(muxed_lsu_req.q.user[CollectiveWidth+:AddrWidth]);
  assign is_collective = (collective_mask != 0);

  reqrsp_demux_mapped #(
    .NrPorts   (2),
    .req_chan_t(lsu_req_chan_t),
    .rsp_chan_t(lsu_rsp_chan_t),
    // TODO(zarubaf): Make a parameter.
    .RspDepth (4),
    .NoRules  (1 + TCDMAliasEnable),
    .addr_t   (logic [AddrWidth-1:0]),
    .rule_t   (reqrsp_rule_t)
  ) i_reqrsp_demux_mapped (
    .clk_i,
    .rst_ni,
    .addr_map_i           (addr_map),
    .default_select_i     (DreqSelectSoc),
    .ext_select_i         (DreqSelectSoc),
    .ext_select_override_i(is_collective),
    .slv_req_i            (muxed_lsu_req),
    .slv_rsp_o            (muxed_lsu_rsp),
    .mst_req_o            ({lsu_tcdm_req, soc_req_o}),
    .mst_rsp_i            ({lsu_tcdm_rsp, soc_rsp_i})
  );

  // Resize LSU request to match TCDM bus widths
  lsu_width_converter #(
    .InAddrWidth (AddrWidth),
    .InDataWidth (DataWidth),
    .InUserWidth (snitch_pkg::UserWidth),
    .OutAddrWidth(TcdmAddrWidth),
    .OutDataWidth(DataWidth),
    .OutUserWidth(TcdmUserWidth)
  ) i_lsu_width_converter (
    .lsu_req_i(lsu_tcdm_req),
    .lsu_rsp_o(lsu_tcdm_rsp),
    .lsu_req_o(core_lsu_req_resized),
    .lsu_rsp_i(core_lsu_rsp_resized)
  );

  // Convert LSU request to TCDM protocol
  lsu_to_tcdm #(
    // TODO(zarubaf): Make a parameter.
    .BufDepth (4),
    .AddrWidth(TcdmAddrWidth),
    .UserWidth(TcdmUserWidth),
    .DataWidth(DataWidth)
  ) i_lsu_to_tcdm (
    .clk_i,
    .rst_ni,
    .lsu_req_i (core_lsu_req_resized),
    .lsu_rsp_o (core_lsu_rsp_resized),
    .tcdm_req_o(core_tcdm_req),
    .tcdm_rsp_i(core_tcdm_rsp)
  );

  //////////
  // SSRs //
  //////////

  snitch_ssr_subsystem #(
    .IsaCfg        (IsaCfg),
    .NumSsrs       (NumSsrs),
    .SsrCfgs       (SsrCfgs),
    .SsrRegs       (SsrRegs),
    .SsrMuxRspDepth(SsrMuxRspDepth),
    .TcdmAddrWidth (TcdmAddrWidth),
    .DataWidth     (DataWidth),
    .TcdmUserWidth (TcdmUserWidth),
    .acc_req_t     (acc_req_t),
    .acc_rsp_t     (acc_rsp_t),
    .tcdm_req_t    (tcdm_req_t),
    .tcdm_rsp_t    (tcdm_rsp_t)
  ) i_snitch_ssr_subsystem (
    .clk_i,
    .rst_ni,
    .acc_req_i            (snitch_acc_req_demuxed[snitch_pkg::SSR_CFG]),
    .acc_rsp_o            (snitch_acc_rsp_demuxed[snitch_pkg::SSR_CFG]),
    .ssr_raddr_i          (ssr_raddr),
    .ssr_rdata_o          (ssr_rdata),
    .ssr_rvalid_i         (ssr_rvalid),
    .ssr_rready_o         (ssr_rready),
    .ssr_rdone_i          (ssr_rdone),
    .ssr_waddr_i          (ssr_waddr),
    .ssr_wdata_i          (ssr_wdata),
    .ssr_wvalid_i         (ssr_wvalid),
    .ssr_wready_o         (ssr_wready),
    .ssr_wdone_i          (ssr_wdone),
    .ssr_streamctl_done_o (ssr_streamctl_done),
    .ssr_streamctl_valid_o(ssr_streamctl_valid),
    .ssr_streamctl_ready_i(ssr_streamctl_ready),
    .tcdm_req_o           (ssr_tcdm_req),
    .tcdm_rsp_i           (ssr_tcdm_rsp)
  );

  // Separate SSR0 TCDM request, to mux with core TCDM request
  assign ssr_tcdm_req_0 = ssr_tcdm_req[0];
  assign ssr_tcdm_rsp[0] = ssr_tcdm_rsp_0;
  if (NumSsrs > 1) begin : gen_multi_ssr
    assign ssr_tcdm_req_extra = ssr_tcdm_req[NumSsrs-1:1];
    assign ssr_tcdm_rsp[NumSsrs-1:1] = ssr_tcdm_rsp_extra;
  end

  // Mux TCDM requests from core and SSR0 onto TCDM port 0
  tcdm_mux #(
    .NrPorts  (2),
    .RspDepth (SsrMuxRspDepth),
    .AddrWidth(TcdmAddrWidth),
    .DataWidth(DataWidth),
    .UserWidth(TcdmUserWidth)
  ) i_tcdm_mux (
    .clk_i,
    .rst_ni,
    .slv_req_i({core_tcdm_req, ssr_tcdm_req_0}),
    .slv_rsp_o({core_tcdm_rsp, ssr_tcdm_rsp_0}),
    .mst_req_o(muxed_tcdm_req),
    .mst_rsp_i(muxed_tcdm_rsp)
  );

  // Pack TCDM requests from SSRs[>0] and Spatz onto higher TCDM ports
  always_comb begin
    automatic int unsigned i;
    i = 0;
    tcdm_req_o[i] = muxed_tcdm_req;
    muxed_tcdm_rsp = tcdm_rsp_i[i];
    i++;
    if (NumSsrs > 1) begin
      for (int j = 0; j < NumSsrs - 1; j++) begin
        tcdm_req_o[i] = ssr_tcdm_req_extra[j];
        ssr_tcdm_rsp_extra[j] = tcdm_rsp_i[i];
        i++;
      end
    end
    if (IsaCfg.RVV) begin
      for (int j = 0; j < NumSpatzMemPorts; j++) begin
        tcdm_req_o[i] = spatz_tcdm_req[j];
        spatz_tcdm_rsp[j] = tcdm_rsp_i[i];
        i++;
      end
    end
  end

  /////////////////
  // Core events //
  /////////////////

  always_comb begin
    core_events_o = snitch_events;
    core_events_o.issue_fpu = fpss_events.issue_fpu;
    core_events_o.issue_fpu_seq = fpss_events.issue_fpu_seq;
    core_events_o.issue_core_to_fpu = fpss_events.issue_core_to_fpu;
  end

  ////////////
  // Tracer //
  ////////////

`ifndef TRACE_OFF
  // pragma translate_off
  snitch_tracer #(
    .FpEn     (NativeFpSupport),
    .Xfrep    (IsaCfg.Xfrep),
    .EnableDca(EnableDca)
  ) i_snitch_tracer (
    .clk_i,
    .rst_ni,
    .hart_id_i,
    .trace_port_i         (snitch_trace),
    .fpss_trace_i         (fpss_trace),
    .fpu_sequencer_trace_i(fpu_sequencer_trace),
    .dca_trace_i          (dca_trace)
  );
  // pragma translate_on
`endif

  ////////////////
  // Assertions //
  ////////////////

  // Boot addr must be aligned to 4 bytes (32-bit instruction)
  `ASSERT_INIT(BootAddrAligned, BootAddr[1:0] == 2'b00)

  // DCA extension currently only supports 64-bit datawidth
  `ASSERT_INIT(DcaCoreConfiguration, (!EnableDca) || IsaCfg.RVD)

  // Spatz and SSRs/FREP are not compatible
  `ASSERT_INIT(IllegalSpatzSsrCombo, (!IsaCfg.RVV) || (!IsaCfg.Xssr))
  `ASSERT_INIT(IllegalSpatzFrepCombo, (!IsaCfg.RVV) || (!IsaCfg.Xfrep))

  // Spatz replaces the iDMA (v1 review)
  `ASSERT_INIT(IllegalSpatzAxiDmaCombo, (!EnableSpatzAxi) || (!IsaCfg.Xdma))
  `ASSERT_INIT(SpatzAxiNeeds1DmaChannel, (!EnableSpatzAxi) || (DMANumChannels == 1))
  `ASSERT_INIT(SpatzAxiNeedsSpatzPorts, (!EnableSpatzAxi) || (NumSpatzMemPorts > 0))
  // Assertion for all Spatz request to all goes to 1 endpoint is at check_is_tcdm function

endmodule
