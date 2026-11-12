// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Generic single-port wide TCDM to AXI bridge.
//
// Need from input len_beats_i (remaining beats of the run)
//
// Assumptions (checked by assertions unless noted):
// - requests & responses arrive in order (single AXI ID)
//
// Reads:
// one AXI INCR burst per contiguous run, up to `MaxOutstanding` bursts in flight
//
// Writes:
// one AW burst per run, up to `MaxWriteOutstanding` bursts in flight
//
// Reads and writes cannot overlap yet

`include "common_cells/assertions.svh"

module tcdm_to_axi
  import lsu_pkg::*;
#(
    parameter int unsigned DataWidth = 512,
    parameter int unsigned TcdmAddrWidth = 48,
    parameter int unsigned AxiAddrWidth = 48,
    parameter int unsigned MaxReadOutstanding = 4,
    parameter int unsigned MaxWriteOutstanding = 2,
    parameter int unsigned MaxBurstBeats = 8,
    // Derived parameters. DO NOT CHANGE!
    localparam int unsigned LenBeatsWidth = cc_pkg::cnt_width(MaxBurstBeats),
    localparam int unsigned LenAxiWidth  = cc_pkg::idx_width(MaxBurstBeats),
    parameter type tcdm_req_t = logic,
    parameter type tcdm_rsp_t = logic,
    parameter type axi_req_t = logic,
    parameter type axi_rsp_t = logic,
    parameter type axi_ar_chan_t = logic
) (
    input  logic      clk_i,
    input  logic      rst_ni,
    // TCDM slave: one coalesced wide beat per request handshake.
    input  tcdm_req_t tcdm_req_i,
    input  logic [LenBeatsWidth-1:0] len_beats_i,
    input  logic tcdm_read_rsp_ready_i,
    output logic write_busy_o,
    output tcdm_rsp_t tcdm_rsp_o,
    // AXI master: read and write bursts.
    output axi_req_t  axi_req_o,
    input  axi_rsp_t  axi_rsp_i
);

  localparam int unsigned BeatBytes = DataWidth / 8;
  localparam int unsigned OffsetWidth = $clog2(BeatBytes);
  localparam int unsigned WriteAwFifoDepth = MaxWriteOutstanding;

  // burst splitting signals
  logic [LenAxiWidth-1:0] len_axi; // = num_beats - 1

  logic read_ar_outstanding, read_burst_active;
  logic write_busy;

  // Read signals
  axi_ar_chan_t read_ar, read_ar_d, read_ar_q;
  logic read_ar_valid, read_ar_ready, read_r_ready;
  logic [LenAxiWidth-1:0] read_beats_left;
  logic read_ar_full;
  // assertion only
  logic [AxiAddrWidth-1:0]  read_feed_next_addr_d, read_feed_next_addr_q;

  logic [DataWidth-1:0] read_data;
  logic read_tcdm_req, read_ar_accept;
  logic read_tcdm_accept, read_r_tcdm_accept, read_tcdm_beat_accept;
  logic read_data_valid;

  // Write signals
  typedef struct packed {
    logic [AxiAddrWidth-1:0] addr;
    logic [LenAxiWidth-1:0] len;
  } write_aw_t;
  write_aw_t write_aw_fifo_data;
  logic write_aw_fifo_valid, write_aw_fifo_ready;
  logic write_aw_capture;

  logic write_w_valid, write_w_last, write_w_beat_accept;
  logic [LenAxiWidth-1:0] write_beats_left;

  logic write_not_empty;
  logic write_full;
  logic write_aw_valid;
  logic write_tcdm_req, write_burst_active, write_first_w, write_last_beat;

  ////////////////////////////////////////////////////////////////////////////
  // Burst splitting
  ////////////////////////////////////////////////////////////////////////////

  tcdm_burst_splitter #(
      .AddrWidth      (AxiAddrWidth),
      .BeatOffsetWidth(OffsetWidth),
      .MaxBurstBeats   (MaxBurstBeats)
  ) i_burst_splitter (
      .addr_i     (tcdm_req_i.q.addr),
      .len_beats_i(len_beats_i),
      .len_axi_o(len_axi)
  );

  ////////////////////////////////////////////////////////////////////////////
  // Read path
  ////////////////////////////////////////////////////////////////////////////

  assign read_data_valid     = axi_rsp_i.r_valid;
  assign read_r_ready        = tcdm_read_rsp_ready_i;
  assign read_r_tcdm_accept  = read_data_valid && read_r_ready;
  assign read_data           = axi_rsp_i.r.data;
  assign read_ar_ready       = axi_rsp_i.ar_ready;

  assign read_burst_active = |read_beats_left;
  assign read_tcdm_req  = tcdm_req_i.q_valid && !tcdm_req_i.q.write;
  assign read_ar_accept = read_ar_valid && read_ar_ready;
  assign read_tcdm_beat_accept = (read_tcdm_req && read_burst_active);
  assign read_tcdm_accept = read_ar_accept || read_tcdm_beat_accept;
  assign read_ar_valid   = !write_busy && read_tcdm_req &&
                           !read_burst_active && !read_ar_full;

  always_comb begin : proc_read_ar
    read_ar_d       = '0;
    read_ar_d.addr  = tcdm_req_i.q.addr;
    read_ar_d.len   = $bits(read_ar_d.len)'(len_axi);
    read_ar_d.size  = $bits(read_ar_d.size)'(OffsetWidth);
    read_ar_d.burst = axi_pkg::BURST_INCR;
    read_ar         = read_ar_valid ? read_ar_d : read_ar_q;
  end

  cc_credit_counter #(
      .NumCredits     (MaxReadOutstanding),
      .InitCreditEmpty(1'b1)
  ) i_read_credit_counter (
      .clk_i,
      .rst_ni,
      .clr_i        (1'b0),
      .credit_o     (/* unused */),
      .credit_give_i(read_ar_accept),
      .credit_take_i(read_r_tcdm_accept && axi_rsp_i.r.last),
      .credit_left_o(read_ar_outstanding),
      .credit_crit_o(/* unused */),
      .credit_full_o(read_ar_full)
  );

  cc_delta_counter #(
      .Width(LenAxiWidth)
  ) i_read_beats_left (
      .clk_i,
      .rst_ni,
      .clr_i     (1'b0),
      .en_i      (read_tcdm_beat_accept),
      .load_i    (read_ar_accept),
      .down_i    (1'b1),
      .delta_i   (LenAxiWidth'(1)),
      .d_i       (len_axi), // first beat is already accepted with AR
      .q_o       (read_beats_left),
      .overflow_o(/* unused */)
  );

  // assertion only
  always_comb begin : proc_read_feed
    read_feed_next_addr_d = read_feed_next_addr_q;
    if (read_ar_accept) begin
      read_feed_next_addr_d = tcdm_req_i.q.addr + BeatBytes;
    end else if (read_tcdm_beat_accept) begin
      read_feed_next_addr_d = read_feed_next_addr_q + BeatBytes;
    end
  end : proc_read_feed

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      read_feed_next_addr_q <= '0;
      read_ar_q             <= '0;
    end else begin
      read_feed_next_addr_q <= read_feed_next_addr_d;
      if (read_ar_accept) read_ar_q <= read_ar_d;
    end
  end

  ////////////////////////////////////////////////////////////////////////////
  // Write path
  ////////////////////////////////////////////////////////////////////////////

  assign write_tcdm_req     = tcdm_req_i.q_valid && tcdm_req_i.q.write;
  assign write_burst_active = |write_beats_left;

  assign write_w_valid = write_tcdm_req &&
                            (write_burst_active ||
                             (write_first_w && !read_ar_outstanding &&
                              !write_full && write_aw_fifo_ready));
  assign write_first_w    = write_tcdm_req && !write_burst_active;
  assign write_aw_capture = write_w_beat_accept && write_first_w;
  assign write_aw_valid   = write_aw_fifo_valid;
  assign write_last_beat  = write_burst_active ?
                            (write_beats_left == LenAxiWidth'(1)) :
                            (len_axi == '0);
  assign write_w_last        = write_w_valid && write_last_beat;
  assign write_w_beat_accept = write_w_valid && axi_rsp_i.w_ready;
  assign write_busy          = write_w_valid || write_burst_active ||
                               write_not_empty;
  assign write_busy_o = write_busy;

  // FallThrough to prevent 1 cycle delay
  cc_stream_fifo #(
      .FallThrough(1'b1),
      .Depth      (WriteAwFifoDepth),
      .data_t     (write_aw_t)
  ) i_write_aw_fifo (
      .clk_i,
      .rst_ni,
      .clr_i  (1'b0),
      .flush_i(1'b0),
      .usage_o(/* unused */),
      .data_i ({tcdm_req_i.q.addr, len_axi}),
      .valid_i(write_aw_capture),
      .ready_o(write_aw_fifo_ready),
      .data_o (write_aw_fifo_data),
      .valid_o(write_aw_fifo_valid),
      .ready_i(axi_rsp_i.aw_ready)
  );

  cc_credit_counter #(
      .NumCredits     (MaxWriteOutstanding),
      .InitCreditEmpty(1'b1)
  ) i_write_credit_counter (
      .clk_i,
      .rst_ni,
      .clr_i        (1'b0),
      .credit_o     (/* unused */),
      .credit_give_i(write_aw_capture),
      .credit_take_i(axi_rsp_i.b_valid && axi_req_o.b_ready),
      .credit_left_o(write_not_empty),
      .credit_crit_o(/* unused */),
      .credit_full_o(write_full)
  );

  cc_delta_counter #(
      .Width(LenAxiWidth)
  ) i_write_beats_left (
      .clk_i,
      .rst_ni,
      .clr_i     (1'b0),
      .en_i      (write_w_beat_accept),
      .load_i    (write_aw_capture),
      .down_i    (1'b1),
      .delta_i   (LenAxiWidth'(1)),
      .d_i       (len_axi),
      .q_o       (write_beats_left),
      .overflow_o(/* unused */)
  );

  ////////////////////////////////////////////////////////////////////////////
  // Shared logic
  ////////////////////////////////////////////////////////////////////////////

  // Share channel between read and write
  assign tcdm_rsp_o.q_ready = tcdm_req_i.q.write ? write_w_beat_accept : read_tcdm_accept;
  assign tcdm_rsp_o.p_valid = read_ar_outstanding ? read_data_valid : write_w_beat_accept;
  assign tcdm_rsp_o.p.data  = read_ar_outstanding ? read_data : '0;

  always_comb begin : proc_axi_request_merge
    axi_req_o = '0;
    axi_req_o.ar = read_ar;
    axi_req_o.ar_valid = read_ar_valid;
    axi_req_o.r_ready = read_r_ready;
    axi_req_o.b_ready = 1'b1;
    if (write_busy) begin
      axi_req_o.aw = '0;
      axi_req_o.aw.addr = write_aw_fifo_data.addr;
      axi_req_o.aw.len = $bits(axi_req_o.aw.len)'(write_aw_fifo_data.len);
      axi_req_o.aw.size = $bits(axi_req_o.aw.size)'(OffsetWidth);
      axi_req_o.aw.burst = axi_pkg::BURST_INCR;
      axi_req_o.aw_valid = write_aw_valid;
      axi_req_o.w = '0;
      axi_req_o.w.data = tcdm_req_i.q.data;
      axi_req_o.w.strb = tcdm_req_i.q.strb;
      axi_req_o.w.last = write_w_last;
      axi_req_o.w_valid = write_w_valid;
    end
  end

  ////////////////////////////////////////////////////////////////////////////
  // Assertions
  ////////////////////////////////////////////////////////////////////////////

  `ASSERT_INIT(AxiAddrCoversTcdm, (AxiAddrWidth >= TcdmAddrWidth))
  `ASSERT_INIT(AxiPageAddressWidth, (AxiAddrWidth >= 12)) // at least 4KB
  `ASSERT_INIT(NonzeroOutstandingCapacity, (MaxReadOutstanding > 0))
  `ASSERT_INIT(NonzeroWriteCapacity, (MaxWriteOutstanding > 0))
  `ASSERT_INIT(NonzeroMaxBurstBeats, (MaxBurstBeats > 0))
  `ASSERT_INIT(MaxBurstFitsAxiLen, (MaxBurstBeats <= 256))
  `ASSERT(SpatzReadBurstAddress_A,
          read_burst_active && read_tcdm_accept |->
              (tcdm_req_i.q.addr == read_feed_next_addr_q))
  `ASSERT(SpatzReadResponseOkay_A,
          read_r_tcdm_accept |-> (axi_rsp_i.r.resp == axi_pkg::RESP_OKAY))
  `ASSERT(SpatzReadBeatNoWriteBurst_A,
          read_burst_active |-> !write_busy)
  `ASSERT(SpatzWriteBeatNoRead_A,
          write_busy |-> !read_ar_outstanding)
  // Catch AMO and AXI errors
  `ASSERT(NoAmo_A, tcdm_req_i.q_valid |-> tcdm_req_i.q.amo == AMONone)
  `ASSERT(SpatzWriteResponseOkay_A,
          axi_rsp_i.b_valid |-> (axi_rsp_i.b.resp == axi_pkg::RESP_OKAY))
  `ASSERT(SpatzWriteBWithPending_A, axi_rsp_i.b_valid |-> write_not_empty)

endmodule
