// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Single-port wide TCDM to AXI bridge.
// Assumptions (checked by assertions unless noted):
// - the request address is aligned to `DataWidth`
// - requests & responses arrive in order (single AXI ID)
//
// Reads: 
// one AXI INCR burst per vle
// Up to `MaxOutstanding` bursts in flight
//
// Writes: 
// one AW burst per vse
// 1 in flight burst
//
// Reads and writes cannot overlap yet because the source carries 
// no response ID or response-ready.

`include "common_cells/assertions.svh"

module tcdm_to_axi
  import lsu_pkg::*;
  import cc_pkg::cnt_width;
#(
    parameter int unsigned DataWidth = 512,
    parameter int unsigned TcdmAddrWidth = 48,
    parameter int unsigned AxiAddrWidth = 48,
    parameter int unsigned MaxOutstanding = 4,
    parameter int unsigned MaxBurstLen = 8,
    // Derived parameter
    parameter int unsigned BurstLenWidth = cc_pkg::cnt_width(MaxBurstLen),
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
    input  logic [BurstLenWidth-1:0] beats_left_i,
    input  logic burst_mode_req_i,
    input  logic read_rsp_ready_i,
    output logic write_busy_o,
    output tcdm_rsp_t tcdm_rsp_o,
    // AXI master: read and write bursts.
    output axi_req_t  axi_req_o,
    input  axi_rsp_t  axi_rsp_i
);

  localparam int unsigned BeatBytes = DataWidth / 8;
  localparam int unsigned OffsetWidth = $clog2(BeatBytes);

  logic [BurstLenWidth-1:0] burst_beats;

  axi_ar_chan_t read_ar, read_ar_d, read_ar_q;
  logic read_ar_valid, read_ar_ready, read_r_ready;

  typedef struct packed {
    logic [BurstLenWidth-1:0] beats_issued;  // beats accepted from the source
    logic [AxiAddrWidth-1:0]  next_addr;     // expected next beat (assertion only)
  } read_feed_t;
  read_feed_t read_feed_d, read_feed_q;
  logic read_feed_busy_d, read_feed_busy_q;

  logic [cc_pkg::cnt_width(MaxOutstanding)-1:0] read_ar_outstanding_d,
                                                read_ar_outstanding_q;
  logic read_outstanding;

  logic [DataWidth-1:0] read_data;
  logic read_beat_req, read_burst_start, read_beat_fire;
  logic read_req_fire, read_rsp_fire;
  logic read_data_valid;

  logic wr_start, wr_w_valid, wr_w_last, wr_accept, wr_b_done;
  typedef struct packed {
    logic [BurstLenWidth-1:0]           len;        // AXI len field (beats - 1)
    logic [BurstLenWidth-1:0]           beats_fed;  // W beats accepted so far
    logic [AxiAddrWidth-1:0]            addr;       // AW base address
  } wr_burst_t;
  wr_burst_t wr_burst_d, wr_burst_q;
  logic wr_burst_active_d, wr_burst_active_q;
  logic wr_aw_done_d, wr_aw_done_q;
  logic write_busy;
  assign write_busy = wr_burst_active_q || wr_start;
  assign write_busy_o = write_busy;
  assign wr_start = tcdm_req_i.q_valid && tcdm_req_i.q.write &&
                    !read_outstanding && !wr_burst_active_q;
  assign wr_w_valid = write_busy &&
                      tcdm_req_i.q_valid && tcdm_req_i.q.write &&
                      (wr_start || (wr_burst_q.beats_fed <= wr_burst_q.len));
  logic [BurstLenWidth-1:0] wr_beats_fed, wr_len;
  assign wr_beats_fed = wr_start ? '0 : wr_burst_q.beats_fed;
  assign wr_len       = wr_start ? (burst_beats - 1'b1) : wr_burst_q.len;
  assign wr_w_last    = wr_w_valid && (wr_beats_fed == wr_len);
  assign wr_accept = wr_w_valid && axi_rsp_i.w_ready &&
                     (wr_aw_done_q || (axi_req_o.aw_valid && axi_rsp_i.aw_ready));
  assign wr_b_done = axi_rsp_i.b_valid && wr_burst_active_q;

  // burst splitter
  always_comb begin : proc_burst_beats
    int unsigned page_beats;
    int unsigned wanted_beats;

    page_beats   = (4096 - int'(tcdm_req_i.q.addr[11:0])) >> OffsetWidth;
    wanted_beats = 1;
    if (burst_mode_req_i && (beats_left_i > 1) &&
        (tcdm_req_i.q.addr[OffsetWidth-1:0] == '0)) begin
      // tcdm_req_i.q.addr alignment to DataWidth is checked with assertion
      wanted_beats = int'(beats_left_i);
      if (wanted_beats > MaxBurstLen) wanted_beats = MaxBurstLen;
      if (wanted_beats > page_beats)  wanted_beats = page_beats;
    end
    burst_beats = BurstLenWidth'(wanted_beats);
  end

  always_comb begin : proc_read_ar
    read_ar_valid   = !write_busy && !read_feed_busy_q &&
                      (read_ar_outstanding_q != MaxOutstanding) &&
                      tcdm_req_i.q_valid && !tcdm_req_i.q.write;
    read_ar_d       = '0;
    read_ar_d.addr  = tcdm_req_i.q.addr;
    read_ar_d.len   = $bits(read_ar_d.len)'(burst_beats - 1'b1);
    read_ar_d.size  = $bits(read_ar_d.size)'(OffsetWidth);
    read_ar_d.burst = axi_pkg::BURST_INCR;
    read_ar         = read_ar_valid ? read_ar_d : read_ar_q;
  end

  assign read_data_valid = axi_rsp_i.r_valid;
  assign read_r_ready    = read_rsp_ready_i;
  assign read_data       = axi_rsp_i.r.data;
  assign read_rsp_fire   = read_data_valid && read_r_ready;
  assign read_ar_ready   = axi_rsp_i.ar_ready;

  assign read_beat_req    = tcdm_req_i.q_valid && !tcdm_req_i.q.write;
  assign read_burst_start = read_ar_valid && read_ar_ready;
  assign read_beat_fire   = read_feed_busy_q &&
                            (read_feed_q.beats_issued <= read_ar_q.len);
  assign read_req_fire    = read_burst_start ||
                            (read_beat_req && read_beat_fire);
  assign read_outstanding = (read_ar_outstanding_q != '0);

  always_comb begin : proc_read_feed
    read_feed_d           = read_feed_q;
    read_feed_busy_d      = read_feed_busy_q;
    read_ar_outstanding_d = read_ar_outstanding_q;
    if (read_burst_start) begin
      read_feed_d.beats_issued = BurstLenWidth'(1);
      read_feed_d.next_addr    = tcdm_req_i.q.addr + AxiAddrWidth'(BeatBytes);
      read_feed_busy_d         = (burst_beats > BurstLenWidth'(1));
      read_ar_outstanding_d    = read_ar_outstanding_q + 1'b1;
    end else if (read_req_fire) begin
      read_feed_d.beats_issued = read_feed_q.beats_issued + 1'b1;
      read_feed_d.next_addr    = read_feed_q.next_addr + AxiAddrWidth'(BeatBytes);
      if (read_feed_q.beats_issued == read_ar_q.len)
        read_feed_busy_d = 1'b0;
    end
    if (read_rsp_fire && axi_rsp_i.r.last)
      read_ar_outstanding_d = read_ar_outstanding_d - 1'b1;
  end : proc_read_feed

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      read_feed_q           <= '0;
      read_feed_busy_q      <= 1'b0;
      read_ar_outstanding_q <= '0;
      read_ar_q             <= '0;
    end else begin
      read_feed_q           <= read_feed_d;
      read_feed_busy_q      <= read_feed_busy_d;
      read_ar_outstanding_q <= read_ar_outstanding_d;
      if (read_burst_start) read_ar_q <= read_ar_d;
    end
  end

  // No write while a read is outstanding.
  assign tcdm_rsp_o.q_ready = tcdm_req_i.q.write ? wr_accept : read_req_fire;
  assign tcdm_rsp_o.p_valid = read_outstanding ? read_data_valid : wr_accept;
  assign tcdm_rsp_o.p.data  = read_outstanding ? read_data : '0;

  always_comb begin : proc_axi_request_merge
    axi_req_o = '0;
    axi_req_o.ar = read_ar;
    axi_req_o.ar_valid = read_ar_valid;
    axi_req_o.r_ready = read_r_ready;
    axi_req_o.b_ready = 1'b1;
    if (write_busy) begin
      axi_req_o.aw = '0;
      axi_req_o.aw.addr = wr_burst_active_q ? wr_burst_q.addr : tcdm_req_i.q.addr;
      axi_req_o.aw.len = $bits(axi_req_o.aw.len)'(
          wr_burst_active_q ? wr_burst_q.len : (burst_beats - 1'b1));
      axi_req_o.aw.size = $bits(axi_req_o.aw.size)'(OffsetWidth);
      axi_req_o.aw.burst = axi_pkg::BURST_INCR;
      axi_req_o.aw_valid = !wr_aw_done_q;
      axi_req_o.w = '0;
      axi_req_o.w.data = tcdm_req_i.q.data;
      axi_req_o.w.strb = tcdm_req_i.q.strb;
      axi_req_o.w.last = wr_w_last;
      axi_req_o.w_valid = wr_w_valid;
    end
  end

  always_comb begin : proc_write_burst_state
    wr_burst_d        = wr_burst_q;
    wr_burst_active_d = wr_burst_active_q;
    wr_aw_done_d      = wr_aw_done_q;
    if (wr_start) begin
      wr_burst_active_d    = 1'b1;
      wr_burst_d.len       = burst_beats - 1'b1;
      wr_burst_d.beats_fed = wr_accept ? BurstLenWidth'(1) : '0;
      wr_burst_d.addr      = tcdm_req_i.q.addr;
      if (axi_req_o.aw_valid && axi_rsp_i.aw_ready) wr_aw_done_d = 1'b1;
    end else if (wr_burst_active_q) begin
      if (axi_req_o.aw_valid && axi_rsp_i.aw_ready) wr_aw_done_d = 1'b1;
      if (wr_accept) wr_burst_d.beats_fed = wr_burst_q.beats_fed + 1'b1;
      if (wr_b_done) begin
        wr_burst_active_d    = 1'b0;
        wr_aw_done_d         = 1'b0;
        wr_burst_d.beats_fed = '0;
      end
    end
  end : proc_write_burst_state

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      wr_burst_q        <= '0;
      wr_burst_active_q <= 1'b0;
      wr_aw_done_q      <= 1'b0;
    end else begin
      wr_burst_q        <= wr_burst_d;
      wr_burst_active_q <= wr_burst_active_d;
      wr_aw_done_q      <= wr_aw_done_d;
    end
  end

  `ASSERT_INIT(AxiAddrCoversTcdm, (AxiAddrWidth >= TcdmAddrWidth))
  `ASSERT_INIT(AxiPageAddressWidth, (AxiAddrWidth >= 12)) // at least 4KB
  `ASSERT_INIT(NonzeroOutstandingCapacity, (MaxOutstanding > 0))
  `ASSERT_INIT(NonzeroMaxBurstLen, (MaxBurstLen > 0))
  `ASSERT_INIT(MaxBurstFitsAxiLen, (MaxBurstLen <= 256))
  `ASSERT(SpatzReadBurstAddress_A,
          read_feed_busy_q && read_req_fire |->
              (tcdm_req_i.q.addr == read_feed_q.next_addr))
  `ASSERT(SpatzReadRspWithAr_A, read_rsp_fire |-> read_outstanding)
  `ASSERT(SpatzReadResponseOkay_A,
          read_rsp_fire |-> (axi_rsp_i.r.resp == axi_pkg::RESP_OKAY))
  // input read burst addr needs to be aligned to AXI DataWidth
  `ASSERT(SpatzReadBurstAligned_A,
          read_ar_valid && burst_mode_req_i |->
              (tcdm_req_i.q.addr[OffsetWidth-1:0] == '0))
  `ASSERT(SpatzReadBeatMetadata_A,
          read_feed_busy_q && read_beat_req |-> burst_mode_req_i)
  `ASSERT(SpatzReadBeatNoWriteBurst_A,
          read_feed_busy_q |-> !write_busy)
  `ASSERT(SpatzWriteBeatNoRead_A,
          write_busy |-> !read_outstanding)
  // Catch AMO and AXI errors
  `ASSERT(NoAmo_A, tcdm_req_i.q_valid |-> tcdm_req_i.q.amo == AMONone)
  `ASSERT(SpatzWriteResponseOkay_A,
          axi_rsp_i.b_valid |-> (axi_rsp_i.b.resp == axi_pkg::RESP_OKAY))

endmodule
