// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Single-port wide TCDM to AXI bridge.
// Assumptions (check by assertions):
// Input request aligned to DataWidth
// Response arrive in order
//
// Reads:
// 1 AXI INCR burst per VLSU op
// Up to `MaxOutstanding` ARs in flight
//
// Writes: single-beat path through `axi_from_mem`

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
    parameter int unsigned BurstLenWidth = cc_pkg::cnt_width(MaxBurstLen);
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
    output tcdm_rsp_t tcdm_rsp_o,
    // AXI master: read bursts and single-beat writes.
    output axi_req_t  axi_req_o,
    input  axi_rsp_t  axi_rsp_i
);

  localparam int unsigned BeatBytes = DataWidth / 8;
  localparam int unsigned OffsetWidth = $clog2(BeatBytes);

  logic write_req, write_gnt, write_rsp_valid, write_err;
  logic [DataWidth-1:0] write_rdata;
  axi_req_t axi_write_req;
  axi_rsp_t axi_write_rsp;
  logic [cc_pkg::cnt_width(MaxOutstanding)-1:0] write_count_d, write_count_q;

  axi_ar_chan_t read_ar, read_ar_d, read_ar_q;
  logic read_ar_valid, read_ar_ready, read_r_ready;
  logic [BurstLenWidth-1:0] read_burst_len;

  typedef struct packed {
    logic [BurstLenWidth-1:0] burst_len;     // beats in this burst
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

  // burst splitter
  always_comb begin : proc_read_burst_length
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
    read_burst_len = BurstLenWidth'(wanted_beats);
  end

  always_comb begin : proc_read_ar
    read_ar_valid   = (write_count_q == '0) && !read_feed_busy_q &&
                      (read_ar_outstanding_q != MaxOutstanding) &&
                      tcdm_req_i.q_valid && !tcdm_req_i.q.write;
    read_ar_d       = '0;
    read_ar_d.addr  = tcdm_req_i.q.addr;
    read_ar_d.len   = $bits(read_ar_d.len)'(read_burst_len - 1'b1);
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
  assign read_beat_fire   = (write_count_q == '0) && read_feed_busy_q &&
                            burst_mode_req_i &&
                            (read_feed_q.beats_issued < read_feed_q.burst_len);
  assign read_req_fire    = read_burst_start ||
                            (read_beat_req && read_beat_fire);
  assign read_outstanding = (read_ar_outstanding_q != '0);

  always_comb begin : proc_read_feed
    read_feed_d           = read_feed_q;
    read_feed_busy_d      = read_feed_busy_q;
    read_ar_outstanding_d = read_ar_outstanding_q;
    if (read_burst_start) begin
      read_feed_d.burst_len    = read_burst_len;
      read_feed_d.beats_issued = BurstLenWidth'(1);
      read_feed_d.next_addr    = tcdm_req_i.q.addr + AxiAddrWidth'(BeatBytes);
      read_feed_busy_d         = (read_burst_len > BurstLenWidth'(1));
      read_ar_outstanding_d    = read_ar_outstanding_q + 1'b1;
    end else if (read_feed_busy_q && read_req_fire) begin
      read_feed_d.beats_issued = read_feed_q.beats_issued + 1'b1;
      read_feed_d.next_addr    = read_feed_q.next_addr + AxiAddrWidth'(BeatBytes);
      if ((read_feed_q.beats_issued + 1'b1) == read_feed_q.burst_len)
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

  always_comb begin : proc_write_count
    write_count_d = write_count_q;
    case ({write_req && write_gnt, write_rsp_valid})
      2'b10: write_count_d = write_count_q + 1'b1;
      2'b01: write_count_d = write_count_q - 1'b1;
      default: ;
    endcase
  end

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) write_count_q <= '0;
    else write_count_q <= write_count_d;
  end

  // No write while a read is outstanding
  assign write_req = tcdm_req_i.q_valid && tcdm_req_i.q.write && !read_outstanding;
  assign tcdm_rsp_o.q_ready = tcdm_req_i.q.write ?
                              (write_gnt && !read_outstanding) : read_req_fire;
  assign tcdm_rsp_o.p_valid = read_outstanding ? read_data_valid : write_rsp_valid;
  assign tcdm_rsp_o.p.data = read_outstanding ? read_data : write_rdata;

  always_comb begin : proc_write_response
    axi_write_rsp = '0;
    axi_write_rsp.aw_ready = axi_rsp_i.aw_ready;
    axi_write_rsp.w_ready = axi_rsp_i.w_ready;
    axi_write_rsp.b_valid = axi_rsp_i.b_valid;
    axi_write_rsp.b = axi_rsp_i.b;
  end

  always_comb begin : proc_axi_request_merge
    axi_req_o = axi_write_req;
    axi_req_o.ar = read_ar;
    axi_req_o.ar_valid = read_ar_valid;
    axi_req_o.r_ready = read_r_ready;
  end

  axi_from_mem #(
      .MemAddrWidth(TcdmAddrWidth),
      .AxiAddrWidth(AxiAddrWidth),
      .DataWidth   (DataWidth),
      .MaxRequests (MaxOutstanding),
      .axi_req_t   (axi_req_t),
      .axi_rsp_t   (axi_rsp_t)
  ) i_axi_from_mem (
      .clk_i,
      .rst_ni,
      .mem_req_i      (write_req),
      .mem_addr_i     (tcdm_req_i.q.addr),
      .mem_we_i       (tcdm_req_i.q.write),
      .mem_wdata_i    (tcdm_req_i.q.data),
      .mem_be_i       (tcdm_req_i.q.strb),
      .mem_gnt_o      (write_gnt),
      .mem_rsp_valid_o(write_rsp_valid),
      .mem_rsp_rdata_o(write_rdata),
      .mem_rsp_error_o(write_err),
      .slv_aw_cache_i ('0),
      .slv_ar_cache_i ('0),
      .axi_req_o      (axi_write_req),
      .axi_rsp_i      (axi_write_rsp)
  );

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
  // Catch AMO and AXI errors
  `ASSERT(NoAmo_A, tcdm_req_i.q_valid |-> tcdm_req_i.q.amo == AMONone)
  `ASSERT(NoAxiError_A, write_rsp_valid |-> !write_err)

endmodule
