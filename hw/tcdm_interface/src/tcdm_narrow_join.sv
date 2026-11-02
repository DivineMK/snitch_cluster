// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Narrow join + fork: coalesces `NrPorts` narrow (8B) contiguous TCDM requests
// into one wide (64B) single-beat TCDM request and fans the wide response back
// out to the joined lanes. use as input for `tcdm_to_axi`

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"

module tcdm_narrow_join
  import lsu_pkg::*;
#(
    parameter int unsigned NrPorts = 8,
    parameter int unsigned NarrowDataWidth = 64,
    parameter int unsigned WideDataWidth = 512,
    parameter int unsigned TcdmAddrWidth = 48,
    // One lane mask is stored per accepted wide beat, so this bounds the number
    // of beats that can be in flight towards the AXI converter.
    parameter int unsigned RspMaskDepth = 4,
    parameter type narrow_req_t = logic,
    parameter type narrow_rsp_t = logic,
    parameter type wide_req_t = logic,
    parameter type wide_rsp_t = logic
) (
    input  logic                      clk_i,
    input  logic                      rst_ni,
    // Narrow side: joined lanes (AXI-bound groups only, see above).
    input  narrow_req_t [NrPorts-1:0] narrow_req_i,
    output narrow_rsp_t [NrPorts-1:0] narrow_rsp_o,
    // Wide side: single coalesced TCDM request per group.
    output wide_req_t                 wide_req_o,
    input  wide_rsp_t                 wide_rsp_i,
    output logic                      rsp_mask_valid_o,
    output logic                      request_fire_o,
    output logic                      response_fire_o
);

  localparam int unsigned NarrowStrbWidth = NarrowDataWidth / 8;
  localparam int unsigned WideStrbWidth = WideDataWidth / 8;
  localparam int unsigned LaneBytes = NarrowStrbWidth;

  // Joined payload.
  logic [             WideDataWidth-1:0] join_wdata;
  logic [             WideStrbWidth-1:0] join_strb;
  logic                                  join_write;
  logic [             TcdmAddrWidth-1:0] join_addr;
  logic [                   NrPorts-1:0] lane_sel;
 
  logic [cc_pkg::idx_width(NrPorts)-1:0] anchor; // first valid port
  logic                                  anchor_empty; // no valid port
  logic [             TcdmAddrWidth-1:0] base_addr;
  always_comb begin
    lane_sel = '0;
    for (int i = 0; i < NrPorts; i++) lane_sel[i] = narrow_req_i[i].q_valid;
  end

  cc_lzc #(
      .Width(NrPorts),
      .Mode (cc_pkg::LZC_TRAILING_ZERO_CNT)
  ) i_anchor_lzc (
      .in_i   (lane_sel),
      .cnt_o  (anchor),
      .empty_o(anchor_empty)
  );
  assign base_addr = narrow_req_i[anchor].q.addr - TcdmAddrWidth'(anchor * LaneBytes);

  logic [NrPorts-1:0] join_inp_ready;
  logic join_oup_valid, join_oup_ready;
  cc_stream_join_dynamic #(
      .NumInp(NrPorts)
  ) i_join (
      .inp_valid_i(lane_sel),
      .inp_ready_o(join_inp_ready),
      .sel_i      (lane_sel),
      .oup_valid_o(join_oup_valid),
      .oup_ready_i(join_oup_ready)
  );

  always_comb begin
    join_wdata = '0;
    join_strb  = '0;
    for (int i = 0; i < NrPorts; i++) begin
      if (lane_sel[i]) begin
        join_wdata[i*NarrowDataWidth+:NarrowDataWidth] = narrow_req_i[i].q.data;
        join_strb[i*LaneBytes+:LaneBytes] = narrow_req_i[i].q.strb;
      end
    end
    join_write = (!anchor_empty) ? narrow_req_i[anchor].q.write : 1'b0;
    join_addr  = (!anchor_empty) ? base_addr : '0;
  end

  // Wide request
  // TODO: AMONone and no user
  assign wide_req_o.q.addr  = join_addr;
  assign wide_req_o.q.write = join_write;
  assign wide_req_o.q.amo   = AMONone;
  assign wide_req_o.q.data  = join_wdata;
  assign wide_req_o.q.strb  = join_strb;
  assign wide_req_o.q.user  = '0;

  typedef logic [NrPorts-1:0] lane_mask_t;
  lane_mask_t rsp_mask_wdata, rsp_mask_rdata;
  logic rsp_mask_valid, rsp_mask_ready;
  assign rsp_mask_wdata = lane_sel;
  assign request_fire_o = join_oup_valid && wide_rsp_i.q_ready && rsp_mask_ready;
  assign response_fire_o = wide_rsp_i.p_valid && rsp_mask_valid;
  assign rsp_mask_valid_o = rsp_mask_valid;
  assign wide_req_o.q_valid = join_oup_valid && rsp_mask_ready;
  assign join_oup_ready = wide_rsp_i.q_ready && rsp_mask_ready;

  cc_stream_fifo_optimal_wrap #(
      .Depth      ( RspMaskDepth ),
      .data_t     ( lane_mask_t ),
      .PrintInfo  ( 1'b0 )
  ) i_rsp_mask_fifo (
      .clk_i,
      .rst_ni,
      .clr_i      ( 1'b0 ),
      .flush_i    ( 1'b0 ),
      .usage_o    ( /* unused */ ),
      .data_i     ( rsp_mask_wdata ),
      .valid_i    ( request_fire_o ),
      .ready_o    ( rsp_mask_ready ),
      .data_o     ( rsp_mask_rdata ),
      .valid_o    ( rsp_mask_valid ),
      .ready_i    ( response_fire_o )
  );

  for (genvar i = 0; i < NrPorts; i++) begin : gen_narrow_rsp
    assign narrow_rsp_o[i].q_ready = join_inp_ready[i];
    assign narrow_rsp_o[i].p_valid = wide_rsp_i.p_valid && rsp_mask_valid && rsp_mask_rdata[i];
    assign narrow_rsp_o[i].p.data  = wide_rsp_i.p.data[i*NarrowDataWidth+:NarrowDataWidth];
  end

  `ASSERT_INIT(MatchingDataWidth, (WideDataWidth == NrPorts * NarrowDataWidth))

  // TODO: now only works for contiguous access
  for (genvar i = 0; i < NrPorts; i++) begin : gen_assert_contig
    `ASSERT(NarrowContiguousRequest,
            narrow_req_i[i].q_valid |->
      (narrow_req_i[i].q.addr == base_addr + unsigned'(i * LaneBytes)))
  end

endmodule
