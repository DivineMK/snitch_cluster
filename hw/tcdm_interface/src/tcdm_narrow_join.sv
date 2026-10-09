// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Narrow join + fork: coalesces `NrPorts` narrow (8B) contiguous TCDM requests
// into one wide (64B) single-beat TCDM request and fans the wide response back
// out to the joined lanes. Feeds `tcdm_to_axi` for the Spatz vector escape path.
//
// Scope (v1):
// - Contiguous unit-stride only, anchored on the lowest presenting lane `f`:
//   every valid lane `i` must satisfy `addr[i] == base + i*8` with group base
//   `base = addr[f] - f*8` 64B-aligned, uniform `write`, no AMO. Holes below
//   `f` (vstart resume, masked lanes) are tolerated as zeroed lanes / masked
//   `strb`. This is a usage CONTRACT, not enforced: violations fire a
//   mis-packed beat (undefined in silicon) and trip `NarrowContiguousRequest`
//   in sim. Narrow fallback for the rest is TODO.
// - No page-crossing check (TODO: reuse `idma_legalizer_page_splitter`).
// - Steps present atomically: no lane may arrive later than its group-mates
//   (Spatz presents each step's lanes simultaneously); staggered arrival hangs.
//
// NOTE on address decode: the caller (`snitch_cc`) widens Spatz VLSU channels
// to full address width and steers each port by the cluster TCDM base/mask
// rule. The join only ever sees AXI-bound groups.

`include "common_cells/registers.svh"
`include "common_cells/assertions.svh"

// Narrow join + fork: N narrow TCDM requesters to 1 wide TCDM master.
module tcdm_narrow_join
  import lsu_pkg::*;
#(
    parameter int unsigned NrPorts = 8,
    parameter int unsigned NarrowDataWidth = 64,
    parameter int unsigned WideDataWidth = 512,
    parameter int unsigned TcdmAddrWidth = 48,
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
    input  wide_rsp_t                 wide_rsp_i
);

  localparam int unsigned NarrowStrbWidth = NarrowDataWidth / 8;
  localparam int unsigned WideStrbWidth = WideDataWidth / 8;
  localparam int unsigned LaneBytes = NarrowStrbWidth;

  // Joined payload.
  logic [             WideDataWidth-1:0] join_wdata;
  logic [             WideStrbWidth-1:0] join_strb;
  logic                                  join_write;
  logic [             TcdmAddrWidth-1:0] join_addr;

  // Active set + first-valid anchor. the group base is reconstructed so the beat
  // stays 64B-aligned. The set doubles as the dynamic select for
  // `cc_stream_join_dynamic` below.
  logic [                   NrPorts-1:0] lane_sel;
  logic [cc_pkg::idx_width(NrPorts)-1:0] anchor;
  logic                                  anchor_empty;
  logic [             TcdmAddrWidth-1:0] base_addr;
  always_comb begin
    lane_sel = '0;
    for (int i = 0; i < NrPorts; i++) lane_sel[i] = narrow_req_i[i].q_valid;
  end
  // Lowest presenting lane as a trailing-zero count; `anchor_empty` gates the
  // logic below instead of `|lane_sel`.
  cc_lzc #(
      .Width(NrPorts),
      .Mode (cc_pkg::LZC_TRAILING_ZERO_CNT)
  ) i_anchor_lzc (
      .in_i   (lane_sel),
      .cnt_o  (anchor),
      .empty_o(anchor_empty)
  );
  // Reconstructed group base `addr[f] - f*8`. Meaningful only when a lane
  // presents; pack below is gated on `!anchor_empty`.
  assign base_addr = narrow_req_i[anchor].q.addr - TcdmAddrWidth'(anchor * LaneBytes);

  // Common-cells join over the active set: output valid once all selected
  // ports are valid (true by construction); ready fans back out to selected
  // ports only.
  logic [NrPorts-1:0] narrow_q_valid_vec;
  for (genvar v = 0; v < NrPorts; v++) begin : gen_valid_vec
    assign narrow_q_valid_vec[v] = narrow_req_i[v].q_valid;
  end
  logic [NrPorts-1:0] join_inp_ready;
  logic join_oup_valid, join_oup_ready;
  cc_stream_join_dynamic #(
      .NumInp(NrPorts)
  ) i_join (
      .inp_valid_i(narrow_q_valid_vec),
      .inp_ready_o(join_inp_ready),
      .sel_i      (lane_sel),
      .oup_valid_o(join_oup_valid),
      .oup_ready_i(join_oup_ready)
  );

  // Pack: each presenting lane contributes its slot; idle lanes stay zeroed
  // with masked `strb`. Contiguity is NOT enforced here: unit-stride,
  // aligned, uniform groups are a usage contract (see header), checked by
  // `NarrowContiguousRequest` in sim. Anything else fires a mis-packed beat.
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

  // Wide request: fires on any presenting set. AMO never occurs (Spatz emits
  // `AMONone`), so the wide beat is never atomic; `user` is tied off
  // (dropped at AXI anyway).
  assign wide_req_o.q.addr  = join_addr;
  assign wide_req_o.q.write = join_write;
  assign wide_req_o.q.amo   = AMONone;
  assign wide_req_o.q.data  = join_wdata;
  assign wide_req_o.q.strb  = join_strb;
  assign wide_req_o.q.user  = '0;
  assign wide_req_o.q_valid = join_oup_valid;
  assign join_oup_ready     = wide_rsp_i.q_ready;

  // Latched fanout mask: captured at the wide handshake, held until the
  // response fans out.
  logic [NrPorts-1:0] active_q, active_d;
  `FF(active_q, active_d, '0, clk_i, rst_ni)
  always_comb begin
    active_d = active_q;
    if (wide_req_o.q_valid && wide_rsp_i.q_ready) active_d = lane_sel;
  end

  for (genvar i = 0; i < NrPorts; i++) begin : gen_narrow_rsp
    assign narrow_rsp_o[i].q_ready = join_inp_ready[i];
    assign narrow_rsp_o[i].p_valid = wide_rsp_i.p_valid && active_q[i];
    assign narrow_rsp_o[i].p.data  = wide_rsp_i.p.data[i*NarrowDataWidth+:NarrowDataWidth];
  end

  `ASSERT_INIT(MatchingDataWidth, (WideDataWidth == NrPorts * NarrowDataWidth))

  // v1 presentation contract (review aids): each step's lanes present
  // simultaneously with unit-stride addresses around the reconstructed base.
  // Holes below the anchor (vstart, masks) are legal; staggered arrival hangs
  // the join (no narrow fallback), so fail loudly instead.
  for (genvar i = 0; i < NrPorts; i++) begin : gen_assert_contig
    `ASSERT(NarrowContiguousRequest,
            narrow_req_i[i].q_valid |->
      (narrow_req_i[i].q.addr == base_addr + unsigned'(i * LaneBytes)))
  end

endmodule
