// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Single-port TCDM to AXI converter via `axi_from_mem`: one wide TCDM port to
// an AXI manager (single-beat, full-size, `BURST_FIXED`, which is equivalent
// to `INCR` for `LEN=0`; ID 0, cache `'0`). Depth-1 (`MaxRequests=1`).

`include "common_cells/assertions.svh"

module tcdm_to_axi
  import lsu_pkg::*;
#(
    parameter int unsigned DataWidth = 512,
    parameter int unsigned TcdmAddrWidth = 48,
    parameter int unsigned AxiAddrWidth = 48,
    parameter type tcdm_req_t = logic,
    parameter type tcdm_rsp_t = logic,
    parameter type axi_req_t = logic,
    parameter type axi_rsp_t = logic
) (
    input  logic      clk_i,
    input  logic      rst_ni,
    // TCDM slave: one coalesced wide request at a time.
    input  tcdm_req_t tcdm_req_i,
    output tcdm_rsp_t tcdm_rsp_o,
    // AXI master: single-beat transactions (LEN=0, SIZE=log2(width/8)).
    output axi_req_t  axi_req_o,
    input  axi_rsp_t  axi_rsp_i
);

  logic mem_req, mem_gnt, mem_rsp_valid, mem_err;
  logic [DataWidth-1:0] mem_rdata;
  assign mem_req = tcdm_req_i.q_valid;
  assign tcdm_rsp_o.q_ready = mem_gnt;
  assign tcdm_rsp_o.p_valid = mem_rsp_valid;
  assign tcdm_rsp_o.p.data = mem_rdata;

  axi_from_mem #(
      .MemAddrWidth(TcdmAddrWidth),
      .AxiAddrWidth(AxiAddrWidth),
      .DataWidth   (DataWidth),
      .MaxRequests (1),
      .axi_req_t   (axi_req_t),
      .axi_rsp_t   (axi_rsp_t)
  ) i_axi_from_mem (
      .clk_i,
      .rst_ni,
      .mem_req_i      (mem_req),
      .mem_addr_i     (tcdm_req_i.q.addr),
      .mem_we_i       (tcdm_req_i.q.write),
      .mem_wdata_i    (tcdm_req_i.q.data),
      .mem_be_i       (tcdm_req_i.q.strb),
      .mem_gnt_o      (mem_gnt),
      .mem_rsp_valid_o(mem_rsp_valid),
      .mem_rsp_rdata_o(mem_rdata),
      .mem_rsp_error_o(mem_err),
      .slv_aw_cache_i ('0),
      .slv_ar_cache_i ('0),
      .axi_req_o      (axi_req_o),
      .axi_rsp_i      (axi_rsp_i)
  );

  `ASSERT_INIT(AxiAddrCoversTcdm, (AxiAddrWidth >= TcdmAddrWidth))
  // AMO and AXI errors have nowhere to go on the TCDM response: flag in sim.
  `ASSERT(NoAmo_A, tcdm_req_i.q_valid |-> tcdm_req_i.q.amo == AMONone)
  `ASSERT(NoAxiError_A, mem_rsp_valid |-> !mem_err)

endmodule
