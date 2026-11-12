// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Burst legalizer for the TCDM-AXI converter.
`include "common_cells/assertions.svh"

module tcdm_burst_splitter
#(
    parameter int unsigned AddrWidth   = 48,
    parameter int unsigned MaxBurstBeats = 8,
    parameter int unsigned BeatOffsetWidth = 6,  // 64B
    parameter int unsigned PageOffsetWidth = 12, // 4KB
    // Derived parameters. DO NOT CHANGE!
    localparam int unsigned BeatBytes   = 1 << BeatOffsetWidth,
    localparam int unsigned PageBytes   = 1 << PageOffsetWidth,
    localparam int unsigned LenBeatsWidth = cc_pkg::cnt_width(MaxBurstBeats),
    localparam int unsigned LenAxiWidth  = cc_pkg::idx_width(MaxBurstBeats)
) (
    input  logic [AddrWidth-1:0]     addr_i,
    input  logic [LenBeatsWidth-1:0] len_beats_i,
    output logic [LenAxiWidth-1:0]   len_axi_o
);

  localparam int unsigned PaddedAddrWidth = AddrWidth+1;
  logic [PaddedAddrWidth-1:0] page_beats;
  logic [PaddedAddrWidth-1:0] burst_beats;

  assign page_beats = (PageBytes - addr_i[PageOffsetWidth-1:0]) >> BeatOffsetWidth;

  always_comb begin : proc_clamp
    burst_beats = PaddedAddrWidth'(len_beats_i);
    if (burst_beats > MaxBurstBeats)
      burst_beats = MaxBurstBeats;
    if (burst_beats > page_beats)
      burst_beats = page_beats;
    len_axi_o = LenAxiWidth'(burst_beats - 1'b1);
  end

  `ASSERT_INIT(NonzeroBeatBytes, BeatBytes > 0)
  `ASSERT_INIT(NonzeroMaxBurstBeats, MaxBurstBeats > 0)
  `ASSERT_INIT(PageCoversBeat, PageOffsetWidth >= BeatOffsetWidth)
  `ASSERT_INIT(PageOffsetFitsAddr, PageOffsetWidth <= AddrWidth)
endmodule
