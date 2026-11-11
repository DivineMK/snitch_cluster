// Copyright 2026 ETH Zurich and University of Bologna.
// Solderpad Hardware License, Version 0.51, see LICENSE for details.
// SPDX-License-Identifier: SHL-0.51

// Burst legalizer for the TCDM-AXI converter.
`include "common_cells/assertions.svh"

module tcdm_burst_splitter
#(
    parameter int unsigned AddrWidth   = 48,
    parameter int unsigned MaxBurstLen = 8,
    parameter int unsigned BeatOffsetWidth = 6,  // 64B
    parameter int unsigned PageOffsetWidth = 12, // 4KB
    parameter int unsigned LenBeatsWidth = 4,
    // Derived parameters. DO NOT CHANGE!
    localparam int unsigned BeatBytes   = 1 << BeatOffsetWidth,
    localparam int unsigned PageBytes   = 1 << PageOffsetWidth,
    localparam int unsigned BurstLenWidth   = cc_pkg::cnt_width(MaxBurstLen)
) (
    input  logic [AddrWidth-1:0]     addr_i,
    input  logic [LenBeatsWidth-1:0] len_beats_i,
    output logic [BurstLenWidth-1:0] burst_len_o
);

  localparam int unsigned PaddedAddrWidth = AddrWidth+1;
  logic [PaddedAddrWidth-1:0] page_beats;
  logic [PaddedAddrWidth-1:0] burst_beats;

  assign page_beats = (PageBytes - addr_i[PageOffsetWidth-1:0]) >> BeatOffsetWidth;

  always_comb begin : proc_clamp
    burst_beats = PaddedAddrWidth'(len_beats_i);
    if (burst_beats > MaxBurstLen)
      burst_beats = MaxBurstLen;
    if (burst_beats > page_beats)
      burst_beats = page_beats;
    burst_len_o = BurstLenWidth'(burst_beats - 1'b1);
  end

  `ASSERT_INIT(NonzeroBeatBytes, BeatBytes > 0)
  `ASSERT_INIT(NonzeroMaxBurstLen, MaxBurstLen > 0)
  `ASSERT_INIT(PageCoversBeat, PageOffsetWidth >= BeatOffsetWidth)
  `ASSERT_INIT(PageOffsetFitsAddr, PageOffsetWidth <= AddrWidth)
endmodule
