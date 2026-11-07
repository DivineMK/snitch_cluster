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
    // Derived parameters. DO NOT CHANGE!
    localparam int unsigned BeatBytes   = 1 << BeatOffsetWidth,
    localparam int unsigned PageBytes   = 1 << PageOffsetWidth,
    localparam int unsigned BurstLenWidth   = cc_pkg::cnt_width(MaxBurstLen)
) (
    input  logic [AddrWidth-1:0]     addr_i,
    // Bytes of the current run not yet covered by an accepted beat.
    input  logic [AddrWidth-1:0]     bytes_remaining_i,
    input  logic                     contiguous_i,
    output logic [BurstLenWidth-1:0] burst_len_o
);

  localparam int unsigned PaddedAddrWidth = AddrWidth+1;
  logic [PaddedAddrWidth-1:0] beats_in_insn;
  logic [PaddedAddrWidth-1:0] page_beats;
  logic [PaddedAddrWidth-1:0] burst_beats;

  // round up to handle partial beats for unaligned transfer
  assign beats_in_insn = ({1'b0, bytes_remaining_i} + BeatBytes - 1'b1) >> BeatOffsetWidth;
  assign page_beats    = (PageBytes - addr_i[PageOffsetWidth-1:0]) >> BeatOffsetWidth;

  always_comb begin : proc_clamp
    burst_beats = 1;
    if (contiguous_i && (addr_i[BeatOffsetWidth-1:0] == '0)) begin
      burst_beats = beats_in_insn;
      if (burst_beats > MaxBurstLen)
        burst_beats = MaxBurstLen;
      if (burst_beats > page_beats)
        burst_beats = page_beats;
    end
    burst_len_o = BurstLenWidth'(burst_beats - 1'b1);
  end

  `ASSERT_INIT(NonzeroBeatBytes, BeatBytes > 0)
  `ASSERT_INIT(NonzeroMaxBurstLen, MaxBurstLen > 0)
  `ASSERT_INIT(PageCoversBeat, PageOffsetWidth >= BeatOffsetWidth)
  `ASSERT_INIT(PageOffsetFitsAddr, PageOffsetWidth <= AddrWidth)
endmodule
