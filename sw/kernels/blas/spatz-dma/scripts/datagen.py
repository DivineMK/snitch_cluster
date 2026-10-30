#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

import sys

import numpy as np
import snitch.util.sim.data_utils as du


class SpatzDmaDataGen(du.DataGen):
    # Create repeating pattern of [0]*8+[1]*8+[2]*8+[3]*8

    BURST_ALIGNMENT = 4096

    def validate(self, **kwargs):
        # the repeating pattern is 32 bytes
        assert kwargs['n'] > 0 and kwargs['n'] % 32 == 0
        du.validate_tcdm_footprint(2 * kwargs['n'] * 8)

    def emit_header(self, **kwargs):
        header = [super().emit_header()]
        self.validate(**kwargs)

        n = kwargs['n']
        pattern = np.repeat(np.arange(4, dtype=np.float64), 8)
        values = np.tile(pattern, n // pattern.size)
        values_b = values + 4.0
        header += [du.format_scalar_definition(
            'extern const uint32_t', 'n', n)]

        header += [du.format_array_definition(
            'double', 'a', values, alignment=self.BURST_ALIGNMENT,
            section=kwargs['section'])]
        header += [du.format_array_definition(
            'double', 'b', values_b, alignment=self.BURST_ALIGNMENT,
            section=kwargs['section'])]
        return '\n\n'.join(header)


if __name__ == '__main__':
    sys.exit(SpatzDmaDataGen().main())
