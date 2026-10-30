#!/usr/bin/env python3
# Copyright 2026 ETH Zurich and University of Bologna.
# Licensed under the Apache License, Version 2.0, see LICENSE for details.
# SPDX-License-Identifier: Apache-2.0

import sys

import numpy as np
from snitch.util.sim.verif_utils import Verifier


class SpatzDmaVerifier(Verifier):
    OUTPUT_UIDS = ['a', 'b']

    def get_actual_results(self):
        a = self.get_output_from_symbol('a', 'double')
        b = self.get_output_from_symbol('b', 'double')
        return np.concatenate((a, b))

    def get_expected_results(self):
        a = self.get_input_from_symbol('a', 'double')
        b = self.get_input_from_symbol('b', 'double')
        expected_b = b.copy()
        return np.concatenate((a, expected_b))

    def check_results(self, *args):
        return super().check_results(*args, atol=0)


if __name__ == '__main__':
    sys.exit(SpatzDmaVerifier().main())
