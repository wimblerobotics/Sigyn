# SPDX-License-Identifier: Apache-2.0
# Copyright 2025 Wimblerobotics
# https://github.com/wimblerobotics/Sigyn

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Found code style errors / warnings'
