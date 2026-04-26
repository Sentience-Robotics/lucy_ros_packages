import warnings
from pathlib import Path

import pytest
from ament_flake8.main import main_with_errors


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    pkg_root = Path(__file__).resolve().parents[1]
    targets = [
        str(pkg_root / "setup.py"),
        str(pkg_root / "lucy_config_pipeline"),
        str(pkg_root / "test"),
    ]
    with warnings.catch_warnings():
        warnings.filterwarnings(
            "ignore",
            message="SelectableGroups dict interface is deprecated",
            category=DeprecationWarning,
        )
        rc, errors = main_with_errors(argv=targets)
    assert rc == 0, (
        "Found %d code style errors / warnings:\n" % len(errors)
        + "\n".join(errors)
    )
