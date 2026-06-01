"""CLI argument parsing: new --version / --log-level / --loop-hz / --watchdog-s."""

from __future__ import annotations

import pytest

from mavpilot import __version__
from mavpilot.cli import _build_argparser


def test_defaults():
    args = _build_argparser().parse_args([])
    assert args.log_level == "INFO"
    assert args.loop_hz == 50.0
    assert args.watchdog_s == 2.0


def test_overrides_parsed():
    args = _build_argparser().parse_args(
        ["--log-level", "DEBUG", "--loop-hz", "30", "--watchdog-s", "1.5"]
    )
    assert args.log_level == "DEBUG"
    assert args.loop_hz == 30.0
    assert args.watchdog_s == 1.5


def test_invalid_log_level_rejected():
    with pytest.raises(SystemExit):
        _build_argparser().parse_args(["--log-level", "TRACE"])


def test_version_flag_prints_and_exits(capsys):
    with pytest.raises(SystemExit) as exc:
        _build_argparser().parse_args(["--version"])
    assert exc.value.code == 0
    assert __version__ in capsys.readouterr().out
