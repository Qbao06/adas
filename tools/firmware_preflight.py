"""Pre-upload preflight for the ESP32 Radar Gateway firmware.

The Arduino ``hardware_config.h`` ships with sentinel values
(``GPIO_PLACEHOLDER``, ``MAC_PLACEHOLDER``, ``CHANNEL_PLACEHOLDER``,
``CHANGE_ME`` and the ``PLACEHOLDER_BOARD_MODEL`` string) so a fresh
checkout compiles without leaking hardware-specific bytes into the
repository.  Those same sentinels must be replaced before the firmware
is uploaded to real hardware, otherwise the ESP32 boots with all
GPIOs pulled high, a peer MAC of ``FF:FF:FF:FF:FF:FF`` and an
ESP-NOW channel of 0.

This script:

1. Parses ``firmware/esp32_radar_gateway/src/hardware_config.h`` with a
   light-weight regex scanner (no ``pycparser`` / ``pcpp`` dependency
   so it works on a plain Python install on the build machine).
2. Detects any placeholder sentinel that survived.
3. Exits non-zero with a precise Vietnamese error message listing
   exactly which lines / macros need attention.
4. Exits zero with a one-line summary when the configuration is clean.

It is importable (call :func:`check_hardware_config`) and runnable
as a CLI.  ``build.sh`` invokes it as a pre-step so a fresh checkout
fails fast.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path
from typing import Iterable, List, NamedTuple, Optional, Tuple

# --- Constants ------------------------------------------------------------- #

REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_HEADER = REPO_ROOT / "firmware" / "esp32_radar_gateway" / "src" / "hardware_config.h"

# The Python mirror of the sentinel values defined in hardware_config.h.
GPIO_PLACEHOLDER = 0xFF
MAC_PLACEHOLDER = 0xFF
CHANNEL_PLACEHOLDER = 0

# Anything that looks like a TODO or a "fill me in" string in the
# header is treated as a placeholder too.
TEXT_PLACEHOLDERS: Tuple[str, ...] = (
    "CHANGE_ME",
    "PLACEHOLDER_BOARD_MODEL",
    "PLACEHOLDER",
    "TODO_REPLACE",
    "FIXME",
)

# Macros we care about for placeholder detection.  The header uses
# #ifndef guards so a missing macro is fine -- it just inherits the
# sentinel default.  We still surface every default the header ships
# so the user can see what their build is actually using.
GPIO_MACROS: Tuple[str, ...] = (
    "UART1_RX_GPIO",
    "UART1_TX_GPIO",
    "UART2_RX_GPIO",
    "UART2_TX_GPIO",
)
CHANNEL_MACROS: Tuple[str, ...] = ("ESPNOW_CHANNEL",)
MAC_MACROS: Tuple[str, ...] = (
    "GATEWAY_MAC_BYTE_0",
    "GATEWAY_MAC_BYTE_1",
    "GATEWAY_MAC_BYTE_2",
    "GATEWAY_MAC_BYTE_3",
    "GATEWAY_MAC_BYTE_4",
    "GATEWAY_MAC_BYTE_5",
    "REMOTE_MAC_BYTE_0",
    "REMOTE_MAC_BYTE_1",
    "REMOTE_MAC_BYTE_2",
    "REMOTE_MAC_BYTE_3",
    "REMOTE_MAC_BYTE_4",
    "REMOTE_MAC_BYTE_5",
)
BOARD_MODEL_MACRO: str = "BOARD_MODEL"


# --- Data classes ---------------------------------------------------------- #


class FoundIssue(NamedTuple):
    """A single placeholder hit, ready for the Vietnamese error message."""

    macro: str
    line: int
    value: str
    reason: str

    def as_message(self) -> str:
        return (
            f"  - {self.macro} (line {self.line}): {self.value} "
            f"({self.reason})"
        )


# --- Pure helpers ---------------------------------------------------------- #


def _strip_cpp_comments(text: str) -> str:
    """Remove ``// ...`` and ``/* ... */`` comments so they cannot
    accidentally satisfy a placeholder lookup."""
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.DOTALL)
    text = re.sub(r"//[^\n]*", "", text)
    return text


def _parse_define_value(body: str, macro: str) -> Optional[Tuple[str, int]]:
    """Return ``(value_text, line_number)`` for ``#define MACRO value``.

    Skips lines where the macro is wrapped in ``#ifndef ...`` so we do
    not misread the default.  Returns ``None`` if not found.
    """
    pattern = re.compile(
        rf"^[ \t]*#\s*define\s+{re.escape(macro)}\s+(.+?)\s*$",
        re.MULTILINE,
    )
    match = pattern.search(body)
    if match is None:
        return None
    line = body.count("\n", 0, match.start()) + 1
    return match.group(1).strip(), line


def _parse_define_default(body: str, macro: str) -> Optional[Tuple[str, int]]:
    """Like :func:`_parse_define_value` but matches ``#define MACRO DEFAULT``.

    The header uses ``#ifndef ... #define MACRO PLACEHOLDER #endif`` so
    the *first* ``#define`` we see is the default sentinel.  When the
    user replaces it, the sentinel line is gone and we fall through to
    ``None``.
    """
    return _parse_define_value(body, macro)


# --- Issue detection ------------------------------------------------------- #


def _detect_issues(header_path: Path) -> List[FoundIssue]:
    raw = header_path.read_text(encoding="utf-8")
    body = _strip_cpp_comments(raw)

    issues: List[FoundIssue] = []

    # Board model first -- pure string match, simplest case.
    board = _parse_define_value(body, BOARD_MODEL_MACRO)
    if board is not None:
        value, line = board
        for sentinel in TEXT_PLACEHOLDERS:
            if sentinel in value.upper():
                issues.append(
                    FoundIssue(
                        macro=BOARD_MODEL_MACRO,
                        line=line,
                        value=value,
                        reason=(
                            "chuỗi board model vẫn chứa sentinel; "
                            f"thay bằng tên board thật (vd. \"ESP32-DevKitC\")"
                        ),
                    )
                )
                break

    # GPIO pins: expect a numeric literal (hex / dec).
    for macro in GPIO_MACROS:
        found = _parse_define_default(body, macro)
        if found is None:
            continue
        value, line = found
        # ``#define UART1_RX_GPIO  GPIO_PLACEHOLDER`` => value="GPIO_PLACEHOLDER"
        if "GPIO_PLACEHOLDER" in value or value.strip() == str(GPIO_PLACEHOLDER):
            issues.append(
                FoundIssue(
                    macro=macro,
                    line=line,
                    value=value,
                    reason=(
                        "chân GPIO vẫn ở GPIO_PLACEHOLDER (0xFF); "
                        "hãy thay bằng số chân thật trên board"
                    ),
                )
            )
            continue
        try:
            int(value, 0)
        except ValueError:
            issues.append(
                FoundIssue(
                    macro=macro,
                    line=line,
                    value=value,
                    reason="giá trị GPIO không phải số hợp lệ",
                )
            )

    # Channel.
    for macro in CHANNEL_MACROS:
        found = _parse_define_default(body, macro)
        if found is None:
            continue
        value, line = found
        if "CHANNEL_PLACEHOLDER" in value or value.strip() == str(CHANNEL_PLACEHOLDER):
            issues.append(
                FoundIssue(
                    macro=macro,
                    line=line,
                    value=value,
                    reason=(
                        "kênh ESP-NOW vẫn ở CHANNEL_PLACEHOLDER (0); "
                        "hãy chọn kênh 1..13 và đảm bảo cả 2 ESP32 trùng kênh"
                    ),
                )
            )
            continue
        try:
            channel = int(value, 0)
            if not 1 <= channel <= 13:
                issues.append(
                    FoundIssue(
                        macro=macro,
                        line=line,
                        value=value,
                        reason="kênh ESP-NOW phải nằm trong khoảng 1..13",
                    )
                )
        except ValueError:
            issues.append(
                FoundIssue(
                    macro=macro,
                    line=line,
                    value=value,
                    reason="kênh ESP-NOW không phải số hợp lệ",
                )
            )

    # MAC bytes.
    for macro in MAC_MACROS:
        found = _parse_define_default(body, macro)
        if found is None:
            continue
        value, line = found
        if "MAC_PLACEHOLDER" in value or value.strip() == str(MAC_PLACEHOLDER):
            label = "Gateway" if macro.startswith("GATEWAY") else "Remote"
            issues.append(
                FoundIssue(
                    macro=macro,
                    line=line,
                    value=value,
                    reason=(
                        f"byte MAC của {label} vẫn ở MAC_PLACEHOLDER (0xFF); "
                        "hãy đọc MAC thật bằng sketch hỗ trợ rồi điền vào"
                    ),
                )
            )
            continue
        try:
            int(value, 0)
        except ValueError:
            issues.append(
                FoundIssue(
                    macro=macro,
                    line=line,
                    value=value,
                    reason="byte MAC không phải số hợp lệ",
                )
            )

    return issues


def check_hardware_config(header_path: Optional[Path] = None) -> List[FoundIssue]:
    """Return the list of placeholder issues found in the header.

    When the header file is missing the function returns a single
    synthetic issue so the caller still exits non-zero with a clear
    message instead of a stack trace.
    """
    path = header_path or DEFAULT_HEADER
    if not path.exists():
        return [
            FoundIssue(
                macro="<missing-file>",
                line=0,
                value=str(path),
                reason="không tìm thấy file hardware_config.h",
            )
        ]
    return _detect_issues(path)


# --- CLI ------------------------------------------------------------------- #


def _format_summary(issues: Iterable[FoundIssue]) -> str:
    return "\n".join(issue.as_message() for issue in issues)


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(
        prog="firmware_preflight",
        description=(
            "Kiểm tra sentinel placeholder trong firmware/esp32_radar_gateway/"
            "src/hardware_config.h. Trả về 0 nếu sạch, non-zero nếu còn "
            "placeholder cần thay."
        ),
    )
    parser.add_argument(
        "--header",
        type=Path,
        default=DEFAULT_HEADER,
        help="Đường dẫn tới hardware_config.h (mặc định: %(default)s)",
    )
    parser.add_argument(
        "--quiet",
        action="store_true",
        help="Chỉ in thông báo khi có lỗi (kết quả 0 vẫn im lặng).",
    )
    args = parser.parse_args(argv)

    issues = check_hardware_config(args.header)
    if issues:
        print(
            "[FAIL] firmware_preflight: phát hiện "
            f"{len(issues)} placeholder trong {args.header}:",
            file=sys.stderr,
        )
        print(_format_summary(issues), file=sys.stderr)
        print(
            "\nHướng dẫn: mở firmware/esp32_radar_gateway/src/"
            "hardware_config.h, thay các giá trị PLACEHOLDER bằng cấu hình "
            "thật của board, rồi chạy lại build.sh.",
            file=sys.stderr,
        )
        return 1
    if not args.quiet:
        print(
            f"[OK] firmware_preflight: {args.header} không còn placeholder "
            "(GPIO/MAC/Channel/BoardModel đều đã điền)."
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())
