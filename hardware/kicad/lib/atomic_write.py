# Adapted from mcp-server-kicad (ProductOfAmerica), MIT License.
# https://github.com/ProductOfAmerica/mcp-server-kicad
# See hardware/kicad/lib/THIRD_PARTY_LICENSES.md for the license text and provenance.
"""Small same-directory atomic-write helper for generated KiCad artifacts."""

from __future__ import annotations

import os
import shutil
import tempfile
import time
from pathlib import Path

_REPLACE_RETRY_DELAYS = (0.05, 0.1, 0.2)


def atomic_write(path: str | Path, data: bytes | str, *, encoding: str = "utf-8") -> None:
    """Write *data* to *path* via a temp file in the same directory and os.replace()."""
    destination = Path(path)
    payload = data if isinstance(data, bytes) else str(data).encode(encoding)
    parent = destination.parent
    fd, temp_name = tempfile.mkstemp(
        prefix=f"{destination.name}.",
        suffix=".tmp",
        dir=str(parent),
    )
    temp_path = Path(temp_name)
    try:
        with os.fdopen(fd, "wb") as handle:
            handle.write(payload)
        if destination.exists():
            shutil.copymode(destination, temp_path)
        for delay in _REPLACE_RETRY_DELAYS:
            try:
                os.replace(temp_path, destination)
                return
            except PermissionError:
                time.sleep(delay)
        try:
            os.replace(temp_path, destination)
        except PermissionError as exc:
            raise OSError(
                f"could not replace {destination}: it is open in another program or held by a sync/antivirus client"
            ) from exc
    except BaseException:
        try:
            temp_path.chmod(0o600)
        except OSError:
            pass
        try:
            temp_path.unlink(missing_ok=True)
        except OSError:
            pass
        raise


__all__ = ["atomic_write"]
