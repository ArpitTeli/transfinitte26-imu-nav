"""Extract one EuRoC sequence from the downloaded Machine Hall archive.

Download "Machine Hall Datasets (ZIP)" by hand from the ETH Research Collection
(https://www.research-collection.ethz.ch, EuRoC MAV dataset, DOI 10.3929/ethz-b-000690084),
then run:
    python scripts/extract_euroc.py path/to/machine_hall.zip MH_01_easy
Only the members for that sequence are extracted, to data/euroc/<name>/. A nested
<name>.zip inside the archive is unpacked too. Result: data/euroc/<name>/mav0/imu0/data.csv
"""
import io
import sys
import zipfile
from pathlib import Path, PurePosixPath


def extract(archive, name, dest_root="data/euroc"):
    dest = Path(dest_root) / name
    with zipfile.ZipFile(archive) as z:
        members = [m for m in z.namelist() if name in PurePosixPath(m).parts or PurePosixPath(m).name == f"{name}.zip"]
        if not members:
            raise SystemExit(f"{name} not found in {archive}")
        inner = [m for m in members if m.endswith(f"{name}.zip")]
        if inner:
            with zipfile.ZipFile(io.BytesIO(z.read(inner[0]))) as zi:
                _extract_mav0(zi, zi.namelist(), dest)
        else:
            _extract_mav0(z, members, dest)
    csv = dest / "mav0" / "imu0" / "data.csv"
    if not csv.exists():
        raise SystemExit(f"extracted, but {csv} is missing; check the archive layout")
    return csv


def _extract_mav0(z, members, dest):
    """Write members below their 'mav0' component to dest/mav0/..., skipping camera images."""
    for m in members:
        parts = PurePosixPath(m).parts
        if "mav0" not in parts or m.endswith("/") or "cam0" in parts or "cam1" in parts:
            continue
        rel = Path(*parts[parts.index("mav0"):])
        out = dest / rel
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_bytes(z.read(m))


if __name__ == "__main__":
    if len(sys.argv) < 2:
        raise SystemExit(__doc__)
    print("ready:", extract(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "MH_01_easy"))
