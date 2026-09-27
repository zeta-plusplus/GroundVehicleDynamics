#!/usr/bin/env python3
"""Convert a Modelica CombiTable2D text table into an ASCII STL terrain mesh.

Double-click operation:
  1. Put this script and ctrl_TerrainTable2stl.txt in the same folder.
  2. Edit the control file to select the input table and output STL.
  3. Double-click TerrainTable2stl.py.

No third-party packages are required; Python standard library only.

The input table must use Modelica text-table syntax, for example:

#1
double z_xy(59,38)
0  -40  -39.99 ...
-20 0    0      ...
...

The STL contains a triangulated height surface. With a CombiTable2Ds using
LinearSegments, its geometry matches the tabulated nodes and is planar within
individual grid cells.
"""

from __future__ import annotations

import math
import re
import sys
import traceback
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

SCRIPT_DIR = Path(__file__).resolve().parent
CONTROL_FILE = SCRIPT_DIR / "ctrl_TerrainTable2stl.txt"

HEADER_RE = re.compile(
    r"^\s*double\s+([^\s(]+)\s*\(\s*(\d+)\s*,\s*(\d+)\s*\)",
    re.IGNORECASE,
)


@dataclass(frozen=True)
class Settings:
    input_file: Path
    output_file: Path
    table_name: str
    z_scale: float
    close_solid: bool
    base_z: float
    pause_on_exit: bool


def parse_bool(value: str, key: str) -> bool:
    normalized = value.strip().lower()
    if normalized in {"1", "true", "yes", "on"}:
        return True
    if normalized in {"0", "false", "no", "off"}:
        return False
    raise ValueError(f"{key} must be true/false (or 1/0), but is: {value!r}")


def resolve_from_script_directory(value: str) -> Path:
    path = Path(value.strip().strip('"').strip("'"))
    return path if path.is_absolute() else SCRIPT_DIR / path


def read_settings(control_file: Path) -> Settings:
    if not control_file.is_file():
        raise FileNotFoundError(
            f"Control file was not found:\n{control_file}\n\n"
            "Put ctrl_TerrainTable2stl.txt in the same folder as this script."
        )

    values: dict[str, str] = {}
    for line_number, raw_line in enumerate(
        control_file.read_text(encoding="utf-8-sig").splitlines(), start=1
    ):
        line = raw_line.strip()
        if not line or line.startswith("#") or line.startswith("//") or line.startswith(";"):
            continue
        if "=" not in line:
            raise ValueError(
                f"Invalid control-file line {line_number}: {raw_line!r}\n"
                "Use KEY = VALUE."
            )
        key, value = line.split("=", 1)
        key = key.strip().upper()
        value = value.strip()
        if not key:
            raise ValueError(f"Control-file line {line_number} has an empty key.")
        values[key] = value

    required = ("INPUT_FILE", "OUTPUT_FILE", "TABLE_NAME")
    missing = [key for key in required if not values.get(key)]
    if missing:
        raise ValueError(
            "Missing required setting(s): " + ", ".join(missing) + "\n"
            "See the template control file specification below."
        )

    z_scale = float(values.get("Z_SCALE", "1.0"))
    base_z = float(values.get("BASE_Z", "0.0"))
    if not math.isfinite(z_scale) or not math.isfinite(base_z):
        raise ValueError("Z_SCALE and BASE_Z must be finite numbers.")

    return Settings(
        input_file=resolve_from_script_directory(values["INPUT_FILE"]),
        output_file=resolve_from_script_directory(values["OUTPUT_FILE"]),
        table_name=values["TABLE_NAME"].strip(),
        z_scale=z_scale,
        close_solid=parse_bool(values.get("CLOSE_SOLID", "false"), "CLOSE_SOLID"),
        base_z=base_z,
        pause_on_exit=parse_bool(values.get("PAUSE_ON_EXIT", "true"), "PAUSE_ON_EXIT"),
    )


def numeric_tokens(lines: Iterable[str]) -> Iterable[float]:
    for line in lines:
        content = line.split("#", 1)[0].strip()
        if not content:
            continue
        for token in re.split(r"[\s,;]+", content):
            if token:
                yield float(token)


def read_combitable2d(file_name: Path, table_name: str) -> list[list[float]]:
    if not file_name.is_file():
        raise FileNotFoundError(f"Input table file was not found:\n{file_name}")

    lines = file_name.read_text(encoding="utf-8-sig", errors="strict").splitlines()
    requested_name = table_name.strip()
    if not requested_name:
        raise ValueError("TABLE_NAME is empty.")

    for index, line in enumerate(lines):
        match = HEADER_RE.match(line)
        if not match:
            continue
        found_name, nrow_text, ncol_text = match.groups()
        if found_name != requested_name:
            continue

        nrow = int(nrow_text)
        ncol = int(ncol_text)
        if nrow < 3 or ncol < 3:
            raise ValueError(
                f"Table {requested_name!r} is {nrow} x {ncol}. "
                "At least 3 x 3 is required."
            )

        count = nrow * ncol
        values: list[float] = []
        for value in numeric_tokens(lines[index + 1 :]):
            values.append(value)
            if len(values) == count:
                break

        if len(values) != count:
            raise ValueError(
                f"Table {requested_name!r} declares {nrow} x {ncol} = {count} values, "
                f"but only {len(values)} numeric values were read after its header."
            )

        table = [values[row * ncol : (row + 1) * ncol] for row in range(nrow)]
        validate_table(table, requested_name)
        return table

    found = []
    for line in lines:
        match = HEADER_RE.match(line)
        if match:
            found.append(match.group(1))
    suffix = "\nAvailable table names: " + ", ".join(found) if found else ""
    raise ValueError(
        f"Table {requested_name!r} was not found in:\n{file_name}{suffix}"
    )


def validate_table(table: list[list[float]], table_name: str) -> None:
    nrow = len(table)
    ncol = len(table[0])
    for row in table:
        if len(row) != ncol:
            raise ValueError(f"Table {table_name!r} has inconsistent row lengths.")
        if not all(math.isfinite(value) for value in row):
            raise ValueError(f"Table {table_name!r} contains NaN or infinity.")

    u1 = [table[i][0] for i in range(1, nrow)]
    u2 = [table[0][j] for j in range(1, ncol)]
    for axis_name, values in (("u1 (first column)", u1), ("u2 (first row)", u2)):
        for k, (a, b) in enumerate(zip(values, values[1:]), start=1):
            if not b > a:
                raise ValueError(
                    f"Table {table_name!r}: {axis_name} must be strictly increasing; "
                    f"value {k} is {a:g}, next value is {b:g}."
                )


def point(table: list[list[float]], i: int, j: int, z_scale: float) -> tuple[float, float, float]:
    return table[i + 1][0], table[0][j + 1], z_scale * table[i + 1][j + 1]


def normal(a: tuple[float, float, float], b: tuple[float, float, float], c: tuple[float, float, float]) -> tuple[float, float, float]:
    ux, uy, uz = b[0] - a[0], b[1] - a[1], b[2] - a[2]
    vx, vy, vz = c[0] - a[0], c[1] - a[1], c[2] - a[2]
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    magnitude = math.sqrt(nx * nx + ny * ny + nz * nz)
    if magnitude <= 1e-300:
        return 0.0, 0.0, 1.0
    return nx / magnitude, ny / magnitude, nz / magnitude


def write_facet(handle, a: tuple[float, float, float], b: tuple[float, float, float], c: tuple[float, float, float]) -> None:
    nx, ny, nz = normal(a, b, c)
    handle.write(f"  facet normal {nx:.17g} {ny:.17g} {nz:.17g}\n")
    handle.write("    outer loop\n")
    for x, y, z in (a, b, c):
        handle.write(f"      vertex {x:.17g} {y:.17g} {z:.17g}\n")
    handle.write("    endloop\n")
    handle.write("  endfacet\n")


def write_top_surface(handle, table: list[list[float]], z_scale: float) -> int:
    nu = len(table) - 1
    nv = len(table[0]) - 1
    facets = 0
    for i in range(nu - 1):
        for j in range(nv - 1):
            p00 = point(table, i, j, z_scale)
            p10 = point(table, i + 1, j, z_scale)
            p11 = point(table, i + 1, j + 1, z_scale)
            p01 = point(table, i, j + 1, z_scale)
            write_facet(handle, p00, p10, p11)
            write_facet(handle, p00, p11, p01)
            facets += 2
    return facets


def at_base(p: tuple[float, float, float], base_z: float) -> tuple[float, float, float]:
    return p[0], p[1], base_z


def write_closed_boundary(handle, table: list[list[float]], z_scale: float, base_z: float) -> int:
    nu = len(table) - 1
    nv = len(table[0]) - 1
    facets = 0

    for i in range(nu - 1):
        a, b = point(table, i, 0, z_scale), point(table, i + 1, 0, z_scale)
        a0, b0 = at_base(a, base_z), at_base(b, base_z)
        write_facet(handle, a, b, b0)
        write_facet(handle, a, b0, a0)
        facets += 2

        a, b = point(table, i, nv - 1, z_scale), point(table, i + 1, nv - 1, z_scale)
        a0, b0 = at_base(a, base_z), at_base(b, base_z)
        write_facet(handle, b, a, a0)
        write_facet(handle, b, a0, b0)
        facets += 2

    for j in range(nv - 1):
        a, b = point(table, 0, j, z_scale), point(table, 0, j + 1, z_scale)
        a0, b0 = at_base(a, base_z), at_base(b, base_z)
        write_facet(handle, b, a, a0)
        write_facet(handle, b, a0, b0)
        facets += 2

        a, b = point(table, nu - 1, j, z_scale), point(table, nu - 1, j + 1, z_scale)
        a0, b0 = at_base(a, base_z), at_base(b, base_z)
        write_facet(handle, a, b, b0)
        write_facet(handle, a, b0, a0)
        facets += 2

    for i in range(nu - 1):
        for j in range(nv - 1):
            p00 = at_base(point(table, i, j, z_scale), base_z)
            p10 = at_base(point(table, i + 1, j, z_scale), base_z)
            p11 = at_base(point(table, i + 1, j + 1, z_scale), base_z)
            p01 = at_base(point(table, i, j + 1, z_scale), base_z)
            write_facet(handle, p11, p10, p00)
            write_facet(handle, p01, p11, p00)
            facets += 2

    return facets


def write_stl(table: list[list[float]], settings: Settings) -> tuple[int, int, int]:
    settings.output_file.parent.mkdir(parents=True, exist_ok=True)
    temporary_file = settings.output_file.with_suffix(settings.output_file.suffix + ".tmp")
    solid_name = re.sub(r"[^A-Za-z0-9_.-]+", "_", settings.table_name) or "terrain"

    nrow, ncol = len(table), len(table[0])
    with temporary_file.open("w", encoding="ascii", newline="\n") as handle:
        handle.write(f"solid terrain_{solid_name}\n")
        facets = write_top_surface(handle, table, settings.z_scale)
        if settings.close_solid:
            facets += write_closed_boundary(handle, table, settings.z_scale, settings.base_z)
        handle.write(f"endsolid terrain_{solid_name}\n")

    temporary_file.replace(settings.output_file)
    return nrow - 1, ncol - 1, facets


def pause_if_requested(settings: Settings | None) -> None:
    should_pause = True if settings is None else settings.pause_on_exit
    if should_pause and sys.stdin.isatty():
        try:
            input("\nPress Enter to close this window...")
        except EOFError:
            pass


def main() -> int:
    settings: Settings | None = None
    try:
        settings = read_settings(CONTROL_FILE)
        table = read_combitable2d(settings.input_file, settings.table_name)
        nu, nv, facets = write_stl(table, settings)
        print("TerrainTable2stl completed successfully.")
        print(f"Input table : {settings.input_file}")
        print(f"Table name  : {settings.table_name}")
        print(f"Grid points : {nu} (u1) x {nv} (u2)")
        print(f"Facets      : {facets}")
        print(f"Output STL  : {settings.output_file}")
        return 0
    except Exception as exc:
        print("TerrainTable2stl failed.\n", file=sys.stderr)
        print(str(exc), file=sys.stderr)
        print("\nDetailed diagnostic information:\n", file=sys.stderr)
        traceback.print_exc()
        return 1
    finally:
        pause_if_requested(settings)


if __name__ == "__main__":
    raise SystemExit(main())
