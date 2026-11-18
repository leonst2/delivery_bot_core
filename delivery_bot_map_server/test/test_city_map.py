"""Format checks for the static city map in maps/city (own map format, #12).

Stands in for `dbot-map validate` until the map tools exist. Contract: map.pgm (binary P5, 8 bit,
0 = occupied, 254 = free, 205 = unknown) + map.yaml (image, resolution, origin [x, y, yaw], negate 0,
occupied_thresh, free_thresh). The map frame origin is the Unity world origin, axes ROS FLU.
"""

from pathlib import Path

import pytest

MAP_DIR = Path(__file__).resolve().parent.parent / 'maps' / 'city'
ALLOWED_VALUES = {0, 205, 254}
REQUIRED_KEYS = {'image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh'}


def read_yaml(path):
    """Read the flat key: value map YAML without a YAML dependency."""
    values = {}
    for line in path.read_text().splitlines():
        line = line.split('#', 1)[0].strip()
        if line:
            key, value = (part.strip() for part in line.split(':', 1))
            values[key] = value
    return values


def read_origin(meta):
    return [float(v) for v in meta['origin'].strip('[]').split(',')]


def read_pgm(path):
    """Return (width, height, pixels) of a binary PGM; pixel row 0 is the top image line (highest y)."""
    data = path.read_bytes()
    magic, size, maxval, pixels = data.split(b'\n', 3)
    assert magic == b'P5', f'{path.name}: not a binary PGM'
    assert maxval == b'255', f'{path.name}: maxval must be 255'
    width, height = (int(v) for v in size.split())
    assert len(pixels) == width * height, f'{path.name}: pixel data does not match {width}x{height}'
    return width, height, pixels


def pixel_at(meta, width, height, pixels, x, y):
    """Value of the cell containing map point (x, y)."""
    resolution = float(meta['resolution'])
    origin_x, origin_y, _ = read_origin(meta)
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    assert 0 <= col < width and 0 <= row < height, f'({x}, {y}) is outside the map'
    return pixels[(height - 1 - row) * width + col]


def map_files():
    return [MAP_DIR / 'map.yaml', *sorted((MAP_DIR / 'tiles').glob('*.yaml'))]


@pytest.mark.parametrize('yaml_path', map_files(), ids=lambda p: p.name)
def test_map_matches_contract(yaml_path):
    meta = read_yaml(yaml_path)
    assert REQUIRED_KEYS <= meta.keys()
    assert float(meta['resolution']) == pytest.approx(0.05)
    assert meta['negate'] == '0'
    origin = read_origin(meta)
    assert len(origin) == 3 and origin[2] == 0.0
    # 205 (unknown) is p = 0.196 and must not read as free; 254 (free) is p = 0.004.
    assert 0.004 < float(meta['free_thresh']) < 0.196 < float(meta['occupied_thresh'])

    width, height, pixels = read_pgm(yaml_path.parent / meta['image'])
    assert set(pixels) <= ALLOWED_VALUES


@pytest.mark.parametrize('yaml_path', map_files()[1:], ids=lambda p: p.name)
def test_tiles_are_aligned_to_the_city_map(yaml_path):
    city = read_yaml(MAP_DIR / 'map.yaml')
    resolution = float(city['resolution'])
    city_x, city_y, _ = read_origin(city)
    tile_x, tile_y, _ = read_origin(read_yaml(yaml_path))
    for offset in (tile_x - city_x, tile_y - city_y):
        cells = offset / resolution
        assert offset >= 0 and cells == pytest.approx(round(cells), abs=1e-6)


def test_city_frame():
    """Spot checks against the Unity scene (map x = Unity +Z, map y = Unity -X)."""
    meta = read_yaml(MAP_DIR / 'map.yaml')
    grid = read_pgm(MAP_DIR / meta['image'])
    # Unity world origin is the central intersection: free road.
    assert pixel_at(meta, *grid, 0.0, 0.0) == 254
    # House_0_A spans Unity x [9, 27], z [9, 24] -> map x [9, 24], y [-27, -9]; its centre is occupied.
    assert pixel_at(meta, *grid, 16.5, -18.0) == 0
