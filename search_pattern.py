"""
Expanding Square Search Pattern
Generates GPS waypoints for a classic maritime/SAR expanding-square pattern.

The pattern starts at `origin`, then spirals outward in a square:
  leg 1: 1×spacing  NORTH
  leg 2: 1×spacing  EAST
  leg 3: 2×spacing  SOUTH
  leg 4: 2×spacing  WEST
  leg 5: 3×spacing  NORTH
  ...
Each pair of perpendicular legs increases the length by one spacing unit.
"""

import math
from typing import Generator, Tuple


EARTH_RADIUS = 6_371_000  # metres


def _offset(lat: float, lon: float, bearing_deg: float, distance_m: float) -> Tuple[float, float]:
    """Return (lat, lon) after travelling `distance_m` on `bearing_deg` from (lat, lon)."""
    R = EARTH_RADIUS
    δ = distance_m / R
    θ = math.radians(bearing_deg)
    φ1 = math.radians(lat)
    λ1 = math.radians(lon)

    φ2 = math.asin(
        math.sin(φ1) * math.cos(δ) +
        math.cos(φ1) * math.sin(δ) * math.cos(θ)
    )
    λ2 = λ1 + math.atan2(
        math.sin(θ) * math.sin(δ) * math.cos(φ1),
        math.cos(δ) - math.sin(φ1) * math.sin(φ2)
    )
    return math.degrees(φ2), math.degrees(λ2)


# Cardinal bearings
BEARINGS = {
    "N": 0,
    "E": 90,
    "S": 180,
    "W": 270,
}

# Expanding-square direction sequence (pairs repeat with growing length)
# N, E, S, S, W, W, N, N, N, E, E, E, ...
# Simpler compact form: each "ring" = [N×k, E×k, S×(k+1), W×(k+1)] where k=1,3,5,…
# We'll use the classic version: direction cycles N→E→S→W, leg length = ceil(i/2)

DIRECTION_CYCLE = ["N", "E", "S", "W"]


class ExpandingSquareSearch:
    """
    Parameters
    ----------
    origin  : (lat, lon) in decimal degrees — centre of the search area
    spacing : distance in metres between parallel legs
    legs    : total number of legs to generate
    """

    def __init__(
        self,
        origin: Tuple[float, float],
        spacing: float = 30.0,
        legs: int = 8,
    ):
        self.origin  = origin
        self.spacing = spacing
        self.legs    = legs

    def waypoints(self) -> Generator[Tuple[float, float], None, None]:
        """Yield (lat, lon) for each waypoint in the search pattern."""
        lat, lon = self.origin
        leg_length = self.spacing  # starts at 1 × spacing

        for i in range(self.legs):
            direction   = DIRECTION_CYCLE[i % 4]
            bearing     = BEARINGS[direction]
            # Leg length increases every 2 legs (classic expanding square)
            leg_distance = leg_length * math.ceil((i + 1) / 2)

            lat, lon = _offset(lat, lon, bearing, leg_distance)
            yield lat, lon

    def visualise(self) -> None:
        """Print a simple ASCII map of the waypoints (debug helper)."""
        wps = list(self.waypoints())
        if not wps:
            return
        lats = [w[0] for w in wps]
        lons = [w[1] for w in wps]
        min_lat, max_lat = min(lats), max(lats)
        min_lon, max_lon = min(lons), max(lons)

        W, H = 60, 30
        print("=" * (W + 4))
        grid = [["." for _ in range(W)] for _ in range(H)]

        def to_grid(lat, lon):
            r = int((max_lat - lat) / (max_lat - min_lat + 1e-9) * (H - 1))
            c = int((lon - min_lon) / (max_lon - min_lon + 1e-9) * (W - 1))
            return max(0, min(H-1, r)), max(0, min(W-1, c))

        # Origin
        r, c = to_grid(*self.origin)
        grid[r][c] = "O"

        for idx, (lt, ln) in enumerate(wps):
            r, c = to_grid(lt, ln)
            grid[r][c] = str(idx % 10)

        for row in grid:
            print("|" + "".join(row) + "|")
        print("=" * (W + 4))
        print(f"O=origin  0-{len(wps)-1}=waypoints")


if __name__ == "__main__":
    # Quick test
    pattern = ExpandingSquareSearch(
        origin=(-35.363261, 149.165230),
        spacing=30,
        legs=8,
    )
    print("Generated waypoints:")
    for i, wp in enumerate(pattern.waypoints()):
        print(f"  WP {i:02d}: {wp[0]:.6f}, {wp[1]:.6f}")
    pattern.visualise()
