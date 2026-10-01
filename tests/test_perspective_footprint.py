"""
Verify the perspective ground footprint and tile selection fix.
Tests that the camera tilt is properly accounted for in tile selection
and that the ground footprint is trapezoidal at oblique angles.
"""
import math
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from orion_shadow.engine.visualizer import TileVisualizer


def test_nadir_footprint_is_symmetric():
    """At -90° tilt (straight down), the footprint should be nearly rectangular."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    fp = viz.compute_footprint(
        lat=40.0, lon=-74.0, alt=1000.0,
        cam_hdg=0.0, cam_pitch=-90.0,
        hfov=47.7, vfov=35.8, zoom=15
    )
    assert fp is not None
    corners = fp['corners_latlon']
    # At nadir, TL and TR should be at roughly the same latitude (both far/near edges equal)
    tl_lat, tr_lat = corners[0][0], corners[1][0]
    bl_lat, br_lat = corners[2][0], corners[3][0]
    # Top edge and bottom edge should be symmetric about center
    top_width = abs(corners[1][1] - corners[0][1])
    bot_width = abs(corners[3][1] - corners[2][1])
    # At nadir, top and bottom widths should be nearly equal (rectangle, not trapezoid)
    assert abs(top_width - bot_width) / max(top_width, bot_width) < 0.05, \
        f"Nadir footprint should be nearly rectangular: top_w={top_width:.6f}, bot_w={bot_width:.6f}"


def test_oblique_footprint_is_trapezoidal():
    """At -45° tilt, the far edge (top of image) should be wider than the near edge."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    fp = viz.compute_footprint(
        lat=40.0, lon=-74.0, alt=1000.0,
        cam_hdg=0.0, cam_pitch=-45.0,  # Looking 45° below horizon
        hfov=47.7, vfov=35.8, zoom=15
    )
    assert fp is not None
    corners = fp['corners_latlon']

    # Top of image = far ground (TL, TR), bottom = near ground (BL, BR)
    far_width = abs(corners[1][1] - corners[0][1])   # TR.lon - TL.lon
    near_width = abs(corners[3][1] - corners[2][1])   # BR.lon - BL.lon

    # At -45° tilt, far edge should be significantly wider than near edge
    assert far_width > near_width * 1.5, \
        f"Far edge ({far_width:.6f}°) should be much wider than near edge ({near_width:.6f}°)"


def test_oblique_footprint_far_edge_farther_north():
    """At -45° tilt looking north, the far edge should be farther north."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    fp = viz.compute_footprint(
        lat=40.0, lon=-74.0, alt=1000.0,
        cam_hdg=0.0, cam_pitch=-45.0,  # Looking north, 45° below horizon
        hfov=47.7, vfov=35.8, zoom=15
    )
    assert fp is not None
    corners = fp['corners_latlon']

    # Top of image (far ground) should be farther north than bottom (near ground)
    far_lat = (corners[0][0] + corners[1][0]) / 2.0  # average of TL, TR
    near_lat = (corners[2][0] + corners[3][0]) / 2.0  # average of BL, BR

    assert far_lat > near_lat, \
        f"Far edge lat ({far_lat:.6f}) should be > near edge lat ({near_lat:.6f}) when looking north"

    # Both should be north of aircraft position
    assert far_lat > 40.0
    assert near_lat > 40.0


def test_oblique_fetches_more_tiles_than_nadir():
    """An oblique view should need more tiles than a nadir view at the same zoom."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")

    nadir_fp = viz.compute_footprint(
        lat=40.0, lon=-74.0, alt=1000.0,
        cam_hdg=0.0, cam_pitch=-90.0,
        hfov=47.7, vfov=35.8, zoom=15
    )
    oblique_fp = viz.compute_footprint(
        lat=40.0, lon=-74.0, alt=1000.0,
        cam_hdg=0.0, cam_pitch=-45.0,
        hfov=47.7, vfov=35.8, zoom=15
    )
    assert nadir_fp is not None and oblique_fp is not None
    assert len(oblique_fp['tiles']) > len(nadir_fp['tiles']), \
        f"Oblique ({len(oblique_fp['tiles'])} tiles) should need more tiles than nadir ({len(nadir_fp['tiles'])} tiles)"


def test_tilt_ignored_old_get_visible_tiles():
    """The old get_visible_tiles always returns the same tiles regardless of tilt — 
    verify compute_footprint does NOT have this bug."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")

    # Old method: always same result
    tiles_nadir_old = viz.get_visible_tiles(40.0, -74.0, 0.0, -90.0, 15)
    tiles_oblique_old = viz.get_visible_tiles(40.0, -74.0, 0.0, -45.0, 15)
    assert tiles_nadir_old == tiles_oblique_old, "Old method should be tilt-agnostic (known bug)"

    # New method: different results for different tilts
    fp_nadir = viz.compute_footprint(40.0, -74.0, 1000.0, 0.0, -90.0, 47.7, 35.8, 15)
    fp_oblique = viz.compute_footprint(40.0, -74.0, 1000.0, 0.0, -45.0, 47.7, 35.8, 15)
    assert fp_nadir['tiles'] != fp_oblique['tiles'], \
        "compute_footprint MUST produce different tiles for different tilt angles"


def test_heading_rotates_footprint():
    """Changing heading should rotate the footprint on the ground."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")

    fp_north = viz.compute_footprint(
        lat=40.0, lon=-74.0, alt=1000.0,
        cam_hdg=0.0, cam_pitch=-45.0,
        hfov=47.7, vfov=35.8, zoom=15
    )
    fp_east = viz.compute_footprint(
        lat=40.0, lon=-74.0, alt=1000.0,
        cam_hdg=90.0, cam_pitch=-45.0,
        hfov=47.7, vfov=35.8, zoom=15
    )
    assert fp_north is not None and fp_east is not None

    # Looking north: far edge should be northward (higher lat)
    far_north = (fp_north['corners_latlon'][0][0] + fp_north['corners_latlon'][1][0]) / 2
    assert far_north > 40.0, "Looking north, far edge should be north of aircraft"

    # Looking east: far edge should be eastward (higher lon)
    far_east_lon = (fp_east['corners_latlon'][0][1] + fp_east['corners_latlon'][1][1]) / 2
    assert far_east_lon > -74.0, "Looking east, far edge should be east of aircraft"


def test_zero_altitude_returns_none():
    """Footprint should return None when altitude is 0."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    fp = viz.compute_footprint(
        lat=40.0, lon=-74.0, alt=0.0,
        cam_hdg=0.0, cam_pitch=-45.0,
        hfov=47.7, vfov=35.8, zoom=15
    )
    assert fp is None


def test_corner_ray_ned_straight_down():
    """With cam_pitch=-90, boresight should point straight down (D > 0, N≈0, E≈0)."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    rays = viz._corner_rays_ned(cam_hdg=0.0, cam_pitch=-90.0, hfov=47.7, vfov=35.8)
    # All 4 rays should have large D component (pointing down)
    for i, (n, e, d) in enumerate(rays):
        assert d > 0.5, f"Corner {i}: ray should point mostly down, got D={d:.3f}"


def test_corner_ray_ned_looking_north_oblique():
    """With cam_hdg=0, cam_pitch=-45, boresight should have N>0 and D>0."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    rays = viz._corner_rays_ned(cam_hdg=0.0, cam_pitch=-45.0, hfov=10.0, vfov=10.0)
    # With small FOV, all rays should roughly point north-and-down
    for i, (n, e, d) in enumerate(rays):
        assert n > 0.3, f"Corner {i}: should have strong N component when looking north, got N={n:.3f}"
        assert d > 0.3, f"Corner {i}: should point down at -45° pitch, got D={d:.3f}"


def test_adaptive_zoom_increases_with_camera_zoom():
    """Higher camera zoom (narrower FOV) should produce higher tile zoom for sharper detail."""
    from orion_shadow.engine.video_server import VideoServer
    from orion_shadow.core.state import GimbalState

    state = GimbalState()
    server = VideoServer(state)

    # Wide FOV (zoom 1x, hfov ≈ 47.7°) at 1000m altitude, -45° tilt
    z_wide = server._compute_tile_zoom(lat=40.0, alt=1000.0, cam_pitch=-45.0, hfov=47.7)

    # Narrow FOV (zoom 10x, hfov ≈ 4.77°) at same conditions
    z_narrow = server._compute_tile_zoom(lat=40.0, alt=1000.0, cam_pitch=-45.0, hfov=4.77)

    assert z_narrow > z_wide, \
        f"Narrow FOV zoom ({z_narrow}) should be higher than wide FOV zoom ({z_wide})"
    assert z_wide >= 15, f"Minimum zoom should be 15, got {z_wide}"
    assert z_narrow <= 19, f"Maximum zoom should be 19, got {z_narrow}"


if __name__ == '__main__':
    tests = [
        test_nadir_footprint_is_symmetric,
        test_oblique_footprint_is_trapezoidal,
        test_oblique_footprint_far_edge_farther_north,
        test_oblique_fetches_more_tiles_than_nadir,
        test_tilt_ignored_old_get_visible_tiles,
        test_heading_rotates_footprint,
        test_zero_altitude_returns_none,
        test_corner_ray_ned_straight_down,
        test_corner_ray_ned_looking_north_oblique,
        test_adaptive_zoom_increases_with_camera_zoom,
    ]
    for t in tests:
        try:
            t()
            print(f"  PASS: {t.__name__}")
        except AssertionError as e:
            print(f"  FAIL: {t.__name__}: {e}")
        except Exception as e:
            print(f"  ERROR: {t.__name__}: {e}")
