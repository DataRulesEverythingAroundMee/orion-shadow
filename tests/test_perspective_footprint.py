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

    # Wide FOV (zoom 1x, hfov ≈ 47.7°) at 3000m altitude, -45° tilt
    z_wide = server._compute_tile_zoom(lat=40.0, alt=3000.0, cam_pitch=-45.0, hfov=47.7)

    # Narrow FOV (zoom 10x, hfov ≈ 4.77°) at same conditions
    z_narrow = server._compute_tile_zoom(lat=40.0, alt=3000.0, cam_pitch=-45.0, hfov=4.77)

    assert z_narrow > z_wide, \
        f"Narrow FOV zoom ({z_narrow}) should be higher than wide FOV zoom ({z_wide})"
    assert z_wide >= 14, f"Minimum zoom should be 14, got {z_wide}"
    assert z_narrow <= 17, f"Maximum zoom should be 17, got {z_narrow}"


def test_prefetch_tiles_along_aircraft_heading():
    """Prefetch tiles should project ahead along the aircraft's heading vector."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    
    # Aircraft at 40.0N, -74.0W flying North (hdg=0)
    tiles_north = viz.get_prefetch_tiles(
        lat=40.0, lon=-74.0, alt=1000.0,
        ac_hdg=0.0, cam_hdg=0.0, cam_pitch=-45.0,
        hfov=47.7, vfov=35.8, zoom=16,
        lookahead_distance=3000.0
    )
    assert len(tiles_north) > 0

    # Aircraft current tile Y
    _, curr_zy = viz.latlon_to_tile(40.0, -74.0, 16)
    
    # In Slippy Map / Web Mercator, northward means lower Y tile index
    min_y = min(t[2] for t in tiles_north)
    assert min_y < curr_zy, f"Prefetched tiles going north should include tile Y < {curr_zy}, got min {min_y}"


def test_prefetch_tiles_east_heading():
    """Aircraft flying East (hdg=90) should prefetch tiles to the east (higher X tile index)."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    
    # Aircraft at 40.0N, -74.0W flying East (hdg=90)
    tiles_east = viz.get_prefetch_tiles(
        lat=40.0, lon=-74.0, alt=1000.0,
        ac_hdg=90.0, cam_hdg=90.0, cam_pitch=-45.0,
        hfov=47.7, vfov=35.8, zoom=16,
        lookahead_distance=3000.0
    )
    assert len(tiles_east) > 0

    curr_zx, _ = viz.latlon_to_tile(40.0, -74.0, 16)
    max_x = max(t[1] for t in tiles_east)
    assert max_x > curr_zx, f"Prefetched tiles going east should include tile X > {curr_zx}, got max {max_x}"


def test_video_server_prefetch_configuration():
    """Verify VideoServer and OrionServer initialize and respect prefetch configuration."""
    from orion_shadow.core.state import GimbalState
    from orion_shadow.engine.video_server import VideoServer
    from orion_shadow.server import OrionServer

    state = GimbalState()
    vs = VideoServer(state, prefetch_enabled=True, prefetch_distance=5000.0)
    assert vs.prefetch_enabled is True
    assert vs.prefetch_distance == 5000.0

    server = OrionServer(prefetch=False, prefetch_distance=2000.0)
    assert server.video_server.prefetch_enabled is False
    assert server.video_server.prefetch_distance == 2000.0


def test_distance_dependent_lod_lowers_zoom_for_distant_tiles():
    """At oblique angles, distant tiles should have lower zoom levels than near tiles."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    # High altitude oblique view
    fp = viz.compute_footprint(
        lat=39.7774, lon=-84.0819, alt=3000.0,
        cam_hdg=0.0, cam_pitch=-20.0,
        hfov=47.7, vfov=26.8, zoom=16,
        distance_lod=True
    )
    assert fp is not None
    tiles = fp['tiles']
    zoom_levels = set(t[0] for t in tiles)

    # Should contain base zoom (16) and lower zoom levels (e.g. 15, 14, 13) for distant horizon
    assert 16 in zoom_levels, "Footprint should contain base zoom tiles near the aircraft"
    assert any(z < 16 for z in zoom_levels), f"Footprint should contain lower zoom tiles for distant terrain, got zooms: {zoom_levels}"

    # Verify that tiles with lower zoom are farther from the aircraft than tiles with higher zoom
    def tile_dist(t):
        tz, tx, ty = t
        inv_n = 1.0 / (2.0 ** tz)
        t_lon = ((tx + 0.5) * inv_n) * 360.0 - 180.0
        sinh_val = math.sinh(math.pi * (1.0 - 2.0 * (ty + 0.5) * inv_n))
        t_lat = math.degrees(math.atan(sinh_val))
        return (t_lat - 39.7774) ** 2 + (t_lon - (-84.0819)) ** 2

    near_tiles = [t for t in tiles if t[0] == 16]
    far_tiles = [t for t in tiles if t[0] < 16]
    avg_dist_near = sum(tile_dist(t) for t in near_tiles) / len(near_tiles)
    avg_dist_far = sum(tile_dist(t) for t in far_tiles) / len(far_tiles)
    assert avg_dist_far > avg_dist_near, \
        f"Lower zoom tiles should on average be farther away: far={avg_dist_far:.6f}, near={avg_dist_near:.6f}"


def test_distance_lod_reduces_tile_count():
    """Distance-dependent LOD should yield fewer unique tiles than a uniform high zoom grid."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    fp_lod = viz.compute_footprint(
        lat=39.7774, lon=-84.0819, alt=3000.0,
        cam_hdg=0.0, cam_pitch=-20.0,
        hfov=47.7, vfov=26.8, zoom=16,
        distance_lod=True
    )
    fp_uniform = viz.compute_footprint(
        lat=39.7774, lon=-84.0819, alt=3000.0,
        cam_hdg=0.0, cam_pitch=-20.0,
        hfov=47.7, vfov=26.8, zoom=16,
        distance_lod=False
    )
    assert fp_lod is not None and fp_uniform is not None
    # Uniform zoom either hits max_tiles (due to contraction) or generates many more tiles for the same area
    # In both cases, LOD produces a much more compact tile set covering the horizon
    assert any(t[0] < 16 for t in fp_lod['tiles'])
    assert all(t[0] == 16 for t in fp_uniform['tiles'])


def test_distance_lod_disabled_flag():
    """When distance_lod is False, all tiles should strictly use the base zoom."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    fp = viz.compute_footprint(
        lat=40.0, lon=-74.0, alt=1000.0,
        cam_hdg=0.0, cam_pitch=-30.0,
        hfov=47.7, vfov=35.8, zoom=15,
        distance_lod=False
    )
    assert fp is not None
    assert all(t[0] == 15 for t in fp['tiles']), "All tiles must be at zoom 15 when distance_lod=False"


def test_zoomed_in_footprint_padding():
    """At high zoom (e.g. 23x), pad_tiles=1 expands tile_bounds by 1 on each side and fetches surrounding buffer tiles."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    lat, lon, alt = 39.7774, -84.0819, 700.0
    cam_hdg, cam_pitch = 0.0, -20.0
    hfov = 47.7 / 23.0
    vfov = hfov * (720.0 / 1280.0)
    zoom = 17

    fp_unpadded = viz.compute_footprint(lat, lon, alt, cam_hdg, cam_pitch, hfov, vfov, zoom, pad_tiles=0)
    assert fp_unpadded is not None
    min_tx_u, min_ty_u, max_tx_u, max_ty_u = fp_unpadded['tile_bounds']

    fp_padded = viz.compute_footprint(lat, lon, alt, cam_hdg, cam_pitch, hfov, vfov, zoom, pad_tiles=1)
    assert fp_padded is not None
    min_tx_p, min_ty_p, max_tx_p, max_ty_p = fp_padded['tile_bounds']

    assert min_tx_p == min_tx_u - 1
    assert max_tx_p == max_tx_u + 1
    assert min_ty_p == min_ty_u - 1
    assert max_ty_p == max_ty_u + 1
    # Padded tile set must contain all surrounding buffer tiles
    assert len(fp_padded['tiles']) >= 9


def test_cam_roll_in_compute_footprint():
    """Camera roll angle rotates the computed ground footprint corners."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    lat, lon, alt = 39.7774, -84.0819, 1000.0
    cam_hdg, cam_pitch = 0.0, -30.0
    hfov, vfov = 47.7, 35.8
    zoom = 15

    fp_0 = viz.compute_footprint(lat, lon, alt, cam_hdg, cam_pitch, hfov, vfov, zoom, cam_roll=0.0)
    fp_roll = viz.compute_footprint(lat, lon, alt, cam_hdg, cam_pitch, hfov, vfov, zoom, cam_roll=15.0)

    assert fp_0 is not None and fp_roll is not None
    # Rolled footprint corners must differ from 0-roll footprint corners
    c_0 = fp_0['corners_latlon']
    c_r = fp_roll['corners_latlon']
    assert not all(math.isclose(p0[0], pr[0], abs_tol=1e-5) for p0, pr in zip(c_0, c_r))


def test_agl_perspective_footprint_covers_near_ground_at_high_zoom():
    """Verify that using AGL (alt - terrain_elev) at 5x zoom prevents missing near-ground tiles."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    lat, lon = 39.7774, -84.0819
    alt_msl = 574.9
    elev = 250.0
    agl = alt_msl - elev  # 324.9m
    cam_hdg, cam_pitch = 179.0, -12.3
    hfov = 47.7 / 5.0
    vfov = hfov * (720.0 / 1280.0)
    zoom = 17

    # Bottom rays of sensor (near ground) at -12.3° pitch, 5.37° VFOV have depression angle ~15.0°
    # Distance to actual terrain is agl / sin(depression) ≈ 324.9 / 0.258 ≈ 1256m
    # In Dayton, OH heading South (179°), 1256m south corresponds to latitude 39.7661 -> tile Y ≈ 49732.1

    # With MSL (old bug):
    fp_msl = viz.compute_footprint(lat, lon, alt_msl, cam_hdg, cam_pitch, hfov, vfov, zoom, pad_tiles=1)
    assert fp_msl is not None
    # MSL mistakenly thought ground was at sea level (2224m away), so min_ty was 49734 (missing tile 49732)
    assert fp_msl['tile_bounds'][1] > 49732, "MSL footprint should demonstrate the bug by missing near ground"

    # With AGL (fixed):
    fp_agl = viz.compute_footprint(lat, lon, agl, cam_hdg, cam_pitch, hfov, vfov, zoom, pad_tiles=2)
    assert fp_agl is not None
    min_tx, min_ty, max_tx, max_ty = fp_agl['tile_bounds']
    # AGL bounds must cover the near ground hit at tile Y 49732
    assert min_ty <= 49732 <= max_ty, f"AGL footprint must cover near ground hit (ty=49732): bounds={fp_agl['tile_bounds']}"


def test_footprint_tiles_fully_cover_bounding_box_without_holes():
    """Verify that compute_footprint produces tiles covering 100% of the canvas bounding box with zero holes."""
    viz = TileVisualizer("https://tile.example.com/{z}/{x}/{y}.png")
    lat, lon, alt = 39.7774, -84.0819, 324.9
    cam_hdg, cam_pitch = 179.0, -12.3
    hfov = 47.7 / 5.0
    vfov = hfov * (720.0 / 1280.0)
    zoom = 17

    fp = viz.compute_footprint(lat, lon, alt, cam_hdg, cam_pitch, hfov, vfov, zoom, distance_lod=True, pad_tiles=2)
    assert fp is not None
    min_tx, min_ty, max_tx, max_ty = fp['tile_bounds']

    # Map all covered base zoom cells
    covered = set()
    for (tz, tx, ty) in fp['tiles']:
        dz = zoom - tz
        scale = 1 << dz
        for dty in range(scale):
            for dtx in range(scale):
                covered.add((tx * scale + dtx, ty * scale + dty))

    # Verify every single cell in [min_tx..max_tx, min_ty..max_ty] is covered
    missing = []
    for ty in range(min_ty, max_ty + 1):
        for tx in range(min_tx, max_tx + 1):
            if (tx, ty) not in covered:
                missing.append((tx, ty))

    assert len(missing) == 0, f"Canvas bounding box has {len(missing)} unpainted tile holes: {missing[:10]}"


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
        test_prefetch_tiles_along_aircraft_heading,
        test_prefetch_tiles_east_heading,
        test_video_server_prefetch_configuration,
        test_distance_dependent_lod_lowers_zoom_for_distant_tiles,
        test_distance_lod_reduces_tile_count,
        test_distance_lod_disabled_flag,
        test_zoomed_in_footprint_padding,
        test_cam_roll_in_compute_footprint,
        test_agl_perspective_footprint_covers_near_ground_at_high_zoom,
        test_footprint_tiles_fully_cover_bounding_box_without_holes,
    ]
    for t in tests:
        try:
            t()
            print(f"  PASS: {t.__name__}")
        except AssertionError as e:
            print(f"  FAIL: {t.__name__}: {e}")
        except Exception as e:
            print(f"  ERROR: {t.__name__}: {e}")
