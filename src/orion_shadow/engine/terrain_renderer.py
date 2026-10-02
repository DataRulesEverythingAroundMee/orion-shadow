import math
import logging
import concurrent.futures
from typing import Dict, Any, Optional, Tuple

from orion_shadow.engine.terrain import TerrainEngine

logger = logging.getLogger(__name__)

try:
    import numpy as np
except ImportError:
    np = None

try:
    import cv2
except ImportError:
    cv2 = None

# ---------------------------------------------------------------------------
# Optional GPU backends — both are purely additive; CPU paths remain intact.
# ---------------------------------------------------------------------------

# CuPy: drop-in NumPy replacement for GPU ray-marching numerics (Fix 3b)
try:
    import cupy as cp
    _CUPY_AVAILABLE = True
except Exception:
    cp = None
    _CUPY_AVAILABLE = False

# OpenCV CUDA modules for GPU remap / resize (Fix 3a)
try:
    _CUDA_CV_AVAILABLE = (
        cv2 is not None
        and hasattr(cv2, 'cuda')
        and int(cv2.cuda.getCudaEnabledDeviceCount()) > 0
    )
except Exception:
    _CUDA_CV_AVAILABLE = False

# Shared thread-pool for offloading blocking render work from the asyncio loop
_render_executor = concurrent.futures.ThreadPoolExecutor(
    max_workers=2, thread_name_prefix="orion_render"
)


class TerrainDraper:
    """
    Renders 3D perspective terrain visualization with draped satellite/map tiles.
    Uses screen-space ray marching against DTED elevation data, inverse texture mapping,
    surface normal hillshading, and 3D mountain horizon skyline compositing.

    Optimizations applied:
    - Fix 1b: cv2.INTER_LANCZOS4 for terrain remap (quality)
    - Fix 1c: cv2.INTER_CUBIC for shade/haze upscale (quality)
    - Fix 2a: Adaptive ray step count based on AGL (accuracy at low altitude)
    - Fix 3a: cv2.cuda.remap when OpenCV CUDA is available (GPU throughput)
    - Fix 3b: CuPy ray marching when available (GPU acceleration)
    """

    def __init__(self, terrain: TerrainEngine, sub_sample: int = 6):
        self.terrain = terrain
        self.sub_sample = max(1, sub_sample)

        # Cached grid geometry to avoid reallocations when FOV/resolution don't change
        self._cached_dims: Optional[Tuple[int, int, float, float]] = None
        self._cached_ray_cam: Optional[Any] = None

        logger.info(
            "Initialized TerrainDraper (sub_sample=%d, CuPy=%s, OpenCV CUDA=%s)",
            self.sub_sample, _CUPY_AVAILABLE, _CUDA_CV_AVAILABLE
        )

    def _get_camera_rays(self, width: int, height: int, hfov: float, vfov: float) -> Any:
        """Computes and caches normalized camera ray directions in camera frame."""
        if np is None:
            return None

        gw = max(16, width // self.sub_sample)
        gh = max(16, height // self.sub_sample)
        cache_key = (gw, gh, round(hfov, 3), round(vfov, 3))

        if self._cached_dims == cache_key and self._cached_ray_cam is not None:
            return self._cached_ray_cam

        xs = (np.arange(gw, dtype=np.float32) + 0.5) * (float(width) / gw)
        ys = (np.arange(gh, dtype=np.float32) + 0.5) * (float(height) / gh)
        X, Y = np.meshgrid(xs, ys)

        cx = width / 2.0
        cy = height / 2.0
        hw = math.tan(math.radians(hfov / 2.0))
        hh = math.tan(math.radians(vfov / 2.0))

        fx = cx / max(1e-6, hw)
        fy = cy / max(1e-6, hh)

        un = (X - cx) / fx
        vn = (Y - cy) / fy
        wn = np.ones_like(un)

        norm = np.sqrt(un * un + vn * vn + wn * wn)
        norm[norm < 1e-6] = 1.0

        ray_cam = np.stack([un / norm, vn / norm, wn / norm], axis=-1)
        self._cached_dims = cache_key
        self._cached_ray_cam = ray_cam
        return ray_cam

    def _rotation_matrix(self, cam_hdg: float, cam_pitch: float, cam_roll: float) -> Any:
        """
        Builds 3x3 rotation matrix from camera frame to NED frame:
        X_cam: Right, Y_cam: Down, Z_cam: Forward.
        NED: X: North, Y: East, Z: Down.
        """
        hdg_rad = math.radians(cam_hdg)
        pitch_rad = math.radians(cam_pitch)
        roll_rad = math.radians(cam_roll)

        cos_h, sin_h = math.cos(hdg_rad), math.sin(hdg_rad)
        cos_p, sin_p = math.cos(pitch_rad), math.sin(pitch_rad)
        cos_r, sin_r = math.cos(roll_rad), math.sin(roll_rad)

        # Forward boresight in NED
        fwd = np.array([cos_p * cos_h, cos_p * sin_h, -sin_p], dtype=np.float32)
        # Unrolled right & down vectors in NED
        r0 = np.array([-sin_h, cos_h, 0.0], dtype=np.float32)
        d0 = np.array([sin_p * cos_h, sin_p * sin_h, cos_p], dtype=np.float32)

        # Apply roll around boresight
        right = cos_r * r0 + sin_r * d0
        down = -sin_r * r0 + cos_r * d0

        # Matrix mapping [cam_x, cam_y, cam_z] to [NED_N, NED_E, NED_D]
        return np.column_stack([right, down, fwd])

    def render(
        self,
        ground_texture: Any,
        tile_bounds: Tuple[int, int, int, int],
        tile_px: int,
        zoom: int,
        telem: Dict[str, Any],
        width: int,
        height: int,
        hfov: float,
        vfov: float,
        base_sky_frame: Optional[Any] = None
    ) -> Optional[Any]:
        """
        Renders a 3D draped terrain frame using ray-surface intersection against DTED.
        """
        if np is None or cv2 is None:
            return None

        min_tx, min_ty, max_tx, max_ty = tile_bounds
        ray_cam = self._get_camera_rays(width, height, hfov, vfov)
        if ray_cam is None:
            return None

        gh, gw = ray_cam.shape[:2]

        lat0 = telem['lat']
        lon0 = telem['lon']
        alt0 = telem['alt']
        cam_hdg = telem['cam_hdg']
        cam_pitch = telem['cam_pitch']
        cam_roll = telem.get('cam_roll', 0.0)

        # Transform rays to NED
        R_cam2ned = self._rotation_matrix(cam_hdg, cam_pitch, cam_roll)
        # ray_ned has shape (gh, gw, 3)
        ray_ned = np.dot(ray_cam, R_cam2ned.T)

        d_N = ray_ned[:, :, 0]
        d_E = ray_ned[:, :, 1]
        d_D = ray_ned[:, :, 2]

        # Ground elevation at aircraft nadir
        ac_terrain_elev = self.terrain.get_elevation(lat0, lon0)
        agl = max(20.0, alt0 - ac_terrain_elev)

        # Ray marching bounds
        s_min = max(5.0, agl * 0.15)
        s_max = min(65000.0, max(15000.0, agl * 28.0))

        # Fix 2a: Adaptive ray step count based on AGL.
        # More steps at low altitudes to resolve narrow ridgelines without
        # over-sampling at cruise altitude where geometry is coarser.
        if agl < 300.0:
            num_steps = 32
        elif agl < 800.0:
            num_steps = 28
        else:
            num_steps = 20

        # Select array backend: CuPy (GPU) when available, NumPy (CPU) otherwise
        # Fix 3b: CuPy drop-in for the entire ray-marching inner loop
        if _CUPY_AVAILABLE and cp is not None:
            xp = cp
            d_N = cp.asarray(d_N)
            d_E = cp.asarray(d_E)
            d_D = cp.asarray(d_D)
        else:
            xp = np

        steps = s_min * ((s_max / s_min) ** (xp.arange(num_steps, dtype=xp.float32) / (num_steps - 1)))

        cos_lat = max(0.01, math.cos(math.radians(lat0)))
        lat_scale = 1.0 / 111320.0
        lon_scale = 1.0 / (111320.0 * cos_lat)

        hit_mask = xp.zeros((gh, gw), dtype=bool)
        hit_s = xp.full((gh, gw), s_max, dtype=xp.float32)
        prev_delta = xp.zeros((gh, gw), dtype=xp.float32)
        prev_s = xp.full((gh, gw), s_min, dtype=xp.float32)

        # Coarse-to-fine ray marching
        for i, s_val in enumerate(steps):
            active = ~hit_mask
            if not xp.any(active):
                break

            s_act = float(s_val)
            # Ray altitude at distance s: Alt(s) = alt0 - s * d_D
            ray_alt = alt0 - s_act * d_D[active]

            # Geographical position of ray at distance s
            s_lat = lat0 + (s_act * d_N[active]) * lat_scale
            s_lon = lon0 + (s_act * d_E[active]) * lon_scale

            # DTED terrain elevation at sample point.
            # When using CuPy, convert to NumPy for the DTED lookup (which is
            # CPU-based), then convert result back to the GPU array.
            if _CUPY_AVAILABLE and cp is not None and xp is cp:
                terr_elev = cp.asarray(
                    self.terrain.get_elevations(
                        cp.asnumpy(s_lat), cp.asnumpy(s_lon)
                    )
                )
            else:
                terr_elev = self.terrain.get_elevations(s_lat, s_lon)

            delta = ray_alt - terr_elev

            if i == 0:
                # Initialize previous values
                prev_delta[active] = delta
                prev_s[active] = s_act
                # Hits at first step (very close to ground)
                hit_now = delta <= 0.0
                if xp.any(hit_now):
                    curr_hit_mask = xp.zeros_like(active)
                    curr_hit_mask[active] = hit_now
                    hit_mask |= curr_hit_mask
                    hit_s[curr_hit_mask] = s_act
            else:
                hit_now = (delta <= 0.0) & (prev_delta[active] > 0.0)
                if xp.any(hit_now):
                    # Linear interpolation (secant root refinement)
                    p_d = prev_delta[active][hit_now]
                    c_d = delta[hit_now]
                    denom = p_d - c_d
                    denom[denom < 1e-4] = 1e-4
                    frac = xp.clip(p_d / denom, 0.0, 1.0)
                    s_interp = prev_s[active][hit_now] + frac * (s_act - prev_s[active][hit_now])

                    # Unpack into full grid
                    curr_hit_mask = xp.zeros_like(active)
                    curr_hit_mask[active] = hit_now
                    hit_mask |= curr_hit_mask
                    hit_s[curr_hit_mask] = s_interp

                prev_delta[active] = delta
                prev_s[active] = s_act

        # Bring results back to CPU NumPy if we used CuPy
        if _CUPY_AVAILABLE and cp is not None and xp is cp:
            hit_mask = cp.asnumpy(hit_mask)
            hit_s = cp.asnumpy(hit_s)
            d_N = cp.asnumpy(d_N)
            d_E = cp.asnumpy(d_E)
            d_D = cp.asnumpy(d_D)

        # Rays that never hit terrain point to the sky
        is_sky = ~hit_mask

        # Hit coordinates in Lat / Lon
        hit_lat = lat0 + (hit_s * d_N) * lat_scale
        hit_lon = lon0 + (hit_s * d_E) * lon_scale

        # Texture coordinates in tile canvas
        n_zoom = 2.0 ** zoom
        hit_tx = (hit_lon + 180.0) / 360.0 * n_zoom
        hit_lat_clamped = np.clip(hit_lat, -85.0511, 85.0511)
        lat_rad = np.radians(hit_lat_clamped)
        hit_ty = (1.0 - np.log(np.tan(lat_rad) + 1.0 / np.cos(lat_rad)) / math.pi) / 2.0 * n_zoom

        canvas_h, canvas_w = ground_texture.shape[:2]
        u_tex = np.clip((hit_tx - min_tx) * float(tile_px), 0.0, float(max(1, canvas_w - 1)))
        v_tex = np.clip((hit_ty - min_ty) * float(tile_px), 0.0, float(max(1, canvas_h - 1)))

        # --- Fast 3D Hillshading calculation via 3D surface gradient ---
        # 3D coordinates in local meters (E, N, Up)
        pos_e = (hit_s * d_E)
        pos_n = (hit_s * d_N)
        pos_up = (alt0 - hit_s * d_D)

        # Gradients along ray columns (dx) and rows (dy)
        de_dx = np.gradient(pos_e, axis=1)
        dn_dx = np.gradient(pos_n, axis=1)
        dup_dx = np.gradient(pos_up, axis=1)

        de_dy = np.gradient(pos_e, axis=0)
        dn_dy = np.gradient(pos_n, axis=0)
        dup_dy = np.gradient(pos_up, axis=0)

        # Cross product of surface tangent vectors: N = dP/dx x dP/dy
        norm_e = dn_dx * dup_dy - dup_dx * dn_dy
        norm_n = dup_dx * de_dy - de_dx * dup_dy
        norm_up = de_dx * dn_dy - dn_dx * de_dy

        n_norm = np.sqrt(norm_e * norm_e + norm_n * norm_n + norm_up * norm_up)
        n_norm[n_norm < 1e-6] = 1.0

        # Ensure normal points upwards
        flip = norm_up < 0.0
        norm_e[flip] = -norm_e[flip]
        norm_n[flip] = -norm_n[flip]
        norm_up[flip] = -norm_up[flip]

        norm_x = norm_e / n_norm
        norm_y = norm_n / n_norm
        norm_z = norm_up / n_norm

        # Sun light direction (e.g. azimuth 315° NW, elevation 45°)
        sun_x = -0.5
        sun_y = 0.5
        sun_z = 0.7071
        sun_len = math.sqrt(sun_x * sun_x + sun_y * sun_y + sun_z * sun_z)
        sun_x /= sun_len
        sun_y /= sun_len
        sun_z /= sun_len

        dot = norm_x * sun_x + norm_y * sun_y + norm_z * sun_z
        shade = np.clip(0.72 + 0.38 * np.maximum(0.0, dot), 0.55, 1.25)

        # Atmospheric haze blending based on distance s
        haze = np.clip((hit_s - s_min) / (s_max - s_min) * 0.7, 0.0, 0.6)

        # Upscale remap coordinates and shading masks to full target frame resolution.
        # Fix 1c: Use INTER_CUBIC for continuous scalar fields (shade, haze) so the
        # hillshading gradient upscales smoothly without the blocky banding that
        # INTER_LINEAR produces at coarse sub-sample factors.
        map_x = cv2.resize(u_tex.astype(np.float32), (width, height), interpolation=cv2.INTER_LINEAR)
        map_y = cv2.resize(v_tex.astype(np.float32), (width, height), interpolation=cv2.INTER_LINEAR)
        sky_mask = cv2.resize(is_sky.astype(np.uint8), (width, height), interpolation=cv2.INTER_NEAREST).astype(bool)
        shade_full = cv2.resize(shade.astype(np.float32), (width, height),
                                interpolation=cv2.INTER_CUBIC)[:, :, np.newaxis]
        haze_full = cv2.resize(haze.astype(np.float32), (width, height),
                               interpolation=cv2.INTER_CUBIC)[:, :, np.newaxis]

        # Warp ground texture over 3D terrain.
        # Fix 1b: INTER_LANCZOS4 produces sharper satellite texture with far less
        # blurring than INTER_LINEAR, at a minor extra CPU cost.
        # Fix 1b (border): BORDER_REPLICATE avoids the bright-green fringe that
        # BORDER_CONSTANT creates at the canvas edge during remap.
        # Fix 3a: Use cv2.cuda.remap when OpenCV was compiled with CUDA support.
        if _CUDA_CV_AVAILABLE:
            try:
                gpu_tex = cv2.cuda_GpuMat()
                gpu_tex.upload(ground_texture)
                gpu_mapx = cv2.cuda_GpuMat()
                gpu_mapx.upload(map_x)
                gpu_mapy = cv2.cuda_GpuMat()
                gpu_mapy.upload(map_y)
                gpu_out = cv2.cuda.remap(gpu_tex, gpu_mapx, gpu_mapy,
                                         interpolation=cv2.INTER_LINEAR,
                                         borderMode=cv2.BORDER_REPLICATE)
                draped_ground = gpu_out.download()
            except Exception as e:
                # Fall back to CPU if the CUDA call fails at runtime
                logger.warning("cv2.cuda.remap failed (%s); falling back to CPU cv2.remap", e)
                draped_ground = cv2.remap(
                    ground_texture, map_x, map_y,
                    interpolation=cv2.INTER_LANCZOS4,
                    borderMode=cv2.BORDER_REPLICATE,
                )
        else:
            draped_ground = cv2.remap(
                ground_texture, map_x, map_y,
                interpolation=cv2.INTER_LANCZOS4,
                borderMode=cv2.BORDER_REPLICATE,
            )

        # Apply 3D relief hillshading
        draped_shaded = np.clip(draped_ground.astype(np.float32) * shade_full, 0, 255).astype(np.uint8)

        # Apply atmospheric distance haze
        haze_color = np.array([185, 165, 140], dtype=np.float32)
        draped_hazed = np.clip(
            draped_shaded.astype(np.float32) * (1.0 - haze_full) + haze_color * haze_full,
            0,
            255
        ).astype(np.uint8)

        if logger.isEnabledFor(logging.DEBUG):
            logger.debug(
                "3D terrain rendered: %dx%d, sky_pixels=%d, num_steps=%d, AGL=%.1fm",
                width, height, int(np.sum(sky_mask)), num_steps, agl
            )

        # Composite with sky background:
        # Pixels that cleared mountain peaks show the atmospheric sky gradient
        if base_sky_frame is not None and isinstance(base_sky_frame, np.ndarray):
            out_frame = base_sky_frame.copy()
            if np.any(sky_mask):
                out_frame[sky_mask] = np.array([185, 165, 140], dtype=np.uint8)
            out_frame[~sky_mask] = draped_hazed[~sky_mask]
            return out_frame
        else:
            return draped_hazed
