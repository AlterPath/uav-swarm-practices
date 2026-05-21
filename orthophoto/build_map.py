#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import math
import os
import sys

import cv2
import numpy as np
import pandas as pd


CAM_HFOV_DEG    = 114.0
CAM_ROTATE_CW90 = True

CANVAS_RES_M_PX = 0.05
CANVAS_MARGIN_M = 3.0
BG_COLOR        = (50, 50, 50)

USE_YAW = False

BLEND_POWER = 6

MIN_AGL_M = 0.3

UAV_LANE_Y = {
    'uav0': 1.6,
    'uav1': 4.9,
    'uav2': 8.6,
}

DEFAULT_FORCE_AGL_M = 4.0


def effective_fovs(hfov_deg: float, img_w: int, img_h: int,
                   rotate_cw90: bool) -> tuple:
    hfov = math.radians(hfov_deg)
    vfov = 2.0 * math.atan(math.tan(hfov / 2.0) * img_h / img_w)

    if rotate_cw90:
        return (math.degrees(vfov), math.degrees(hfov))
    else:
        return (math.degrees(hfov), math.degrees(vfov))


def footprint_meters(fov_w_deg: float, fov_h_deg: float, agl: float) -> tuple:
    fw = 2.0 * agl * math.tan(math.radians(fov_w_deg) / 2.0)
    fh = 2.0 * agl * math.tan(math.radians(fov_h_deg) / 2.0)
    return fw, fh


def load_photos(paths: list) -> pd.DataFrame:
    frames = []
    for p in paths:
        df = pd.read_csv(p)
        df.columns = [c.strip().lower() for c in df.columns]
        df['_src'] = os.path.basename(p)
        frames.append(df)
    data = pd.concat(frames, ignore_index=True)
    data = data[data.get('ok', 1) == 1].copy()
    data = data.dropna(subset=['filename'])
    data = data[data['filename'].astype(str).str.strip() != '']
    data = data.reset_index(drop=True)
    print(f'[load] Снимков с ok=1: {len(data)}')
    return data


def apply_hardcoded_lane_offsets(photos: pd.DataFrame,
                                 lane_y: dict = None) -> pd.DataFrame:
    photos = photos.copy()
    lane_y = lane_y or UAV_LANE_Y
    srcs = photos['_src'].unique()

    print(f'[offset/hardcoded] Применяю смещения из UAV_LANE_Y: {lane_y}')
    for src in srcs:
        matched_key = None
        for key in lane_y:
            if key in src.lower():
                matched_key = key
                break
        if matched_key is None:
            print(f'  {src}: WARN — не нашёл совпадения в UAV_LANE_Y, пропускаю')
            continue

        mask    = photos['_src'] == src
        first_y = photos.loc[mask, 'pos_y'].iloc[0]
        target  = lane_y[matched_key]
        dy      = target - first_y
        photos.loc[mask, 'pos_y'] += dy
        print(f'  {src}: матч={matched_key}, first_y={first_y:+.3f} → '
              f'target={target:+.3f} м, dy={dy:+.3f} м '
              f'({mask.sum()} снимков)')

    return photos


def apply_lane_offsets(photos: pd.DataFrame,
                       mode: str,
                       field_wid: float = None,
                       cam_fov_deg: float = None,
                       overlap: float = None,
                       manual_offsets: dict = None) -> pd.DataFrame:
    photos = photos.copy()
    srcs = photos['_src'].unique()

    if mode == 'auto':
        if field_wid is None or cam_fov_deg is None or overlap is None:
            raise ValueError('auto-режим требует field_wid, cam_fov_deg, overlap')

        mean_agl = photos['agl_m'].abs().mean()
        fov_rad  = math.radians(cam_fov_deg)
        stripe   = 2.0 * mean_agl * math.tan(fov_rad / 2.0) * (1.0 - overlap)
        n        = len(srcs)

        print(f'[offset/auto] agl_mean={mean_agl:.2f} м, stripe={stripe:.2f} м, '
              f'дронов={n}, field_wid={field_wid} м')

        for rank, src in enumerate(sorted(srcs)):
            lane_center = stripe * (rank + 0.5)
            mask        = photos['_src'] == src
            first_y     = photos.loc[mask, 'pos_y'].iloc[0]
            first_x     = photos.loc[mask, 'pos_x'].iloc[0]
            dy = lane_center - first_y
            dx = -first_x if abs(first_x) > 0.5 else 0.0
            photos.loc[mask, 'pos_y'] += dy
            photos.loc[mask, 'pos_x'] += dx
            print(f'  {src}: rank={rank}, lane_center={lane_center:.2f} м, '
                  f'first_y={first_y:.3f} м → dy={dy:+.3f} м, dx={dx:+.3f} м')

    elif mode == 'manual':
        if not manual_offsets:
            raise ValueError('manual-режим требует --uav-offset')
        for src, (dx, dy) in manual_offsets.items():
            mask = photos['_src'].str.contains(src, regex=False)
            if mask.sum() == 0:
                print(f'[offset/manual] WARN: источник {src!r} не найден в CSV')
                continue
            photos.loc[mask, 'pos_x'] += dx
            photos.loc[mask, 'pos_y'] += dy
            print(f'[offset/manual] {src}: dx={dx:+.3f} м, dy={dy:+.3f} м '
                  f'({mask.sum()} снимков)')
    else:
        raise ValueError(f'Неизвестный режим offset: {mode!r}')

    return photos


def load_obstacles(path: str) -> list:
    obs = []
    if not path or not os.path.exists(path):
        return obs
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            parts = line.split(',')
            if len(parts) >= 3:
                obs.append((float(parts[0]), float(parts[1]), float(parts[2])))
    print(f'[load] Препятствий: {len(obs)}')
    return obs


def find_image(filename: str, imgdir: str) -> str:
    basename = os.path.basename(filename)
    for c in (filename, os.path.join(imgdir, filename),
              os.path.join(imgdir, basename)):
        if os.path.isfile(c):
            return c
    try:
        for e in os.scandir(imgdir):
            if e.is_dir():
                p = os.path.join(e.path, basename)
                if os.path.isfile(p):
                    return p
    except OSError:
        pass
    for root, _, files in os.walk(imgdir):
        if basename in files:
            return os.path.join(root, basename)
    return None


def center_weight_map(h_px: int, w_px: int, soft: bool = True) -> np.ndarray:
    yy, xx = np.meshgrid(np.arange(h_px, dtype=np.float32),
                         np.arange(w_px, dtype=np.float32),
                         indexing='ij')
    cx = (w_px - 1) / 2.0
    cy = (h_px - 1) / 2.0
    dx = (xx - cx) / max(cx, 1.0)
    dy = (yy - cy) / max(cy, 1.0)
    dist = np.sqrt(dx * dx + dy * dy)
    if not soft:
        return dist
    clipped = np.minimum(dist, 1.0)
    return np.cos(np.pi / 2.0 * clipped) ** BLEND_POWER


def build_map(photos: pd.DataFrame, imgdir: str,
              obstacles: list = None,
              resolution: float = CANVAS_RES_M_PX,
              margin: float = CANVAS_MARGIN_M,
              use_yaw: bool = USE_YAW,
              hard_blend: bool = False) -> tuple:
    obstacles = obstacles or []

    max_agl = float(photos['agl_m'].abs().max())
    fov_w, fov_h = effective_fovs(CAM_HFOV_DEG, 640, 480, CAM_ROTATE_CW90)
    max_fw, max_fh = footprint_meters(fov_w, fov_h, max_agl)
    half = max(max_fw, max_fh) / 2.0

    x_min = float(photos['pos_x'].min()) - half - margin
    x_max = float(photos['pos_x'].max()) + half + margin
    y_min = float(photos['pos_y'].min()) - half - margin
    y_max = float(photos['pos_y'].max()) + half + margin

    canvas_w = int(math.ceil((x_max - x_min) / resolution))
    canvas_h = int(math.ceil((y_max - y_min) / resolution))

    if hard_blend:
        canvas = np.full((canvas_h, canvas_w, 3), BG_COLOR, dtype=np.uint8)
        best_dist = np.full((canvas_h, canvas_w), 999.0, dtype=np.float32)
        canvas_acc = weight_acc = None
    else:
        canvas_acc = np.zeros((canvas_h, canvas_w, 3), dtype=np.float32)
        weight_acc = np.zeros((canvas_h, canvas_w),    dtype=np.float32)
        canvas = best_dist = None

    print(f'[canvas] X=[{x_min:.1f}; {x_max:.1f}] м, '
          f'Y=[{y_min:.1f}; {y_max:.1f}] м')
    print(f'[canvas] Размер: {canvas_w}×{canvas_h} пкс '
          f'(~{canvas_w*resolution:.1f}×{canvas_h*resolution:.1f} м), '
          f'res={resolution} м/пкс, режим=' +
          ('hard' if hard_blend else 'soft'))

    placed   = 0
    skipped  = 0

    for _, row in photos.iterrows():
        fpath = find_image(str(row['filename']), imgdir)
        if fpath is None:
            skipped += 1
            continue

        img = cv2.imread(fpath)
        if img is None:
            skipped += 1
            continue

        orig_h, orig_w = img.shape[:2]

        if CAM_ROTATE_CW90:
            img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)

        agl = abs(float(row['agl_m']))
        if agl < MIN_AGL_M:
            skipped += 1
            continue

        fov_w_i, fov_h_i = effective_fovs(CAM_HFOV_DEG, orig_w, orig_h, CAM_ROTATE_CW90)
        fw_m, fh_m = footprint_meters(fov_w_i, fov_h_i, agl)
        fw_px = max(1, int(round(fw_m / resolution)))
        fh_px = max(1, int(round(fh_m / resolution)))

        patch = cv2.resize(img, (fw_px, fh_px), interpolation=cv2.INTER_AREA)
        if hard_blend:
            wmap = center_weight_map(fh_px, fw_px, soft=False)
        else:
            wmap = center_weight_map(fh_px, fw_px, soft=True)

        if use_yaw and 'yaw_rad' in row and not pd.isna(row['yaw_rad']):
            yaw_deg = math.degrees(float(row['yaw_rad']))
            M = cv2.getRotationMatrix2D((fw_px / 2.0, fh_px / 2.0),
                                        yaw_deg, 1.0)
            patch = cv2.warpAffine(patch, M, (fw_px, fh_px),
                                   flags=cv2.INTER_LINEAR,
                                   borderMode=cv2.BORDER_CONSTANT,
                                   borderValue=BG_COLOR)
            bv = 999.0 if hard_blend else 0.0
            wmap = cv2.warpAffine(wmap, M, (fw_px, fh_px),
                                  flags=cv2.INTER_LINEAR,
                                  borderMode=cv2.BORDER_CONSTANT,
                                  borderValue=bv)

        cx = (float(row['pos_x']) - x_min) / resolution
        cy = canvas_h - 1 - (float(row['pos_y']) - y_min) / resolution
        x0 = int(round(cx - fw_px / 2.0))
        y0 = int(round(cy - fh_px / 2.0))
        x1 = x0 + fw_px
        y1 = y0 + fh_px

        sx0 = max(0, -x0);                 sy0 = max(0, -y0)
        sx1 = fw_px  - max(0, x1 - canvas_w)
        sy1 = fh_px  - max(0, y1 - canvas_h)
        dx0 = max(0, x0);                  dy0 = max(0, y0)
        dx1 = min(canvas_w, x1);           dy1 = min(canvas_h, y1)

        if dx1 > dx0 and dy1 > dy0 and sx1 > sx0 and sy1 > sy0:
            sub_bgr = patch[sy0:sy1, sx0:sx1]
            sub_w   = wmap [sy0:sy1, sx0:sx1]
            valid = sub_w > (0.01 if hard_blend else 1e-4)

            if hard_blend:
                cv_bgr  = canvas   [dy0:dy1, dx0:dx1]
                cv_dist = best_dist[dy0:dy1, dx0:dx1]
                improve = valid & (sub_w < cv_dist)
                cv_bgr [improve] = sub_bgr [improve]
                cv_dist[improve] = sub_w   [improve]
            else:
                w = sub_w * valid.astype(np.float32)
                canvas_acc[dy0:dy1, dx0:dx1] += sub_bgr.astype(np.float32) \
                                                * w[:, :, None]
                weight_acc[dy0:dy1, dx0:dx1] += w
            placed += 1

        if (placed + skipped) % 20 == 0:
            print(f'  обработано: {placed + skipped}/{len(photos)}', end='\r')

    if hard_blend:
        covered = int((best_dist < 999.0).sum())
    else:
        covered = int((weight_acc > 1e-6).sum())
        canvas = np.full((canvas_h, canvas_w, 3), BG_COLOR, dtype=np.uint8)
        valid = weight_acc > 1e-6
        wsafe = np.where(valid, weight_acc, 1.0)[:, :, None]
        avg   = (canvas_acc / wsafe).clip(0, 255).astype(np.uint8)
        canvas[valid] = avg[valid]

    pct = 100.0 * covered / (canvas_w * canvas_h)
    print(f'\n[stitch] Положено: {placed}, пропущено: {skipped}, '
          f'покрытие холста: {pct:.1f}%')

    if obstacles:
        for ox, oy, orad in obstacles:
            cx_o = int(round((ox - x_min) / resolution))
            cy_o = int(round(canvas_h - 1 - (oy - y_min) / resolution))
            r    = max(1, int(round(orad / resolution)))
            overlay = canvas.copy()
            cv2.circle(overlay, (cx_o, cy_o), r, (0, 0, 220), -1)
            cv2.addWeighted(overlay, 0.35, canvas, 0.65, 0, canvas)
            cv2.circle(canvas, (cx_o, cy_o), r, (0, 0, 255), 2)
            cv2.circle(canvas, (cx_o, cy_o), 3, (0, 0, 255), -1)

    return canvas, x_min, y_min


def draw_grid(mosaic: np.ndarray, origin_x: float, origin_y: float,
              resolution: float, step_m: float = 10.0) -> np.ndarray:
    out = mosaic.copy()
    h, w = out.shape[:2]
    color_grid, color_label = (180, 180, 180), (240, 240, 240)
    font, fs = cv2.FONT_HERSHEY_SIMPLEX, 0.4

    x = math.ceil(origin_x / step_m) * step_m
    while x <= origin_x + w * resolution:
        px = int((x - origin_x) / resolution)
        cv2.line(out, (px, 0), (px, h), color_grid, 1)
        cv2.putText(out, f'{x:.0f}m', (px + 2, h - 4),
                    font, fs, color_label, 1, cv2.LINE_AA)
        x += step_m

    y = math.ceil(origin_y / step_m) * step_m
    while y <= origin_y + h * resolution:
        py = h - 1 - int((y - origin_y) / resolution)
        cv2.line(out, (0, py), (w, py), color_grid, 1)
        cv2.putText(out, f'{y:.0f}m', (2, py - 2),
                    font, fs, color_label, 1, cv2.LINE_AA)
        y += step_m

    return out


def generate_demo(out_dir: str = '/tmp/demo_simple',
                  field_len: float = 60.0,
                  field_wid: float = 30.0,
                  altitude: float = 2.5,
                  n_drones: int = 3,
                  overlap: float = 0.75) -> tuple:
    imgdir = os.path.join(out_dir, 'raw_images')
    os.makedirs(imgdir, exist_ok=True)
    res = CANVAS_RES_M_PX

    fw_px = int(field_len / res)
    fh_px = int(field_wid / res)

    field = np.full((fh_px, fw_px, 3), (35, 75, 50), dtype=np.uint8)
    np.random.seed(0)
    noise = np.random.randint(-12, 12, field.shape, dtype=np.int16)
    field = np.clip(field.astype(np.int16) + noise, 0, 255).astype(np.uint8)

    pts = np.array([
        [int(0.55 * fw_px), int(0.20 * fh_px)],
        [int(0.95 * fw_px), int(0.85 * fh_px)],
    ], np.int32)
    cv2.line(field, tuple(pts[0]), tuple(pts[1]), (50, 110, 150), 18)

    for lx, ly in [(10, 5), (30, 15), (50, 25), (15, 20), (45, 10)]:
        cv2.circle(field, (int(lx / res), int(ly / res)), 6, (0, 200, 0), -1)

    fov_w, fov_h = effective_fovs(CAM_HFOV_DEG, 640, 480, CAM_ROTATE_CW90)
    foot_w, foot_h = footprint_meters(fov_w, fov_h, altitude)
    overlap = max(0.01, min(0.99, overlap))
    step_x = foot_w * (1.0 - overlap)
    print(f'[demo] overlap={overlap:.0%}, шаг={step_x:.2f} м (footprint={foot_w:.2f}×{foot_h:.2f} м)')
    rows, pid = [], 0

    for drone_id in range(n_drones):
        lane_y = field_wid * (drone_id + 0.5) / n_drones
        x = 0.0
        while x <= field_len + 1e-3:
            fx0_px = max(0, int((x - foot_w / 2) / res))
            fy0_px = max(0, int((lane_y - foot_h / 2) / res))
            fx1_px = min(fw_px, int((x + foot_w / 2) / res))
            fy1_px = min(fh_px, int((lane_y + foot_h / 2) / res))

            if fx1_px > fx0_px and fy1_px > fy0_px:
                patch = field[fh_px - fy1_px:fh_px - fy0_px,
                              fx0_px:fx1_px]
                patch = cv2.resize(patch, (480, 640) if CAM_ROTATE_CW90
                                          else (640, 480))
                if CAM_ROTATE_CW90:
                    patch = cv2.rotate(patch, cv2.ROTATE_90_COUNTERCLOCKWISE)

                fname = f'uav{drone_id}_photo_{pid:05d}.jpg'
                cv2.imwrite(os.path.join(imgdir, fname), patch)

                rows.append({
                    'time_s': pid * 2.0,
                    'filename': fname,
                    'pos_x': x, 'pos_y': lane_y, 'pos_z': altitude,
                    'agl_m': altitude,
                    'yaw_rad': 0.0, 'roll_rad': 0.0, 'pitch_rad': 0.0,
                    'vel_mps': 2.0, 'ok': 1, 'event': 'SHOT',
                })
                pid += 1
            x += step_x

    csv_path = os.path.join(out_dir, 'photos_demo.csv')
    pd.DataFrame(rows).to_csv(csv_path, index=False)
    print(f'[demo] Снимков: {len(rows)}, поле {field_len}×{field_wid} м')
    return [csv_path], imgdir


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--photos', nargs='+', default=None)
    p.add_argument('--imgdir', default='.')
    p.add_argument('--out', default='mosaic.jpg')
    p.add_argument('--obstacles', default=None)
    p.add_argument('--resolution', type=float, default=CANVAS_RES_M_PX)
    p.add_argument('--yaw', action='store_true')
    p.add_argument('--grid', action='store_true')
    p.add_argument('--overlap', type=float, default=0.75)
    p.add_argument('--force-agl', type=float, default=None, metavar='METERS')
    p.add_argument('--offset-mode', choices=['none', 'auto', 'manual'], default='none')
    p.add_argument('--field-wid', type=float, default=30.0)
    p.add_argument('--cam-fov', type=float, default=78.0)
    p.add_argument('--drone-overlap', type=float, default=0.2)
    p.add_argument('--uav-offset', nargs='+', default=None, metavar='NAME:DX:DY')
    args = p.parse_args()


    if not args.photos:
        print('[err] Укажите --photos или используйте --demo')
        sys.exit(1)
    photos = load_photos(args.photos)
    imgdir = args.imgdir
    obstacles = load_obstacles(args.obstacles)
    if len(photos) == 0:
        print('[err] Нет снимков с ok=1.')
        sys.exit(1)

    effective_force_agl = args.force_agl
    if effective_force_agl is None and DEFAULT_FORCE_AGL_M is not None:
        effective_force_agl = DEFAULT_FORCE_AGL_M
        print(f'[info] --force-agl не задан, использую DEFAULT_FORCE_AGL_M = {DEFAULT_FORCE_AGL_M} м')

    if effective_force_agl is not None:
        if effective_force_agl < MIN_AGL_M:
            print(f'[warn] force-agl {effective_force_agl} м меньше MIN_AGL_M={MIN_AGL_M} м — '
                  f'все снимки будут пропущены!')
        print(f'[info] Высота принудительно установлена: {effective_force_agl} м '
              f'(было: min={photos["agl_m"].min():.2f}, max={photos["agl_m"].max():.2f} м)')
        photos = photos.copy()
        photos['agl_m'] = effective_force_agl

    if args.offset_mode == 'none':
        if not args.demo and UAV_LANE_Y:
            photos = apply_hardcoded_lane_offsets(photos)
    else:
        if args.offset_mode == 'manual':
            if not args.uav_offset:
                print('[err] --offset-mode manual требует --uav-offset NAME:DX:DY ...')
                sys.exit(1)
            manual = {}
            for entry in args.uav_offset:
                parts = entry.split(':')
                if len(parts) != 3:
                    print(f'[err] Неверный формат --uav-offset: {entry!r} '
                          f'(ожидается NAME:DX:DY)')
                    sys.exit(1)
                manual[parts[0]] = (float(parts[1]), float(parts[2]))
            photos = apply_lane_offsets(photos, mode='manual',
                                        manual_offsets=manual)
        else:
            photos = apply_lane_offsets(photos, mode='auto',
                                        field_wid=args.field_wid,
                                        cam_fov_deg=args.cam_fov,
                                        overlap=args.drone_overlap)

    mosaic, ox, oy = build_map(photos, imgdir, obstacles,
                                resolution=args.resolution,
                                use_yaw=args.yaw)

    if args.grid:
        mosaic = draw_grid(mosaic, ox, oy, args.resolution, step_m=10.0)

    cv2.imwrite(args.out, mosaic)
    print(f'[done] {args.out}  ({mosaic.shape[1]}×{mosaic.shape[0]} пкс)')

    thumb_path = args.out.rsplit('.', 1)[0] + '_thumb.jpg'
    scale = min(1.0, 1200 / max(mosaic.shape[:2]))
    thumb = cv2.resize(mosaic,
                       (int(mosaic.shape[1] * scale),
                        int(mosaic.shape[0] * scale)))
    cv2.imwrite(thumb_path, thumb)
    print(f'[done] {thumb_path} (миниатюра)')


if __name__ == '__main__':
    main()