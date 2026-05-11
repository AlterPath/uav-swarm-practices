#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
build_map.py — формирование ортофотоплана из серии снимков квадрокоптеров
               (алгоритм APF + полосы).

Алгоритм:
  1. Читает photos_*.csv (список снимков с телеметрией: позиция, ориентация).
  2. Для каждого снимка с ok==1:
       a. Загружает изображение.
       b. Выполняет устранение дисторсии объектива.
       c. По высоте, углам рысканья/тангажа/крена и FOV вычисляет проекцию
          на плоскость земли (гомография).
       d. Накладывает на общий холст методом взвешенного смешивания.
  3. (Опционально) рисует препятствия поверх карты.
  4. Сохраняет готовый ортофотоплан + отчёт покрытия.

Запуск:
    python3 build_map.py \
        --photos  photos_apf_lanes_uav0_XXXXXX.csv \
        --imgdir  raw_images/uav0 \
        --out     mosaic_uav0.jpg

    # Несколько дронов:
    python3 build_map.py \
        --photos  photos_uav0.csv photos_uav1.csv photos_uav2.csv \
        --imgdir  raw_images \
        --out     mosaic_all.jpg \
        --obstacles obstacles.txt

Требования:
    pip install opencv-python numpy pandas
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np
import pandas as pd

# =============================================================================
# КОНСТАНТЫ — ВСЕ НАСТРАИВАЕМЫЕ ПАРАМЕТРЫ В ОДНОМ МЕСТЕ
# =============================================================================

# ── Параметры камеры (Gazebo down_camera_link) ────────────────────────────────
# Реальный FOV камеры симулятора — НЕ путать с FOV планирования полос
# (в apf_with_lanes_drones.py стоит 78° для расчёта ширины полосы,
#  но физическая камера в Gazebo имеет horizontal_fov ≈ 114°)
CAM_HFOV_DEG = 114.0         # горизонтальный угол обзора камеры, °

# Перекрытие снимков (используется только в демо-режиме для расстановки точек съёмки)
# В реальных данных расстановка уже задана APF-алгоритмом; реальное перекрытие ≈ 53%
CAM_OVERLAP = 0.53

# Коэффициенты дисторсии [k1, k2, p1, p2, k3]
# d ≈ 0 для симулятора; для реальной камеры заменить на значения калибровки
DIST_COEFFS = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float64)

# ── Параметры выходного холста ────────────────────────────────────────────────
CANVAS_RESOLUTION = 0.05     # ГРД (ground sample distance), метров на пиксель
CANVAS_MARGIN_M   = 3.0      # отступ вокруг крайних снимков, метры

# ── Параметры поля (используются в демо-режиме) ───────────────────────────────
DEMO_FIELD_LEN_M = 60.0      # длина поля, м
DEMO_FIELD_WID_M = 30.0      # ширина поля, м
DEMO_ALTITUDE_M  = 5.0       # высота полёта, м
DEMO_N_DRONES    = 3         # количество дронов
DEMO_IMG_W       = 640       # ширина синтетического снимка, пкс
DEMO_IMG_H       = 480       # высота синтетического снимка, пкс

# =============================================================================


# ─────────────────────────────────────────────────────────────────────────────
# Вспомогательные функции
# ─────────────────────────────────────────────────────────────────────────────

def rotation_x(angle: float) -> np.ndarray:
    """Матрица поворота вокруг оси X (крен / roll)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1, 0,  0],
                     [0, c, -s],
                     [0, s,  c]], dtype=np.float64)

def rotation_y(angle: float) -> np.ndarray:
    """Матрица поворота вокруг оси Y (тангаж / pitch)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[ c, 0, s],
                     [ 0, 1, 0],
                     [-s, 0, c]], dtype=np.float64)

def rotation_z(angle: float) -> np.ndarray:
    """Матрица поворота вокруг оси Z (рысканье / yaw)."""
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0],
                     [s,  c, 0],
                     [0,  0, 1]], dtype=np.float64)


def camera_intrinsics(img_w: int, img_h: int,
                      hfov_deg: float) -> np.ndarray:
    """
    Строит матрицу внутренних параметров камеры K (3×3).
    Предполагаем квадратные пиксели: fx = fy.
    """
    hfov_rad = math.radians(hfov_deg)
    fx = img_w / (2.0 * math.tan(hfov_rad / 2.0))
    fy = fx                         # квадратные пиксели
    cx = img_w / 2.0
    cy = img_h / 2.0
    K = np.array([[fx,  0, cx],
                  [ 0, fy, cy],
                  [ 0,  0,  1]], dtype=np.float64)
    return K


def pixel_to_ground(u: float, v: float,
                    K: np.ndarray, R_cam2world: np.ndarray,
                    cam_pos_x: float, cam_pos_y: float,
                    agl: float) -> tuple:
    """
    Вычисляет точку пересечения луча из пикселя (u, v) с плоскостью земли.

    Параметры
    ----------
    u, v         : координата пикселя
    K            : матрица внутренних параметров (3×3)
    R_cam2world  : матрица поворота «камера → мир» (3×3)
    cam_pos_x/y  : горизонтальная позиция дрона, м
    agl          : высота над землёй, м (положительная)

    Возвращает
    ----------
    (world_x, world_y) : координаты на земле, м
    """
    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]

    # Луч в системе координат камеры (ось Z смотрит вниз)
    ray_cam = np.array([(u - cx) / fx,
                        (v - cy) / fy,
                         1.0])

    # Переводим луч в мировую систему координат
    ray_world = R_cam2world @ ray_cam

    # Камера смотрит вниз — ось Z луча должна быть отрицательной (ENU: Z вверх)
    if ray_world[2] >= 0:
        ray_world[2] = -abs(ray_world[2])

    # t: параметр луча, при котором z = 0 (уровень земли)
    t = -agl / ray_world[2]

    world_x = cam_pos_x + t * ray_world[0]
    world_y = cam_pos_y + t * ray_world[1]
    return world_x, world_y


def image_footprint(img_w: int, img_h: int,
                    K: np.ndarray, R_cam2world: np.ndarray,
                    pos_x: float, pos_y: float, agl: float) -> np.ndarray:
    """
    Возвращает 4 угловые точки проекции снимка на землю (shape: 4×2).
    Порядок: верхний-левый, верхний-правый, нижний-правый, нижний-левый.
    """
    corners_px = [(0, 0), (img_w, 0), (img_w, img_h), (0, img_h)]
    corners_world = []
    for u, v in corners_px:
        gx, gy = pixel_to_ground(u, v, K, R_cam2world, pos_x, pos_y, agl)
        corners_world.append([gx, gy])
    return np.array(corners_world, dtype=np.float64)


def world_to_canvas(wx: np.ndarray, wy: np.ndarray,
                    origin_x: float, origin_y: float,
                    resolution: float) -> np.ndarray:
    """
    Переводит мировые координаты (м) в пиксельные координаты холста.
    origin_x/y — левый-нижний угол холста в мировых единицах.
    """
    px = (wx - origin_x) / resolution
    py = (wy - origin_y) / resolution
    return np.stack([px, py], axis=-1).astype(np.float32)


# ─────────────────────────────────────────────────────────────────────────────
# Чтение данных
# ─────────────────────────────────────────────────────────────────────────────

def load_photos_csv(paths: list) -> pd.DataFrame:
    """
    Читает один или несколько photos_*.csv, возвращает отфильтрованный
    DataFrame только с успешными снимками (ok == 1).
    """
    frames = []
    for p in paths:
        df = pd.read_csv(p)
        frames.append(df)
    data = pd.concat(frames, ignore_index=True)

    # Только успешно сохранённые снимки
    data = data[data['ok'] == 1].copy()
    data = data.dropna(subset=['filename'])
    data = data[data['filename'].str.strip() != '']
    data = data.reset_index(drop=True)

    print(f'[load] Снимков с ok=1: {len(data)}')
    return data


def load_obstacles(path: str) -> list:
    """
    Читает obstacles.txt — список препятствий (x, y, radius).
    Возвращает список кортежей [(x, y, r), ...].
    """
    obstacles = []
    if not path or not os.path.exists(path):
        return obstacles
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) >= 3:
                obstacles.append((float(parts[0]),
                                  float(parts[1]),
                                  float(parts[2])))
    print(f'[load] Препятствий: {len(obstacles)}')
    return obstacles


# ─────────────────────────────────────────────────────────────────────────────
# Основной класс формирования карты
# ─────────────────────────────────────────────────────────────────────────────

class MapBuilder:
    """
    Формирует ортофотоплан из серии аэрофотоснимков с телеметрией.

    Используется взвешенное пиксельное смешивание (weight map):
    каждый снимок вносит вклад обратно пропорционально расстоянию
    до края снимка — это уменьшает «швы» на стыках.
    """

    def __init__(self, resolution: float = CANVAS_RESOLUTION,
                 margin: float = CANVAS_MARGIN_M,
                 hfov_deg: float = CAM_HFOV_DEG):
        self.resolution = resolution      # м/пиксель
        self.margin     = margin          # отступ, м
        self.hfov_deg   = hfov_deg

        # Будут инициализированы после первого прохода (определение границ)
        self.origin_x: float = 0.0       # мировые координаты левого-нижнего угла
        self.origin_y: float = 0.0
        self.canvas_w:  int  = 0         # ширина холста, пикселей
        self.canvas_h:  int  = 0         # высота холста, пикселей

        # Аккумуляторы для взвешенного смешивания
        self.sum_bgr:    np.ndarray | None = None   # float32 (H, W, 3)
        self.sum_weight: np.ndarray | None = None   # float32 (H, W)

    # ── шаг 1: определить границы холста ─────────────────────────────────────

    def compute_canvas_bounds(self, photos: pd.DataFrame, imgdir: str):
        """
        Обходит все снимки, вычисляет охватывающий прямоугольник в мировых
        координатах и инициализирует холст.
        """
        all_corners = []

        for _, row in photos.iterrows():
            fpath = self._find_image(row['filename'], imgdir)
            if fpath is None:
                continue

            img = cv2.imread(fpath)
            if img is None:
                continue

            h_px, w_px = img.shape[:2]
            K = camera_intrinsics(w_px, h_px, self.hfov_deg)
            R = self._build_rotation(row['yaw_rad'],
                                     row['roll_rad'],
                                     row['pitch_rad'])
            agl = abs(float(row['agl_m']))
            if agl < 0.3:          # дрон ещё не взлетел
                continue

            corners = image_footprint(w_px, h_px, K, R,
                                      float(row['pos_x']),
                                      float(row['pos_y']),
                                      agl)
            all_corners.append(corners)

        if not all_corners:
            raise RuntimeError('Не найдено ни одного валидного снимка.')

        pts = np.vstack(all_corners)   # (N*4, 2)
        x_min, y_min = pts[:, 0].min() - self.margin, pts[:, 1].min() - self.margin
        x_max, y_max = pts[:, 0].max() + self.margin, pts[:, 1].max() + self.margin

        self.origin_x = x_min
        self.origin_y = y_min
        self.canvas_w = int(math.ceil((x_max - x_min) / self.resolution))
        self.canvas_h = int(math.ceil((y_max - y_min) / self.resolution))

        # Инициализируем аккумуляторы
        self.sum_bgr    = np.zeros((self.canvas_h, self.canvas_w, 3),
                                   dtype=np.float32)
        self.sum_weight = np.zeros((self.canvas_h, self.canvas_w),
                                   dtype=np.float32)

        print(f'[canvas] Мировые границы: X=[{x_min:.1f}, {x_max:.1f}] м, '
              f'Y=[{y_min:.1f}, {y_max:.1f}] м')
        print(f'[canvas] Размер холста:   {self.canvas_w} × {self.canvas_h} пкс '
              f'({self.canvas_w*self.resolution:.1f} × '
              f'{self.canvas_h*self.resolution:.1f} м)')

    # ── шаг 2: наложить все снимки ────────────────────────────────────────────

    def process_all(self, photos: pd.DataFrame, imgdir: str):
        """
        Основной цикл: для каждого снимка выполняет ундисторсию,
        геопривязку и смешивание с холстом.
        """
        total = len(photos)
        processed = 0

        for idx, row in photos.iterrows():
            fpath = self._find_image(row['filename'], imgdir)
            if fpath is None:
                continue

            img = cv2.imread(fpath)
            if img is None:
                print(f'  [!] Не удалось прочитать: {fpath}')
                continue

            agl = abs(float(row['agl_m']))
            if agl < 0.3:
                continue

            h_px, w_px = img.shape[:2]
            K = camera_intrinsics(w_px, h_px, self.hfov_deg)

            # ── a) Устранение дисторсии ───────────────────────────────────────
            img_undist = self._undistort(img, K)

            # ── b) Гомография: изображение → холст ───────────────────────────
            R = self._build_rotation(row['yaw_rad'],
                                     row['roll_rad'],
                                     row['pitch_rad'])

            corners_world = image_footprint(w_px, h_px, K, R,
                                            float(row['pos_x']),
                                            float(row['pos_y']),
                                            agl)

            # Переводим мировые координаты в пиксели холста
            corners_canvas = self._world_to_canvas_pts(corners_world)

            # Исходные пиксели (4 угла снимка)
            corners_img = np.array([[0,    0],
                                     [w_px, 0],
                                     [w_px, h_px],
                                     [0,    h_px]], dtype=np.float32)

            # ── c) Вычисляем гомографию и проецируем снимок на холст ─────────
            H, _ = cv2.findHomography(corners_img, corners_canvas)
            if H is None:
                continue

            warped = cv2.warpPerspective(
                img_undist.astype(np.float32), H,
                (self.canvas_w, self.canvas_h),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=0
            )

            # ── d) Маска и весовая карта (feathering) ────────────────────────
            mask_src = np.ones((h_px, w_px), dtype=np.float32)
            weight_map = self._feather_weight(h_px, w_px)

            warped_mask   = cv2.warpPerspective(
                mask_src, H, (self.canvas_w, self.canvas_h),
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            warped_weight = cv2.warpPerspective(
                weight_map, H, (self.canvas_w, self.canvas_h),
                flags=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

            # Маскируем нули (зоны вне снимка)
            valid = (warped_mask > 0.5)
            self.sum_bgr[valid]    += warped[valid] * warped_weight[valid, np.newaxis]
            self.sum_weight[valid] += warped_weight[valid]

            processed += 1
            if processed % 10 == 0 or processed == total:
                print(f'  Обработано: {processed}/{total}', end='\r')

        print(f'\n[process] Готово. Снимков добавлено: {processed}')

    # ── шаг 3: финализировать карту ───────────────────────────────────────────

    def finalize(self) -> np.ndarray:
        """
        Нормализует аккумуляторы и возвращает итоговое изображение (BGR uint8).
        Пиксели без покрытия окрашиваются в серый (50 50 50).
        """
        result = np.full((self.canvas_h, self.canvas_w, 3),
                         50, dtype=np.float32)          # фон — тёмно-серый
        covered = self.sum_weight > 0
        result[covered] = (self.sum_bgr[covered] /
                           self.sum_weight[covered, np.newaxis])
        return result.clip(0, 255).astype(np.uint8)

    # ── шаг 4: отрисовать препятствия ─────────────────────────────────────────

    def draw_obstacles(self, mosaic: np.ndarray, obstacles: list) -> np.ndarray:
        """
        Рисует препятствия (окружности) поверх ортофотоплана.
        Центр окружности соответствует реальным координатам.
        """
        out = mosaic.copy()
        for ox, oy, orad in obstacles:
            # Центр препятствия → пиксели холста
            cx = int((ox - self.origin_x) / self.resolution)
            cy = int((oy - self.origin_y) / self.resolution)
            r  = max(1, int(orad / self.resolution))
            # Полупрозрачный красный кружок
            overlay = out.copy()
            cv2.circle(overlay, (cx, cy), r, (0, 0, 220), -1)
            cv2.addWeighted(overlay, 0.35, out, 0.65, 0, out)
            cv2.circle(out, (cx, cy), r, (0, 0, 255), 2)
            cv2.circle(out, (cx, cy), 3, (0, 0, 255), -1)
        return out

    # ── шаг 5: нанести координатную сетку ────────────────────────────────────

    def draw_grid(self, mosaic: np.ndarray,
                  step_m: float = 10.0) -> np.ndarray:
        """
        Добавляет координатную сетку с подписями осей (шаг step_m метров).
        """
        out = mosaic.copy()
        h, w = out.shape[:2]
        color_grid  = (180, 180, 180)
        color_label = (240, 240, 240)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fs   = 0.4

        # Вертикальные линии (по X)
        x_start = math.ceil(self.origin_x / step_m) * step_m
        x = x_start
        while x <= self.origin_x + w * self.resolution:
            px = int((x - self.origin_x) / self.resolution)
            cv2.line(out, (px, 0), (px, h), color_grid, 1)
            cv2.putText(out, f'{x:.0f}m', (px + 2, h - 4),
                        font, fs, color_label, 1, cv2.LINE_AA)
            x += step_m

        # Горизонтальные линии (по Y)
        y_start = math.ceil(self.origin_y / step_m) * step_m
        y = y_start
        while y <= self.origin_y + h * self.resolution:
            py = int((y - self.origin_y) / self.resolution)
            cv2.line(out, (0, py), (w, py), color_grid, 1)
            cv2.putText(out, f'{y:.0f}m', (2, py - 2),
                        font, fs, color_label, 1, cv2.LINE_AA)
            y += step_m

        return out

    # ── вспомогательные методы ────────────────────────────────────────────────

    def _build_rotation(self, yaw: float, roll: float,
                        pitch: float) -> np.ndarray:
        """
        Строит матрицу поворота «тело дрона → мир» (ENU).
        Порядок: сначала крен, затем тангаж, затем рысканье.
        Камера смотрит вниз (в систему координат которой ось Z направлена
        к земле), поэтому добавляем разворот на 180° вокруг X.
        """
        R_body = rotation_z(yaw) @ rotation_y(pitch) @ rotation_x(roll)
        # Камера смотрит вниз: поворачиваем на 180° вокруг оси X
        R_down = rotation_x(math.pi)
        return R_body @ R_down

    def _undistort(self, img: np.ndarray, K: np.ndarray) -> np.ndarray:
        """
        Устраняет дисторсию объектива.
        При d ≈ 0 изображение не меняется, но структура готова для
        работы с реальной камерой.
        """
        return cv2.undistort(img, K, DIST_COEFFS)

    @staticmethod
    def _feather_weight(h: int, w: int) -> np.ndarray:
        """
        Генерирует карту весов: 1.0 в центре → 0.0 у краёв (feathering).
        Уменьшает видимость «швов» при смешивании перекрывающихся снимков.
        """
        ys = np.linspace(0, 1, h)
        xs = np.linspace(0, 1, w)
        wx = np.minimum(xs, 1 - xs) * 2       # 0 → 1 → 0 по горизонтали
        wy = np.minimum(ys, 1 - ys) * 2       # 0 → 1 → 0 по вертикали
        weight = np.outer(wy, wx).astype(np.float32)
        weight = np.clip(weight, 0.01, 1.0)   # минимальный вес ≠ 0
        return weight

    def _world_to_canvas_pts(self, corners_world: np.ndarray) -> np.ndarray:
        """
        Переводит 4 угла (world coords, м) в пиксели холста (float32).
        """
        out = []
        for gx, gy in corners_world:
            cx = (gx - self.origin_x) / self.resolution
            cy = (gy - self.origin_y) / self.resolution
            out.append([cx, cy])
        return np.array(out, dtype=np.float32)

    def _find_image(self, filename: str, imgdir: str) -> str | None:
        """
        Ищет файл снимка в нескольких возможных местах.
        """
        candidates = [
            filename,
            os.path.join(imgdir, filename),
            os.path.join(imgdir, os.path.basename(filename)),
        ]
        for c in candidates:
            if os.path.isfile(c):
                return c
        return None

    def coverage_stats(self) -> dict:
        """
        Считает процент покрытия холста и статистику весов.
        """
        total_px   = self.canvas_w * self.canvas_h
        covered_px = int((self.sum_weight > 0).sum())
        return {
            'total_pixels':   total_px,
            'covered_pixels': covered_px,
            'coverage_pct':   100.0 * covered_px / total_px,
        }


# ─────────────────────────────────────────────────────────────────────────────
# Демо-режим: генерация синтетических снимков для проверки алгоритма
# ─────────────────────────────────────────────────────────────────────────────

def generate_demo_data(out_dir: str = '/tmp/demo_map',
                       n_drones: int = DEMO_N_DRONES,
                       field_len: float = DEMO_FIELD_LEN_M,
                       field_wid: float = DEMO_FIELD_WID_M,
                       altitude: float = DEMO_ALTITUDE_M,
                       img_w: int = DEMO_IMG_W,
                       img_h: int = DEMO_IMG_H) -> tuple:
    """
    Создаёт синтетические данные:
      - Тестовое изображение поля (чёрно-белая текстура)
      - DataFrame с телеметрией (photos_*.csv)
    Возвращает (DataFrame, imgdir).
    """
    import shutil
    imgdir = os.path.join(out_dir, 'raw_images')
    os.makedirs(imgdir, exist_ok=True)

    # Создаём текстуру поля (разрешение = CANVAS_RESOLUTION)
    res   = CANVAS_RESOLUTION
    fw_px = int(field_len / res)
    fh_px = int(field_wid / res)
    field = np.full((fh_px, fw_px, 3), 100, dtype=np.uint8)

    # Клетчатый узор для визуальной проверки искажений
    for i in range(0, fw_px, 20):
        for j in range(0, fh_px, 20):
            if (i // 20 + j // 20) % 2 == 0:
                field[j:j+20, i:i+20] = (80, 80, 80)

    # Несколько ярких ориентиров
    landmarks = [(10, 5), (30, 15), (50, 25), (15, 20), (45, 10)]
    for lx, ly in landmarks:
        px = int(lx / res)
        py = int(ly / res)
        cv2.circle(field, (px, py), 8, (0, 200, 0), -1)
        cv2.putText(field, f'({lx},{ly})', (px+5, py-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (255, 255, 255), 1)

    rows = []
    photo_id = 0

    # FOV → размер footprint на земле
    hfov_rad  = math.radians(CAM_HFOV_DEG)
    foot_w    = 2.0 * altitude * math.tan(hfov_rad / 2.0)   # м
    foot_h    = foot_w * img_h / img_w                       # м
    overlap   = CAM_OVERLAP
    step_x    = foot_w * (1 - overlap)
    step_y    = foot_h * (1 - overlap)

    for drone_id in range(n_drones):
        lane_y = field_wid * (drone_id + 0.5) / n_drones

        x = 0.0
        direction = 1
        while True:
            # Вырезаем footprint из синтетического поля
            fx0 = x - foot_w / 2
            fy0 = lane_y - foot_h / 2
            fx1 = x + foot_w / 2
            fy1 = lane_y + foot_h / 2

            if fx1 > field_len and direction == 1:
                direction = -1
                lane_y_next = lane_y + step_y
                if lane_y_next > field_wid:
                    break
                x = field_len
                lane_y = lane_y_next
                continue
            elif fx0 < 0 and direction == -1:
                direction = 1
                lane_y_next = lane_y + step_y
                if lane_y_next > field_wid:
                    break
                x = 0.0
                lane_y = lane_y_next
                continue

            # Координаты в пикселях синтетического поля
            px0 = int(max(0, fx0 / res))
            py0 = int(max(0, fy0 / res))
            px1 = int(min(fw_px, fx1 / res))
            py1 = int(min(fh_px, fy1 / res))

            if px1 > px0 and py1 > py0:
                patch = field[py0:py1, px0:px1]
                patch_resized = cv2.resize(patch, (img_w, img_h))

                # Добавляем небольшой шум (имитация реальной камеры)
                noise = np.random.randint(-8, 8,
                                         patch_resized.shape,
                                         dtype=np.int16)
                patch_noisy = (patch_resized.astype(np.int16) + noise)
                patch_noisy = patch_noisy.clip(0, 255).astype(np.uint8)

                fname = f'uav{drone_id}_photo_{photo_id:05d}.jpg'
                fpath = os.path.join(imgdir, fname)
                cv2.imwrite(fpath, patch_noisy)

                # Небольшие случайные шумы телеметрии
                yaw   = 0.0 + np.random.randn() * 0.02
                roll  = np.random.randn() * 0.01
                pitch = np.random.randn() * 0.01

                rows.append({
                    'time_s':    photo_id * 2.0,
                    'filename':  fname,
                    'pos_x':     x,
                    'pos_y':     lane_y,
                    'pos_z':     altitude,
                    'agl_m':     altitude,
                    'yaw_rad':   yaw,
                    'roll_rad':  roll,
                    'pitch_rad': pitch,
                    'vel_mps':   2.0,
                    'ok':        1,
                    'event':     'SHOT',
                })
                photo_id += 1

            x += step_x * direction

    df = pd.DataFrame(rows)
    csv_path = os.path.join(out_dir, 'photos_demo.csv')
    df.to_csv(csv_path, index=False)

    print(f'[demo] Синтетических снимков: {len(df)} '
          f'(3 дрона, поле {field_len}×{field_wid} м)')
    return df, imgdir


# ─────────────────────────────────────────────────────────────────────────────
# Точка входа
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Формирование ортофотоплана из серии аэрофотоснимков.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__
    )
    parser.add_argument('--photos', nargs='+', default=None,
                        help='Пути к photos_*.csv (можно несколько файлов)')
    parser.add_argument('--imgdir', default='.',
                        help='Папка с JPEG-снимками (raw_images/uavN)')
    parser.add_argument('--out', default='mosaic.jpg',
                        help='Путь для сохранения ортофотоплана')
    parser.add_argument('--obstacles', default=None,
                        help='Путь к obstacles.txt (необязательно)')
    parser.add_argument('--resolution', type=float, default=CANVAS_RESOLUTION,
                        help=f'ГРД, м/пиксель (по умолчанию {CANVAS_RESOLUTION})')
    parser.add_argument('--fov', type=float, default=CAM_HFOV_DEG,
                        help=f'Горизонтальный FOV камеры, ° (по умолчанию {CAM_HFOV_DEG})')
    parser.add_argument('--grid', action='store_true',
                        help='Нанести координатную сетку на карту')
    parser.add_argument('--demo', action='store_true',
                        help='Запустить на синтетических данных (тест алгоритма)')
    args = parser.parse_args()

    # ── Режим демо ────────────────────────────────────────────────────────────
    if args.demo:
        print('=== Режим демо: генерация синтетического поля ===')
        photos, imgdir = generate_demo_data()
        obstacles = []
        out_path = args.out if args.out != 'mosaic.jpg' else 'mosaic_demo.jpg'
    else:
        if not args.photos:
            print('[ошибка] Укажите --photos или используйте --demo')
            sys.exit(1)
        photos = load_photos_csv(args.photos)
        imgdir = args.imgdir
        obstacles = load_obstacles(args.obstacles)
        out_path = args.out

        if len(photos) == 0:
            print('[ошибка] Нет снимков с ok=1. Проверьте CSV и imgdir.')
            sys.exit(1)

    # ── Построение карты ──────────────────────────────────────────────────────
    builder = MapBuilder(resolution=args.resolution, hfov_deg=args.fov)

    print('\n[1/4] Определение границ холста...')
    builder.compute_canvas_bounds(photos, imgdir)

    print('\n[2/4] Наложение снимков...')
    builder.process_all(photos, imgdir)

    print('\n[3/4] Финализация...')
    mosaic = builder.finalize()

    # Препятствия
    if obstacles:
        mosaic = builder.draw_obstacles(mosaic, obstacles)

    # Сетка
    if args.grid:
        mosaic = builder.draw_grid(mosaic, step_m=10.0)

    # ── Сохранение ────────────────────────────────────────────────────────────
    cv2.imwrite(out_path, mosaic)

    stats = builder.coverage_stats()
    print(f'\n[4/4] Готово!')
    print(f'      Файл:     {out_path}')
    print(f'      Размер:   {mosaic.shape[1]} × {mosaic.shape[0]} пкс')
    print(f'      Покрытие: {stats["coverage_pct"]:.1f}% '
          f'({stats["covered_pixels"]:,} / {stats["total_pixels"]:,} пкс)')

    # Сохраняем миниатюру для быстрого просмотра
    thumb_path = out_path.rsplit('.', 1)[0] + '_thumb.jpg'
    scale = min(1.0, 1200 / max(mosaic.shape[:2]))
    thumb = cv2.resize(mosaic,
                       (int(mosaic.shape[1] * scale),
                        int(mosaic.shape[0] * scale)))
    cv2.imwrite(thumb_path, thumb)
    print(f'      Миниатюра: {thumb_path}')


if __name__ == '__main__':
    main()
