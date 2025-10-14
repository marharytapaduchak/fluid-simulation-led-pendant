"""Inspired by video Matthias Müller - Ten Minute Physics"""

import math
import pygame
import numpy as np
from numba import jit

U_FIELD = 0
V_FIELD = 1
S_FIELD = 2 #стіна

@jit(nopython=True)
def integrate_jit(v, s, num_x, num_y, gravity, dt):
    """
    додає гравітацію до вертикальної швидкості v у клітинках рідини
    """

    n = num_y
    for i in range(1, num_x):
        for j in range(1, num_y - 1):
            if s[i*n + j] != 0.0 and s[i*n + j-1] != 0.0:
                v[i*n + j] += gravity * dt

@jit(nopython=True)
def solve_incompressibility_jit(u, v, p, s, num_x, num_y, num_iters, cp, over_relaxation):
    """
    приводить поле швидкостей до бездивергентного (нестисного) стану та обчислює тиск p
    Гаус–Зайдель з overrelaxation
    """

    n = num_y
    for _ in range(num_iters):
        for i in range(1, num_x - 1):
            for j in range(1, num_y - 1):
                if s[i*n + j] == 0.0:
                    continue

                sx0 = s[(i-1)*n + j]
                sx1 = s[(i+1)*n + j]
                sy0 = s[i*n + j-1]
                sy1 = s[i*n + j+1]
                s_sum = sx0 + sx1 + sy0 + sy1

                if s_sum == 0.0:
                    continue

                div = (u[(i+1)*n + j] - u[i*n + j] +
                       v[i*n + j+1] - v[i*n + j])

                p_val = -div / s_sum
                p_val *= over_relaxation
                p[i*n + j] += cp * p_val

                u[i*n + j] -= sx0 * p_val
                u[(i+1)*n + j] += sx1 * p_val
                v[i*n + j] -= sy0 * p_val
                v[i*n + j+1] += sy1 * p_val

@jit(nopython=True)
def extrapolate_jit(u, v, num_x, num_y):
    """
    адекватні граничні умови на бордерах сітки
    дає нульовий градієнт на межі
    """

    n = num_y
    for i in range(num_x):
        u[i*n + 0] = u[i*n + 1]
        u[i*n + num_y-1] = u[i*n + num_y-2]

    for j in range(num_y):
        v[0*n + j] = v[1*n + j]
        v[(num_x-1)*n + j] = v[(num_x-2)*n + j]

@jit(nopython=True)
def sample_field_jit(x, y, field, u, v, m, num_x, num_y, h):
    """
    білінійна інтерполяція значення поля у довільній точці
    """

    n = num_y
    h1 = 1.0 / h
    h2 = 0.5 * h

    x = max(min(x, num_x * h), h)
    y = max(min(y, num_y * h), h)

    dx = 0.0
    dy = 0.0

    if field == U_FIELD:
        f = u
        dy = h2
    elif field == V_FIELD:
        f = v
        dx = h2
    else:  # S_FIELD
        f = m
        dx = h2
        dy = h2

    x0 = min(int((x-dx)*h1), num_x-1)
    tx = ((x-dx) - x0*h) * h1
    x1 = min(x0 + 1, num_x-1)

    y0 = min(int((y-dy)*h1), num_y-1)
    ty = ((y-dy) - y0*h) * h1
    y1 = min(y0 + 1, num_y-1)

    sx = 1.0 - tx
    sy = 1.0 - ty

    val = (sx*sy * f[x0*n + y0] +
           tx*sy * f[x1*n + y0] +
           tx*ty * f[x1*n + y1] +
           sx*ty * f[x0*n + y1])

    return val

@jit(nopython=True)
def advect_vel_jit(u, v, new_u, new_v, s, num_x, num_y, h, dt):
    """
    адвекція швидкостей (перенесення значень уздовж потоку)
    """

    new_u[:] = u
    new_v[:] = v
    n = num_y
    h2 = 0.5 * h

    for i in range(1, num_x):
        for j in range(1, num_y):
            # u component
            if (s[i*n + j] != 0.0 and s[(i-1)*n + j] != 0.0 
                and j < num_y - 1):
                x = i*h
                y = j*h + h2
                u_val = u[i*n + j]
                # avg_v
                v_val = (v[(i-1)*n + j] + v[i*n + j] +
                        v[(i-1)*n + j+1] + v[i*n + j+1]) * 0.25
                x = x - dt*u_val
                y = y - dt*v_val
                u_val = sample_field_jit(x, y, U_FIELD, u, v, u, num_x, num_y, h)
                new_u[i*n + j] = u_val

            # v component
            if (s[i*n + j] != 0.0 and s[i*n + j-1] != 0.0 
                and i < num_x - 1):
                x = i*h + h2
                y = j*h
                # avg_u
                u_val = (u[i*n + j-1] + u[i*n + j] +
                        u[(i+1)*n + j-1] + u[(i+1)*n + j]) * 0.25
                v_val = v[i*n + j]
                x = x - dt*u_val
                y = y - dt*v_val
                v_val = sample_field_jit(x, y, V_FIELD, u, v, v, num_x, num_y, h)
                new_v[i*n + j] = v_val

    u[:] = new_u
    v[:] = new_v

@jit(nopython=True)
def advect_smoke_jit(u, v, m, new_m, s, num_x, num_y, h, dt):
    """
    переносить скалярне поле (в нас дим)
    """

    new_m[:] = m
    n = num_y
    h2 = 0.5 * h

    for i in range(1, num_x - 1):
        for j in range(1, num_y - 1):
            if s[i*n + j] != 0.0:
                u_val = (u[i*n + j] + u[(i+1)*n + j]) * 0.5
                v_val = (v[i*n + j] + v[i*n + j+1]) * 0.5
                x = i*h + h2 - dt*u_val
                y = j*h + h2 - dt*v_val

                new_m[i*n + j] = sample_field_jit(x, y, S_FIELD, u, v, m, num_x, num_y, h)

    m[:] = new_m

@jit(nopython=True)
def get_sci_color(val, min_val, max_val):
    """
    перетворює скаляр у псевдокольори для візуалізації тиску
    """

    val = max(min(val, max_val - 0.0001), min_val)
    d = max_val - min_val
    val = 0.5 if d == 0.0 else (val - min_val) / d

    m = 0.25
    num = int(val / m)
    s = (val - num * m) / m

    if num == 0:
        r, g, b = 0.0, s, 1.0
    elif num == 1:
        r, g, b = 0.0, 1.0, 1.0-s
    elif num == 2:
        r, g, b = s, 1.0, 0.0
    else:  # num == 3
        r, g, b = 1.0, 1.0 - s, 0.0

    return int(255*r), int(255*g), int(255*b)

@jit(nopython=True)
def render_grid(pixels, p, m, s, num_x, num_y, width, height, h, c_scale,
                show_pressure, show_smoke, scene_nr, min_p, max_p):
    """
    малює grid
    """

    n = num_y
    cell_scale = 1.1

    for i in range(num_x):
        for j in range(num_y):
            if show_pressure:
                p_val = p[i*n + j]
                r, g, b = get_sci_color(p_val, min_p, max_p)

                if show_smoke:
                    s_val = m[i*n + j]
                    r = max(0, r - int(255*s_val))
                    g = max(0, g - int(255*s_val))
                    b = max(0, b - int(255*s_val))
            elif show_smoke:
                s_val = m[i*n + j]
                if scene_nr == 2:
                    r, g, b = get_sci_color(s_val, 0.0, 1.0)
                else:
                    val = int(255*s_val)
                    r, g, b = val, val, val
            elif s[i*n + j] == 0.0:
                r, g, b = 0, 0, 0
            else:
                r, g, b = 255, 255, 255

            x = int(i * h * c_scale)
            y = int(height - (j+1) * h * c_scale)
            cx = int(c_scale * cell_scale * h) + 1
            cy = int(c_scale * cell_scale * h) + 1

            #прямокутник
            for yi in range(max(0, y), min(height, y + cy)):
                for xi in range(max(0, x), min(width, x + cx)):
                    pixels[yi, xi, 0] = r
                    pixels[yi, xi, 1] = g
                    pixels[yi, xi, 2] = b

class Fluid:
    """
    клас для полів рідини
    """

    def __init__(self, density, num_x, num_y, h):
        self.density = density
        self.num_x = num_x + 2
        self.num_y = num_y + 2
        self.num_cells = self.num_x * self.num_y
        self.h = h

        self.u = np.zeros(self.num_cells, dtype=np.float32)
        self.v = np.zeros(self.num_cells, dtype=np.float32)
        self.new_u = np.zeros(self.num_cells, dtype=np.float32)
        self.new_v = np.zeros(self.num_cells, dtype=np.float32)
        self.p = np.zeros(self.num_cells, dtype=np.float32)
        self.s = np.zeros(self.num_cells, dtype=np.float32)
        self.m = np.ones(self.num_cells, dtype=np.float32)
        self.new_m = np.zeros(self.num_cells, dtype=np.float32)

    def sample_field(self, x, y, field):
        """
        обгортка над sample_field_jit()
        """

        return sample_field_jit(x, y, field, self.u, self.v, self.m,
                                self.num_x, self.num_y, self.h)

    def simulate(self, dt, gravity, num_iters, over_relaxation):
        """
        один крок симуляції
        """

        integrate_jit(self.v, self.s, self.num_x, self.num_y, gravity, dt)

        self.p.fill(0.0)
        cp = self.density * self.h / dt
        solve_incompressibility_jit(self.u, self.v, self.p, self.s, 
                                    self.num_x, self.num_y, num_iters, cp, over_relaxation)

        extrapolate_jit(self.u, self.v, self.num_x, self.num_y)
        advect_vel_jit(self.u, self.v, self.new_u, self.new_v, self.s, 
                      self.num_x, self.num_y, self.h, dt)
        advect_smoke_jit(self.u, self.v, self.m, self.new_m, self.s,
                        self.num_x, self.num_y, self.h, dt)

class Scene:
    """
    UI
    """

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.sim_height = 1.1
        self.c_scale = height / self.sim_height
        self.sim_width = width / self.c_scale

        self.gravity = -9.81
        self.dt = 1.0 / 120.0
        self.num_iters = 100
        self.frame_nr = 0
        self.over_relaxation = 1.9
        self.obstacle_x = 0.0
        self.obstacle_y = 0.0
        self.obstacle_radius = 0.15
        self.paused = False
        self.scene_nr = 0
        self.show_obstacle = False
        self.show_streamlines = False
        self.show_velocities = False
        self.show_pressure = False
        self.show_smoke = True
        self.fluid = None

    def c_x(self, x):
        """
        координати симуляції у пікселі екрана
        """

        return int(x * self.c_scale)

    def c_y(self, y):
        """
        координати симуляції у пікселі екрана
        """

        return int(self.height - y * self.c_scale)

    def setup_scene(self, scene_nr):
        """
        Створює Fluid з потрібною роздільністю і виставляє маску s, 
        граничні умови, вхідну швидкість, гравітацію і режими візуалізації для різних сцен
        """

        self.scene_nr = scene_nr
        self.obstacle_radius = 0.15
        self.over_relaxation = 1.9
        self.dt = 1.0 / 60.0
        self.num_iters = 40

        res = 100
        if scene_nr == 0:
            res = 50
        elif scene_nr == 3:
            res = 200

        domain_height = 1.0
        domain_width = domain_height / self.sim_height * self.sim_width
        h = domain_height / res

        num_x = int(domain_width / h)
        num_y = int(domain_height / h)

        density = 1000.0
        self.fluid = Fluid(density, num_x, num_y, h)
        f = self.fluid
        n = f.num_y

        if scene_nr == 0:  # Tank - резервуар з водою та гравітацією
            for i in range(f.num_x):
                for j in range(f.num_y):
                    s = 1.0
                    if i == 0 or i == f.num_x-1 or j == 0:
                        s = 0.0
                    f.s[i*n + j] = s

            self.gravity = -9.81
            self.show_pressure = True
            self.show_smoke = False

        elif scene_nr == 1 or scene_nr == 3:  # Wind tunnel - аеродинамічна труба
            in_vel = 2.0
            for i in range(f.num_x):
                for j in range(f.num_y):
                    s = 1.0
                    if i == 0 or j == 0 or j == f.num_y-1:
                        s = 0.0
                    f.s[i*n + j] = s

                    if i == 1:
                        f.u[i*n + j] = in_vel

            pipe_h = 0.1 * f.num_y
            min_j = int(0.5 * f.num_y - 0.5*pipe_h)
            max_j = int(0.5 * f.num_y + 0.5*pipe_h)

            for j in range(min_j, max_j):
                f.m[j] = 0.0

            self.set_obstacle(0.4, 0.5, True)

            self.gravity = 0.0
            self.show_pressure = False
            self.show_smoke = True

            if scene_nr == 3:
                self.dt = 1.0 / 120.0
                self.num_iters = 100
                self.show_pressure = True

        elif scene_nr == 2:  # Paint - малювання кольоровим димом
            self.gravity = 0.0
            self.over_relaxation = 1.0
            self.show_pressure = False
            self.show_smoke = True
            self.obstacle_radius = 0.1

    def set_obstacle(self, x, y, reset):
        """
        рухома перешкода
        """
        vx = 0.0
        vy = 0.0

        if not reset:
            vx = (x - self.obstacle_x) / self.dt
            vy = (y - self.obstacle_y) / self.dt

        self.obstacle_x = x
        self.obstacle_y = y
        r = self.obstacle_radius
        f = self.fluid
        n = f.num_y

        for i in range(1, f.num_x - 2):
            for j in range(1, f.num_y - 2):
                f.s[i*n + j] = 1.0

                dx = (i + 0.5) * f.h - x
                dy = (j + 0.5) * f.h - y

                if dx * dx + dy * dy < r * r:
                    f.s[i*n + j] = 0.0
                    if self.scene_nr == 2:
                        f.m[i*n + j] = 0.5 + 0.5 * math.sin(0.1 * self.frame_nr)
                    else:
                        f.m[i*n + j] = 1.0
                    f.u[i*n + j] = vx
                    f.u[(i+1)*n + j] = vx
                    f.v[i*n + j] = vy
                    f.v[i*n + j+1] = vy

        self.show_obstacle = True


def main():
    """
    let the show begin
    """

    pygame.init()

    width = 1200
    height = 700
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("Симуляція рідини Ейлера")

    scene = Scene(width, height)
    scene.setup_scene(1)

    clock = pygame.time.Clock()
    running = True
    mouse_down = False

    font = pygame.font.Font(None, 20)
    font_large = pygame.font.Font(None, 24)

    pixels = np.zeros((height, width, 3), dtype=np.uint8)

    # scene_names = [
    #     "Tank (Резервуар з водою)",
    #     "Wind Tunnel (Аеродинамічна труба)",
    #     "Paint (Малювання димом)",
    #     "Hires Tunnel (Високоякісна труба)"
    # ]

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:
                    scene.paused = not scene.paused
                elif event.key == pygame.K_1:
                    scene.setup_scene(0)
                elif event.key == pygame.K_2:
                    scene.setup_scene(1)
                elif event.key == pygame.K_3:
                    scene.setup_scene(2)
                elif event.key == pygame.K_4:
                    scene.setup_scene(3)
                elif event.key == pygame.K_s:
                    scene.show_streamlines = not scene.show_streamlines
                elif event.key == pygame.K_v:
                    scene.show_velocities = not scene.show_velocities
                elif event.key == pygame.K_r:
                    scene.show_pressure = not scene.show_pressure
                elif event.key == pygame.K_m:
                    scene.show_smoke = not scene.show_smoke
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_down = True
                mx, my = event.pos
                x = mx / scene.c_scale
                y = (height - my) / scene.c_scale
                scene.set_obstacle(x, y, True)
            elif event.type == pygame.MOUSEBUTTONUP:
                mouse_down = False
            elif event.type == pygame.MOUSEMOTION:
                if mouse_down:
                    mx, my = event.pos
                    x = mx / scene.c_scale
                    y = (height - my) / scene.c_scale
                    scene.set_obstacle(x, y, False)

        # Симуляція
        if not scene.paused:
            scene.fluid.simulate(scene.dt, scene.gravity, scene.num_iters, 
                                scene.over_relaxation)
            scene.frame_nr += 1

        f = scene.fluid
        min_p = np.min(f.p)
        max_p = np.max(f.p)

        pixels.fill(255)
        render_grid(pixels, f.p, f.m, f.s, f.num_x, f.num_y, width, height,
                   f.h, scene.c_scale, scene.show_pressure, scene.show_smoke,
                   scene.scene_nr, min_p, max_p)

        pygame.surfarray.blit_array(screen, pixels.transpose(1, 0, 2))

        # Перешкода
        if scene.show_obstacle:
            r = scene.obstacle_radius + f.h
            color = (0, 0, 0) if scene.show_pressure else (221, 221, 221)
            pygame.draw.circle(screen, color,
                             (scene.c_x(scene.obstacle_x), 
                              scene.c_y(scene.obstacle_y)),
                             int(scene.c_scale * r))
            pygame.draw.circle(screen, (0, 0, 0),
                             (scene.c_x(scene.obstacle_x), 
                              scene.c_y(scene.obstacle_y)),
                             int(scene.c_scale * r), 3)

        #стрілки показують напрямок потоку
        if scene.show_velocities:
            scale = 0.02
            n = f.num_y
            step = 5  # Малюємо кожну 5-ту стрілку

            for i in range(0, f.num_x, step):
                for j in range(0, f.num_y, step):
                    u = f.u[i*n + j]
                    v = f.v[i*n + j]

                    x0 = scene.c_x(i * f.h)
                    y0 = scene.c_y((j + 0.5) * f.h)
                    x1 = scene.c_x(i * f.h + u * scale)
                    y1 = y0

                    pygame.draw.line(screen, (255, 0, 0), (x0, y0), (x1, y1), 1)

                    x0 = scene.c_x((i + 0.5) * f.h)
                    y0 = scene.c_y(j * f.h)
                    x1 = x0
                    y1 = scene.c_y(j * f.h + v * scale)

                    pygame.draw.line(screen, (0, 0, 255), (x0, y0), (x0, y1), 1)

        #показують шлях руху частинок
        if scene.show_streamlines:
            seg_len = f.h * 0.2
            num_segs = 15

            for i in range(1, f.num_x - 1, 5):
                for j in range(1, f.num_y - 1, 5):
                    x = (i + 0.5) * f.h
                    y = (j + 0.5) * f.h

                    points = [(scene.c_x(x), scene.c_y(y))]

                    for n in range(num_segs):
                        u = f.sample_field(x, y, U_FIELD)
                        v = f.sample_field(x, y, V_FIELD)
                        x += u * 0.01
                        y += v * 0.01

                        if x > f.num_x * f.h:
                            break

                        points.append((scene.c_x(x), scene.c_y(y)))

                    if len(points) > 1:
                        pygame.draw.lines(screen, (0, 0, 0), False, points, 1)

        info_bg = pygame.Surface((width, 120))
        info_bg.set_alpha(200)
        info_bg.fill((240, 240, 240))
        screen.blit(info_bg, (0, 0))

        scene_text = font_large.render(" ", True, (0, 0, 0))
        screen.blit(scene_text, (10, 10))

        controls = [
            "P-Пауза",
            "S-Лінії потоку  V-Вектори швидкості  R-Тиск  M-Дим"
        ]

        for i, text in enumerate(controls):
            surf = font.render(text, True, (50, 50, 50))
            screen.blit(surf, (10, 40 + i * 25))

        # FPS
        fps_text = font_large.render(f"FPS: {int(clock.get_fps())}", True, (0, 100, 0))
        screen.blit(fps_text, (width - 100, 10))

        status = []
        if scene.paused:
            status.append("ПАУЗА")
        if scene.show_streamlines:
            status.append("Streamlines")
        if scene.show_velocities:
            status.append("Velocities")
        if scene.show_pressure:
            status.append("Pressure")
        if scene.show_smoke:
            status.append("Smoke")

        if status:
            status_text = font.render(" | ".join(status), True, (0, 0, 200))
            screen.blit(status_text, (width - 400, 40))

        pygame.display.flip()
        clock.tick(60)

    pygame.quit()


if __name__ == "__main__":
    main()
