from __future__ import annotations

import ctypes
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import numpy as np
import trimesh
from OpenGL.GL import (
    GL_ARRAY_BUFFER,
    GL_BLEND,
    GL_COLOR_BUFFER_BIT,
    GL_COMPILE_STATUS,
    GL_DEPTH_BUFFER_BIT,
    GL_DEPTH_TEST,
    GL_ELEMENT_ARRAY_BUFFER,
    GL_FALSE,
    GL_FLOAT,
    GL_FRAGMENT_SHADER,
    GL_LEQUAL,
    GL_LINES,
    GL_LINK_STATUS,
    GL_ONE_MINUS_SRC_ALPHA,
    GL_SRC_ALPHA,
    GL_STATIC_DRAW,
    GL_TRIANGLES,
    GL_TRUE,
    GL_UNSIGNED_INT,
    GL_VERTEX_SHADER,
    glAttachShader,
    glBindAttribLocation,
    glBindBuffer,
    glBindVertexArray,
    glBlendFunc,
    glBufferData,
    glClear,
    glClearColor,
    glCompileShader,
    glCreateProgram,
    glCreateShader,
    glDeleteBuffers,
    glDeleteProgram,
    glDeleteShader,
    glDeleteVertexArrays,
    glDepthFunc,
    glDepthMask,
    glDisable,
    glDrawArrays,
    glDrawElements,
    glEnable,
    glEnableVertexAttribArray,
    glGenBuffers,
    glGenVertexArrays,
    glGetProgramInfoLog,
    glGetProgramiv,
    glGetShaderInfoLog,
    glGetShaderiv,
    glGetUniformLocation,
    glLineWidth,
    glLinkProgram,
    glShaderSource,
    glUniform4f,
    glUniformMatrix4fv,
    glUseProgram,
    glVertexAttribPointer,
    glViewport,
)
from PyQt5 import QtCore
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QColor, QFont, QPainter
from PyQt5.QtWidgets import QOpenGLWidget, QSizePolicy

from UR_Utils.ur5e_model_cache import build_models_from_mjcf


MESH_VERTEX_SHADER = """
#version 120
attribute vec3 a_position;
attribute vec3 a_normal;

uniform mat4 u_mvp;
uniform mat4 u_model;

varying vec3 v_normal;

void main()
{
    v_normal = normalize(mat3(u_model) * a_normal);
    gl_Position = u_mvp * vec4(a_position, 1.0);
}
"""


MESH_FRAGMENT_SHADER = """
#version 120
uniform vec4 u_color;

varying vec3 v_normal;

void main()
{
    vec3 normal = normalize(v_normal);
    vec3 light_dir = normalize(vec3(1.0, 2.0, 1.0));
    vec3 half_dir = normalize(light_dir + vec3(0.0, 0.0, 1.0));

    float diffuse = max(dot(normal, light_dir), 0.0);
    float specular = 0.0;
    if (diffuse > 0.0) {
        specular = pow(max(dot(normal, half_dir), 0.0), 32.0);
    }

    vec3 rgb = u_color.rgb * (0.25 + 0.55 * diffuse) + vec3(0.20 * specular);
    gl_FragColor = vec4(rgb, u_color.a);
}
"""


LINE_VERTEX_SHADER = """
#version 120
attribute vec3 a_position;
attribute vec4 a_color;

uniform mat4 u_mvp;

varying vec4 v_color;

void main()
{
    v_color = a_color;
    gl_Position = u_mvp * vec4(a_position, 1.0);
}
"""


LINE_FRAGMENT_SHADER = """
#version 120
varying vec4 v_color;

void main()
{
    gl_FragColor = v_color;
}
"""


def _normalize(v: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(v))
    if norm < 1e-12:
        return v
    return v / norm


def _look_at(eye: np.ndarray, target: np.ndarray, up: np.ndarray) -> np.ndarray:
    forward = _normalize(target - eye)
    side = _normalize(np.cross(forward, up))
    true_up = np.cross(side, forward)

    rotation = np.eye(4, dtype=np.float32)
    rotation[0, :3] = side
    rotation[1, :3] = true_up
    rotation[2, :3] = -forward

    translation = np.eye(4, dtype=np.float32)
    translation[:3, 3] = -eye
    return rotation @ translation


def _perspective(fov_degrees: float, aspect: float, near: float, far: float) -> np.ndarray:
    aspect = max(float(aspect), 1e-6)
    f = 1.0 / math.tan(math.radians(fov_degrees) * 0.5)
    matrix = np.zeros((4, 4), dtype=np.float32)
    matrix[0, 0] = f / aspect
    matrix[1, 1] = f
    matrix[2, 2] = (far + near) / (near - far)
    matrix[2, 3] = (2.0 * far * near) / (near - far)
    matrix[3, 2] = -1.0
    return matrix


def _as_mat4(matrix: np.ndarray) -> np.ndarray:
    return np.asarray(matrix, dtype=np.float32).reshape(4, 4)


def _set_mat4(location: int, matrix: np.ndarray) -> None:
    glUniformMatrix4fv(location, 1, GL_TRUE, _as_mat4(matrix))


def _compile_shader(source: str, shader_type: int) -> int:
    shader = glCreateShader(shader_type)
    glShaderSource(shader, source)
    glCompileShader(shader)
    if not glGetShaderiv(shader, GL_COMPILE_STATUS):
        info = glGetShaderInfoLog(shader).decode("utf-8", errors="replace")
        glDeleteShader(shader)
        raise RuntimeError(f"OpenGL shader 编译失败: {info}")
    return shader


def _compile_program(
    vertex_source: str,
    fragment_source: str,
    attrib_bindings: Sequence[Tuple[int, str]],
) -> int:
    vertex_shader = _compile_shader(vertex_source, GL_VERTEX_SHADER)
    fragment_shader = _compile_shader(fragment_source, GL_FRAGMENT_SHADER)

    program = glCreateProgram()
    glAttachShader(program, vertex_shader)
    glAttachShader(program, fragment_shader)
    for index, name in attrib_bindings:
        glBindAttribLocation(program, index, name)
    glLinkProgram(program)

    glDeleteShader(vertex_shader)
    glDeleteShader(fragment_shader)

    if not glGetProgramiv(program, GL_LINK_STATUS):
        info = glGetProgramInfoLog(program).decode("utf-8", errors="replace")
        glDeleteProgram(program)
        raise RuntimeError(f"OpenGL shader 链接失败: {info}")
    return program


@dataclass(frozen=True)
class MeshSpec:
    name: str
    path: Path
    color: np.ndarray


@dataclass
class _MeshGpuResource:
    vao: int
    vbo: int
    ebo: int
    index_count: int


class ArcballCamera:
    """围绕目标点旋转的简易相机。"""

    def __init__(
        self,
        azimuth_degrees: float = 45.0,
        elevation_degrees: float = 30.0,
        distance: float = 2.0,
        target: Sequence[float] = (0.0, 0.0, 0.3),
        fov_degrees: float = 45.0,
    ) -> None:
        self.azimuth = math.radians(float(azimuth_degrees))
        self.elevation = math.radians(float(elevation_degrees))
        self.distance = max(0.25, min(8.0, float(distance)))
        self.target = np.asarray(target, dtype=np.float32).reshape(3)
        self.fov_degrees = max(10.0, min(120.0, float(fov_degrees)))
        self._world_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
        self._clamp_elevation()

    def position(self) -> np.ndarray:
        cos_el = math.cos(self.elevation)
        return self.target + self.distance * np.array(
            [
                cos_el * math.cos(self.azimuth),
                cos_el * math.sin(self.azimuth),
                math.sin(self.elevation),
            ],
            dtype=np.float32,
        )

    def view_matrix(self) -> np.ndarray:
        return _look_at(self.position(), self.target, self._world_up)

    def proj_matrix(self, aspect: float) -> np.ndarray:
        return _perspective(self.fov_degrees, aspect, near=0.02, far=20.0)

    def rotate(self, dx: float, dy: float) -> None:
        self.azimuth -= math.radians(dx * 0.35)
        self.elevation += math.radians(dy * 0.35)
        self._clamp_elevation()

    def _clamp_elevation(self) -> None:
        limit = math.radians(88.0)
        self.elevation = max(-limit, min(limit, self.elevation))

    def pan(self, dx: float, dy: float, viewport_height: int) -> None:
        height = max(int(viewport_height), 1)
        pan_scale = self.distance * math.tan(math.radians(self.fov_degrees) * 0.5)
        pan_scale = 2.0 * pan_scale / float(height)

        eye = self.position()
        forward = _normalize(self.target - eye)
        right = _normalize(np.cross(forward, self._world_up))
        up = _normalize(np.cross(right, forward))

        self.target -= right * float(dx) * pan_scale
        self.target += up * float(dy) * pan_scale

    def zoom(self, wheel_steps: float) -> None:
        self.distance *= math.exp(-float(wheel_steps) * 0.12)
        self.distance = max(0.25, min(8.0, self.distance))

    def debug_lines(self) -> List[str]:
        return [
            f"az {math.degrees(self.azimuth):5.1f}  el {math.degrees(self.elevation):5.1f}",
            f"dst {self.distance:4.2f}m  fov {self.fov_degrees:4.1f}",
            (
                "tgt "
                f"{self.target[0]: .2f},{self.target[1]: .2f},{self.target[2]: .2f}m"
            ),
        ]


class GLMesh:
    """单个 OBJ mesh 的 GPU 资源缓存。"""

    _gpu_cache: Dict[Tuple[str, int, int], _MeshGpuResource] = {}

    def __init__(self, path: Path) -> None:
        self.path = Path(path)
        self._cache_key = self._make_cache_key(self.path)
        resource = self._gpu_cache.get(self._cache_key)
        if resource is None:
            resource = self._upload_mesh(self.path)
            self._gpu_cache[self._cache_key] = resource
        self._resource = resource

    @staticmethod
    def _make_cache_key(path: Path) -> Tuple[str, int, int]:
        resolved = Path(path).resolve()
        stat = resolved.stat()
        return str(resolved), stat.st_mtime_ns, stat.st_size

    @staticmethod
    def _load_trimesh(path: Path) -> trimesh.Trimesh:
        mesh = trimesh.load(str(path), force="mesh", process=False)
        if isinstance(mesh, trimesh.Scene):
            geometries = tuple(mesh.geometry.values())
            if not geometries:
                raise ValueError(f"mesh 文件没有可渲染几何: {path}")
            mesh = trimesh.util.concatenate(geometries)
        if not isinstance(mesh, trimesh.Trimesh):
            raise TypeError(f"无法识别的 mesh 类型: {type(mesh)!r}")
        if mesh.vertices.size == 0 or mesh.faces.size == 0:
            raise ValueError(f"mesh 文件为空: {path}")
        return mesh

    @classmethod
    def _upload_mesh(cls, path: Path) -> _MeshGpuResource:
        mesh = cls._load_trimesh(path)

        vertices = np.asarray(mesh.vertices, dtype=np.float32)
        normals = np.asarray(mesh.vertex_normals, dtype=np.float32)
        if normals.shape != vertices.shape:
            normals = np.zeros_like(vertices, dtype=np.float32)
            for face, normal in zip(mesh.faces, mesh.face_normals):
                normals[face] += normal.astype(np.float32)
            lengths = np.linalg.norm(normals, axis=1)
            valid = lengths > 1e-12
            normals[valid] /= lengths[valid, None]
            normals[~valid] = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        interleaved = np.ascontiguousarray(
            np.column_stack((vertices, normals)).astype(np.float32)
        )
        indices = np.ascontiguousarray(mesh.faces.reshape(-1).astype(np.uint32))

        vao = glGenVertexArrays(1)
        vbo = glGenBuffers(1)
        ebo = glGenBuffers(1)

        glBindVertexArray(vao)

        glBindBuffer(GL_ARRAY_BUFFER, vbo)
        glBufferData(GL_ARRAY_BUFFER, interleaved.nbytes, interleaved, GL_STATIC_DRAW)

        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo)
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.nbytes, indices, GL_STATIC_DRAW)

        stride = 6 * ctypes.sizeof(ctypes.c_float)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(0))
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(
            1,
            3,
            GL_FLOAT,
            GL_FALSE,
            stride,
            ctypes.c_void_p(3 * ctypes.sizeof(ctypes.c_float)),
        )

        glBindVertexArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0)

        return _MeshGpuResource(vao=vao, vbo=vbo, ebo=ebo, index_count=int(indices.size))

    def draw(self) -> None:
        glBindVertexArray(self._resource.vao)
        glDrawElements(GL_TRIANGLES, self._resource.index_count, GL_UNSIGNED_INT, None)
        glBindVertexArray(0)

    @classmethod
    def clear_gpu_cache(cls) -> None:
        for resource in cls._gpu_cache.values():
            glDeleteBuffers(1, [resource.vbo])
            glDeleteBuffers(1, [resource.ebo])
            glDeleteVertexArrays(1, [resource.vao])
        cls._gpu_cache.clear()


@dataclass
class _MeshRenderItem:
    spec: MeshSpec
    mesh: GLMesh


@dataclass
class _LineResource:
    vao: int
    vbo: int
    vertex_count: int


class RobotGLWidget(QOpenGLWidget):
    """UR5e 双机械臂原生 OpenGL 可视化控件。"""

    def __init__(
        self,
        mjcf_path: str,
        visual_model=None,
        camera_azimuth_deg: float = 45.0,
        camera_elevation_deg: float = 30.0,
        camera_distance: float = 2.0,
        camera_target: Sequence[float] = (0.0, 0.0, 0.3),
        camera_fov_deg: float = 45.0,
        show_camera_info: bool = True,
        parent=None,
    ) -> None:
        super().__init__(parent)
        self.mjcf_path = str(Path(mjcf_path).resolve())
        self.mesh_specs = self._extract_mesh_specs(self.mjcf_path, visual_model)
        self.camera = ArcballCamera(
            azimuth_degrees=camera_azimuth_deg,
            elevation_degrees=camera_elevation_deg,
            distance=camera_distance,
            target=camera_target,
            fov_degrees=camera_fov_deg,
        )
        self.show_camera_info = show_camera_info

        self._mesh_program: Optional[int] = None
        self._line_program: Optional[int] = None
        self._mesh_u_mvp = -1
        self._mesh_u_model = -1
        self._mesh_u_color = -1
        self._line_u_mvp = -1

        self._items: List[_MeshRenderItem] = []
        self._grid: Optional[_LineResource] = None
        self._base_frame: Optional[_LineResource] = None
        self._ee_frame: Optional[_LineResource] = None
        self._actual_transforms: Optional[List[np.ndarray]] = None
        self._virtual_transforms: Optional[List[np.ndarray]] = None
        self._actual_ee_tf: Optional[np.ndarray] = None
        self._virtual_ee_tf: Optional[np.ndarray] = None
        self._last_mouse_pos: Optional[QtCore.QPoint] = None
        self._cleanup_connected = False
        self._gl_cleaned = False

        self.setFocusPolicy(Qt.StrongFocus)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    @staticmethod
    def _extract_mesh_specs(mjcf_path: str, visual_model) -> List[MeshSpec]:
        if visual_model is None:
            _, _, _, visual_model = build_models_from_mjcf(mjcf_path)

        specs: List[MeshSpec] = []
        for geom in visual_model.geometryObjects:
            color = np.asarray(getattr(geom, "meshColor", [0.8, 0.8, 0.8, 1.0]))
            if color.size == 3:
                color = np.append(color, 1.0)
            specs.append(
                MeshSpec(
                    name=str(geom.name),
                    path=Path(geom.meshPath).resolve(),
                    color=np.asarray(color[:4], dtype=np.float32),
                )
            )
        return specs

    def minimumSizeHint(self) -> QtCore.QSize:
        return QtCore.QSize(320, 240)

    def initializeGL(self) -> None:
        if not self._cleanup_connected and self.context() is not None:
            self.context().aboutToBeDestroyed.connect(self.cleanup_gl)
            self._cleanup_connected = True

        glClearColor(0.075, 0.082, 0.09, 1.0)
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)

        self._mesh_program = _compile_program(
            MESH_VERTEX_SHADER,
            MESH_FRAGMENT_SHADER,
            ((0, "a_position"), (1, "a_normal")),
        )
        self._line_program = _compile_program(
            LINE_VERTEX_SHADER,
            LINE_FRAGMENT_SHADER,
            ((0, "a_position"), (1, "a_color")),
        )

        self._mesh_u_mvp = glGetUniformLocation(self._mesh_program, "u_mvp")
        self._mesh_u_model = glGetUniformLocation(self._mesh_program, "u_model")
        self._mesh_u_color = glGetUniformLocation(self._mesh_program, "u_color")
        self._line_u_mvp = glGetUniformLocation(self._line_program, "u_mvp")

        start = time.perf_counter()
        self._items = [
            _MeshRenderItem(spec=spec, mesh=GLMesh(spec.path))
            for spec in self.mesh_specs
        ]
        elapsed = time.perf_counter() - start
        print(f"UR5e OpenGL mesh 加载完成: {len(self._items)} 个, {elapsed:.3f}s")

        self._grid = self._create_line_resource(self._build_grid_vertices())
        self._base_frame = self._create_line_resource(
            self._build_frame_vertices(length=10.0)
        )
        self._ee_frame = self._create_line_resource(self._build_frame_vertices(length=0.12))
        self._gl_cleaned = False

    def resizeGL(self, width: int, height: int) -> None:
        glViewport(0, 0, max(1, int(width)), max(1, int(height)))

    def paintGL(self) -> None:
        glEnable(GL_DEPTH_TEST)
        glDepthFunc(GL_LEQUAL)
        glDepthMask(GL_TRUE)
        glDisable(GL_BLEND)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        width = max(1, self.width())
        height = max(1, self.height())
        aspect = float(width) / float(height)
        view = self.camera.view_matrix()
        proj = self.camera.proj_matrix(aspect)

        self._draw_grid(proj, view)
        self._draw_base_frame(proj, view)

        # 先画半透明虚拟机械臂，再画实体机械臂，避免同位姿时虚拟层罩住实体。
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glDepthMask(GL_FALSE)
        self._draw_robot(self._virtual_transforms, proj, view, alpha=0.4)
        glDepthMask(GL_TRUE)
        glDisable(GL_BLEND)

        self._draw_robot(self._actual_transforms, proj, view, alpha=1.0)

        self._draw_ee_frame(self._actual_ee_tf, proj, view)
        self._draw_ee_frame(self._virtual_ee_tf, proj, view)
        self._draw_camera_info_overlay()

    def update_actual_transforms(
        self,
        transforms: Iterable[np.ndarray],
        ee_tf: np.ndarray,
    ) -> None:
        self._actual_transforms = self._copy_transforms(transforms)
        self._actual_ee_tf = _as_mat4(ee_tf).copy()
        self.update()

    def update_virtual_transforms(
        self,
        transforms: Iterable[np.ndarray],
        ee_tf: np.ndarray,
    ) -> None:
        self._virtual_transforms = self._copy_transforms(transforms)
        self._virtual_ee_tf = _as_mat4(ee_tf).copy()
        self.update()

    @staticmethod
    def _copy_transforms(transforms: Iterable[np.ndarray]) -> List[np.ndarray]:
        return [_as_mat4(transform).copy() for transform in transforms]

    def _draw_robot(
        self,
        transforms: Optional[Sequence[np.ndarray]],
        proj: np.ndarray,
        view: np.ndarray,
        alpha: float,
    ) -> None:
        if not transforms or self._mesh_program is None:
            return

        glUseProgram(self._mesh_program)
        for item, transform in zip(self._items, transforms):
            model = _as_mat4(transform)
            mvp = proj @ view @ model
            color = item.spec.color
            glUniform4f(
                self._mesh_u_color,
                float(color[0]),
                float(color[1]),
                float(color[2]),
                float(alpha),
            )
            _set_mat4(self._mesh_u_model, model)
            _set_mat4(self._mesh_u_mvp, mvp)
            item.mesh.draw()
        glUseProgram(0)

    def _draw_grid(self, proj: np.ndarray, view: np.ndarray) -> None:
        if self._grid is None or self._line_program is None:
            return

        glUseProgram(self._line_program)
        _set_mat4(self._line_u_mvp, proj @ view)
        glLineWidth(1.0)
        glBindVertexArray(self._grid.vao)
        glDrawArrays(GL_LINES, 0, self._grid.vertex_count)
        glBindVertexArray(0)
        glUseProgram(0)

    def _draw_ee_frame(
        self,
        transform: Optional[np.ndarray],
        proj: np.ndarray,
        view: np.ndarray,
    ) -> None:
        self._draw_frame(self._ee_frame, transform, proj, view, line_width=3.0)

    def _draw_base_frame(self, proj: np.ndarray, view: np.ndarray) -> None:
        self._draw_frame(
            self._base_frame,
            np.eye(4, dtype=np.float32),
            proj,
            view,
            line_width=5.0,
        )

    def _draw_camera_info_overlay(self) -> None:
        if not self.show_camera_info:
            return

        painter = QPainter(self)
        painter.setRenderHint(QPainter.TextAntialiasing, True)
        font = QFont("Monospace")
        font.setStyleHint(QFont.TypeWriter)
        font.setPointSize(7)
        painter.setFont(font)

        lines = self.camera.debug_lines()
        metrics = painter.fontMetrics()
        line_height = metrics.height()
        text_width = max(metrics.horizontalAdvance(line) for line in lines)
        padding = 4
        rect = QtCore.QRect(
            8,
            8,
            text_width + padding * 2,
            line_height * len(lines) + padding * 2,
        )

        painter.fillRect(rect, QColor(0, 0, 0, 150))
        painter.setPen(QColor(235, 238, 242))
        y = rect.top() + padding + metrics.ascent()
        for line in lines:
            painter.drawText(rect.left() + padding, y, line)
            y += line_height
        painter.end()

    def _draw_frame(
        self,
        resource: Optional[_LineResource],
        transform: Optional[np.ndarray],
        proj: np.ndarray,
        view: np.ndarray,
        line_width: float,
    ) -> None:
        if transform is None or resource is None or self._line_program is None:
            return

        glUseProgram(self._line_program)
        _set_mat4(self._line_u_mvp, proj @ view @ _as_mat4(transform))
        glLineWidth(line_width)
        glBindVertexArray(resource.vao)
        glDrawArrays(GL_LINES, 0, resource.vertex_count)
        glBindVertexArray(0)
        glUseProgram(0)

    @staticmethod
    def _build_grid_vertices() -> np.ndarray:
        # 在 XY 平面绘制 21x21 参考地面网格，Z 轴为 0。
        vertices: List[List[float]] = []
        color = [0.5, 0.5, 0.5, 0.55]
        for i in range(21):
            value = -1.0 + 0.1 * i
            vertices.append([-1.0, value, 0.0, *color])
            vertices.append([1.0, value, 0.0, *color])
            vertices.append([value, -1.0, 0.0, *color])
            vertices.append([value, 1.0, 0.0, *color])
        return np.asarray(vertices, dtype=np.float32)

    @staticmethod
    def _build_frame_vertices(length: float) -> np.ndarray:
        return np.asarray(
            [
                [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0],
                [length, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0],
                [0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 1.0],
                [0.0, length, 0.0, 0.0, 1.0, 0.0, 1.0],
                [0.0, 0.0, 0.0, 0.0, 0.25, 1.0, 1.0],
                [0.0, 0.0, length, 0.0, 0.25, 1.0, 1.0],
            ],
            dtype=np.float32,
        )

    @staticmethod
    def _create_line_resource(vertices: np.ndarray) -> _LineResource:
        data = np.ascontiguousarray(vertices, dtype=np.float32)
        vao = glGenVertexArrays(1)
        vbo = glGenBuffers(1)

        glBindVertexArray(vao)
        glBindBuffer(GL_ARRAY_BUFFER, vbo)
        glBufferData(GL_ARRAY_BUFFER, data.nbytes, data, GL_STATIC_DRAW)

        stride = 7 * ctypes.sizeof(ctypes.c_float)
        glEnableVertexAttribArray(0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, ctypes.c_void_p(0))
        glEnableVertexAttribArray(1)
        glVertexAttribPointer(
            1,
            4,
            GL_FLOAT,
            GL_FALSE,
            stride,
            ctypes.c_void_p(3 * ctypes.sizeof(ctypes.c_float)),
        )

        glBindVertexArray(0)
        glBindBuffer(GL_ARRAY_BUFFER, 0)
        return _LineResource(vao=vao, vbo=vbo, vertex_count=int(data.shape[0]))

    def mousePressEvent(self, event) -> None:
        self._last_mouse_pos = event.pos()
        event.accept()

    def mouseMoveEvent(self, event) -> None:
        if self._last_mouse_pos is None:
            self._last_mouse_pos = event.pos()
            event.accept()
            return

        delta = event.pos() - self._last_mouse_pos
        self._last_mouse_pos = event.pos()

        if event.buttons() & Qt.LeftButton:
            self.camera.rotate(delta.x(), delta.y())
            self.update()
        elif event.buttons() & Qt.MiddleButton:
            self.camera.pan(delta.x(), delta.y(), self.height())
            self.update()
        event.accept()

    def mouseReleaseEvent(self, event) -> None:
        self._last_mouse_pos = None
        event.accept()

    def wheelEvent(self, event) -> None:
        delta = event.angleDelta().y() / 120.0
        if abs(delta) > 1e-6:
            self.camera.zoom(delta)
            self.update()
        event.accept()

    def cleanup_gl(self) -> None:
        if self._gl_cleaned:
            return
        if self.context() is None or not self.context().isValid():
            self._gl_cleaned = True
            return

        self.makeCurrent()

        if self._grid is not None:
            glDeleteBuffers(1, [self._grid.vbo])
            glDeleteVertexArrays(1, [self._grid.vao])
            self._grid = None
        if self._base_frame is not None:
            glDeleteBuffers(1, [self._base_frame.vbo])
            glDeleteVertexArrays(1, [self._base_frame.vao])
            self._base_frame = None
        if self._ee_frame is not None:
            glDeleteBuffers(1, [self._ee_frame.vbo])
            glDeleteVertexArrays(1, [self._ee_frame.vao])
            self._ee_frame = None

        GLMesh.clear_gpu_cache()

        if self._mesh_program is not None:
            glDeleteProgram(self._mesh_program)
            self._mesh_program = None
        if self._line_program is not None:
            glDeleteProgram(self._line_program)
            self._line_program = None

        self.doneCurrent()
        self._gl_cleaned = True

    def close(self) -> bool:
        self.cleanup_gl()
        return super().close()
