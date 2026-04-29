import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import meshcat.geometry as mg

from ur5e_kinematics import UR5eKinematics


class UR5eDualVisualizer:
    """Dual UR5e robot visualization with actual (opaque) and virtual (semi-transparent) states."""

    def __init__(self, mjcf_path: str, tool_offset: np.ndarray = np.array([0, 0.1, 0]),
                 tool_rotation: np.ndarray = None, zmq_url: str = None):
        """
        Args:
            mjcf_path: Path to the UR5e MJCF XML file.
            tool_offset: 3-element [dx, dy, dz] translation offset (meters) from the
                URDF end-effector frame to the actual tool tip. Default: zero.
            tool_rotation: 3-element [rx, ry, rz] rotation offset (radians) as
                intrinsic XYZ Euler angles. Default: zero.
            zmq_url: Optional Meshcat ZMQ URL.
        """
        # Tool offset
        if tool_offset is None:
            self.tool_offset = np.zeros(3)
        else:
            self.tool_offset = np.asarray(tool_offset, dtype=np.float64)

        # Tool rotation (Euler angles to rotation matrix)
        if tool_rotation is None:
            self.tool_rotation = np.eye(3)
        else:
            r = np.asarray(tool_rotation, dtype=np.float64)
            self.tool_rotation = pin.rpy.rpyToMatrix(r[0], r[1], r[2])

        # Fix end-effector frame: wrist_3_link in MJCF has Y/Z axes swapped
        # compared to the real UR tool frame. Rotate -90 degrees around X.
        R_x_neg90 = np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], dtype=np.float64)

        self.kinematics = UR5eKinematics(
            mjcf_path=mjcf_path,
            end_frame_name="wrist_3_link",
            tcp_offset=self.tool_offset,
            tcp_rotation=self.tool_rotation @ R_x_neg90,
        )

        # Load visual geometry. The visualizer uses the corrected model owned by
        # UR5eKinematics so FK conventions match numerical kinematics.
        result = pin.shortcuts.buildModelsFromMJCF(mjcf_path)
        self.model = self.kinematics.model
        self.visual_model_actual = result[3]

        # Clone visual model for virtual robot
        self.visual_model_virtual = self.visual_model_actual.clone()

        # Kinematic data for both robots
        self.data_actual = self.model.createData()
        self.data_virtual = self.model.createData()

        # Create visualizer for actual robot
        self.viz_actual = MeshcatVisualizer(
            model=self.model,
            visual_model=self.visual_model_actual,
            data=self.data_actual,
        )
        self.viz_actual.initViewer(open=False, zmq_url=zmq_url)
        self.viz_actual.loadViewerModel(rootNodeName="actual")

        # Create visualizer for virtual robot, sharing the same viewer
        self.viz_virtual = MeshcatVisualizer(
            model=self.model,
            visual_model=self.visual_model_virtual,
            data=self.data_virtual,
        )
        self.viz_virtual.initViewer(viewer=self.viz_actual.viewer)
        self.viz_virtual.loadViewerModel(
            rootNodeName="virtual",
            visual_color=[0.3, 0.6, 0.9, 0.4],
        )

        # Create end-effector frame visuals for both robots
        self._create_ee_frames()

        print("UR5e dual visualizer initialized.")
        print(f"Open Meshcat URL in browser: {self.viz_actual.viewer.url()}")

    def _create_ee_frames(self):
        """Create shorter RGB axis lines for end-effector frames."""
        ee_axis_length = 0.06  # shorter than the default 0.2 base frame

        positions = (
            ee_axis_length
            * np.array(
                [[0, 0, 0], [1, 0, 0], [0, 0, 0], [0, 1, 0], [0, 0, 0], [0, 0, 1]]
            )
            .astype(np.float32)
            .T
        )
        colors = (
            np.array(
                [[1, 0, 0], [1, 0.6, 0], [0, 1, 0], [0.6, 1, 0], [0, 0, 1], [0, 0.6, 1]]
            )
            .astype(np.float32)
            .T
        )

        line_geom = mg.LineSegments(
            mg.PointsGeometry(position=positions, color=colors),
            mg.LineBasicMaterial(vertexColors=True, linewidth=2),
        )

        # Actual robot EE frame
        self.viz_actual.viewer["actual/ee_frame"].set_object(line_geom)
        # Virtual robot EE frame
        self.viz_actual.viewer["virtual/ee_frame"].set_object(line_geom)

    def update_actual(self, q: np.ndarray):
        """Update the actual (opaque) robot joint angles.

        Args:
            q: 7-element array [q1..q6, gripper]. Only q1..q6 (radians) are used.
        """
        q6 = np.asarray(q[:6], dtype=np.float64)
        self.kinematics.update_forward_kinematics(q6, self.data_actual)
        pin.updateGeometryPlacements(
            self.model, self.data_actual,
            self.visual_model_actual, self.viz_actual.visual_data,
        )
        self.viz_actual.display()

        # Update end-effector frame
        ee_tf = self.kinematics.forward_kinematics(q6)[0]
        self.viz_actual.viewer["actual/ee_frame"].set_transform(ee_tf)

    def update_virtual(self, q: np.ndarray):
        """Update the virtual (semi-transparent) robot joint angles.

        Args:
            q: 7-element array [q1..q6, gripper]. Only q1..q6 (radians) are used.
        """
        q6 = np.asarray(q[:6], dtype=np.float64)
        self.kinematics.update_forward_kinematics(q6, self.data_virtual)
        pin.updateGeometryPlacements(
            self.model, self.data_virtual,
            self.visual_model_virtual, self.viz_virtual.visual_data,
        )
        self.viz_virtual.display()

        # Update end-effector frame
        ee_tf = self.kinematics.forward_kinematics(q6)[0]
        self.viz_actual.viewer["virtual/ee_frame"].set_transform(ee_tf)

    @property
    def url(self) -> str:
        """Return the Meshcat viewer URL."""
        return self.viz_actual.viewer.url()

    def close(self):
        """Close the visualizer and clean up resources."""
        pass  # Meshcat viewer manages its own lifecycle
