# quat_visual.py # - visualizes quaternion from serial input for debugging purposes
import sys
import serial
import numpy as np
from collections import deque
from threading import Thread, Lock
from PyQt6.QtWidgets import (
    QApplication,
    QMainWindow,
    QWidget,
    QVBoxLayout,
    QSplitter,
    QProgressBar,
    QLabel,
    QHBoxLayout,
)
from PyQt6.QtCore import QTimer, Qt
import pyqtgraph as pg
from vispy import scene
from vispy.visuals.transforms import MatrixTransform


class SerialReader(Thread):
    """Non-blocking serial reader thread"""

    def __init__(self, port="/dev/ttyACM0", baudrate=115200):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.running = True
        self.lock = Lock()
        self.latest_quat = None
        self.fsr_percent = None

    def run(self):
        """Continuously read from serial port with error handling"""
        while self.running:
            try:
                # Attempt to open serial port if not already open
                if self.ser is None or not self.ser.is_open:
                    self.ser = serial.Serial(
                        port=self.port, baudrate=self.baudrate, timeout=0.1
                    )
                    print(f"Connected to {self.port}")

                # Read line with timeout
                line = self.ser.readline().decode("utf-8", errors="ignore").strip()

                if line.startswith("DATA"):
                    parts = line[5:].split()
                    if len(parts) == 5:
                        w, x, y, z, p = map(float, parts)
                        with self.lock:
                            self.latest_quat = (w, x, y, z)
                            self.fsr_percent = p

            except (serial.SerialException, ValueError, IndexError) as e:
                # Handle serial errors gracefully
                if self.ser and self.ser.is_open:
                    self.ser.close()
                self.ser = None
                # Don't spam errors, just wait and retry

            except Exception as e:
                print(f"Unexpected error: {e}")

    def get_quaternion(self):
        """Thread-safe getter for latest quaternion"""
        with self.lock:
            return self.latest_quat

    def get_force(self):
        """Thread-safe getter for latest fsr reading"""
        with self.lock:
            return self.fsr_percent

    def stop(self):
        """Stop the reader thread"""
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.close()


class QuaternionVisualizer(QMainWindow):
    """Main window with split panes for graph and 3D visualization"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Quaternion Visualizer")
        self.setGeometry(100, 100, 1000, 600)

        # Set dark mode style
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #000000;
            }
            QSplitter::handle {
                background-color: #3e3e3e;
            }
        """)

        # Data storage for plotting
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.w_data = deque(maxlen=self.max_points)
        self.x_data = deque(maxlen=self.max_points)
        self.y_data = deque(maxlen=self.max_points)
        self.z_data = deque(maxlen=self.max_points)
        self.time_counter = 0
        self.last_quat = None
        self.graph_update_counter = 0

        # Initialize UI
        self.init_ui()

        # Start serial reader
        self.serial_reader = SerialReader()
        self.serial_reader.start()

        # Setup update timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_visualizations)
        self.timer.start(16)  # ~60 FPS

    def init_ui(self):
        """Setup the UI with split panes"""
        # Create central widget and main layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        layout = QVBoxLayout(central_widget)

        # Create splitter for two panes
        splitter = QSplitter(Qt.Orientation.Horizontal)

        # Left pane: pyqtgraph plot (dark mode) with OpenGL acceleration
        self.graph_widget = pg.PlotWidget(useOpenGL=True)
        self.graph_widget.setBackground("#000000")
        self.graph_widget.setTitle("Quaternion Components", color="w", size="12pt")
        self.graph_widget.setLabel("left", "Value", color="w")
        self.graph_widget.setLabel("bottom", "Time (samples)", color="w")
        self.graph_widget.addLegend()
        self.graph_widget.showGrid(x=True, y=True, alpha=0.3)
        # Set axis colors for dark mode
        self.graph_widget.getAxis("left").setPen("w")
        self.graph_widget.getAxis("bottom").setPen("w")
        self.graph_widget.getAxis("left").setTextPen("w")
        self.graph_widget.getAxis("bottom").setTextPen("w")
        # Disable auto-ranging for better performance
        self.graph_widget.setYRange(-1.1, 1.1)
        self.graph_widget.enableAutoRange(axis="y", enable=False)

        # Create plot curves with optimized settings
        self.w_curve = self.graph_widget.plot(
            pen=pg.mkPen("r", width=2), name="w", skipFiniteCheck=True
        )
        self.x_curve = self.graph_widget.plot(
            pen=pg.mkPen("g", width=2), name="x", skipFiniteCheck=True
        )
        self.y_curve = self.graph_widget.plot(
            pen=pg.mkPen("b", width=2), name="y", skipFiniteCheck=True
        )
        self.z_curve = self.graph_widget.plot(
            pen=pg.mkPen("y", width=2), name="z", skipFiniteCheck=True
        )

        # Right pane: Container for Euler angles and 3D visualization
        right_pane = QWidget()
        right_pane.setStyleSheet("background-color: #000000;")
        right_layout = QVBoxLayout(right_pane)
        right_layout.setContentsMargins(0, 0, 0, 0)
        right_layout.setSpacing(5)

        # Euler angles display at the top of right pane
        euler_container = QWidget()
        euler_container.setMaximumHeight(60)
        euler_container.setStyleSheet("background-color: #000000;")
        euler_layout = QHBoxLayout(euler_container)
        euler_layout.setContentsMargins(10, 5, 10, 5)

        # Roll label
        roll_label = QLabel("Roll:")
        roll_label.setStyleSheet("color: white; font-size: 11pt; font-weight: bold;")
        roll_label.setFixedWidth(50)
        euler_layout.addWidget(roll_label)

        self.roll_value = QLabel("0.0°")
        self.roll_value.setStyleSheet("color: #00ff00; font-size: 14pt; font-weight: bold;")
        self.roll_value.setFixedWidth(100)
        euler_layout.addWidget(self.roll_value)

        # Pitch label
        pitch_label = QLabel("Pitch:")
        pitch_label.setStyleSheet("color: white; font-size: 11pt; font-weight: bold;")
        pitch_label.setFixedWidth(60)
        euler_layout.addWidget(pitch_label)

        self.pitch_value = QLabel("0.0°")
        self.pitch_value.setStyleSheet("color: #00ffff; font-size: 14pt; font-weight: bold;")
        self.pitch_value.setFixedWidth(100)
        euler_layout.addWidget(self.pitch_value)

        # Yaw label
        yaw_label = QLabel("Yaw:")
        yaw_label.setStyleSheet("color: white; font-size: 11pt; font-weight: bold;")
        yaw_label.setFixedWidth(50)
        euler_layout.addWidget(yaw_label)

        self.yaw_value = QLabel("0.0°")
        self.yaw_value.setStyleSheet("color: #ffff00; font-size: 14pt; font-weight: bold;")
        self.yaw_value.setFixedWidth(100)
        euler_layout.addWidget(self.yaw_value)

        euler_layout.addStretch()

        # Add Euler display to right pane layout
        right_layout.addWidget(euler_container)

        # vispy 3D visualization with black background
        self.canvas = scene.SceneCanvas(keys="interactive", show=True, bgcolor="black")
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = "turntable"
        self.view.camera.distance = 5

        # Create a 3D grid background for depth perception
        self.create_grid()

        # Create a box to visualize orientation
        self.create_box()

        # Add canvas to right pane layout
        right_layout.addWidget(self.canvas.native)

        # Add widgets to splitter
        splitter.addWidget(self.graph_widget)
        splitter.addWidget(right_pane)
        splitter.setSizes([500, 500])

        # Add splitter with stretch factor to take most of the space
        layout.addWidget(splitter, stretch=10)

        # Create FSR bar widget
        fsr_container = QWidget()
        fsr_container.setMaximumHeight(60)  # Keep FSR bar small
        fsr_layout = QHBoxLayout(fsr_container)
        fsr_layout.setContentsMargins(10, 5, 10, 5)

        # Label for FSR
        fsr_label = QLabel("FSR:")
        fsr_label.setStyleSheet("color: white; font-size: 11pt; font-weight: bold;")
        fsr_label.setFixedWidth(50)
        fsr_layout.addWidget(fsr_label)

        # Progress bar for FSR
        self.fsr_bar = QProgressBar()
        self.fsr_bar.setMinimum(0)
        self.fsr_bar.setMaximum(100)
        self.fsr_bar.setValue(0)
        self.fsr_bar.setTextVisible(True)
        self.fsr_bar.setFormat("%p%")
        self.fsr_bar.setFixedHeight(35)
        self.fsr_bar.setStyleSheet("""
            QProgressBar {
                border: 2px solid #3e3e3e;
                border-radius: 5px;
                background-color: #1e1e1e;
                text-align: center;
                color: white;
                font-size: 11pt;
                font-weight: bold;
            }
            QProgressBar::chunk {
                background-color: qlineargradient(x1:0, y1:0, x2:1, y2:0,
                    stop:0 #00ff00, stop:0.5 #ffff00, stop:1 #ff0000);
                border-radius: 3px;
            }
        """)
        fsr_layout.addWidget(self.fsr_bar)

        # Add FSR container with minimal stretch
        layout.addWidget(fsr_container, stretch=0)

    def create_grid(self):
        """Create a 3D grid for depth perception"""
        # Create a brighter grid plane
        self.grid = scene.visuals.GridLines(
            parent=self.view.scene, color=(0.5, 0.5, 0.5, 0.8)
        )

        # Position the grid below the cube
        self.grid.transform = MatrixTransform()
        grid_matrix = np.eye(4, dtype=np.float32)
        grid_matrix[3, 2] = -2.5  # Move grid down on Z axis
        self.grid.transform.matrix = grid_matrix

    def create_box(self):
        """Create a box mesh to visualize quaternion rotation"""
        # Define box vertices
        vertices = np.array(
            [
                [-0.5, -0.5, -0.5],
                [0.5, -0.5, -0.5],
                [0.5, 0.5, -0.5],
                [-0.5, 0.5, -0.5],
                [-0.5, -0.5, 0.5],
                [0.5, -0.5, 0.5],
                [0.5, 0.5, 0.5],
                [-0.5, 0.5, 0.5],
            ],
            dtype=np.float32,
        )

        # Scale the box to make it more visible
        vertices *= 1.5

        # Define faces
        faces = np.array(
            [
                [0, 1, 2],
                [0, 2, 3],
                [1, 5, 6],
                [1, 6, 2],
                [5, 4, 7],
                [5, 7, 6],
                [4, 0, 3],
                [4, 3, 7],
                [3, 2, 6],
                [3, 6, 7],
                [4, 5, 1],
                [4, 1, 0],
            ],
            dtype=np.uint32,
        )

        # Create bright solid red mesh with flat shading
        self.box_mesh = scene.visuals.Mesh(
            vertices=vertices,
            faces=faces,
            color=(0.9, 0.0, 0.0, 1.0),
            shading="flat",
            parent=self.view.scene,
        )
        self.box_mesh.set_gl_state(depth_test=True, blend=False)

        # Store initial transform
        self.box_transform = MatrixTransform()
        self.box_mesh.transform = self.box_transform

    def update_visualizations(self):
        """Update both graph and 3D visualization"""
        # Get latest quaternion
        quat = self.serial_reader.get_quaternion()

        # Only update if we have new data
        if quat is not None and quat != self.last_quat:
            w, x, y, z = quat
            self.last_quat = quat

            # Update graph data
            self.time_data.append(self.time_counter)
            self.w_data.append(w)
            self.x_data.append(x)
            self.y_data.append(y)
            self.z_data.append(z)
            self.time_counter += 1

            # Update plot curves less frequently (every 2 frames = ~30 FPS)
            self.graph_update_counter += 1
            if self.graph_update_counter >= 2:
                time_array = np.array(self.time_data)
                self.w_curve.setData(time_array, np.array(self.w_data))
                self.x_curve.setData(time_array, np.array(self.x_data))
                self.y_curve.setData(time_array, np.array(self.y_data))
                self.z_curve.setData(time_array, np.array(self.z_data))
                self.graph_update_counter = 0

            # Always update 3D visualization for smooth rotation
            self.update_3d_rotation(w, x, y, z)

            # Calculate and update Euler angles
            roll, pitch, yaw = self.quaternion_to_euler(w, x, y, z)
            self.roll_value.setText(f"{roll:.1f}°")
            self.pitch_value.setText(f"{pitch:.1f}°")
            self.yaw_value.setText(f"{yaw:.1f}°")

        # Update FSR bar
        fsr_percent = self.serial_reader.get_force()
        if fsr_percent is not None:
            self.fsr_bar.setValue(int(fsr_percent))

    def quaternion_to_euler(self, w, x, y, z):
        """Convert quaternion to Euler angles (roll, pitch, yaw) in degrees"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Convert to degrees
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def update_3d_rotation(self, w, x, y, z):
        """Apply quaternion rotation to 3D objects using vispy"""
        from vispy.util.quaternion import Quaternion

        # Create vispy Quaternion (w, x, y, z)
        quat = Quaternion(w, x, y, z)

        # Get rotation matrix from quaternion
        rotation_matrix = quat.get_matrix()

        # Create 4x4 transform matrix
        transform_matrix = np.eye(4, dtype=np.float32)
        transform_matrix[:3, :3] = rotation_matrix[:3, :3]

        # Apply transform to box mesh (canvas updates automatically)
        self.box_transform.matrix = transform_matrix

    def closeEvent(self, event):
        """Cleanup when window closes"""
        self.serial_reader.stop()
        self.timer.stop()
        event.accept()


def main():
    app = QApplication(sys.argv)
    window = QuaternionVisualizer()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
