# quat_visual.py # - visualizes quaternion from serial input for debugging purposes
import sys
import serial
import numpy as np
from collections import deque
from threading import Thread, Lock
from PyQt6.QtWidgets import QApplication, QMainWindow, QWidget, QVBoxLayout, QSplitter
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
                    if len(parts) == 4:
                        w, x, y, z = map(float, parts)
                        with self.lock:
                            self.latest_quat = (w, x, y, z)

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
        self.setGeometry(100, 100, 1400, 800)

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

        # Left pane: pyqtgraph plot (dark mode)
        self.graph_widget = pg.PlotWidget()
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

        # Create plot curves
        self.w_curve = self.graph_widget.plot(pen=pg.mkPen("r", width=2), name="w")
        self.x_curve = self.graph_widget.plot(pen=pg.mkPen("g", width=2), name="x")
        self.y_curve = self.graph_widget.plot(pen=pg.mkPen("b", width=2), name="y")
        self.z_curve = self.graph_widget.plot(pen=pg.mkPen("y", width=2), name="z")

        # Right pane: vispy 3D visualization with black background
        self.canvas = scene.SceneCanvas(keys="interactive", show=True, bgcolor="black")
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = "turntable"
        self.view.camera.distance = 5

        # Create a 3D grid background for depth perception
        self.create_grid()

        # Create a box to visualize orientation
        self.create_box()

        # Add widgets to splitter
        splitter.addWidget(self.graph_widget)
        splitter.addWidget(self.canvas.native)
        splitter.setSizes([700, 700])

        layout.addWidget(splitter)

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

        if quat is not None:
            w, x, y, z = quat

            # Update graph data
            self.time_data.append(self.time_counter)
            self.w_data.append(w)
            self.x_data.append(x)
            self.y_data.append(y)
            self.z_data.append(z)
            self.time_counter += 1

            # Update plot curves
            time_array = np.array(self.time_data)
            self.w_curve.setData(time_array, np.array(self.w_data))
            self.x_curve.setData(time_array, np.array(self.x_data))
            self.y_curve.setData(time_array, np.array(self.y_data))
            self.z_curve.setData(time_array, np.array(self.z_data))

            # Update 3D visualization using vispy's quaternion
            self.update_3d_rotation(w, x, y, z)

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

        # Apply transform to box mesh
        self.box_transform.matrix = transform_matrix
        self.box_mesh.transform = self.box_transform

        # Trigger canvas update
        self.canvas.update()

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
