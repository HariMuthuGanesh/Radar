import sys
import os
import csv
import struct
import time
import random
import numpy as np
from datetime import datetime

from PySide6.QtWidgets import (QApplication, QMainWindow, QWidget, QHBoxLayout, 
                             QVBoxLayout, QSplitter, QPushButton, QComboBox, 
                             QLabel, QGroupBox, QTextEdit, QStackedWidget, QStatusBar)
from PySide6.QtCore import Qt, QTimer, QObject, QThread, Signal, Slot
from PySide6.QtGui import QTextCursor

import pyqtgraph as pg
import pyqtgraph.opengl as gl

# =============================================================================
# LOGGER COMPONENT (from csv_logger.py)
# =============================================================================
class CSVLogger:
    """
    Handles saving session data to CSV files.
    """
    def __init__(self, directory="radar/radar_ui_app/saved_files"):
        self.directory = directory
        if not os.path.exists(self.directory):
            os.makedirs(self.directory)

    def save_session(self, session_data):
        """
        Saves a list of dictionaries to a CSV file.
        Returns the absolute path of the saved file.
        """
        if not session_data:
            return None

        timestamp_str = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"radar_capture_{timestamp_str}.csv"
        filepath = os.path.join(self.directory, filename)

        # Extract headers from the first data point
        headers = session_data[0].keys()

        try:
            with open(filepath, mode='w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=headers)
                writer.writeheader()
                writer.writerows(session_data)
            return os.path.abspath(filepath)
        except Exception as e:
            print(f"Error saving CSV: {e}")
            return None

# =============================================================================
# PARSER COMPONENT (from frame_parser.py)
# =============================================================================
class FrameParser:
    """
    Parses raw radar data into structured frames.
    """
    def __init__(self):
        self.leftover_data = b''

    def parse(self, raw_data):
        all_points = []
        data_to_parse = self.leftover_data + raw_data
        point_size = 20
        
        num_points = len(data_to_parse) // point_size
        self.leftover_data = data_to_parse[num_points * point_size:]
        
        timestamp = datetime.now().isoformat(timespec='milliseconds')
        
        for i in range(num_points):
            segment = data_to_parse[i*point_size : (i+1)*point_size]
            try:
                vals = struct.unpack('fffff', segment)
                all_points.append({
                    'timestamp': timestamp,
                    'x': vals[0],
                    'y': vals[1],
                    'z': vals[2],
                    'velocity': vals[3],
                    'intensity': vals[4]
                })
            except Exception:
                continue
        return all_points

    def get_mock_frame(self, frame_id):
        num_points = random.randint(10, 50)
        timestamp = datetime.now().isoformat(timespec='milliseconds')
        points = []
        for _ in range(num_points):
            points.append({
                'timestamp': timestamp,
                'frame_id': frame_id,
                'x': random.uniform(-5, 5),
                'y': random.uniform(0, 10),
                'z': random.uniform(-2, 2),
                'velocity': random.uniform(-2, 2),
                'intensity': random.uniform(0, 100)
            })
        return points

# =============================================================================
# COMMUNICATION COMPONENT (from serial_manager.py)
# =============================================================================
class SerialReader(QObject):
    data_received = Signal(bytes)
    error_occurred = Signal(str)
    finished = Signal()

    def __init__(self, port, baudrate):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.is_running = False
        self.serial_port = None

    @Slot()
    def run(self):
        try:
            import serial
            self.serial_port = serial.Serial(self.port, self.baudrate, timeout=1)
            self.is_running = True
            while self.is_running:
                if self.serial_port.in_waiting > 0:
                    data = self.serial_port.read(self.serial_port.in_waiting)
                    self.data_received.emit(data)
                else:
                    time.sleep(0.01)
        except Exception as e:
            self.error_occurred.emit(str(e))
        finally:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.finished.emit()

    def stop(self):
        self.is_running = False

class SerialManager(QObject):
    log_message = Signal(str)
    data_ready = Signal(bytes)
    connection_status = Signal(bool)

    # def __init__(self, config_port="COM6", data_port="COM7", config_baud=115200, data_baud=921600):
    def __init__(self, config_port="COM12", data_port="COM13", config_baud=115200, data_baud=921600):
        super().__init__()
        self.config_port = config_port
        self.data_port = data_port
        self.config_baud = config_baud
        self.data_baud = data_baud
        self.config_ser = None
        self.reader_thread = None
        self.reader_worker = None

    def connect_ports(self):
        try:
            import serial
            self.config_ser = serial.Serial(self.config_port, self.config_baud, timeout=1)
            self.log_message.emit(f"Connected to config port {self.config_port}")
            self.connection_status.emit(True)
            return True
        except Exception as e:
            self.log_message.emit(f"Failed to connect to config port: {str(e)}")
            self.connection_status.emit(False)
            return False

    def disconnect_ports(self):
        self.stop_reading()
        if self.config_ser and self.config_ser.is_open:
            self.config_ser.close()
            self.log_message.emit("Disconnected config port.")
        self.connection_status.emit(False)

    def send_config(self, filepath):
        if not self.config_ser or not self.config_ser.is_open:
            self.log_message.emit("Error: Config port not open.")
            return False
        try:
            with open(filepath, 'r') as f:
                commands = f.readlines()
            for cmd in commands:
                cmd = cmd.strip()
                if cmd and not cmd.startswith('%'):
                    self.config_ser.write((cmd + '\n').encode())
                    self.log_message.emit(f"Sent: {cmd}")
                    time.sleep(0.1)
            return True
        except Exception as e:
            self.log_message.emit(f"Error sending config: {str(e)}")
            return False

    def start_reading(self):
        if self.reader_thread and self.reader_thread.isRunning():
            return
        self.reader_thread = QThread()
        self.reader_worker = SerialReader(self.data_port, self.data_baud)
        self.reader_worker.moveToThread(self.reader_thread)
        self.reader_thread.started.connect(self.reader_worker.run)
        self.reader_worker.data_received.connect(self.data_ready)
        self.reader_worker.error_occurred.connect(lambda e: self.log_message.emit(f"Data Serial Error: {e}"))
        self.reader_worker.finished.connect(self.reader_thread.quit)
        self.reader_worker.finished.connect(self.reader_worker.deleteLater)
        self.reader_thread.finished.connect(self.reader_thread.deleteLater)
        self.reader_thread.start()
        self.log_message.emit(f"Started reading data from {self.data_port}")

    def stop_reading(self):
        if self.reader_worker:
            self.reader_worker.stop()
        if self.reader_thread:
            self.reader_thread.quit()
            self.reader_thread.wait()
            self.reader_thread = None
            self.reader_worker = None
        self.log_message.emit("Stopped reading data.")

# =============================================================================
# CONTROLLER COMPONENT (from radar_controller.py)
# =============================================================================
class RadarController(QObject):
    data_updated = Signal(list)
    status_message = Signal(str)
    connection_changed = Signal(bool)

    def __init__(self):
        super().__init__()
        self.serial_manager = SerialManager()
        self.parser = FrameParser()
        self.logger = CSVLogger()
        self.session_data = []
        self.is_streaming = False
        self.frame_count = 0
        self.serial_manager.log_message.connect(self.status_message)
        self.serial_manager.data_ready.connect(self.process_incoming_data)
        self.serial_manager.connection_status.connect(self.connection_changed)

    def connect_radar(self):
        return self.serial_manager.connect_ports()

    def disconnect_radar(self):
        self.stop_streaming()
        self.serial_manager.disconnect_ports()

    def send_config(self, cfg_path):
        if self.serial_manager.send_config(cfg_path):
            self.status_message.emit(f"Config {cfg_path} sent successfully.")
            return True
        return False

    def start_streaming(self):
        if not self.is_streaming:
            self.session_data = []
            self.frame_count = 0
            self.is_streaming = True
            self.serial_manager.start_reading()
            self.status_message.emit("Streaming started.")

    def stop_streaming(self):
        if self.is_streaming:
            self.is_streaming = False
            self.serial_manager.stop_reading()
            self.status_message.emit("Streaming stopped.")

    @Slot(bytes)
    def process_incoming_data(self, raw_data):
        if not self.is_streaming:
            return
        parsed_points = self.parser.parse(raw_data)
        if parsed_points:
            self.frame_count += 1
            for pt in parsed_points:
                pt['frame_id'] = self.frame_count
                self.session_data.append(pt)
            self.data_updated.emit(parsed_points)

    def download_data(self):
        if not self.session_data:
            self.status_message.emit("No data to download.")
            return None
        filepath = self.logger.save_session(self.session_data)
        if filepath:
            self.status_message.emit(f"Data saved to {filepath}")
        else:
            self.status_message.emit("Failed to save data.")
        return filepath

    def get_mock_data(self):
        if self.is_streaming:
            self.frame_count += 1
            pts = self.parser.get_mock_frame(self.frame_count)
            self.session_data.extend(pts)
            self.data_updated.emit(pts)

# =============================================================================
# PLOTTING COMPONENTS (from plot2d.py, plot3d.py, plot_manager.py)
# =============================================================================
class Plot2D(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.plot_widget = pg.PlotWidget(title="Radar 2D View (X-Y)")
        self.layout.addWidget(self.plot_widget)
        self.scatter = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))
        self.plot_widget.addItem(self.scatter)
        self.plot_widget.setLabel('left', 'Y (m)')
        self.plot_widget.setLabel('bottom', 'X (m)')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setXRange(-5, 5)
        self.plot_widget.setYRange(0, 10)

    def update_plot(self, points):
        if not points: return
        x_data = [p['x'] for p in points]
        y_data = [p['y'] for p in points]
        self.scatter.setData(x=x_data, y=y_data)

    def clear(self):
        self.scatter.clear()

class Plot3D(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout(self)
        self.view_widget = gl.GLViewWidget()
        self.layout.addWidget(self.view_widget)
        grid = gl.GLGridItem()
        self.view_widget.addItem(grid)
        axis = gl.GLAxisItem()
        self.view_widget.addItem(axis)
        self.scatter = gl.GLScatterPlotItem(size=5, pxMode=True)
        self.view_widget.addItem(self.scatter)
        self.view_widget.setCameraPosition(distance=15, elevation=30, azimuth=45)

    def update_plot(self, points):
        if not points: return
        pos = np.array([[p['x'], p['y'], p['z']] for p in points])
        colors = np.array([[1, 1, 1, 0.8] for _ in points]) 
        self.scatter.setData(pos=pos, color=colors)

    def clear(self):
        self.scatter.setData(pos=np.empty((0, 3)))

class PlotManager(QStackedWidget):
    def __init__(self):
        super().__init__()
        self.plot2d = Plot2D()
        self.plot3d = Plot3D()
        self.addWidget(self.plot2d)
        self.addWidget(self.plot3d)
        self.current_mode = "2D"

    def set_mode(self, mode):
        if mode == "2D":
            self.setCurrentIndex(0)
            self.current_mode = "2D"
        elif mode == "3D":
            self.setCurrentIndex(1)
            self.current_mode = "3D"

    def update_data(self, points):
        if self.current_mode == "2D":
            self.plot2d.update_plot(points)
        else:
            self.plot3d.update_plot(points)

    def clear_plots(self):
        self.plot2d.clear()
        self.plot3d.clear()

# =============================================================================
# UI COMPONENTS (from control_panel.py, log_widget.py, status_bar.py)
# =============================================================================
class ControlPanel(QWidget):
    def __init__(self, config_dir=None):
        super().__init__()
        if config_dir is None:
            # Use the directory where main.py is located
            config_dir = os.path.dirname(os.path.abspath(__file__))
        self.config_dir = config_dir
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)
        config_group = QGroupBox("Configuration")
        config_layout = QVBoxLayout()
        self.config_combo = QComboBox()
        self.refresh_configs()
        self.browse_btn = QPushButton("Browse...")
        
        config_layout.addWidget(QLabel("Select .cfg file:"))
        config_layout.addWidget(self.config_combo)
        config_layout.addWidget(self.browse_btn)
        config_group.setLayout(config_layout)
        layout.addWidget(config_group)

        mode_group = QGroupBox("Plotting Mode")
        mode_layout = QVBoxLayout()
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["2D", "3D"])
        mode_layout.addWidget(self.mode_combo)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)

        btn_group = QGroupBox("Controls")
        btn_layout = QVBoxLayout()
        self.connect_btn = QPushButton("Connect")
        self.start_btn = QPushButton("Start")
        self.stop_btn = QPushButton("Stop")
        self.download_btn = QPushButton("Download")
        self.start_btn.setEnabled(False)
        self.stop_btn.setEnabled(False)
        self.download_btn.setEnabled(False)
        btn_layout.addWidget(self.connect_btn)
        btn_layout.addWidget(self.start_btn)
        btn_layout.addWidget(self.stop_btn)
        btn_layout.addWidget(self.download_btn)
        btn_group.setLayout(btn_layout)
        layout.addWidget(btn_group)
        layout.addStretch()

    def refresh_configs(self):
        self.config_combo.clear()
        if os.path.exists(self.config_dir):
            configs = [f for f in os.listdir(self.config_dir) if f.endswith('.cfg')]
            self.config_combo.addItems(configs)
        else:
            self.config_combo.addItem("No config files found")

    def set_connected(self, connected):
        self.connect_btn.setText("Disconnect") if connected else self.connect_btn.setText("Connect")
        self.start_btn.setEnabled(connected)
        self.config_combo.setEnabled(not connected)
        self.browse_btn.setEnabled(not connected)

    def set_streaming(self, streaming):
        self.start_btn.setEnabled(not streaming)
        self.stop_btn.setEnabled(streaming)
        self.connect_btn.setEnabled(not streaming)
        self.download_btn.setEnabled(not streaming)
        self.mode_combo.setEnabled(not streaming)

class LogWidget(QTextEdit):
    def __init__(self):
        super().__init__()
        self.setReadOnly(True)
        self.setPlaceholderText("Application logs will appear here...")
        self.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: 'Consolas', monospace;")

    def log(self, message):
        self.append(message)
        self.moveCursor(QTextCursor.End)

class StatusBar(QStatusBar):
    def __init__(self):
        super().__init__()
        self.status_label = QLabel("Disconnected")
        self.addWidget(self.status_label)
        self.fps_label = QLabel("FPS: 0")
        self.addPermanentWidget(self.fps_label)

    def set_connection_status(self, connected):
        if connected:
            self.status_label.setText("Connected")
            self.status_label.setStyleSheet("color: green;")
        else:
            self.status_label.setText("Disconnected")
            self.status_label.setStyleSheet("color: red;")

    def set_fps(self, fps):
        self.fps_label.setText(f"FPS: {fps}")

# =============================================================================
# MAIN WINDOW (from main_window.py)
# =============================================================================
class MainWindow(QMainWindow):
    def __init__(self, controller):
        super().__init__()
        self.controller = controller
        self.setWindowTitle("Radar Visualization Application")
        self.resize(1200, 800)
        self.init_ui()
        self.connect_signals()

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        self.control_panel = ControlPanel()
        main_layout.addWidget(self.control_panel, 1)
        right_panel = QWidget()
        right_layout = QVBoxLayout(right_panel)
        self.splitter = QSplitter(Qt.Vertical)
        self.plot_manager = PlotManager()
        self.log_widget = LogWidget()
        self.splitter.addWidget(self.plot_manager)
        self.splitter.addWidget(self.log_widget)
        self.splitter.setStretchFactor(0, 3)
        self.splitter.setStretchFactor(1, 1)
        right_layout.addWidget(self.splitter)
        main_layout.addWidget(right_panel, 4)
        self.status_bar = StatusBar()
        self.setStatusBar(self.status_bar)

    def connect_signals(self):
        self.control_panel.connect_btn.clicked.connect(self.handle_connection)
        self.control_panel.browse_btn.clicked.connect(self.handle_browse)
        self.control_panel.start_btn.clicked.connect(self.handle_start)
        self.control_panel.stop_btn.clicked.connect(self.handle_stop)
        self.control_panel.download_btn.clicked.connect(self.controller.download_data)
        self.controller.status_message.connect(self.log_widget.log)
        self.controller.connection_changed.connect(self.status_bar.set_connection_status)
        self.controller.connection_changed.connect(self.control_panel.set_connected)
        self.controller.data_updated.connect(self.plot_manager.update_data)

    def handle_browse(self):
        from PySide6.QtWidgets import QFileDialog
        file_path, _ = QFileDialog.getOpenFileName(self, "Select Configuration File", self.control_panel.config_dir, "Config Files (*.cfg)")
        if file_path:
            # Add to combo if not already there and select it
            filename = os.path.basename(file_path)
            index = self.control_panel.config_combo.findText(filename)
            if index == -1:
                self.control_panel.config_combo.addItem(filename)
                index = self.control_panel.config_combo.count() - 1
            self.control_panel.config_combo.setCurrentIndex(index)
            # Update config_dir to the folder of selected file for next time
            self.control_panel.config_dir = os.path.dirname(file_path)

    def handle_connection(self):
        if self.control_panel.connect_btn.text() == "Connect":
            if self.controller.connect_radar():
                cfg_file = self.control_panel.config_combo.currentText()
                if cfg_file and ".cfg" in cfg_file:
                    cfg_path = os.path.join(self.control_panel.config_dir, cfg_file)
                    self.controller.send_config(cfg_path)
        else:
            self.controller.disconnect_radar()

    def handle_start(self):
        mode = self.control_panel.mode_combo.currentText()
        self.plot_manager.set_mode(mode)
        self.plot_manager.clear_plots()
        self.controller.start_streaming()
        self.control_panel.set_streaming(True)

    def handle_stop(self):
        self.controller.stop_streaming()
        self.control_panel.set_streaming(False)

    def closeEvent(self, event):
        self.controller.disconnect_radar()
        event.accept()

# =============================================================================
# APPLICATION ENTRY POINT
# =============================================================================
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    controller = RadarController()
    window = MainWindow(controller)
    window.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
