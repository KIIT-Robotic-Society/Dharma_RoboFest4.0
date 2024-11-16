import sys
from PyQt5.QtWidgets import (QApplication, QMainWindow, QLabel, QWidget, 
                           QVBoxLayout, QFrame, QHBoxLayout)
from PyQt5.QtCore import Qt, QTimer, pyqtSlot, QUrl
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWebEngineWidgets import QWebEngineView
import cv2

class DharmaApp(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("DHARMA")
        self.setFixedSize(1024, 600)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #1a1a1a;
            }
            QLabel {
                color: white;
                padding-bottom: 5px;
                min-height: 30px;
            }
            QFrame {
                border-radius: 10px;
                padding: 10px;
            }
        """)

        # MapBox configuration
        self.MAPBOX_ACCESS_TOKEN = 'pk.eyJ1IjoidGR1dHRhLTEzNCIsImEiOiJjbTI5N2FzMTgwMXh6Mm9zOTVtbmw1cXEwIn0.R2rSfpDex4BIsnXb-t13cA'
        self.LATITUDE = 23.0803
        self.LONGITUDE = 72.4957
        self.zoom_level = 12.0

        # Video stream configuration
        self.stream_url = 'http://192.168.149.210:81/stream'
        
        self.init_ui()
        self.setup_video_capture()
        self.init_timers()

    def init_timers(self):
        self.timers = {
            'value': (QTimer(), self.update_value_display1),
            'dogmode': (QTimer(), self.update_value_display2),
            'watch': (QTimer(), self.update_value_display3),
            'vital': (QTimer(), self.update_value_display4)
        }
        
        for timer, callback in self.timers.values():
            timer.timeout.connect(callback)
            timer.start(1000)

    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(10)  # Reduced from 15
        main_layout.setContentsMargins(20, 0, 20, 20)  # Reduced top margin to 0

        # Header Section
        self.create_header(main_layout)
        
        # Content Section
        content_widget = QWidget()
        content_layout = QHBoxLayout(content_widget)
        content_layout.setSpacing(20)
        content_layout.setContentsMargins(0, 0, 0, 0)  # Remove margins

        # Left Section (Map and Video)
        left_section = self.create_left_section()
        content_layout.addWidget(left_section)

        # Right Section (Info Panels)
        right_section = self.create_right_section()
        content_layout.addWidget(right_section)

        main_layout.addWidget(content_widget)

    def create_header(self, main_layout):
        header_widget = QWidget()
        header_widget.setFixedHeight(90)  # Reduced from 100
        header_layout = QHBoxLayout(header_widget)
        header_layout.setContentsMargins(0, 0, 0, 0)  # Removed all margins
        
        header_layout.addStretch()
        
        center_container = QWidget()
        center_layout = QHBoxLayout(center_container)
        center_layout.setSpacing(20)
        center_layout.setContentsMargins(0, 0, 0, 0)  # Removed margins
        
        logo_label = QLabel()
        try:
            logo_pixmap = QPixmap("kutta.png").scaled(70, 70, Qt.KeepAspectRatio, Qt.SmoothTransformation)  # Reduced size
            logo_label.setPixmap(logo_pixmap)
            logo_label.setContentsMargins(0, 0, 0, 0)
        except Exception as e:
            print(f"Error loading logo: {e}")
            logo_label.setText("LOGO")
        
        title_label = QLabel("DHARMA")
        title_label.setStyleSheet("""
            color: white;
            font-size: 55px;  /* Reduced from 60px */
            font-weight: bold;
            padding: 0px;
            margin: 0px;
            qproperty-alignment: AlignVCenter;
        """)
        title_label.setContentsMargins(0, 0, 0, 0)
        
        center_layout.addWidget(logo_label, 0, Qt.AlignVCenter)
        center_layout.addWidget(title_label, 0, Qt.AlignVCenter)
        
        header_layout.addWidget(center_container, 0, Qt.AlignCenter)
        header_layout.addStretch()
        
        main_layout.addWidget(header_widget)

    def create_left_section(self):
        left_frame = QFrame()
        left_frame.setStyleSheet("""
            QFrame {
                background-color: #ab6f09;
                border-radius: 15px;
                padding: 12px;
                margin: 3px;
            }
        """)
        left_layout = QVBoxLayout(left_frame)
        left_layout.setSpacing(8)  # Reduced spacing
        left_layout.setContentsMargins(8, 0, 8, 8)  # Reduced margins, especially top

        # Create a container for video and map
        content_container = QWidget()
        content_layout = QVBoxLayout(content_container)
        content_layout.setSpacing(8)  # Reduced spacing
        content_layout.setContentsMargins(0, 0, 0, 0)

        # Video label
        self.video_label = QLabel()
        self.video_label.setFixedSize(350, 200)
        self.video_label.setStyleSheet("""
            background-color: white;
            margin-top: 0px;
        """)

        # Map view
        self.map_view = QWebEngineView()
        self.map_view.setFixedSize(350, 150)
        self.load_map()

        # Add widgets to content container
        content_layout.addWidget(self.video_label, 0, Qt.AlignTop)
        content_layout.addWidget(self.map_view, 0, Qt.AlignTop)
        
        # Add container to main layout
        left_layout.addWidget(content_container, 0, Qt.AlignTop)
        left_layout.addStretch()

        return left_frame

    def create_right_section(self):
        right_frame = QFrame()
        right_frame.setStyleSheet("""
            QFrame {
                background-color: #ab6f09;
                border-radius: 15px;
                padding: 12px;
                margin: 3px;
            }
        """)
        right_layout = QVBoxLayout(right_frame)
        right_layout.setContentsMargins(8, 0, 8, 8)  # Reduced margins, especially top
        right_layout.setSpacing(8)  # Reduced spacing
        
        content_container = QWidget()
        content_layout = QVBoxLayout(content_container)
        content_layout.setContentsMargins(0, 0, 0, 0)
        content_layout.setSpacing(8)
        
        self.create_info_panels(content_layout)
        
        right_layout.addWidget(content_container, 0, Qt.AlignTop)
        right_layout.addStretch()
        
        return right_frame

    def create_info_panels(self, layout):
        panel_style = """
            QFrame {
                background-color: #3d3d3d;
                border-radius: 10px;
                padding: 10px;
                margin: 2px;
            }
            QLabel {
                color: white;
                font-size: 18px;
                padding: 3px;
                min-height: 25px;
                qproperty-alignment: AlignCenter;
            }
        """

        # Dog MPU Panel
        dog_mpu_frame = self.create_info_panel("Failsafe", panel_style)
        self.value_label = dog_mpu_frame.findChild(QLabel, "value_label")
        layout.addWidget(dog_mpu_frame)

        # Watch MPU Panel
        watch_mpu_frame = self.create_info_panel("Joystick Mode", panel_style)
        self.watch_label = watch_mpu_frame.findChild(QLabel, "value_label")
        layout.addWidget(watch_mpu_frame)

        # Bottom section with Dog Mode and Vital signs
        bottom_layout = QHBoxLayout()
        bottom_layout.setSpacing(8)

        # Dog Mode Panel
        dog_mode_frame = self.create_info_panel("Dog Mode", panel_style)
        self.dogmode_label = dog_mode_frame.findChild(QLabel, "value_label")
        bottom_layout.addWidget(dog_mode_frame)

        # Vital Signs Panel
        vital_frame = self.create_info_panel("Vital Signs", panel_style)
        self.vital_label = vital_frame.findChild(QLabel, "value_label")
        bottom_layout.addWidget(vital_frame)

        layout.addLayout(bottom_layout)

    def create_info_panel(self, title, style):
        frame = QFrame()
        frame.setStyleSheet(style)
        layout = QVBoxLayout(frame)
        layout.setSpacing(5)  # Reduced spacing
        layout.setContentsMargins(8, 3, 8, 8)  # Reduced margins
        
        title_label = QLabel(title)
        title_label.setStyleSheet("""
            font-weight: bold;
            padding-bottom: 5px;
        """)
        layout.addWidget(title_label)
        
        value_label = QLabel()
        value_label.setObjectName("value_label")
        value_label.setWordWrap(True)
        layout.addWidget(value_label)
        
        return frame

    def update_value_display1(self):
        try:
            with open('Failsafe.txt', 'r') as file:
                value = file.read().strip()
                self.value_label.setText(value)
        except FileNotFoundError:
            self.value_label.setText("File not found")
        except Exception as e:
            self.value_label.setText(f"Error: {str(e)}")

    def update_value_display2(self):
        try:
            with open('Dogmode.txt', 'r') as file:
                value = file.read().strip()
                self.dogmode_label.setText(value)
        except FileNotFoundError:
            self.dogmode_label.setText("File not found")
        except Exception as e:
            self.dogmode_label.setText(f"Error: {str(e)}")

    def update_value_display3(self):
        try:
            with open('joystick.txt', 'r') as file:
                values = file.read().strip()
                self.watch_label.setText(values)
        except FileNotFoundError:
            self.watch_label.setText("Joystick data not available")
        except Exception as e:
            self.watch_label.setText(f"Error: {str(e)}")

    def update_value_display4(self):
        try:
            with open('vital.txt', 'r') as file:
                value = file.read().strip()
                self.vital_label.setText(value)
        except FileNotFoundError:
            self.vital_label.setText("File not found")
        except Exception as e:
            self.vital_label.setText(f"Error: {str(e)}")

    def load_map(self):
        try:
            map_url = f'https://api.mapbox.com/styles/v1/mapbox/streets-v11/static/pin-s({self.LONGITUDE},{self.LATITUDE})/{self.LONGITUDE},{self.LATITUDE},{self.zoom_level},0,0/960x720?access_token={self.MAPBOX_ACCESS_TOKEN}'
            qurl = QUrl(map_url)
            self.map_view.setUrl(qurl)
        except Exception as e:
            print(f"Error loading map: {e}")
            self.map_view.setHtml("<html><body><h2>Error loading map</h2></body></html>")

    def setup_video_capture(self):
        try:
            self.cap = cv2.VideoCapture(self.stream_url)
            if not self.cap.isOpened():
                print(f"Error: Could not open video stream from {self.stream_url}")
                self.video_label.setText("Video stream not available")
            else:
                print("Video stream opened successfully")
                self.timer = QTimer()
                self.timer.timeout.connect(self.update_frame)
                self.timer.start(30)
        except Exception as e:
            print(f"Error setting up video capture: {e}")
            self.video_label.setText("Error initializing video")

    @pyqtSlot()
    def update_frame(self):
        try:
            ret, frame = self.cap.read()
            if ret:
                frame_resized = cv2.resize(frame, (350, 200))
                frame_rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
                h, w, ch = frame_rgb.shape
                bytes_per_line = ch * w
                qt_image = QImage(frame_rgb.data, w, h, bytes_per_line, QImage.Format_RGB888)
                scaled_pixmap = QPixmap.fromImage(qt_image).scaled(
                    350, 200, Qt.KeepAspectRatio, Qt.SmoothTransformation)
                self.video_label.setPixmap(scaled_pixmap)
            else:
                print("Failed to capture video frame")
        except Exception as e:
            print(f"Error updating frame: {e}")

    def wheelEvent(self, event):
        try:
            if event.angleDelta().y() > 0:
                self.zoom_level = min(self.zoom_level + 1, 20)
            else:
                self.zoom_level = max(self.zoom_level - 1, 0)
            self.load_map()
        except Exception as e:
            print(f"Error handling wheel event: {e}")

    def closeEvent(self, event):
        try:
            if hasattr(self, 'cap'):
                self.cap.release()
            if hasattr(self, 'timer'):
                self.timer.stop()
            for timer, _ in self.timers.values():
                timer.stop()
        except Exception as e:
            print(f"Error during cleanup: {e}")
        super().closeEvent(event)

def main():
    try:
        app = QApplication(sys.argv)
        window = DharmaApp()
        window.show()
        sys.exit(app.exec_())
    except Exception as e:
        print(f"Error in main: {e}")

if __name__ == '__main__':
    main()