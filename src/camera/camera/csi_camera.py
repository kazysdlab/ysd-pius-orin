import cv2
import atexit

class CSICamera:
    def __init__(
        self,
        sensor_id: int = 0,
        capture_size: tuple[int] = (1280, 720),
        capture_fps: int = 60,
        resize: tuple[int] = (1280, 720),
        flip_method: int = 0,
        nvargs: str = ""
    ):
        self.sensor_id = sensor_id
        self.capture_size = capture_size
        self.capture_fps = capture_fps
        self.resize = resize
        self.flip_method = flip_method
        self.nvargs = nvargs

        pipeline = self._pipeline()
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError("Failed to open CSI camera via GStreamer.")

        atexit.register(self.cap.release)
        
    def _pipeline(self):
        w, h = self.capture_size
        rw, rh = self.resize
        return (
            f"nvarguscamerasrc sensor-id={self.sensor_id} ! video/x-raw(memory:NVMM), width={w}, height={h}, format=NV12, framerate={self.capture_fps}/1 ! "
            f"nvvidconv flip-method={self.flip_method} ! video/x-raw, width={rw}, height={rh}, format=BGRx ! "
            f"videoconvert ! video/x-raw, format=BGR ! "
            f"appsink drop=true max-buffers=1 sync=false"
        )

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            raise RuntimeError("Camera read() failed.")
        return frame

    def release(self):
        if hasattr(self, "cap") and self.cap:
            self.cap.release()