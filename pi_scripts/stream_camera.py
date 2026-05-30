"""
Pi Zero 2W camera streaming server.
Streams the Pi camera as MJPEG over HTTP on port 8080.

Deploy to each Pi and run: python3 stream_camera.py

Dependencies (install on Pi):
  pip install picamera2

USB gadget network setup (add to /boot/firmware/config.txt):
  dtoverlay=dwc2
  (and add modules-load=dwc2,g_ether to /boot/firmware/cmdline.txt)

Pi #1 (overhead): static IP 192.168.7.2  → main computer 192.168.7.1
Pi #2 (gaze):     static IP 192.168.8.2  → main computer 192.168.8.1

Main computer reads stream:
  cv2.VideoCapture('http://192.168.7.2:8080/stream.mjpg')
"""

import io
import threading
from http import server

# TODO: uncomment on actual Pi hardware
# from picamera2 import Picamera2
# from picamera2.encoders import MJPEGEncoder
# from picamera2.outputs import FileOutput

PORT = 8080
RESOLUTION = (640, 480)
FRAMERATE = 30

PAGE = b"""<html>
<head><title>Pi Camera Stream</title></head>
<body><img src="/stream.mjpg" /></body>
</html>"""


class StreamOutput(io.BufferedIOBase):
    def __init__(self):
        self.frame = None
        self.condition = threading.Condition()

    def write(self, buf):
        with self.condition:
            self.frame = buf
            self.condition.notify_all()


class StreamHandler(server.BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # suppress per-request logs

    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.send_header('Content-Length', len(PAGE))
            self.end_headers()
            self.wfile.write(PAGE)

        elif self.path == '/stream.mjpg':
            self.send_response(200)
            self.send_header('Age', 0)
            self.send_header('Cache-Control', 'no-cache, private')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
            self.end_headers()
            try:
                while True:
                    with output.condition:
                        output.condition.wait()
                        frame = output.frame
                    self.wfile.write(b'--FRAME\r\n')
                    self.send_header('Content-Type', 'image/jpeg')
                    self.send_header('Content-Length', len(frame))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b'\r\n')
            except Exception:
                pass
        else:
            self.send_error(404)


output = StreamOutput()


def main():
    # TODO: uncomment on actual Pi hardware
    # picam2 = Picamera2()
    # picam2.configure(picam2.create_video_configuration(
    #     main={'size': RESOLUTION, 'format': 'RGB888'},
    # ))
    # picam2.start_recording(MJPEGEncoder(), FileOutput(output))

    addr = ('', PORT)
    httpd = server.HTTPServer(addr, StreamHandler)
    print(f'Streaming on http://0.0.0.0:{PORT}/stream.mjpg')
    try:
        httpd.serve_forever()
    finally:
        pass
        # picam2.stop_recording()


if __name__ == '__main__':
    main()
