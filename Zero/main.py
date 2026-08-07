import time
import subprocess
from datetime import datetime
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput

print("Initializing Camera Module 3...")
picam = Picamera2()

config = picam.create_video_configuration(
    main={"size": (1280, 720), "format": "XRGB8888"}
)
picam.configure(config)

picam.set_controls({"AfMode": 0, "LensPosition": 0.0})
print("Camera focus LOCKED to Infinity.")

# Create a unique filename based on boot time
timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
output_path = f"/home/pi/flight_{timestamp}.mp4"

ffmpeg_cmd = [
    "ffmpeg",
    "-y",
    "-f", "h264",
    "-i", "pipe:0",
    "-c", "copy",
    "-movflags", "frag_keyframe+empty_moov+default_base_moof",
    output_path
]

print(f"Starting ffmpeg, writing to {output_path}...")
ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

encoder = H264Encoder()
output = FileOutput(ffmpeg_process.stdin)

print("Starting recording...")
picam.start_recording(encoder, output)

try:
    while True:
        time.sleep(1)
except (KeyboardInterrupt, SystemExit):
    print("Stopping recording...")
    picam.stop_recording()
    ffmpeg_process.stdin.close()
    ffmpeg_process.wait()