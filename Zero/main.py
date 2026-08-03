import time
import subprocess
from picamera2 import Picamera2
from picamera2.encoders import H264Encoder
from picamera2.outputs import FileOutput

print("Initializing Camera Module 3...")
picam = Picamera2()

config = picam.create_video_configuration(
    main={"size": (1280, 720), "format": "XRGB8888"},
    video={"size": (1280, 720), "format": "H264"}
)
picam.configure(config)

picam.set_controls({"AfMode": 0})
picam.set_controls({"LensPosition": 0.0})
print("Camera focus LOCKED to Infinity.")

output_path = "/home/pi/rocket_flight.mp4"

# Spawn ffmpeg manually so we control the mp4 fragmentation flags directly.
# frag_keyframe+empty_moov: write self-contained fragments as data arrives,
# instead of relying on a single index written at the very end of the file.
ffmpeg_cmd = [
    "ffmpeg",
    "-y",
    "-f", "h264",
    "-i", "pipe:0",
    "-c", "copy",
    "-movflags", "frag_keyframe+empty_moov+default_base_moof",
    output_path
]

print(f"Starting ffmpeg, writing fragmented mp4 to {output_path}...")
ffmpeg_process = subprocess.Popen(ffmpeg_cmd, stdin=subprocess.PIPE)

encoder = H264Encoder()
output = FileOutput(ffmpeg_process.stdin)

print("Starting recording — will run until power is cut or process is killed...")
picam.start_recording(encoder, output)

try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Manually stopped.")
    picam.stop_recording()
    ffmpeg_process.stdin.close()
    ffmpeg_process.wait()