import time
import subprocess
from picamera2 import Picamera2

print("Initializing Camera Module 3...")
picam = Picamera2()

# Configure camera settings (720p balanced mode)
config = picam.create_video_configuration(
    main={"size": (1280, 720), "format": "XRGB8888"},
    video={"size": (1280, 720), "format": "H264"}
)
picam.configure(config)

# ==========================================================
# LOCK FOCUS TO INFINITY FOR FLIGHT
# ==========================================================
picam.set_controls({"AfMode": 0})
picam.set_controls({"LensPosition": 0.0})
print("Camera focus LOCKED to Infinity.")
# ==========================================================

output_filename = "rocket_flight.h264"
mp4_filename = "rocket_flight.mp4"

# IMPORTANT: set this to match the actual FPS you're achieving at this
# resolution on the Zero W (you measured ~20-24 FPS at 720p single-core).
# If it doesn't match reality, playback speed will be off.
RECORDED_FPS = 22

print(f"Starting hardware recording... Saving to {output_filename}")

picam.start_recording(output_filename)
picam.start()

try:
    flight_duration = 60
    print(f"Recording flight for {flight_duration} seconds...")
    time.sleep(flight_duration)

except KeyboardInterrupt:
    print("\nRecording manually stopped.")

finally:
    print("Saving video file and shutting down camera assets...")
    picam.stop_recording()
    picam.stop()
    picam.close()
    print("Camera closed.")

    # ==========================================================
    # AUTO-CONVERT TO MP4
    # ==========================================================
    print(f"Converting {output_filename} to {mp4_filename}...")
    try:
        subprocess.run(
            [
                "ffmpeg", "-y",
                "-r", str(RECORDED_FPS),
                "-i", output_filename,
                "-c", "copy",
                mp4_filename
            ],
            check=True
        )
        print(f"Conversion complete: {mp4_filename}")
    except FileNotFoundError:
        print("ffmpeg not found. Install it with: sudo apt install ffmpeg")
    except subprocess.CalledProcessError as e:
        print(f"ffmpeg conversion failed: {e}")

    print("Done! Safe landing.")