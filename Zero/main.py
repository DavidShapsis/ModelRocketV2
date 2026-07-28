import time
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
# 1. Turn off Autofocus (set mode to Manual)
picam.set_controls({"AfMode": 0}) 

# 2. Set the physical lens position to Infinity (0.0)
# (In libcamera, 0.0 is infinity, and larger numbers are closer focus)
picam.set_controls({"LensPosition": 0.0}) 
print("Camera focus LOCKED to Infinity.")
# ==========================================================

output_filename = "rocket_flight.h264"
print(f"Starting hardware recording... Saving to {output_filename}")

picam.start_recording(output_filename)
picam.start()

try:
    # Adjust this to match your flight window timeline
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
    print("Done! Safe landing.")