import os

os.system(
    "chromium --kiosk --start-fullscreen "
    "--ignore-certificate-errors --allow-insecure-localhost "
    "--enable-features=TouchpadAndWheelScrollLatching,TouchEventFeatureDetection "
    "--touch-events=enabled "
    "\"http://localhost\""
)
