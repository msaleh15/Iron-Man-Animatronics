import serial
import time
import pygame

# === CONFIGURE THESE ===
COM_PORT = "COM5"  # <-- change this to your Arduino's port
BAUD_RATE = 9600

DETECT_MP3_PATH = r"C:\Users\kingl\Documents\Solidworks\Mechatronics Project\Open.mp3"
BUTTON_MP3_PATH = r"C:\Users\kingl\Documents\Solidworks\Mechatronics Project\Close.mp3"
# =======================


def main():
    # Initialize audio
    pygame.mixer.init()

    print(f"Opening serial port {COM_PORT} at {BAUD_RATE} baud...")
    ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # Give Arduino time to reset

    print("Listening for AUDIO_* commands from Arduino...")

    try:
        while True:
            if ser.in_waiting > 0:
                line = ser.readline().decode(errors="ignore").strip()
                if not line:
                    continue

                print(f"Serial: {line}")

                if line == "AUDIO_DETECT_START":
                    # Load and play the detect MP3 (loop while active)
                    pygame.mixer.music.load(DETECT_MP3_PATH)
                    pygame.mixer.music.play(-1)  # -1 = loop
                elif line == "AUDIO_DETECT_STOP":
                    if pygame.mixer.music.get_busy():
                        pygame.mixer.music.stop()

                elif line == "AUDIO_BUTTON_START":
                    # Load and play the button MP3 (loop while active)
                    pygame.mixer.music.load(BUTTON_MP3_PATH)
                    pygame.mixer.music.play(-1)
                elif line == "AUDIO_BUTTON_STOP":
                    if pygame.mixer.music.get_busy():
                        pygame.mixer.music.stop()

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        if pygame.mixer.music.get_busy():
            pygame.mixer.music.stop()
        ser.close()
        pygame.mixer.quit()


if __name__ == "__main__":
    main()
