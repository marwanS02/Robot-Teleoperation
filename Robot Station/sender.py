# sender_udp_tunable.py

import cv2
import socket
import struct
import time

#WINDOWS_IP = "100.118.89.87" #Marwan Laptop
WINDOWS_IP = "192.168.4.3" #short distance tp link exo
#WINDOWS_IP = "100.66.145.56"  #Huimin Laptop tailscale IP
PORT = 5057

CHUNK_SIZE = 1400

# >>> tweak these two by trial and error <<<
REQ_WIDTH  = 1024  # try 640, 800, 1024, 1280, 1600, 1920...
REQ_HEIGHT = 576

def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        raise RuntimeError("Could not open webcam")

    # Request resolution
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  REQ_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, REQ_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, 20)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # Read back what we *actually* got
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    real_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"[SENDER] Requested: {REQ_WIDTH}x{REQ_HEIGHT} @ 20 fps")
    print(f"[SENDER] Actual:    {real_w}x{real_h} @ {real_fps:.1f} fps")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    frame_id = 0
    t_last_log = time.time()
    bytes_this_sec = 0
    frames_this_sec = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            print("[SENDER] Failed to grab frame.")
            continue

        ok, encoded = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
        if not ok:
            print("[SENDER] JPEG encode failed, skipping frame.")
            continue

        data = encoded.tobytes()
        total_chunks = (len(data) + CHUNK_SIZE - 1) // CHUNK_SIZE

        for chunk_idx in range(total_chunks):
            chunk = data[chunk_idx * CHUNK_SIZE:(chunk_idx + 1) * CHUNK_SIZE]
            header = struct.pack("!IHH", frame_id, total_chunks, chunk_idx)
            sock.sendto(header + chunk, (WINDOWS_IP, PORT))

        frame_id = (frame_id + 1) % 2**32

        # bandwidth & fps logging once per second
        now = time.time()
        bytes_this_sec += len(data)
        frames_this_sec += 1
        if now - t_last_log >= 1.0:
            mbps = (bytes_this_sec * 8) / 1e6
            print(f"[SENDER] ~{frames_this_sec} fps, avg frame {bytes_this_sec//frames_this_sec} B, ~{mbps:.2f} Mbit/s")
            bytes_this_sec = 0
            frames_this_sec = 0
            t_last_log = now

        time.sleep(0.005)

if __name__ == "__main__":
    main()
