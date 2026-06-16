import socket
import struct
import cv2
import numpy as np

HOST = "0.0.0.0"
PORT = 5057

WINDOW_NAME = "Remote Camera"

def main():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((HOST, PORT))

    print(f"[RECEIVER] Listening on UDP {HOST}:{PORT}")

    # Allow the window to be resized by the user (OS standard resizing)
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

    # Frame assembly buffer
    buffers = {}

    while True:
        packet, _ = sock.recvfrom(65535)

        if len(packet) < 8:
            continue

        frame_id, total_chunks, chunk_idx = struct.unpack("!IHH", packet[:8])
        chunk = packet[8:]

        if frame_id not in buffers:
            buffers[frame_id] = {
                "total": total_chunks,
                "received": 0,
                "chunks": {}
            }

        buf = buffers[frame_id]
        if chunk_idx not in buf["chunks"]:
            buf["chunks"][chunk_idx] = chunk
            buf["received"] += 1

        # Completed frame?
        if buf["received"] == buf["total"]:
            # Reassemble
            ordered = b"".join(buf["chunks"][i] for i in range(buf["total"]))

            # Clean up old buffers
            keys = list(buffers.keys())
            for k in keys:
                if k != frame_id:
                    del buffers[k]

            # Decode JPEG
            arr = np.frombuffer(ordered, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                continue

            # Display the frame directly
            # The window size is handled by the OS/Mouse because of WINDOW_NORMAL
            cv2.imshow(WINDOW_NAME, frame)

            if cv2.waitKey(1) & 0xFF == 27: # ESC to quit
                break

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()