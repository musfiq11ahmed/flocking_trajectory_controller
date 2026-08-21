import socket
import time

# NETWORK SETUP
# The IP address assigned by router
UDP_IP = "192.168.0.101" 
UDP_PORT = 4210

# Create a UDP socket
print(f"Opening UDP socket to target IP: {UDP_IP}:{UDP_PORT}")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# THE MESSAGE
# Formatted as: "TARGET, Left_m/s, Right_m/s"
message = "TARGET,0.33,0.33"

print("\nPress Ctrl+C to stop sending.")
try:
    while True:
        # Encode the string into bytes and send it over Wi-Fi
        sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
        print(f"Sent: {message}")
        
        # Pause for 100ms (simulate a 10Hz camera tracking loop)
        time.sleep(0.1)
        
except KeyboardInterrupt:
    print("\nTransmission stopped by user.")
    sock.close()