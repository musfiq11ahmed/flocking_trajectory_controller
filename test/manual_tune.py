import socket

# --- NETWORK SETUP ---

from config import UDP_IP, UDP_PORT

print(f"Opening UDP socket to target IP: {UDP_IP}:{UDP_PORT}")
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("\n=======================================")
print("      SWARM COMMAND & TUNE CENTER      ")
print("=======================================")
print("Commands:")
print("  t [left] [right] -> Set target m/s (e.g., t 0.3 0.3)")
print("  p [Kp] [Ki] [Kd] -> Tune PID (e.g., p 0.25 0.02 0.05)")
print("  s                -> STOP (Targets to 0.0)")
print("  q                -> Quit")
print("=======================================\n")

try:
    while True:
        cmd_input = input("Enter command: ").strip().split()
        
        if not cmd_input:
            continue
            
        cmd = cmd_input[0].lower()

        # 1. SEND TARGET (Simple Trajectory)
        if cmd == 't' and len(cmd_input) == 3:
            left_mps = cmd_input[1]
            right_mps = cmd_input[2]
            message = f"TARGET,{left_mps},{right_mps}"
            sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
            print(f" >> Sent: {message}")

        # 2. SEND PID TUNE
        elif cmd == 'p' and len(cmd_input) == 4:
            kp = cmd_input[1]
            ki = cmd_input[2]
            kd = cmd_input[3]
            message = f"TUNE,{kp},{ki},{kd}"
            sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
            print(f" >> Sent: {message}")

        # 3. EMERGENCY STOP
        elif cmd == 's':
            message = "TARGET,0.0,0.0"
            sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
            print(" >> Sent: EMERGENCY STOP")

        # 4. QUIT
        elif cmd == 'q':
            print("Shutting down Command Center.")
            break
            
        else:
            print(" !! Invalid format. Check the command list.")

except KeyboardInterrupt:
    print("\nShutting down Command Center.")
    sock.close()