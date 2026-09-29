import re

with open('auto/auto_1.3.0.py', 'r') as f:
    content = f.read()

# Add print to send_udp
old_send = """        try:
            sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
        except Exception:
            pass
        _last_cmd      = cmd
        _last_cmd_time = now"""

new_send = """        try:
            sock.sendto(cmd.encode(), (ESP32_IP, UDP_PORT))
            if cmd != _last_cmd:
                print(f"[UDP] Sent: {cmd}")
        except Exception as e:
            print(f"[UDP] Error: {e}")
        _last_cmd      = cmd
        _last_cmd_time = now"""

content = content.replace(old_send, new_send)

with open('auto/auto_1.3.0.py', 'w') as f:
    f.write(content)
