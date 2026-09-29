import re

with open('src/auto_1.3.0.ino', 'r') as f:
    content = f.read()

content = content.replace("#define FORWARD_SPEED    135", "#define FORWARD_SPEED    100")
content = content.replace("#define TURN_SPEED       120", "#define TURN_SPEED       60")
content = content.replace("#define RETREAT_SPEED   -130", "#define RETREAT_SPEED   -100")

with open('src/auto_1.3.0.ino', 'w') as f:
    f.write(content)
