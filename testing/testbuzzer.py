import sys
sys.path.append('/src')
from src import hw

h = hw.HW()
h.turnOn()
h.read()
h.beep()
print(h.values["BAT_TEMP"])