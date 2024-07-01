import sys
sys.path.append('/src')
from src import hw

h = hw.HW()
h.read()
print(h.values["BAT_TEMP"])