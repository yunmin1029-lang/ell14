import time
import numpy as np
from elliptec import ElliptecRotationStage

mount = ElliptecRotationStage(port='COM11')

#초기화
mount.home()
time.sleep(1.0)
mount.tare()
time.sleep(1.0)
print("Start angle scan")

#scan
angles = np.arange(0,181,5)

for ang in angles:
    mount.angle = ang
    time.sleep(5.0)
    print(f"Angle = {mount.angle:.2f} deg")
 
mount.angle = 0
mount.close()
print("Scan finished")
