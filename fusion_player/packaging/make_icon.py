"""Draw the app icon (a walked path on a blue tile) and save packaging/icon.ico."""
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.patches import FancyBboxPatch  # noqa: E402
from PIL import Image  # noqa: E402

here = Path(__file__).parent
fig = plt.figure(figsize=(2.56, 2.56), dpi=100)
ax = fig.add_axes([0, 0, 1, 1])
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.axis("off")
ax.add_patch(FancyBboxPatch((0.04, 0.04), 0.92, 0.92, boxstyle="round,pad=0,rounding_size=0.18",
                            fc="#1f6feb", ec="none"))
t = np.linspace(0, 1, 200)
x = 0.2 + 0.6 * t
y = 0.28 + 0.38 * t + 0.08 * np.sin(2 * np.pi * 1.5 * t)
ax.plot(x, y, color="white", lw=9, solid_capstyle="round")
ax.plot(x[0], y[0], "o", color="white", ms=16)
ax.plot(x[-1], y[-1], "o", color="#ffb000", ms=22, mec="white", mew=3)
png = here / "icon.png"
fig.savefig(png, transparent=True)
Image.open(png).save(here / "icon.ico", sizes=[(16, 16), (32, 32), (48, 48), (64, 64), (128, 128), (256, 256)])
print("wrote", here / "icon.ico")
