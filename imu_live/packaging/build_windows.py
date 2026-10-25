"""Build the Windows app and its installer.

Run from the project root:  python packaging/build_windows.py
Needs: pip install pyinstaller, and Inno Setup 6 (winget install JRSoftware.InnoSetup).
Output: <temp>/imulive-build/dist/IMU Live/IMU Live.exe (portable app) and build/installer/IMU-Live-Setup-<version>.exe
"""
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

VERSION = "1.2.0"
ROOT = Path(__file__).resolve().parents[1]
PKG = ROOT / "packaging"
OUT = ROOT / "build"
# The intermediate build goes outside the project unless IMULIVE_BUILD says otherwise: PyInstaller writes
# deeply nested files (a long path breaks Windows' 260-char limit), and OneDrive locks folders while it
# syncs them, which makes --clean fail with "Access is denied".
BUILD = Path(os.environ.get("IMULIVE_BUILD") or Path(tempfile.gettempdir()) / "imulive-build")
APK = ROOT / "android" / "app" / "build" / "outputs" / "apk" / "debug" / "app-debug.apk"


def find_iscc():
    for p in (Path(os.environ.get("LOCALAPPDATA", "")) / "Programs/Inno Setup 6/ISCC.exe",
              Path("C:/Program Files (x86)/Inno Setup 6/ISCC.exe"),
              Path("C:/Program Files/Inno Setup 6/ISCC.exe")):
        if p.exists():
            return p
    found = shutil.which("iscc")
    if found:
        return Path(found)
    raise SystemExit("Inno Setup 6 not found: winget install JRSoftware.InnoSetup")


def main():
    if not (PKG / "icon.ico").exists():
        subprocess.run([sys.executable, str(PKG / "make_icon.py")], check=True)
    subprocess.run([
        sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean", "--windowed",
        "--name", "IMU Live",
        "--icon", str(PKG / "icon.ico"),
        "--paths", str(ROOT),
        # the window backend; without it the app starts, then crashes opening its window
        "--hidden-import", "matplotlib.backends.backend_tkagg",
        # not used by the live app; keeps the bundle small
        "--exclude-module", "scipy", "--exclude-module", "pandas", "--exclude-module", "pytest",
        "--distpath", str(BUILD / "dist"), "--workpath", str(BUILD / "work"), "--specpath", str(BUILD),
        str(ROOT / "scripts" / "live.py"),
    ], check=True)
    app = BUILD / "dist" / "IMU Live"
    print("app:", app / "IMU Live.exe")

    extras = BUILD / "extras"
    shutil.rmtree(extras, ignore_errors=True)
    extras.mkdir(parents=True)
    shutil.copy(ROOT / "docs" / "live_demo.md", extras / "How to use.md")
    shutil.copy(ROOT / "docs" / "data_collection.md", extras / "Data collection guide.md")
    if APK.exists():
        shutil.copy(APK, extras / "ImuStream.apk")
    else:
        print("warning: phone APK not built; installer will not include it")

    subprocess.run([str(find_iscc()), f"/DAppVersion={VERSION}", f"/DAppDir={app}", f"/DExtrasDir={extras}",
                    f"/DIconFile={PKG / 'icon.ico'}", f"/O{OUT / 'installer'}", str(PKG / "installer.iss")],
                   check=True)
    print("installer:", OUT / "installer" / f"IMU-Live-Setup-{VERSION}.exe")


if __name__ == "__main__":
    main()
