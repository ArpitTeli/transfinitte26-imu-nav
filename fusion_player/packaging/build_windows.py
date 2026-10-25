"""Build the Fusion Player Windows app and its installer (it replaces the older IMU Live app).

Run from fusion_player/:  python packaging/build_windows.py
Needs: pip install pyinstaller, and Inno Setup 6 (winget install JRSoftware.InnoSetup).
Output: <temp>/fusion-build/dist/Fusion Player/Fusion Player.exe (portable app) and
        build/installer/Fusion-Player-Setup-<version>.exe
"""
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

VERSION = "2.0.0"
ROOT = Path(__file__).resolve().parents[1]                 # fusion_player/
IMU_LIVE = ROOT.parent / "imu_live"
PKG = ROOT / "packaging"
OUT = ROOT / "build"
# The intermediate build goes outside the project (PyInstaller's nested files can pass Windows' 260-character path
# limit, and OneDrive locks folders while syncing them, which makes --clean fail).
BUILD = Path(os.environ.get("FUSION_BUILD") or Path(tempfile.gettempdir()) / "fusion-build")
ICON = PKG / "icon.ico"                                   # made by make_icon.py
APK = IMU_LIVE / "android" / "app" / "build" / "outputs" / "apk" / "debug" / "app-debug.apk"


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
    subprocess.run([
        sys.executable, "-m", "PyInstaller", "--noconfirm", "--clean", "--windowed",
        "--name", "Fusion Player", "--icon", str(ICON),
        "--paths", str(ROOT), "--paths", str(IMU_LIVE),            # app modules + imunav (the foot filter)
        "--add-data", f"{ROOT / 'filter' / 'replay.exe'};filter",   # the robot filter, built from fusion.c
        "--add-data", f"{ROOT / 'settings'};settings",              # its settings profiles
        # not used by the app; keeps the bundle small
        "--exclude-module", "matplotlib", "--exclude-module", "scipy", "--exclude-module", "tkinter",
        "--exclude-module", "PyQt5", "--exclude-module", "PyQt6", "--exclude-module", "IPython",
        "--exclude-module", "pytest",
        "--distpath", str(BUILD / "dist"), "--workpath", str(BUILD / "work"), "--specpath", str(BUILD),
        str(ROOT / "app.py"),
    ], check=True)
    app = BUILD / "dist" / "Fusion Player"
    print("app:", app / "Fusion Player.exe")

    extras = BUILD / "extras"
    shutil.rmtree(extras, ignore_errors=True)
    extras.mkdir(parents=True)
    shutil.copy(ROOT / "README.md", extras / "How to use.md")
    shutil.copy(IMU_LIVE / "docs" / "data_collection.md", extras / "Data collection guide.md")
    if APK.exists():
        shutil.copy(APK, extras / "ImuStream.apk")
    else:
        print("warning: phone APK not built; installer will not include it")

    subprocess.run([str(find_iscc()), f"/DAppVersion={VERSION}", f"/DAppDir={app}", f"/DExtrasDir={extras}",
                    f"/DIconFile={ICON}", f"/O{OUT / 'installer'}", str(PKG / "installer.iss")], check=True)
    print("installer:", OUT / "installer" / f"Fusion-Player-Setup-{VERSION}.exe")


if __name__ == "__main__":
    main()
