; Inno Setup script for IMU Live. Built by packaging/build_windows.py, which passes
; AppVersion, AppDir (PyInstaller output folder), ExtrasDir (APK + guide) and IconFile.

#ifndef AppVersion
  #define AppVersion "1.2.0"
#endif

[Setup]
AppId={{6F3A9C2E-4B71-4E8D-9A35-2C1D7B0E5F44}
AppName=IMU Live
AppVersion={#AppVersion}
AppPublisher=Transfinitte'26 Problem 4 team
DefaultDirName={autopf}\IMU Live
DefaultGroupName=IMU Live
DisableProgramGroupPage=yes
; Per-user install by default (no admin needed); the user can choose all users.
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
OutputBaseFilename=IMU-Live-Setup-{#AppVersion}
SetupIconFile={#IconFile}
UninstallDisplayIcon={app}\IMU Live.exe
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; GroupDescription: "Shortcuts:"
Name: "firewall"; Description: "Allow IMU Live to receive phone data through Windows Firewall (private networks, UDP 5555)"; GroupDescription: "Network:"; Check: IsAdminInstallMode

[Files]
Source: "{#AppDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#ExtrasDir}\*"; DestDir: "{app}\extras"; Flags: ignoreversion skipifsourcedoesntexist

[Icons]
Name: "{group}\IMU Live"; Filename: "{app}\IMU Live.exe"
Name: "{group}\Phone app (APK) folder"; Filename: "{app}\extras"
Name: "{group}\How to use"; Filename: "{app}\extras\How to use.md"
Name: "{group}\Data collection guide"; Filename: "{app}\extras\Data collection guide.md"
Name: "{group}\Recordings"; Filename: "{userdocs}\IMU Live"
Name: "{group}\Uninstall IMU Live"; Filename: "{uninstallexe}"
Name: "{autodesktop}\IMU Live"; Filename: "{app}\IMU Live.exe"; Tasks: desktopicon

[Dirs]
Name: "{userdocs}\IMU Live"; Flags: uninsneveruninstall

[Run]
Filename: "{sys}\netsh.exe"; Parameters: "advfirewall firewall add rule name=""IMU Live"" dir=in action=allow program=""{app}\IMU Live.exe"" protocol=UDP localport=5555 profile=private"; Flags: runhidden; Tasks: firewall
Filename: "{app}\IMU Live.exe"; Description: "Start IMU Live now"; Flags: nowait postinstall skipifsilent

[UninstallRun]
Filename: "{sys}\netsh.exe"; Parameters: "advfirewall firewall delete rule name=""IMU Live"""; Flags: runhidden; RunOnceId: "DelFirewallRule"; Check: IsAdminInstallMode
