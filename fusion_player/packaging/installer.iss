; Inno Setup script for Fusion Player. Built by packaging/build_windows.py, which passes
; AppVersion, AppDir (PyInstaller output folder), ExtrasDir (APK + guides) and IconFile.
; Fusion Player replaces the older IMU Live app: installing it removes IMU Live (recordings are kept).

#ifndef AppVersion
  #define AppVersion "2.0.0"
#endif
#define ImuLiveKey "Software\Microsoft\Windows\CurrentVersion\Uninstall\{6F3A9C2E-4B71-4E8D-9A35-2C1D7B0E5F44}_is1"

[Setup]
AppId={{9B05F0A7-ECDE-475B-A16E-28F227D9CB55}
AppName=Fusion Player
AppVersion={#AppVersion}
AppPublisher=Transfinitte'26 Problem 4 team
DefaultDirName={autopf}\Fusion Player
DefaultGroupName=Fusion Player
DisableProgramGroupPage=yes
; Per-user install by default (no admin needed); the user can choose all users.
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
OutputBaseFilename=Fusion-Player-Setup-{#AppVersion}
SetupIconFile={#IconFile}
UninstallDisplayIcon={app}\Fusion Player.exe
Compression=lzma2/max
SolidCompression=yes
WizardStyle=modern
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible

[Tasks]
Name: "desktopicon"; Description: "Create a desktop shortcut"; GroupDescription: "Shortcuts:"
Name: "firewall"; Description: "Allow Fusion Player to receive phone data through Windows Firewall (private networks, UDP 5555)"; GroupDescription: "Network:"; Check: IsAdminInstallMode

[Files]
Source: "{#AppDir}\*"; DestDir: "{app}"; Flags: ignoreversion recursesubdirs createallsubdirs
Source: "{#ExtrasDir}\*"; DestDir: "{app}\extras"; Flags: ignoreversion skipifsourcedoesntexist

[Icons]
Name: "{group}\Fusion Player"; Filename: "{app}\Fusion Player.exe"
Name: "{group}\Phone app (APK) folder"; Filename: "{app}\extras"
Name: "{group}\How to use"; Filename: "{app}\extras\How to use.md"
Name: "{group}\Data collection guide"; Filename: "{app}\extras\Data collection guide.md"
Name: "{group}\Recordings"; Filename: "{userdocs}\IMU Live"
Name: "{group}\Uninstall Fusion Player"; Filename: "{uninstallexe}"
Name: "{autodesktop}\Fusion Player"; Filename: "{app}\Fusion Player.exe"; Tasks: desktopicon

[Dirs]
Name: "{userdocs}\IMU Live"; Flags: uninsneveruninstall

[Run]
Filename: "{sys}\netsh.exe"; Parameters: "advfirewall firewall add rule name=""Fusion Player"" dir=in action=allow program=""{app}\Fusion Player.exe"" protocol=UDP localport=5555 profile=private"; Flags: runhidden; Tasks: firewall
Filename: "{app}\Fusion Player.exe"; Description: "Start Fusion Player now"; Flags: nowait postinstall skipifsilent

[UninstallRun]
Filename: "{sys}\netsh.exe"; Parameters: "advfirewall firewall delete rule name=""Fusion Player"""; Flags: runhidden; RunOnceId: "DelFirewallRule"; Check: IsAdminInstallMode

[Code]
{ Remove the older IMU Live app (per-user or all-users install) before installing. Its uninstaller keeps the
  recordings in Documents\IMU Live. A failure here never stops the Fusion Player install. }
procedure RemoveImuLive(Root: Integer);
var
  Cmd: String;
  Code: Integer;
begin
  if RegQueryStringValue(Root, '{#ImuLiveKey}', 'UninstallString', Cmd) then
    Exec(RemoveQuotes(Cmd), '/VERYSILENT /SUPPRESSMSGBOXES /NORESTART', '', SW_HIDE, ewWaitUntilTerminated, Code);
end;

procedure CurStepChanged(CurStep: TSetupStep);
begin
  if CurStep = ssInstall then
  begin
    RemoveImuLive(HKCU);
    if IsAdminInstallMode then
      RemoveImuLive(HKLM);
  end;
end;
