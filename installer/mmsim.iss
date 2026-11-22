; Inno Setup 6 script for Micromouse Studio (MMSim).
; Build with tools/build_installer.ps1, which passes the defines below:
;   /DAppVersion=0.2.1  /DStageDir=<dist\mmsim-v0.2.1-win64>  /DRepoRoot=<repo>  /DOutName=MMSim-v0.2.1-Setup
; The stage folder is the same one the portable zip is made from (tools/package_release.ps1).

#ifndef AppVersion
  #define AppVersion "0.2.1"
#endif
#ifndef RepoRoot
  #define RepoRoot ".."
#endif
#ifndef StageDir
  #define StageDir RepoRoot + "\dist\mmsim-v" + AppVersion + "-win64"
#endif
#ifndef OutName
  #define OutName "MMSim-v" + AppVersion + "-Setup"
#endif

#define AppName "Micromouse Studio"
#define AppExe  "mmsim.exe"

[Setup]
; Stable id: lets a newer Setup.exe upgrade this install in place.
AppId={{6F0B7C1E-3D52-4B7A-9A64-5C0D1E7A2B91}
AppName={#AppName}
AppVersion={#AppVersion}
AppVerName={#AppName} {#AppVersion}
AppPublisher=MMSim contributors
AppPublisherURL=https://github.com/madukamalruk/micromouse-studio
AppSupportURL=https://github.com/madukamalruk/micromouse-studio/issues
AppUpdatesURL=https://github.com/madukamalruk/micromouse-studio/releases
VersionInfoVersion={#AppVersion}
VersionInfoDescription={#AppName} Setup
; Per-user install: no UAC prompt, nothing outside the user profile is touched.
PrivilegesRequired=lowest
PrivilegesRequiredOverridesAllowed=dialog
DefaultDirName={localappdata}\Programs\MMSim
DefaultGroupName={#AppName}
DisableProgramGroupPage=yes
DisableDirPage=no
LicenseFile={#RepoRoot}\LICENSE
SetupIconFile={#RepoRoot}\assets\mmsim.ico
UninstallDisplayIcon={app}\{#AppExe}
UninstallDisplayName={#AppName}
OutputDir={#RepoRoot}\dist
OutputBaseFilename={#OutName}
Compression=lzma2/max
SolidCompression=yes
ArchitecturesAllowed=x64compatible
ArchitecturesInstallIn64BitMode=x64compatible
WizardStyle=modern
CloseApplications=yes
RestartApplications=no
MinVersion=10.0

[Languages]
Name: "english"; MessagesFile: "compiler:Default.isl"

[Tasks]
Name: "desktopicon"; Description: "Create a &Desktop shortcut"; GroupDescription: "Shortcuts:"; Flags: unchecked

[Files]
; Everything the portable zip carries, except the portable-only bits (launcher .bat, marker).
Source: "{#StageDir}\*"; DestDir: "{app}"; Excludes: "mmsim.portable,Launch_MMSim.bat"; Flags: ignoreversion recursesubdirs createallsubdirs
; Tells the app to keep its data in %LOCALAPPDATA%\MMSim instead of the install folder.
Source: "{#RepoRoot}\installer\mmsim.installed"; DestDir: "{app}"; Flags: ignoreversion
; Icon file for the shortcuts (the exe embeds the same icon).
Source: "{#RepoRoot}\assets\mmsim.ico"; DestDir: "{app}"; Flags: ignoreversion

[Icons]
Name: "{group}\{#AppName}"; Filename: "{app}\{#AppExe}"; WorkingDir: "{app}"; IconFilename: "{app}\mmsim.ico"
Name: "{group}\Uninstall {#AppName}"; Filename: "{uninstallexe}"
Name: "{autodesktop}\{#AppName}"; Filename: "{app}\{#AppExe}"; WorkingDir: "{app}"; IconFilename: "{app}\mmsim.ico"; Tasks: desktopicon

[Run]
Filename: "{app}\{#AppExe}"; Description: "Launch {#AppName}"; WorkingDir: "{app}"; Flags: nowait postinstall skipifsilent

[UninstallDelete]
; Files the app may create next to the exe if the user ran it with --portable.
Type: files; Name: "{app}\mmsim.json"
Type: files; Name: "{app}\imgui.ini"
Type: filesandordirs; Name: "{app}\runs"
Type: filesandordirs; Name: "{app}\reports"
Type: filesandordirs; Name: "{app}\screenshots"

[Code]
// The uninstaller never removes the user's own data silently: settings, saved robot configs,
// run recordings, reports and screenshots live in %LOCALAPPDATA%\MMSim. Ask once; a silent
// uninstall (/VERYSILENT) keeps the data.
procedure CurUninstallStepChanged(CurUninstallStep: TUninstallStep);
var
  DataDir: String;
begin
  if CurUninstallStep = usPostUninstall then
  begin
    DataDir := ExpandConstant('{localappdata}\MMSim');
    if DirExists(DataDir) and (not UninstallSilent) then
    begin
      if MsgBox('Also delete your Micromouse Studio settings, saved robot configs, runs, reports and screenshots?' + #13#10 + #13#10 + DataDir,
                mbConfirmation, MB_YESNO or MB_DEFBUTTON2) = IDYES then
        DelTree(DataDir, True, True, True);
    end;
  end;
end;
