<#
.SYNOPSIS
  Smoke-test dist/MMSim-v<version>-Setup.exe: silent install into a temp folder, launch with the
  toolchain removed from PATH, check the per-user data folder, silent uninstall.

.DESCRIPTION
  Nothing outside a new temp folder is touched: the app runs with LOCALAPPDATA redirected into
  that folder (child process only), the installer is told /DIR=<temp>, /CURRENTUSER and no
  optional tasks (no shortcuts), and the uninstaller removes the HKCU uninstall entry again.
  Exit code 0 = all checks passed.
#>
[CmdletBinding()]
param(
    [string]$Setup,
    [string]$Version
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest
$root = Split-Path -Parent $PSScriptRoot
if (-not $Version) {
    $m = Select-String -Path (Join-Path $root 'CMakeLists.txt') -Pattern 'project\s*\([^)]*VERSION\s+([0-9][0-9.]*)' | Select-Object -First 1
    $Version = $m.Matches[0].Groups[1].Value
}
if (-not $Setup) { $Setup = Join-Path $root "dist\MMSim-v$Version-Setup.exe" }
$Setup = (Resolve-Path $Setup).Path

$tmp = Join-Path ([System.IO.Path]::GetTempPath()) ("mmsim-setup-test-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
$inst = Join-Path $tmp 'install'
$fakeLocal = Join-Path $tmp 'localappdata'
New-Item -ItemType Directory -Path $tmp, $fakeLocal | Out-Null
$log = Join-Path $tmp 'setup.log'

$failures = New-Object System.Collections.Generic.List[string]
function Test-Check([string]$Name, [bool]$Ok, [string]$Detail = '') {
    if ($Ok) { Write-Host "  PASS  $Name $Detail" -ForegroundColor Green }
    else { Write-Host "  FAIL  $Name $Detail" -ForegroundColor Red; $script:failures.Add($Name) }
}

$uninstKey = 'HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\{6F0B7C1E-3D52-4B7A-9A64-5C0D1E7A2B91}_is1'
Test-Check 'no previous MMSim install registered' (-not (Test-Path $uninstKey))

# --- install ---------------------------------------------------------------------------------
$p = Start-Process -FilePath $Setup -Wait -PassThru -ArgumentList @(
    '/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART', '/CURRENTUSER', "/DIR=`"$inst`"", "/LOG=`"$log`"", '/TASKS=""')
Test-Check 'silent install exit code 0' ($p.ExitCode -eq 0) "(exit $($p.ExitCode))"
foreach ($f in 'mmsim.exe', 'mmsim-cli.exe', 'SDL2.dll', 'mmsim.ico', 'mmsim.installed', 'LICENSE',
    'THIRD_PARTY_NOTICES.md', 'README.md', 'config\robots\generic_classic.json', 'unins000.exe',
    'firmware_bin\demo_floodfill.dll', 'firmware_bin\demo_dijkstra.dll', 'firmware_bin\demo_astar.dll',
    'firmware_bin\demo_tremaux.dll', 'firmware_bin\demo_wallfollower.dll', 'mazes\test\gen16_loops_a.txt',
    'fonts\Roboto-Medium.ttf', 'licenses\Apache-2.0.txt') {
    Test-Check "installed: $f" (Test-Path (Join-Path $inst $f))
}
Test-Check 'portable marker NOT installed' (-not (Test-Path (Join-Path $inst 'mmsim.portable')))
Test-Check 'Launch_MMSim.bat NOT installed' (-not (Test-Path (Join-Path $inst 'Launch_MMSim.bat')))
Test-Check 'uninstall entry registered (Apps & Features)' (Test-Path $uninstKey)
if (Test-Path $uninstKey) {
    $k = Get-ItemProperty $uninstKey
    Test-Check 'DisplayName / DisplayVersion' (($k.DisplayName -like 'Micromouse Studio*') -and ($k.DisplayVersion -eq $Version)) "($($k.DisplayName) $($k.DisplayVersion))"
}

# --- launch with a clean PATH and a redirected LOCALAPPDATA ------------------------------------
$clean = ($env:PATH -split ';' | Where-Object { $_ -and ($_ -notmatch 'msys64|mingw|\\ucrt64|\\clang64|gcc') }) -join ';'
function Invoke-Installed([string]$Exe, [string[]]$Arguments) {
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = (Join-Path $inst $Exe)
    $psi.Arguments = ($Arguments | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } }) -join ' '
    $psi.WorkingDirectory = $env:TEMP   # not the install folder
    $psi.UseShellExecute = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.EnvironmentVariables['PATH'] = $clean
    $psi.EnvironmentVariables['LOCALAPPDATA'] = $fakeLocal
    $proc = [System.Diagnostics.Process]::Start($psi)
    $so = $proc.StandardOutput.ReadToEndAsync(); $se = $proc.StandardError.ReadToEndAsync()
    if (-not $proc.WaitForExit(180000)) { $proc.Kill(); return @{ Code = -1; Out = 'timeout' } }
    return @{ Code = $proc.ExitCode; Out = $so.Result + $se.Result }
}
$r = Invoke-Installed 'mmsim.exe' @('--frames', '6', '--exit', '--hidden', '--software',
    '--maze', (Join-Path $inst 'mazes\test\gen16_loops_a.txt'), '--firmware', 'demo_floodfill')
Test-Check 'installed mmsim.exe starts (clean PATH, no MSYS2)' ($r.Code -eq 0) "(exit $($r.Code))"
$r = Invoke-Installed 'mmsim-cli.exe' @('--firmware', 'floodfill', '--mazes', (Join-Path $inst 'mazes\test\gen16_loops_b.txt'), '--quiet')
Test-Check 'installed mmsim-cli.exe solves a maze' ($r.Code -eq 0 -and $r.Out -match '1/1 solved')

# Interactive run (scripted --frames runs never write session files): open the window for a few
# seconds, close it normally, and expect the session files in the per-user folder.
$psi = New-Object System.Diagnostics.ProcessStartInfo
$psi.FileName = (Join-Path $inst 'mmsim.exe')
$psi.Arguments = '--software --maze "' + (Join-Path $inst 'mazes\test\gen16_loops_a.txt') + '" --firmware demo_floodfill'
$psi.WorkingDirectory = $env:TEMP
$psi.UseShellExecute = $false
$psi.EnvironmentVariables['PATH'] = $clean
$psi.EnvironmentVariables['LOCALAPPDATA'] = $fakeLocal
$gui = [System.Diagnostics.Process]::Start($psi)
Start-Sleep -Seconds 6
$closed = $false
if (-not $gui.HasExited) {
    [void]$gui.CloseMainWindow()
    $closed = $gui.WaitForExit(20000)
    if (-not $closed) { $gui.Kill() }
}
Test-Check 'interactive window closes normally' $closed
$userDir = Join-Path $fakeLocal 'MMSim'
Test-Check 'per-user folder %LOCALAPPDATA%\MMSim created' (Test-Path $userDir) "($userDir)"
Test-Check 'session file written to the user folder' ((Test-Path (Join-Path $userDir 'mmsim.json')) -and (Test-Path (Join-Path $userDir 'imgui.ini')))
Test-Check 'install folder stayed clean (no mmsim.json / imgui.ini / runs)' (-not ((Test-Path (Join-Path $inst 'mmsim.json')) -or (Test-Path (Join-Path $inst 'imgui.ini')) -or (Test-Path (Join-Path $inst 'runs'))))

# --- uninstall ---------------------------------------------------------------------------------
$unins = Join-Path $inst 'unins000.exe'
$p = Start-Process -FilePath $unins -Wait -PassThru -ArgumentList @('/VERYSILENT', '/SUPPRESSMSGBOXES', '/NORESTART')
Test-Check 'silent uninstall exit code 0' ($p.ExitCode -eq 0) "(exit $($p.ExitCode))"
# Inno finishes by spawning a helper that deletes unins000.exe; give it a moment.
for ($i = 0; $i -lt 30 -and (Test-Path (Join-Path $inst 'mmsim.exe')); $i++) { Start-Sleep -Milliseconds 300 }
Test-Check 'mmsim.exe removed' (-not (Test-Path (Join-Path $inst 'mmsim.exe')))
Test-Check 'uninstall entry removed' (-not (Test-Path $uninstKey))
Test-Check 'silent uninstall keeps the user data folder' (Test-Path $userDir)

Remove-Item $tmp -Recurse -Force -ErrorAction SilentlyContinue
if ($failures.Count -gt 0) { Write-Host ("FAILED: " + ($failures -join ', ')) -ForegroundColor Red; exit 1 }
Write-Host 'Installer smoke test passed.' -ForegroundColor Green
exit 0
