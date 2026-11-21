<#
.SYNOPSIS
  Prove that a release zip runs on a machine without the build toolchain.

.DESCRIPTION
  Extracts the zip into a new temp folder outside the repository, then starts mmsim.exe and
  mmsim-cli.exe there as child processes whose PATH has the MSYS2/MinGW entries removed.
  The PATH change exists only in those child processes (the persistent user and machine PATH
  is never touched). Exit code 0 = all checks passed.
#>
[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Zip,
    [string]$Msys2,
    [switch]$SoftwareRender
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$zipFull = (Resolve-Path $Zip).Path
$tmp = Join-Path ([System.IO.Path]::GetTempPath()) ("mmsim-portable-" + [guid]::NewGuid().ToString('N').Substring(0, 8))
New-Item -ItemType Directory -Path $tmp | Out-Null
Add-Type -AssemblyName System.IO.Compression.FileSystem
[System.IO.Compression.ZipFile]::ExtractToDirectory($zipFull, $tmp)
$pkg = (Get-ChildItem $tmp -Directory | Select-Object -First 1).FullName
Write-Host "Extracted to $pkg"

# PATH without any MSYS2 / MinGW / gcc folder (child processes only).
$clean = ($env:PATH -split ';' | Where-Object {
        $_ -and ($_ -notmatch 'msys64|mingw|\\ucrt64|\\clang64|gcc')
    }) -join ';'
if ($Msys2) { $clean = ($clean -split ';' | Where-Object { $_ -notlike "$Msys2*" }) -join ';' }

$failures = New-Object System.Collections.Generic.List[string]
function Test-Check([string]$Name, [bool]$Ok, [string]$Detail = '') {
    if ($Ok) { Write-Host "  PASS  $Name $Detail" -ForegroundColor Green }
    else { Write-Host "  FAIL  $Name $Detail" -ForegroundColor Red; $script:failures.Add($Name) }
}

function Invoke-Clean([string]$Exe, [string[]]$Arguments, [int]$TimeoutSec = 180) {
    $psi = New-Object System.Diagnostics.ProcessStartInfo
    $psi.FileName = $Exe
    $psi.Arguments = ($Arguments | ForEach-Object { if ($_ -match '\s') { '"' + $_ + '"' } else { $_ } }) -join ' '
    $psi.WorkingDirectory = $pkg
    $psi.UseShellExecute = $false
    $psi.RedirectStandardOutput = $true
    $psi.RedirectStandardError = $true
    $psi.EnvironmentVariables['PATH'] = $clean
    $p = [System.Diagnostics.Process]::Start($psi)
    $so = $p.StandardOutput.ReadToEndAsync()
    $se = $p.StandardError.ReadToEndAsync()
    if (-not $p.WaitForExit($TimeoutSec * 1000)) { $p.Kill(); return @{ Code = -1; Out = 'timeout'; Err = '' } }
    return @{ Code = $p.ExitCode; Out = $so.Result; Err = $se.Result }
}

$cliPath = Join-Path $pkg 'mmsim-cli.exe'
$guiPath = Join-Path $pkg 'mmsim.exe'
Test-Check 'MSYS2 is not on the child PATH' (-not ($clean -match 'msys64')) ''

# 1. GUI: five scripted frames with a bundled maze and solver, hidden window, no session files.
$guiArgs = @('--frames', '5', '--exit', '--hidden', '--no-session',
    '--maze', (Join-Path $pkg 'mazes\test\gen16_loops_a.txt'), '--firmware', 'demo_floodfill')
if ($SoftwareRender) { $guiArgs += '--software' }
$r = Invoke-Clean $guiPath $guiArgs
Test-Check 'mmsim.exe --frames 5 --exit' ($r.Code -eq 0) "(exit $($r.Code))"

# 2. CLI: every bundled solver on a bundled maze.
foreach ($s in 'floodfill', 'dijkstra', 'astar', 'tremaux') {
    $r = Invoke-Clean $cliPath @('--firmware', $s, '--mazes', (Join-Path $pkg 'mazes\test\gen16_loops_b.txt'), '--quiet')
    Test-Check "mmsim-cli --firmware $s" ($r.Code -eq 0 -and $r.Out -match '1/1 solved') (($r.Out -split "`n" | Where-Object { $_ -match 'solved' } | Select-Object -First 1))
}
$r = Invoke-Clean $cliPath @('--firmware', 'wallfollower', '--mazes', (Join-Path $pkg 'mazes\test\gen16_perfect_b.txt'), '--quiet')
Test-Check 'mmsim-cli --firmware wallfollower (perfect maze)' ($r.Code -eq 0 -and $r.Out -match '1/1 solved')

# 3. Contents: no mazefiles content, no state files, no private paths in text files.
$files = Get-ChildItem $pkg -Recurse -File
Test-Check 'no external/ or runs/ folder' (-not (Get-ChildItem $pkg -Directory | Where-Object { $_.Name -in 'external', 'runs', 'reports' }))
Test-Check 'mmsim.portable marker present' (Test-Path (Join-Path $pkg 'mmsim.portable'))
Test-Check 'no mmsim.json / imgui.ini'(-not ($files | Where-Object { $_.Name -in 'mmsim.json', 'imgui.ini' }))
$textHits = $files | Where-Object { $_.Extension -in '.md', '.txt', '.bat', '.json' } |
    Select-String -Pattern 'C:\\Users\\|D:\\Projects|/Users/' -SimpleMatch:$false -ErrorAction SilentlyContinue
Test-Check 'no private paths in text files' (-not $textHits)

$size = [math]::Round((Get-Item $zipFull).Length / 1MB, 2)
Write-Host ("Zip: {0} MB, {1} files" -f $size, $files.Count)
$files | ForEach-Object { $_.FullName.Substring($pkg.Length + 1) } | Sort-Object | ForEach-Object { Write-Host "  $_" }

Remove-Item $tmp -Recurse -Force -ErrorAction SilentlyContinue
if ($failures.Count -gt 0) { Write-Host ("FAILED: " + ($failures -join ', ')) -ForegroundColor Red; exit 1 }
Write-Host 'Portable verification passed.' -ForegroundColor Green
exit 0
