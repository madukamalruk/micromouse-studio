<#
.SYNOPSIS
  Build the release preset, run the tests and pack a portable Windows zip.

.DESCRIPTION
  Produces dist/mmsim-v<version>-win64.zip with mmsim.exe, mmsim-cli.exe, SDL2.dll, fonts/,
  firmware_bin/demo_*.dll, config/robots/, mazes/test/, Launch_MMSim.bat, mmsim.portable,
  README.md, LICENSE, THIRD_PARTY_NOTICES.md and licenses/. The staged folder (dist/mmsim-v<version>-win64/)
  is also the input of tools/build_installer.ps1. Then (unless -SkipVerify) extracts the zip into a
  fresh temp folder OUTSIDE the repository and runs mmsim.exe and mmsim-cli.exe there with the
  MSYS2 toolchain removed from PATH *for those child processes only* (nothing persistent is
  changed). The mazefiles collection is never bundled: it is fetched on demand with
  tools/fetch_mazes.ps1.

.PARAMETER Version   Version string; default: read from the top-level CMakeLists.txt project().
.PARAMETER Msys2     MSYS2 root (default: $env:MSYS2_ROOT, else D:\msys64, else C:\msys64).
.PARAMETER SkipTests Skip `ctest --preset release` (CI runs it as a separate step).
.PARAMETER SkipBuild Package what is already built in build/release.
.PARAMETER SkipVerify Skip the clean-PATH extraction test.
.PARAMETER SoftwareRender Use the SDL software renderer in the GUI smoke test (CI runners have no GPU).
#>
[CmdletBinding()]
param(
    [string]$Version,
    [string]$Msys2,
    [switch]$SkipTests,
    [switch]$SkipBuild,
    [switch]$SkipVerify,
    [switch]$SoftwareRender
)
$ErrorActionPreference = 'Stop'
Set-StrictMode -Version Latest

$root = Split-Path -Parent $PSScriptRoot
Set-Location $root

if (-not $Version) {
    $m = Select-String -Path (Join-Path $root 'CMakeLists.txt') -Pattern 'project\s*\([^)]*VERSION\s+([0-9][0-9.]*)' | Select-Object -First 1
    if (-not $m) { throw 'Could not read the project version from CMakeLists.txt; pass -Version.' }
    $Version = $m.Matches[0].Groups[1].Value
}
$Version = $Version.TrimStart('v')
$pkgName = "mmsim-v$Version-win64"

# --- toolchain (this process only) ---------------------------------------------------------
if (-not $Msys2) {
    foreach ($c in @($env:MSYS2_ROOT, 'D:\msys64', 'C:\msys64')) {
        if ($c -and (Test-Path (Join-Path $c 'ucrt64\bin\g++.exe'))) { $Msys2 = $c; break }
    }
}
$toolBin = if ($Msys2) { Join-Path $Msys2 'ucrt64\bin' } else { $null }
if ($toolBin -and (Test-Path $toolBin) -and ($env:PATH -notlike "*$toolBin*")) {
    $env:PATH = "$toolBin;$env:PATH"   # process scope only
}

function Invoke-Step([string]$Title, [scriptblock]$Block) {
    Write-Host "==> $Title" -ForegroundColor Cyan
    & $Block
    if ($LASTEXITCODE -ne 0) { throw "$Title failed (exit $LASTEXITCODE)" }
}

if (-not $SkipBuild) {
    Invoke-Step 'Configure (release)' { cmake --preset release }
    Invoke-Step 'Build (release)'     { cmake --build --preset release }
}
if (-not $SkipTests) {
    Invoke-Step 'Test (release)'      { ctest --preset release --output-on-failure -j 4 }
}

# --- stage -----------------------------------------------------------------------------------
$build = Join-Path $root 'build\release'
$dist  = Join-Path $root 'dist'
$stage = Join-Path $dist $pkgName
if (Test-Path $stage) { Remove-Item $stage -Recurse -Force }
New-Item -ItemType Directory -Force -Path $stage | Out-Null

function Copy-Required([string]$From, [string]$ToDir) {
    if (-not (Test-Path $From)) { throw "Missing build output: $From" }
    New-Item -ItemType Directory -Force -Path $ToDir | Out-Null
    Copy-Item $From $ToDir -Force
}
Copy-Required (Join-Path $build 'sim\app\mmsim.exe')     $stage
Copy-Required (Join-Path $build 'sim\cli\mmsim-cli.exe') $stage
Copy-Required (Join-Path $build 'sim\app\SDL2.dll')      $stage
$fontSrc = Join-Path $build 'sim\app\fonts'
if (-not (Test-Path $fontSrc)) { throw "Missing fonts folder: $fontSrc" }
Copy-Item $fontSrc (Join-Path $stage 'fonts') -Recurse -Force

$solvers = 'demo_floodfill', 'demo_dijkstra', 'demo_astar', 'demo_tremaux', 'demo_wallfollower'
foreach ($s in $solvers) { Copy-Required (Join-Path $build "firmware_bin\$s.dll") (Join-Path $stage 'firmware_bin') }

Copy-Required (Join-Path $root 'config\robots\generic_classic.json') (Join-Path $stage 'config\robots')
Get-ChildItem (Join-Path $root 'mazes\test') -Filter *.txt | ForEach-Object {
    Copy-Required $_.FullName (Join-Path $stage 'mazes\test')
}
Copy-Required (Join-Path $root 'tools\package\Launch_MMSim.bat') $stage
Copy-Item (Join-Path $root 'tools\package\README_PORTABLE.md') (Join-Path $stage 'README.md') -Force
# Marker: "keep everything next to the exe" (settings, runs, reports). The installer drops this
# file and ships mmsim.installed instead (per-user data in %LOCALAPPDATA%\MMSim).
Copy-Required (Join-Path $root 'tools\package\mmsim.portable') $stage
Copy-Required (Join-Path $root 'LICENSE')                $stage
Copy-Required (Join-Path $root 'THIRD_PARTY_NOTICES.md') $stage
Copy-Item (Join-Path $root 'licenses') (Join-Path $stage 'licenses') -Recurse -Force

# Never ship mazefiles content or runtime state.
foreach ($bad in 'external', 'runs', 'reports', 'screenshots', 'mmsim.json', 'imgui.ini') {
    if (Test-Path (Join-Path $stage $bad)) { throw "Refusing to package '$bad'" }
}

# --- zip (forward-slash entry names, fixed order) ----------------------------------------
Add-Type -AssemblyName System.IO.Compression
Add-Type -AssemblyName System.IO.Compression.FileSystem
$zipPath = Join-Path $dist "$pkgName.zip"
if (Test-Path $zipPath) { Remove-Item $zipPath -Force }
$zip = [System.IO.Compression.ZipFile]::Open($zipPath, [System.IO.Compression.ZipArchiveMode]::Create)
try {
    $stageFull = (Resolve-Path $stage).Path.TrimEnd('\')
    Get-ChildItem $stage -Recurse -File | Sort-Object FullName | ForEach-Object {
        $rel = $_.FullName.Substring($stageFull.Length + 1).Replace('\', '/')
        [void][System.IO.Compression.ZipFileExtensions]::CreateEntryFromFile(
            $zip, $_.FullName, "$pkgName/$rel", [System.IO.Compression.CompressionLevel]::Optimal)
    }
} finally { $zip.Dispose() }

$sizeMb = [math]::Round((Get-Item $zipPath).Length / 1MB, 2)
$sha = (Get-FileHash $zipPath -Algorithm SHA256).Hash.ToLower()
Set-Content -Path (Join-Path $dist "$pkgName.zip.sha256") -Value "$sha  $pkgName.zip" -Encoding ascii
Write-Host "==> Packed $zipPath ($sizeMb MB)" -ForegroundColor Green
Write-Host "    sha256 $sha"

# --- portability proof ---------------------------------------------------------------------
if (-not $SkipVerify) {
    & (Join-Path $PSScriptRoot 'verify_portable.ps1') -Zip $zipPath -Msys2 $Msys2 -SoftwareRender:$SoftwareRender
    if ($LASTEXITCODE -ne 0) { throw 'Portable verification failed' }
}
Write-Host "Done. Upload $zipPath to a GitHub Release (tag v$Version) after review." -ForegroundColor Green
