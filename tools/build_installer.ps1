<#
.SYNOPSIS
  Build dist/MMSim-v<version>-Setup.exe with Inno Setup 6 (+ a .sha256 file).

.DESCRIPTION
  Uses the staged folder dist/mmsim-v<version>-win64/ produced by tools/package_release.ps1 (it
  runs that script first when the folder is missing or -Repackage is given), then compiles
  installer/mmsim.iss. ISCC.exe is looked up in $env:INNO_SETUP_DIR, D:\Tools\InnoSetup6 and the
  usual Program Files folders; nothing is installed by this script.

.PARAMETER Version     Default: read from the top-level CMakeLists.txt project().
.PARAMETER Iscc        Full path of ISCC.exe (overrides the search).
.PARAMETER Repackage   Rebuild + re-stage even if the stage folder exists.
.PARAMETER SoftwareRender  Passed through to package_release.ps1 (CI has no GPU).
#>
[CmdletBinding()]
param(
    [string]$Version,
    [string]$Iscc,
    [switch]$Repackage,
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

if (-not $Iscc) {
    $candidates = @()
    if ($env:INNO_SETUP_DIR) { $candidates += (Join-Path $env:INNO_SETUP_DIR 'ISCC.exe') }
    $candidates += 'D:\Tools\InnoSetup6\ISCC.exe'
    $candidates += (Join-Path ${env:ProgramFiles(x86)} 'Inno Setup 6\ISCC.exe')
    $candidates += (Join-Path $env:ProgramFiles 'Inno Setup 6\ISCC.exe')
    $candidates += (Join-Path $env:LOCALAPPDATA 'Programs\Inno Setup 6\ISCC.exe')
    $Iscc = $candidates | Where-Object { $_ -and (Test-Path $_) } | Select-Object -First 1
}
if (-not $Iscc -or -not (Test-Path $Iscc)) {
    throw 'ISCC.exe (Inno Setup 6) not found. Install it (winget install --id JRSoftware.InnoSetup) or pass -Iscc <path>.'
}
Write-Host "Inno Setup: $Iscc"

$stage = Join-Path $root "dist\mmsim-v$Version-win64"
if ($Repackage -or -not (Test-Path (Join-Path $stage 'mmsim.exe'))) {
    Write-Host '==> Staging with package_release.ps1' -ForegroundColor Cyan
    & (Join-Path $PSScriptRoot 'package_release.ps1') -Version $Version -SkipVerify -SoftwareRender:$SoftwareRender
    if ($LASTEXITCODE -ne 0) { throw 'package_release.ps1 failed' }
}
if (-not (Test-Path (Join-Path $stage 'mmsim.exe'))) { throw "Stage folder is missing mmsim.exe: $stage" }

$outName = "MMSim-v$Version-Setup"
$setup = Join-Path $root "dist\$outName.exe"
if (Test-Path $setup) { Remove-Item $setup -Force }

Write-Host '==> Compiling installer/mmsim.iss' -ForegroundColor Cyan
& $Iscc "/DAppVersion=$Version" "/DStageDir=$stage" "/DRepoRoot=$root" "/DOutName=$outName" (Join-Path $root 'installer\mmsim.iss')
if ($LASTEXITCODE -ne 0) { throw "ISCC failed (exit $LASTEXITCODE)" }
if (-not (Test-Path $setup)) { throw "Expected output not found: $setup" }

$sha = (Get-FileHash $setup -Algorithm SHA256).Hash.ToLower()
Set-Content -Path "$setup.sha256" -Value "$sha  $outName.exe" -Encoding ascii
$sizeMb = [math]::Round((Get-Item $setup).Length / 1MB, 2)
Write-Host "==> Built $setup ($sizeMb MB)" -ForegroundColor Green
Write-Host "    sha256 $sha"
