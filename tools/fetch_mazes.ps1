# Fetch micromouseonline/mazefiles into external/mazefiles (gitignored).
# The repo has no licence, so its files are never committed here.
# Pinned to the commit studied in docs/reference/mms-analysis.md.
param(
    [string]$Commit = "070f716d23799573021c142c1cf4e618d023f01e"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$dest = Join-Path $root "external\mazefiles"

if (-not (Test-Path (Join-Path $dest ".git"))) {
    New-Item -ItemType Directory -Force $dest | Out-Null
    git -C $dest init -q
    git -C $dest remote add origin https://github.com/micromouseonline/mazefiles.git
}
git -C $dest fetch -q --depth 1 origin $Commit
git -C $dest -c advice.detachedHead=false checkout -q --force FETCH_HEAD
$head = git -C $dest rev-parse HEAD
if ($head -ne $Commit) { throw "mazefiles at $head, expected $Commit" }

$classic = (Get-ChildItem (Join-Path $dest "classic") -Filter *.txt).Count
$half = (Get-ChildItem (Join-Path $dest "halfsize") -Filter *.txt).Count
Write-Host "mazefiles @ $($head.Substring(0,12)): $classic classic, $half halfsize -> $dest"
