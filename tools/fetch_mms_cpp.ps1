# Fetch mackorone/mms-cpp (the upstream C++ mms example solver, MIT) into external/mms-cpp
# (gitignored). tests/CMakeLists.txt builds it unchanged and runs it against the simulator in
# the "mms" test suite. Pinned to the commit studied in docs/reference/mms-analysis.md.
param(
    [string]$Commit = "a67bb5aa060f634f1650e8efe6ae896dc69aecbe"
)
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$dest = Join-Path $root "external\mms-cpp"

if (-not (Test-Path (Join-Path $dest ".git"))) {
    New-Item -ItemType Directory -Force $dest | Out-Null
    git -C $dest init -q
    git -C $dest remote add origin https://github.com/mackorone/mms-cpp.git
}
git -C $dest fetch -q --depth 1 origin $Commit
git -C $dest -c advice.detachedHead=false checkout -q --force FETCH_HEAD
$head = git -C $dest rev-parse HEAD
if ($head -ne $Commit) { throw "mms-cpp at $head, expected $Commit" }
foreach ($f in "Main.cpp", "API.cpp", "API.h") {
    if (-not (Test-Path (Join-Path $dest $f))) { throw "mms-cpp is missing $f" }
}
Write-Host "mms-cpp @ $($head.Substring(0,12)) -> $dest (re-run cmake to build the example)"
