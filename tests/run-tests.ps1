# run-tests.ps1 -- run the joukowsky tests and keep a readable log.
#
#   .\tests\run-tests.ps1                 every test in the workspace
#   .\tests\run-tests.ps1 -Only single_pipe   one folder: units | joukowsky | single_pipe
#
# Each run writes tests\logs\test-run_<date>_<time>.log and copies it to
# tests\logs\latest.log. Tests print what they measured next to what the
# physics predicts, so the log reads like a lab notebook.
#
# Tests run one at a time (--test-threads=1) so their printed output stays
# in order. They take well under a second; the time is in compiling.

[CmdletBinding()]
param(
    [ValidateSet('', 'units', 'joukowsky', 'single_pipe')]
    [string]$Only = ''
)

$root = Split-Path $PSScriptRoot -Parent
$cargoBin = Join-Path $env:USERPROFILE '.cargo\bin'
if ($env:Path -notlike "*$cargoBin*") { $env:Path += ";$cargoBin" }
$env:CARGO_TERM_COLOR = 'never'   # no colour codes in the log file

$logDir = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Force $logDir | Out-Null
$stamp = Get-Date -Format 'yyyy-MM-dd_HH-mm-ss'
$logFile = Join-Path $logDir "test-run_$stamp.log"

if ($Only) {
    $cargoArgs = "test -p jk-tests --test $Only --no-fail-fast -- --nocapture --test-threads=1"
} else {
    $cargoArgs = "test --workspace --no-fail-fast -- --nocapture --test-threads=1"
}

Push-Location $root
try {
    $commit = (git rev-parse --short HEAD 2>$null)
    $dirty = if (git status --porcelain 2>$null) { ' (with uncommitted changes)' } else { '' }
    $rustc = (rustc --version 2>$null)
    $header = @(
        'joukowsky test run'
        "  when     $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
        "  commit   $commit$dirty"
        "  rust     $rustc"
        "  command  cargo $cargoArgs"
        ('=' * 78)
    )
    Write-Host "Running: cargo $cargoArgs"
    $started = Get-Date
    # cmd /c keeps cargo's stderr as plain text (PowerShell 5.1 would wrap it).
    $output = cmd /c "cargo $cargoArgs 2>&1"
    $exit = $LASTEXITCODE
    $seconds = ((Get-Date) - $started).TotalSeconds
} finally {
    Pop-Location
}

# Totals from every "test result:" line.
$passed = 0; $failed = 0
foreach ($line in $output) {
    if ($line -match 'test result: \w+\. (\d+) passed; (\d+) failed') {
        $passed += [int]$Matches[1]; $failed += [int]$Matches[2]
    }
}
$verdict = if ($exit -eq 0) { 'ALL PASSED' } else { 'FAILURES - search this log for "FAILED" or "panicked"' }
$footer = @(
    ('=' * 78)
    "  passed   $passed"
    "  failed   $failed"
    ("  time     {0:N1} s (including compile)" -f $seconds)
    "  result   $verdict"
)

$utf8 = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllLines($logFile, [string[]]($header + $output + $footer), $utf8)
Copy-Item -LiteralPath $logFile -Destination (Join-Path $logDir 'latest.log') -Force

$footer | ForEach-Object { Write-Host $_ }
Write-Host "  log      $logFile"
exit $exit
