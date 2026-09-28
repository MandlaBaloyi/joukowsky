# run-tests.ps1 -- run the joukowsky tests and keep a readable log.
#
#   .\tests\run-tests.ps1                          quick suite, everything
#   .\tests\run-tests.ps1 -Suite full              also the slow/exhaustive tests
#   .\tests\run-tests.ps1 -Only hydraulics         one bounded context
#   .\tests\run-tests.ps1 -Filter single_pipe      only tests whose path contains this
#   .\tests\run-tests.ps1 -Only hydraulics -Filter va1
#
# Each run writes tests\logs\test-run_<date>_<time>_<suite>.log and copies it
# to tests\logs\latest.log. Tests print what they measured next to what the
# physics predicts, so the log reads like a lab notebook.
#
# QUICK runs in seconds and is for every change. FULL adds tests marked
# #[ignore] in the code (exhaustive sweeps, long simulations, datasets) -
# run it before a commit that touches physics, and later nightly in CI.
#
# Tests run one at a time (--test-threads=1) so printed output stays in
# order. That is gentle on the laptop too.

[CmdletBinding()]
param(
    [ValidateSet('quick', 'full')]
    [string]$Suite = 'quick',

    # A bounded context = one test program in tests\Cargo.toml.
    [ValidateSet('', 'kernel', 'hydraulics')]
    [string]$Only = '',

    # Substring of the test path, e.g. single_pipe, invariants, va1.
    [string]$Filter = ''
)

$root = Split-Path $PSScriptRoot -Parent
$cargoBin = Join-Path $env:USERPROFILE '.cargo\bin'
if ($env:Path -notlike "*$cargoBin*") { $env:Path += ";$cargoBin" }
$env:CARGO_TERM_COLOR = 'never'   # no colour codes in the log file

$logDir = Join-Path $PSScriptRoot 'logs'
New-Item -ItemType Directory -Force $logDir | Out-Null
$stamp = Get-Date -Format 'yyyy-MM-dd_HH-mm-ss'
$logFile = Join-Path $logDir "test-run_${stamp}_$Suite.log"

$target = if ($Only) { "-p jk-tests --test $Only" } else { '--workspace' }
$harness = '--nocapture --test-threads=1'
if ($Suite -eq 'full') { $harness += ' --include-ignored' }
if ($Filter) { $harness += " $Filter" }
$cargoArgs = "test $target --no-fail-fast -- $harness"

Push-Location $root
try {
    $commit = (git rev-parse --short HEAD 2>$null)
    $dirty = if (git status --porcelain 2>$null) { ' (with uncommitted changes)' } else { '' }
    $rustc = (rustc --version 2>$null)
    $header = @(
        'joukowsky test run'
        "  when     $(Get-Date -Format 'yyyy-MM-dd HH:mm:ss')"
        "  suite    $Suite$(if ($Only) { ", only $Only" })$(if ($Filter) { ", filter '$Filter'" })"
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
$passed = 0; $failed = 0; $ignored = 0; $filtered = 0
foreach ($line in $output) {
    if ($line -match 'test result: \w+\. (\d+) passed; (\d+) failed; (\d+) ignored; \d+ measured; (\d+) filtered out') {
        $passed += [int]$Matches[1]; $failed += [int]$Matches[2]
        $ignored += [int]$Matches[3]; $filtered += [int]$Matches[4]
    }
}
$verdict = if ($exit -eq 0) { 'ALL PASSED' } else { 'FAILURES - search this log for "FAILED" or "panicked"' }
$footer = @(
    ('=' * 78)
    "  passed   $passed"
    "  failed   $failed"
    "  skipped  $ignored slow test(s) not in this suite$(if ($Suite -eq 'quick' -and $ignored) { ' (run -Suite full)' })"
    "  filtered $filtered test(s) outside -Only/-Filter"
    ("  time     {0:N1} s (including compile)" -f $seconds)
    "  result   $verdict"
)

$utf8 = New-Object System.Text.UTF8Encoding($false)
[System.IO.File]::WriteAllLines($logFile, [string[]]($header + $output + $footer), $utf8)
Copy-Item -LiteralPath $logFile -Destination (Join-Path $logDir 'latest.log') -Force

$footer | ForEach-Object { Write-Host $_ }
Write-Host "  log      $logFile"
exit $exit
