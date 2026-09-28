# run-tests.ps1 -- run the joukowsky tests and keep a readable log.
#
#   .\tests\run-tests.ps1                          quick suite, everything
#   .\tests\run-tests.ps1 -Suite full              also the slow/exhaustive tests
#   .\tests\run-tests.ps1 -Only hydraulics         one bounded context
#   .\tests\run-tests.ps1 -Filter single_pipe      only tests whose path contains this
#   .\tests\run-tests.ps1 -Only hydraulics -Filter va1
#
# Logs (tests\logs\):
#   runs\<yyyy-MM>\<yyyy-MM-dd>\<HH-mm-ss>_<suite>[_<only>][_<filter>].log
#                       every run, grouped by month and day; kept $KeepDays
#                       days, then deleted (local only, not in git)
#   latest.log          copy of the most recent run
#   latest_quick.log    copy of the most recent quick run
#   latest_full.log     copy of the most recent full run
#   history.csv         one line per run, kept forever: when, commit, suite,
#                       passed, failed... - opens as a table in Excel
# Tests print what they measured next to what the physics predicts, so a
# log reads like a lab notebook.
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
    [ValidateSet('', 'kernel', 'hydraulics', 'detection', 'localisation', 'classification', 'pipeline')]
    [string]$Only = '',

    # Substring of the test path, e.g. single_pipe, invariants, va1.
    [string]$Filter = '',

    # Run logs older than this are deleted (history.csv keeps their summary).
    [int]$KeepDays = 30
)

$root = Split-Path $PSScriptRoot -Parent
$cargoBin = Join-Path $env:USERPROFILE '.cargo\bin'
if ($env:Path -notlike "*$cargoBin*") { $env:Path += ";$cargoBin" }
$env:CARGO_TERM_COLOR = 'never'   # no colour codes in the log file

$logDir = Join-Path $PSScriptRoot 'logs'
$runsDir = Join-Path $logDir 'runs'
$now = Get-Date
$dayDir = Join-Path (Join-Path $runsDir $now.ToString('yyyy-MM')) $now.ToString('yyyy-MM-dd')
New-Item -ItemType Directory -Force $dayDir | Out-Null
$name = $now.ToString('HH-mm-ss') + "_$Suite"
if ($Only) { $name += "_$Only" }
if ($Filter) { $name += '_' + ($Filter -replace '[^\w-]', '_') }
$logFile = Join-Path $dayDir "$name.log"

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
        "  when     $($now.ToString('yyyy-MM-dd HH:mm:ss'))"
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
Copy-Item -LiteralPath $logFile -Destination (Join-Path $logDir "latest_$Suite.log") -Force

# One summary line per run, kept forever. Plain CSV so Excel opens it.
$history = Join-Path $logDir 'history.csv'
if (-not (Test-Path -LiteralPath $history)) {
    [System.IO.File]::WriteAllText($history,
        "when,suite,only,filter,commit,uncommitted,passed,failed,skipped,seconds,result,log`r`n", $utf8)
}
$relLog = $logFile.Substring($logDir.Length + 1) -replace '\\', '/'
$row = @(
    $now.ToString('yyyy-MM-dd HH:mm:ss'), $Suite, $Only, ($Filter -replace ',', ';'),
    $commit, $(if ($dirty) { 'yes' } else { 'no' }),
    $passed, $failed, $ignored, ('{0:F1}' -f $seconds),
    $(if ($exit -eq 0) { 'pass' } else { 'FAIL' }), $relLog
) -join ','
[System.IO.File]::AppendAllText($history, "$row`r`n", $utf8)

# Retention: delete run logs older than $KeepDays, then empty day/month folders.
$cutoff = $now.AddDays(-$KeepDays)
Get-ChildItem -LiteralPath $runsDir -Recurse -File -Filter '*.log' -ErrorAction SilentlyContinue |
    Where-Object { $_.LastWriteTime -lt $cutoff } |
    Remove-Item -Force -ErrorAction SilentlyContinue
Get-ChildItem -LiteralPath $runsDir -Recurse -Directory -ErrorAction SilentlyContinue |
    Sort-Object { $_.FullName.Length } -Descending |
    Where-Object { -not (Get-ChildItem -LiteralPath $_.FullName -Force) } |
    Remove-Item -Force -ErrorAction SilentlyContinue

$footer | ForEach-Object { Write-Host $_ }
Write-Host "  log      $logFile"
exit $exit
