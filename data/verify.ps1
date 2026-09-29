# Re-checks every file listed in data/manifest.csv: present, same size,
# same SHA-256 fingerprint. Run from anywhere:  .\data\verify.ps1
# Exit code 0 = all good; 1 = a file is missing or has changed.

$ErrorActionPreference = 'Stop'
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$rows = @(Import-Csv (Join-Path $here 'manifest.csv'))

if ($rows.Count -eq 0) {
    Write-Output 'No datasets approved yet - manifest.csv is empty.'
    exit 0
}

$bad = 0
foreach ($r in $rows) {
    $path = Join-Path $here $r.file
    if (-not (Test-Path -LiteralPath $path)) {
        Write-Output ("MISSING   {0}  ({1})" -f $r.id, $r.file)
        $bad++
        continue
    }
    $item = Get-Item -LiteralPath $path
    $hash = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash
    if ($hash -ne $r.sha256.ToUpper() -or [string]$item.Length -ne $r.bytes) {
        Write-Output ("CHANGED   {0}  ({1}): expected {2} bytes / {3}, found {4} bytes / {5}" -f `
            $r.id, $r.file, $r.bytes, $r.sha256, $item.Length, $hash)
        $bad++
    } else {
        Write-Output ("OK        {0}  ({1}, {2} bytes, licence: {3})" -f $r.id, $r.file, $r.bytes, $r.licence)
    }
}

if ($bad -gt 0) {
    Write-Output "$bad file(s) missing or changed - do not use them until checked."
    exit 1
}
Write-Output 'All datasets present and unchanged.'
exit 0
