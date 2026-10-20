# Explicit recovery helper: release only the connected D435 from USBIP sharing.
[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$resultPath=Join-Path $PSScriptRoot 'logs\usbip-release.json'
try {
    $usbipd='C:\Program Files\usbipd-win\usbipd.exe'
    $lines=& $usbipd list
    $targets=@($lines | Where-Object {$_ -match '^\s*(\S+)\s+8086:0b07\s+'})
    if($targets.Count -ne 1){throw 'Expected exactly one connected D435 (8086:0b07); no changes made.'}
    $bus=([regex]::Match($targets[0],'^\s*(\S+)')).Groups[1].Value
    $output=& $usbipd unbind --busid $bus 2>&1
    if($LASTEXITCODE -ne 0){throw ($output -join "`n")}
    @{ok=$true;hardware_id='8086:0b07';busid=$bus;action='unbind';time=[DateTimeOffset]::Now.ToString('o')} |
        ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding utf8
} catch {
    @{ok=$false;error=$_.Exception.Message;time=[DateTimeOffset]::Now.ToString('o')} |
        ConvertTo-Json | Set-Content -LiteralPath $resultPath -Encoding utf8
    exit 1
}
