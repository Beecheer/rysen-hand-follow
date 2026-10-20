[CmdletBinding()]
param([ValidateRange(5, 180)][int]$TimeoutSeconds = 45)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'Rysen.Common.ps1')

$sdkRaw=& wslc inspect rysen_sdk_readonly 2>$null
if ($LASTEXITCODE -eq 0) {
    $sdkContainer=@(($sdkRaw -join "`n") | ConvertFrom-Json)[0]
    if ($sdkContainer.State.Running) { throw 'Stop the SDK/camera-follow container before Explorer: hand-tracking\Stop-HandSdkMonitor.ps1 or Stop-HandIndexFollow.ps1.' }
}

$patchPath = Join-Path $PSScriptRoot 'configure-rysen-web.py'
if (-not (Test-Path -LiteralPath $patchPath)) { throw "Missing file: $patchPath" }
# Preflight both containers before changing either one.
foreach ($name in @('rysen_backend', 'rysen_explorer')) {
    $state = Get-RysenContainer $name
    Write-Host "$name : $($state.State.Status)"
}
$started = [System.Collections.Generic.List[string]]::new()
try {
    foreach ($name in @('rysen_backend', 'rysen_explorer')) {
        if (-not (Get-RysenContainer $name).State.Running) {
            & wslc start $name
            if ($LASTEXITCODE -ne 0) { throw "Cannot start $name" }
            $started.Add($name)
        }
    }
    Get-Content -Raw -LiteralPath $patchPath | & wslc exec -i rysen_explorer python3 -
    if ($LASTEXITCODE -ne 0) { throw 'Cannot configure frontend API URL' }
    $deadline = [DateTime]::UtcNow.AddSeconds($TimeoutSeconds)
    $ready = $false
    do {
        try {
            $page = Invoke-WebRequest 'http://127.0.0.1:8080/config/config.js' -UseBasicParsing -TimeoutSec 3
            if (-not $page.Content.Contains(': (window.location.origin)')) { throw 'Unexpected web config' }
            Test-RysenWebSocket
            $ready = $true
        } catch {
            $lastFailure = $_.Exception.Message
            if ([DateTime]::UtcNow -lt $deadline) { Start-Sleep -Milliseconds 500 }
        }
    } until ($ready -or [DateTime]::UtcNow -ge $deadline)
    if (-not $ready) { throw "Services did not become ready: $lastFailure" }
    Write-Host 'Ready: http://127.0.0.1:8080 (refresh with Ctrl+F5)'
    Write-Host 'Left hand: 192.168.0.102. Connect and enable power manually in the UI.'
} catch {
    # Roll back only containers started by this invocation.
    foreach ($name in @('rysen_explorer', 'rysen_backend')) {
        if ($started.Contains($name)) {
            & wslc stop --time 20 $name
            if ($LASTEXITCODE -ne 0) { Write-Warning "Rollback failed for $name; run Stop-Rysen.ps1." }
        }
    }
    throw
}
