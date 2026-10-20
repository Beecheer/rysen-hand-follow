[CmdletBinding()]
param([switch]$NoBrowser)
$ErrorActionPreference='Stop'
$containerName='rysen_sdk_readonly'
foreach ($otherName in @('rysen_backend','rysen_explorer')) {
    $otherRaw=& wslc inspect $otherName
    if ($LASTEXITCODE -ne 0) { throw "Cannot inspect $otherName" }
    $other=@(($otherRaw -join "`n") | ConvertFrom-Json)[0]
    if ($other.State.Running) { throw "Stop $otherName first; avoid competing device controllers." }
}
$containerRaw=& wslc inspect $containerName 2>$null
if ($LASTEXITCODE -ne 0) {
    & wslc create --pull never --name $containerName --label codex.purpose=hand-follow-readonly --publish 127.0.0.1:8092:8092 --entrypoint /bin/sleep rysen_system:latest infinity
    if ($LASTEXITCODE -ne 0) { throw 'Cannot create read-only SDK container from existing image.' }
    $containerRaw=& wslc inspect $containerName
}
$container=@(($containerRaw -join "`n") | ConvertFrom-Json)[0]
$portBinding=@($container.Ports.'8092/tcp')
if ($container.Labels.'codex.purpose' -ne 'hand-follow-readonly' -or $container.Config.Image -ne 'rysen_system:latest' -or
    $container.Config.Entrypoint[0] -ne '/bin/sleep' -or $portBinding.Count -ne 1 -or
    $portBinding[0].HostIp -ne '127.0.0.1' -or $portBinding[0].HostPort -ne '8092') {
    throw 'Existing SDK container configuration does not match; no changes made.'
}
if (-not $container.State.Running) {
    & wslc start $containerName
    if ($LASTEXITCODE -ne 0) { throw 'Cannot start read-only SDK container.' }
}
# Stop the previously deployed bridge before updating its source files.
& wslc exec $containerName python3 -c 'from pathlib import Path; import subprocess; p=Path("/opt/hand-follow/sdk_follow_control.py"); raise SystemExit(subprocess.call(["python3",str(p),"stop"]) if p.exists() else 0)'
if ($LASTEXITCODE -ne 0) { throw 'Cannot stop camera-follow worker.' }
& wslc exec $containerName python3 -c 'from pathlib import Path; import subprocess; p=Path("/opt/hand-follow/sdk_bridge_control.py"); raise SystemExit(subprocess.call(["python3",str(p),"stop"]) if p.exists() else 0)'
if ($LASTEXITCODE -ne 0) { throw 'Cannot stop old read-only bridge.' }
$bridgeSources=@{}
foreach ($sourceName in @('sdk_bridge.py','sdk_bridge_control.py','joint_layout.py')) {
    $bridgeSources[$sourceName]=[IO.File]::ReadAllText((Join-Path $PSScriptRoot "hand_preview\$sourceName"))
}
$bridgeSources | ConvertTo-Json -Compress | & wslc exec -i $containerName python3 -c 'import json,sys,pathlib; root=pathlib.Path("/opt/hand-follow"); root.mkdir(exist_ok=True); files=json.load(sys.stdin); [(root/name).write_text(data,encoding="utf-8") for name,data in files.items()]'
if ($LASTEXITCODE -ne 0) { throw 'Cannot deploy read-only bridge.' }
& wslc exec $containerName python3 /opt/hand-follow/sdk_bridge_control.py start
if ($LASTEXITCODE -ne 0) { throw 'Cannot launch read-only bridge.' }
$deadline=[DateTime]::UtcNow.AddSeconds(25);$ready=$false
do {
    try {
        $sdkStatus=Invoke-RestMethod http://127.0.0.1:8092/status -TimeoutSec 2
        $ready=$sdkStatus.mode -eq 'READ_ONLY' -and $sdkStatus.connected -and $sdkStatus.hand -eq 'LEFT' -and
            $sdkStatus.packet_id -gt 0 -and $null -ne $sdkStatus.feedback_age_ms -and $sdkStatus.feedback_age_ms -lt 250
    } catch { $ready=$false }
    if (-not $ready) { Start-Sleep -Milliseconds 300 }
} until ($ready -or [DateTime]::UtcNow -ge $deadline)
if (-not $ready) {
    & wslc exec $containerName cat /tmp/hand-sdk.log
    & wslc exec $containerName python3 /opt/hand-follow/sdk_bridge_control.py stop
    throw 'No fresh left-hand feedback; check device power, cable and IP.'
}
& (Join-Path $PSScriptRoot 'Stop-HandMapping.ps1')
$previewArguments=@('-NoProfile','-File',(Join-Path $PSScriptRoot 'Start-HandMapping.ps1'),'-SdkMonitor')
if ($NoBrowser) { $previewArguments+='-NoBrowser' }
$null=Start-Process pwsh -ArgumentList $previewArguments -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $PSScriptRoot 'logs\mapping.stdout.log') -RedirectStandardError (Join-Path $PSScriptRoot 'logs\mapping.stderr.log')
$deadline=[DateTime]::UtcNow.AddSeconds(25);$ready=$false
do {
    try {
        $previewStatus=Invoke-RestMethod http://127.0.0.1:8090/api/status -TimeoutSec 2
        $ready=$previewStatus.hardware.mode -eq 'DRY_RUN' -and $previewStatus.hardware.connected
    } catch { $ready=$false }
    if (-not $ready) { Start-Sleep -Milliseconds 300 }
} until ($ready -or [DateTime]::UtcNow -ge $deadline)
if (-not $ready) { throw 'Read-only bridge is running, but preview did not become ready. See logs or run Stop-HandSdkMonitor.ps1.' }
Write-Host 'Ready: http://127.0.0.1:8090 - feedback + simulated commands only; no enable/motion calls.'
