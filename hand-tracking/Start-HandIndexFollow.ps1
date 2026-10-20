[CmdletBinding()]
param([switch]$NoBrowser,[switch]$FullHand,[ValidateSet('mediapipe','wilor','d435','wilor-d435')][string]$Tracker='mediapipe',[string]$DepthSerial)
$ErrorActionPreference='Stop'
if($Tracker -eq 'wilor-d435' -and -not $FullHand){throw 'WiLoR + D435 supports full-hand follow, not the index-only trial.'}
foreach ($otherName in @('rysen_backend','rysen_explorer')) {
    $otherRaw=& wslc inspect $otherName
    if ($LASTEXITCODE -ne 0) { throw "Cannot inspect $otherName" }
    if (@(($otherRaw -join "`n") | ConvertFrom-Json)[0].State.Running) { throw "Stop $otherName before camera control." }
}
$raw=& wslc inspect rysen_sdk_readonly
if ($LASTEXITCODE -ne 0) { throw 'Initialize the existing SDK container with Start-HandSdkMonitor.ps1 first.' }
$container=@(($raw -join "`n") | ConvertFrom-Json)[0]
if ($container.Labels.'codex.purpose' -ne 'hand-follow-readonly' -or $container.Config.Image -ne 'rysen_system:latest' -or
    $container.Config.Entrypoint[0] -ne '/bin/sleep' -or $container.Ports.'8092/tcp'[0].HostIp -ne '127.0.0.1') {
    throw 'Unexpected SDK container configuration; no changes made.'
}
if (-not $container.State.Running) {
    & wslc start rysen_sdk_readonly
    if ($LASTEXITCODE -ne 0) { throw 'Cannot start existing SDK container.' }
}
foreach ($controlName in @('sdk_bridge_control.py','sdk_follow_control.py')) {
    & wslc exec rysen_sdk_readonly python3 -c 'from pathlib import Path; import subprocess,sys; p=Path("/opt/hand-follow")/sys.argv[1]; raise SystemExit(subprocess.call(["python3",str(p),"stop"]) if p.exists() else 0)' $controlName
    if ($LASTEXITCODE -ne 0) { throw "Cannot stop $controlName" }
}
$sources=@{}
foreach ($name in @('sdk_bridge_control.py','sdk_follow_control.py','sdk_follow_bridge.py','joint_layout.py','follow_bounds.py')) {
    $sources[$name]=[IO.File]::ReadAllText((Join-Path $PSScriptRoot "hand_preview\$name"))
}
$sources['index_camera_worker.cpp']=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'hardware\index_camera_worker.cpp'))
$sources['preview-left.urdf']=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'assets\apex_left\apex_hand_left.urdf'))
$sources['device-limits.json']=[IO.File]::ReadAllText((Join-Path $PSScriptRoot 'hardware\device-limits.json'))
$sources['follow-profile.json']=@{full_hand=[bool]$FullHand}|ConvertTo-Json -Compress
$token=[Convert]::ToHexString([Security.Cryptography.RandomNumberGenerator]::GetBytes(32))
$sources['control-token.txt']=$token
$null=New-Item -ItemType Directory -Path (Join-Path $PSScriptRoot 'profiles') -Force
[IO.File]::WriteAllText((Join-Path $PSScriptRoot 'profiles\sdk-control-token.txt'),$token)
$sources | ConvertTo-Json -Compress | & wslc exec -i rysen_sdk_readonly python3 -c 'import json,sys,pathlib; root=pathlib.Path("/opt/hand-follow"); files=json.load(sys.stdin); [(root/name).write_text(data,encoding="utf-8") for name,data in files.items()]; (root/"control-token.txt").chmod(0o600)'
if ($LASTEXITCODE -ne 0) { throw 'Follow source deployment failed.' }
if ($FullHand) {
    $boundsJson=& wslc exec rysen_sdk_readonly python3 /opt/hand-follow/follow_bounds.py --preview /opt/hand-follow/preview-left.urdf --sdk /workspace/install/rysen_apexhand_description/share/rysen_apexhand_description/urdf/apex_hand_left.urdf --device-limits /opt/hand-follow/device-limits.json
    if ($LASTEXITCODE -ne 0) { throw 'SDK/preview limit intersection failed; no device enabled.' }
    [IO.File]::WriteAllText((Join-Path $PSScriptRoot 'profiles\full-hand-limits.json'),($boundsJson -join "`n"))
}
& wslc exec rysen_sdk_readonly g++ -std=c++17 -O2 -pthread -I/workspace/rysen_sdk/include /opt/hand-follow/index_camera_worker.cpp -L/workspace/rysen_sdk/lib/x86_64 -lrysen_sdk '-Wl,-rpath,/workspace/rysen_sdk/lib/x86_64' -o /opt/hand-follow/index_camera_worker
if ($LASTEXITCODE -ne 0) { throw 'Native worker compilation failed; no device enabled.' }
& wslc exec rysen_sdk_readonly python3 /opt/hand-follow/sdk_follow_control.py start
if ($LASTEXITCODE -ne 0) { throw 'Cannot launch follow worker.' }
$deadline=[DateTime]::UtcNow.AddSeconds(25);$ready=$false
do {
    try {
        $status=Invoke-RestMethod http://127.0.0.1:8092/status -TimeoutSec 2
        $ready=$status.control_capable -and $status.connected -and $status.feedback_age_ms -lt 250 -and $status.fault_code -eq 0
    } catch { $ready=$false }
    if (-not $ready) { Start-Sleep -Milliseconds 300 }
} until ($ready -or [DateTime]::UtcNow -ge $deadline)
if (-not $ready) {
    & wslc exec rysen_sdk_readonly cat /tmp/hand-sdk-follow.log
    & wslc exec rysen_sdk_readonly python3 /opt/hand-follow/sdk_follow_control.py stop
    throw 'Follow worker did not become ready; device was not enabled.'
}
if($FullHand){
    $deviceLimits=Get-Content (Join-Path $PSScriptRoot 'hardware\device-limits.json') -Raw | ConvertFrom-Json
    if($status.version.sdk_version -ne $deviceLimits.sdk_version -or $status.version.hand_firmware_version -ne $deviceLimits.firmware_version){
        & wslc exec rysen_sdk_readonly python3 /opt/hand-follow/sdk_follow_control.py stop
        throw 'SDK/firmware differs from the device limit evidence; review device-limits.json before enabling.'
    }
}
& (Join-Path $PSScriptRoot 'Stop-HandMapping.ps1')
$arguments=@('-NoProfile','-File',(Join-Path $PSScriptRoot 'Start-HandMapping.ps1'))
$arguments+=@('-Tracker',$Tracker)
if($DepthSerial){$arguments+=@('-DepthSerial',$DepthSerial)}
if($Tracker -in @('wilor','wilor-d435')){
    $savedProfile=Get-Content (Join-Path $PSScriptRoot 'profiles\wilor-current.json') -Raw | ConvertFrom-Json
    if($savedProfile.context.swap_handedness){$arguments+='-SwapHandedness'}
}
if($Tracker -eq 'd435'){
    $profilePath=Join-Path $PSScriptRoot 'profiles\d435-articulated.json'
    if(Test-Path -LiteralPath $profilePath){
        $savedDepth=Get-Content $profilePath -Raw | ConvertFrom-Json
        if($savedDepth.context.swap_handedness){$arguments+='-SwapHandedness'}
    }
}
if ($FullHand) { $arguments+='-FullHandFollow' } else { $arguments+='-IndexFollow' }
if ($NoBrowser) { $arguments+='-NoBrowser' }
$null=Start-Process pwsh -ArgumentList $arguments -WindowStyle Hidden -PassThru -RedirectStandardOutput (Join-Path $PSScriptRoot 'logs\mapping.stdout.log') -RedirectStandardError (Join-Path $PSScriptRoot 'logs\mapping.stderr.log')
$deadline=[DateTime]::UtcNow.AddSeconds(25);$ready=$false
do {
    try {
        $preview=Invoke-RestMethod http://127.0.0.1:8090/api/status -TimeoutSec 2
        $ready=$null -ne $preview.index_follow -and $preview.hardware.control_capable -and $preview.hardware.connected
    } catch { $ready=$false }
    if (-not $ready) { Start-Sleep -Milliseconds 300 }
} until ($ready -or [DateTime]::UtcNow -ge $deadline)
if (-not $ready) {
    & wslc exec rysen_sdk_readonly python3 /opt/hand-follow/sdk_follow_control.py stop
    throw 'Camera page did not become ready; follow worker stopped. Check logs.'
}
Write-Host 'Follow worker ready and disabled. Open http://127.0.0.1:8090 and click Start when your left hand is tracked.'
