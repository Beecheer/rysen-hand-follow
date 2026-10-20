[CmdletBinding()]
param([switch]$NoBrowser,[switch]$Restart,[ValidateSet('mediapipe','wilor','d435','wilor-d435')][string]$Tracker='wilor',[string]$DepthSerial)
$ErrorActionPreference='Stop'
. (Join-Path $PSScriptRoot 'Rysen.Common.ps1')
foreach ($command in @('uv','pwsh')) {
    if (-not (Get-Command $command -ErrorAction SilentlyContinue)) { throw "$command is not on PATH." }
}
if($Tracker -in @('wilor','wilor-d435')){
    $profilePath=Join-Path $PSScriptRoot 'hand-tracking\profiles\wilor-current.json'
    if (-not (Test-Path -LiteralPath $profilePath)) { throw 'WiLoR calibration is missing; see hand-tracking/README-Comparison.md.' }
    $profile=Get-Content $profilePath -Raw | ConvertFrom-Json
    if($profile.schema -notin @(3,5) -or $profile.context.tracker -ne 'wilor'){throw 'Complete and save a supported WiLoR calibration first.'}
    if($Tracker -eq 'wilor-d435' -and ($profile.schema -ne 5 -or -not $profile.side_sway_zero)){
        throw 'WiLoR + D435 requires the saved three-pose profile and paired side-sway zero.'
    }
}
$container=Get-RysenContainer 'rysen_sdk_readonly'
if ($container.Labels.'codex.purpose' -ne 'hand-follow-readonly') {
    throw 'Unexpected SDK container owner; no changes made.'
}
$origin='http://127.0.0.1:8090'
$state=$null
try { $state=Invoke-RestMethod "$origin/api/status" -TimeoutSec 2 } catch {}
if ($null -ne $state) {
    if ($state.units -ne 'radians' -or $null -eq $state.joints.left_index_j1) {
        throw 'Port 8090 belongs to an unexpected service; no changes made.'
    }
    $healthy=$state.index_follow.full_hand -and $state.hardware.full_hand -and
        $state.hardware.control_capable -and $state.hardware.connected -and
        $state.hardware.feedback_age_ms -lt 250 -and $state.hardware.monitor_age_ms -lt 250
    $sameAlgorithm=$Tracker -ne 'd435' -or $state.depth.algorithm -eq 'd435-articulated-v2'
    if ($healthy -and $state.tracker -eq $Tracker -and $sameAlgorithm -and -not $Restart) {
        Write-Host "Hand follow is already running: $origin"
        Write-Host "Motion active: $($state.hardware.follow.active). Existing session was preserved."
        if (-not $NoBrowser) { Start-Process $origin }
        return
    }
}
try {
    if ($Tracker -in @('wilor','wilor-d435')) { & (Join-Path $PSScriptRoot 'wilor-backend\Ensure-Backend.ps1') }
    & (Join-Path $PSScriptRoot 'hand-tracking\Start-HandFullFollow.ps1') -NoBrowser:$NoBrowser -Tracker $Tracker -DepthSerial $DepthSerial
} catch {
    $failure=$_
    Write-Warning 'Start failed; stopping this workspace camera/SDK/WiLoR services.'
    try { & (Join-Path $PSScriptRoot 'Stop-HandFollow.ps1') }
    catch { Write-Warning "Cleanup could not be confirmed: $($_.Exception.Message)" }
    throw $failure
}
Write-Host "Ready: $origin"
Write-Host 'Starts disabled. Resume camera tracking, then click Start full-hand follow in the page.'
