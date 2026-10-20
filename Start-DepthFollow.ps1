[CmdletBinding()]
param([switch]$PreviewOnly,[switch]$NoBrowser,[switch]$Restart,[string]$Serial)
$ErrorActionPreference='Stop'
$origin='http://127.0.0.1:8090';$project=Join-Path $PSScriptRoot 'hand-tracking'
foreach($name in @('uv','pwsh')){if(-not(Get-Command $name -ErrorAction SilentlyContinue)){throw "$name is not on PATH."}}
function Test-DepthFollowPort {
 $client=[Net.Sockets.TcpClient]::new()
 try{return $client.ConnectAsync('127.0.0.1',8090).Wait(1000) -and $client.Connected}catch{return $false}finally{$client.Dispose()}
}
$state=$null;try{$state=Invoke-RestMethod "$origin/api/status" -TimeoutSec 2}catch{}
if(-not $state -and (Test-DepthFollowPort)){throw 'Cannot verify service on 8090; left untouched.'}
if($state){
 if($state.units -ne 'radians' -or $null -eq $state.joints.left_index_j1){throw 'Unknown service on 8090; left untouched.'}
 $sameMode=([bool]$state.index_follow.full_hand -eq (-not [bool]$PreviewOnly))
 $sameSerial=(-not $Serial -or $state.depth.device.serial -eq $Serial)
 $sameAlgorithm=($state.depth.algorithm -eq 'd435-articulated-v2')
 $healthy= $PreviewOnly -or ($state.hardware.control_capable -and $state.hardware.connected -and $state.hardware.feedback_age_ms -lt 250 -and $state.hardware.monitor_age_ms -lt 250)
 if($state.tracker -eq 'd435' -and $sameMode -and $sameSerial -and $sameAlgorithm -and $healthy -and -not $Restart){
  if($state.depth.error -or $state.state -eq 'ERROR' -or -not $state.depth.bound){throw "D435 is not ready: $($state.reason). Use Resume or -Restart."}
  Write-Host "D435 chain already running: $origin (motion state preserved)."
  if(-not $NoBrowser){Start-Process $origin};return
 }
 & (Join-Path $PSScriptRoot 'Stop-HandFollow.ps1')
}
# Shared camera: stop only the identified standalone D435 service.
& (Join-Path $PSScriptRoot 'Stop-DepthCamera.ps1')
$env:UV_CACHE_DIR=Join-Path $PSScriptRoot '.uv-cache'
& uv run --project (Join-Path $PSScriptRoot 'realsense-tools') --frozen python (Join-Path $PSScriptRoot 'realsense-tools\prepare_hand_model.py')
if($LASTEXITCODE){throw 'D435 MediaPipe model preparation failed.'}
try{
if(-not $PreviewOnly){
 & (Join-Path $PSScriptRoot 'Start-HandFollow.ps1') -Tracker d435 -NoBrowser:$NoBrowser -Restart:$Restart -DepthSerial $Serial
}else{
$logs=Join-Path $project 'logs';$null=New-Item -ItemType Directory -Path $logs -Force
$arguments=@('-NoProfile','-File',(Join-Path $project 'Start-HandMapping.ps1'),'-Tracker','d435','-NoBrowser')
if($Serial){$arguments+=@('-DepthSerial',$Serial)}
$profilePath=Join-Path $project 'profiles\d435-articulated.json'
if(Test-Path -LiteralPath $profilePath){if((Get-Content $profilePath -Raw | ConvertFrom-Json).context.swap_handedness){$arguments+='-SwapHandedness'}}
Start-Process pwsh -ArgumentList $arguments -WindowStyle Hidden -RedirectStandardOutput (Join-Path $logs 'mapping.stdout.log') -RedirectStandardError (Join-Path $logs 'mapping.stderr.log')
}
$deadline=[DateTime]::UtcNow.AddSeconds(30)
do{
 $state=$null;try{$state=Invoke-RestMethod "$origin/api/status" -TimeoutSec 2}catch{}
 if($state -and $state.tracker -ne 'd435'){throw 'Unexpected service on 8090; left untouched.'}
 if($state.depth.error -or $state.state -eq 'ERROR'){throw "D435 failed: $($state.reason). See hand-tracking/logs."}
 if($state.depth.bound -and $state.depth.state -eq 'RUNNING'){
  Write-Host "D435 chain ready: $origin; preview-only=$([bool]$PreviewOnly). Startup never enables the robot."
  if(-not $NoBrowser){Start-Process $origin};return
 }
 Start-Sleep -Milliseconds 250
}while([DateTime]::UtcNow -lt $deadline)
throw 'D435 startup not confirmed; use Stop-DepthFollow.cmd and inspect logs.'
}catch{
 $failure=$_
 try{& (Join-Path $PSScriptRoot 'Stop-DepthFollow.ps1')}catch{Write-Warning "Cleanup not confirmed: $($_.Exception.Message)"}
 throw $failure
}
