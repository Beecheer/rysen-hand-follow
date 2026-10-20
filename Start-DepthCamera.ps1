[CmdletBinding()]
param([switch]$NoBrowser,[switch]$Restart,[string]$Serial,[switch]$Hands)
$ErrorActionPreference='Stop'
$project=Join-Path $PSScriptRoot 'realsense-tools';$origin='http://127.0.0.1:8095'
foreach($name in @('uv','pwsh')){if(-not(Get-Command $name -ErrorAction SilentlyContinue)){throw "$name is not on PATH."}}
$existing=$null
try{$existing=Invoke-RestMethod "$origin/api/status" -TimeoutSec 2}catch{}
if($existing){
 if($existing.service -ne 'rysen-realsense-basic'){throw 'Unexpected service on 8095; left untouched.'}
 if([bool]$existing.hand_tracking.enabled -ne [bool]$Hands){$Restart=$true}
 if($Serial -and $existing.device.serial -and $existing.device.serial -ne $Serial -and -not $Restart){throw 'Another camera is active; stop it or use -Restart.'}
 if($Restart){& (Join-Path $PSScriptRoot 'Stop-DepthCamera.ps1')}
 else{
  Write-Host "Depth camera service already running: $origin ($($existing.state)). Use Resume to retry camera errors."
  if(-not $NoBrowser){Start-Process $origin};return
 }
}
$env:UV_CACHE_DIR=Join-Path $PSScriptRoot '.uv-cache'
$usbipd=Get-Command usbipd -ErrorAction SilentlyContinue
if($usbipd){
 $shared=& $usbipd.Source list
 if($LASTEXITCODE -eq 0 -and @($shared | Where-Object {$_ -match '^\s*\S+\s+8086:0b07\s+.*Attached\s*$'}).Count){
  throw 'D435 is attached through USBIP. Release this camera for Windows first; see realsense-tools/README.md (USBIP).'
 }
}
Push-Location $project
try{& uv sync --frozen;if($LASTEXITCODE -ne 0){throw 'Depth camera uv sync failed.'}}finally{Pop-Location}
if($Hands){
 & uv run --project $project --frozen python (Join-Path $project 'prepare_hand_model.py')
 if($LASTEXITCODE -ne 0){throw 'MediaPipe model preparation failed.'}
}
$logs=Join-Path $project 'logs';$null=New-Item -ItemType Directory -Path $logs -Force
$arguments=@('-NoProfile','-File',(Join-Path $project 'Run-Camera.ps1'))
if($Serial){$arguments+=@('-Serial',$Serial)}
if($Hands){$arguments+='-Hands'}
Start-Process pwsh -ArgumentList $arguments -WindowStyle Hidden -RedirectStandardOutput (Join-Path $logs 'camera.stdout.log') -RedirectStandardError (Join-Path $logs 'camera.stderr.log')
$deadline=[DateTime]::UtcNow.AddSeconds(25)
do{
 $state=$null;try{$state=Invoke-RestMethod "$origin/api/status" -TimeoutSec 2}catch{}
 if($state -and $state.service -ne 'rysen-realsense-basic'){throw 'Unexpected service on 8095.'}
 if($state.error){
  if(-not $NoBrowser){Start-Process $origin}
  throw "Camera error: $($state.error). Page $origin supports Resume / retry; see realsense-tools/logs."
 }
 if($state.state -eq 'RUNNING'){
  Write-Host "D435 preview ready: $origin (no robot connection)."
  if(-not $NoBrowser){Start-Process $origin};return
 }
 Start-Sleep -Milliseconds 250
}while([DateTime]::UtcNow -lt $deadline)
throw 'Depth camera startup not confirmed; run Stop-DepthCamera.cmd and inspect realsense-tools/logs.'
