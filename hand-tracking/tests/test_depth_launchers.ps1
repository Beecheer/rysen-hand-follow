$ErrorActionPreference='Stop'
$root=Split-Path (Split-Path $PSScriptRoot)
$fixture=Join-Path ([IO.Path]::GetTempPath()) ('rysen-depth-launchers-'+[guid]::NewGuid().ToString('N'))
function Assert-Depth($condition,$message){if(-not $condition){throw $message}}
function New-DepthState([bool]$full=$false){
 return @{units='radians';tracker='d435';joints=@{left_index_j1=0};state='WAITING';
  index_follow=@{full_hand=$full};depth=@{state='RUNNING';bound=$true;algorithm='d435-articulated-v2';device=@{serial='test-camera'}};
  hardware=@{control_capable=$true;connected=$true;feedback_age_ms=1;monitor_age_ms=1}}
}
try{
 $null=New-Item -ItemType Directory -Path $fixture,(Join-Path $fixture 'hand-tracking'),(Join-Path $fixture 'realsense-tools') -Force
 $source=Join-Path $root 'Start-DepthFollow.ps1';$text=[IO.File]::ReadAllText($source)
 $ast=[Management.Automation.Language.Parser]::ParseFile($source,[ref]$null,[ref]$null)
 $probe=$ast.Find({param($n)$n -is [Management.Automation.Language.FunctionDefinitionAst] -and $n.Name -eq 'Test-DepthFollowPort'},$true)
 $text=$text.Remove($probe.Extent.StartOffset,$probe.Extent.EndOffset-$probe.Extent.StartOffset).Insert($probe.Extent.StartOffset,'function Test-DepthFollowPort {return $global:DepthTestPortOpen}')
 Set-Content (Join-Path $fixture 'Start-DepthFollow.ps1') $text
 Copy-Item (Join-Path $root 'Stop-DepthFollow.ps1') $fixture
 Set-Content (Join-Path $fixture 'Stop-DepthCamera.ps1') '$global:DepthTrace.Add("camera-stop")'
 Set-Content (Join-Path $fixture 'Stop-HandFollow.ps1') '$global:DepthTrace.Add("sdk-stop"); if($global:DepthFailStop){throw "stop-failed"}'
 Set-Content (Join-Path $fixture 'Start-HandFollow.ps1') 'param($Tracker,$NoBrowser,$Restart,$DepthSerial);$global:DepthTrace.Add("full-start");if($global:DepthFailFull){throw "full-start-failed"};$global:DepthState=New-DepthState $true'
 function uv {$global:DepthTrace.Add('model');$global:LASTEXITCODE=0}
 function Invoke-RestMethod {param($Uri,$TimeoutSec);return $global:DepthState}
 function Start-Process {param($FilePath,$ArgumentList,$WindowStyle,$RedirectStandardOutput,$RedirectStandardError);$global:DepthTrace.Add('spawn');$global:DepthArguments=$ArgumentList;$global:DepthState=New-DepthState}
 $global:DepthTrace=[Collections.Generic.List[string]]::new()
 $global:DepthState=$null;$global:DepthTestPortOpen=$false;$global:DepthFailFull=$false;$global:DepthFailStop=$false
 & (Join-Path $fixture 'Start-DepthFollow.ps1') -PreviewOnly -NoBrowser -Serial test-camera
 Assert-Depth (($global:DepthTrace -join ',') -eq 'camera-stop,model,spawn') 'Preview cold-start sequence'
 Assert-Depth ($global:DepthArguments -contains '-DepthSerial') 'Serial forwarding'
 $global:DepthTrace.Clear()
 & (Join-Path $fixture 'Start-DepthFollow.ps1') -PreviewOnly -NoBrowser
 Assert-Depth ($global:DepthTrace.Count -eq 0) 'Healthy preview must be reused'
 & (Join-Path $fixture 'Start-DepthFollow.ps1') -PreviewOnly -Restart -NoBrowser
 Assert-Depth (($global:DepthTrace -join ',') -eq 'sdk-stop,camera-stop,model,spawn') 'Explicit preview restart'
 $global:DepthTrace.Clear();$global:DepthState=$null
 & (Join-Path $fixture 'Start-DepthFollow.ps1') -NoBrowser
 Assert-Depth (($global:DepthTrace -join ',') -eq 'camera-stop,model,full-start') 'Full-hand path must use production launcher'
 $global:DepthTrace.Clear()
 & (Join-Path $fixture 'Start-DepthFollow.ps1') -NoBrowser
 Assert-Depth ($global:DepthTrace.Count -eq 0) 'Healthy hardware session must be preserved'
 $global:DepthState=@{units='other'};$caught=$false
 try{& (Join-Path $fixture 'Start-DepthFollow.ps1') -PreviewOnly -NoBrowser}catch{$caught=$true}
 Assert-Depth ($caught -and $global:DepthTrace.Count -eq 0) 'Unknown service guard'
 $global:DepthState=$null;$global:DepthTestPortOpen=$true;$caught=$false
 try{& (Join-Path $fixture 'Start-DepthFollow.ps1') -PreviewOnly -NoBrowser}catch{$caught=$true}
 Assert-Depth ($caught -and $global:DepthTrace.Count -eq 0) 'Unverifiable occupied port guard'
 $global:DepthTestPortOpen=$false;$global:DepthFailFull=$true;$caught=$false
 try{& (Join-Path $fixture 'Start-DepthFollow.ps1') -NoBrowser}catch{$caught=$_.Exception.Message -match 'full-start-failed'}
 Assert-Depth ($caught -and ($global:DepthTrace -join ',') -eq 'camera-stop,model,full-start,sdk-stop,camera-stop') 'Failed full start must clean up'
 $global:DepthTrace.Clear();$global:DepthFailStop=$true;$caught=$false
 try{& (Join-Path $fixture 'Stop-DepthFollow.ps1')}catch{$caught=$true}
 Assert-Depth ($caught -and ($global:DepthTrace -join ',') -eq 'sdk-stop,camera-stop') 'Stop must continue camera cleanup after SDK error'
 Write-Host 'PASS: D435 preview/full startup, reuse, restart, serial, failure cleanup and unknown-service guards.'
}finally{
 $resolved=[IO.Path]::GetFullPath($fixture)
 $tempRoot=[IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\')+'\'
 if($resolved.StartsWith($tempRoot,[StringComparison]::OrdinalIgnoreCase) -and (Split-Path $resolved -Leaf) -match '^rysen-depth-launchers-[0-9a-f]{32}$'){
  Remove-Item -LiteralPath $resolved -Recurse -Force
 }
 foreach($name in @('DepthState','DepthTrace','DepthArguments','DepthTestPortOpen','DepthFailFull','DepthFailStop')){Remove-Variable -Name $name -Scope Global -ErrorAction SilentlyContinue}
}
