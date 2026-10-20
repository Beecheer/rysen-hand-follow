[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$failures=@()
try{& (Join-Path $PSScriptRoot 'Stop-HandFollow.ps1')}catch{$failures+=$_.Exception.Message}
try{& (Join-Path $PSScriptRoot 'Stop-DepthCamera.ps1')}catch{$failures+=$_.Exception.Message}
if($failures.Count){throw ($failures -join '; ')}
