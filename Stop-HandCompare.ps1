$ErrorActionPreference='Stop'
$state=$null
try{$state=Invoke-RestMethod 'http://127.0.0.1:8090/api/status' -TimeoutSec 2}catch{}
if($state -and -not $state.comparison.enabled){throw 'Port 8090 is not a comparison session; left untouched'}
$stopErrors=[System.Collections.Generic.List[string]]::new()
if($state){
    try { & (Join-Path $PSScriptRoot 'hand-tracking\Stop-HandMapping.ps1') }
    catch { $stopErrors.Add($_.Exception.Message) }
}
try { & (Join-Path $PSScriptRoot 'wilor-backend\Stop-Backend.ps1') }
catch { $stopErrors.Add($_.Exception.Message) }
if($stopErrors.Count){throw ($stopErrors -join "`n")}
Write-Host 'Comparison services are stopped.'
