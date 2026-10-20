[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$stopErrors=[System.Collections.Generic.List[string]]::new()
foreach ($script in @('hand-tracking\Stop-HandFullFollow.ps1','wilor-backend\Stop-Backend.ps1')) {
    try { & (Join-Path $PSScriptRoot $script) }
    catch { $stopErrors.Add("${script}: $($_.Exception.Message)") }
}
if ($stopErrors.Count) { throw ($stopErrors -join "`n") }
Write-Host 'Hand follow services stopped. No automatic return-to-zero or hardware power-off.'
