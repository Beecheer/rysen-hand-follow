[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$errorsFound=[System.Collections.Generic.List[string]]::new()
try { & (Join-Path $PSScriptRoot 'Stop-HandMapping.ps1') } catch { $errorsFound.Add($_.Exception.Message) }
$raw=& wslc inspect rysen_sdk_readonly
if ($LASTEXITCODE -ne 0) {
    $errorsFound.Add('Cannot inspect SDK container; no container stop attempted.')
    throw ($errorsFound -join "`n")
}
$container=@(($raw -join "`n") | ConvertFrom-Json)[0]
if ($container.Labels.'codex.purpose' -ne 'hand-follow-readonly') { throw 'Refusing to stop an unrelated container.' }
if ($container.State.Running) {
& wslc exec rysen_sdk_readonly python3 /opt/hand-follow/sdk_follow_control.py stop
if ($LASTEXITCODE -ne 0) { $errorsFound.Add('Follow worker exit/disable not confirmed; inspect device and container.') }
else {
    & wslc stop --time 10 rysen_sdk_readonly
    if ($LASTEXITCODE -ne 0) { $errorsFound.Add('Container stop failed.') }
}
}
if ($errorsFound.Count) { throw ($errorsFound -join "`n") }
Write-Host 'Camera and SDK services stopped. This is not hardware power-off; device state requires live feedback to confirm.'
