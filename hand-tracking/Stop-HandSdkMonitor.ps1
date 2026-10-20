[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$stopErrors=[System.Collections.Generic.List[string]]::new()
try { & (Join-Path $PSScriptRoot 'Stop-HandMapping.ps1') } catch { $stopErrors.Add($_.Exception.Message) }
$containerRaw=& wslc inspect rysen_sdk_readonly
if ($LASTEXITCODE -eq 0) {
    $container=@(($containerRaw -join "`n") | ConvertFrom-Json)[0]
    if ($container.Labels.'codex.purpose' -ne 'hand-follow-readonly') { throw 'Refusing to stop an unrelated container.' }
    if ($container.State.Running) {
        & wslc exec rysen_sdk_readonly python3 -c 'from pathlib import Path; import subprocess; p=Path("/opt/hand-follow/sdk_follow_control.py"); raise SystemExit(subprocess.call(["python3",str(p),"stop"]) if p.exists() else 0)'
        if ($LASTEXITCODE -ne 0) { throw 'Follow worker exit/disable not confirmed; container remains for diagnosis.' }
        & wslc exec rysen_sdk_readonly python3 /opt/hand-follow/sdk_bridge_control.py stop
        if ($LASTEXITCODE -ne 0) { $stopErrors.Add('SDK disconnect did not complete; container remains for diagnosis.') }
        else {
            & wslc stop --time 10 rysen_sdk_readonly
            if ($LASTEXITCODE -ne 0) { $stopErrors.Add('Could not stop read-only container.') }
        }
    }
} else { $stopErrors.Add('Cannot inspect read-only container.') }
if ($stopErrors.Count) { throw ($stopErrors -join "`n") }
Write-Host 'Preview and SDK bridge stopped. Native SDK disconnect may disable fingers; this is not hardware power-off.'
