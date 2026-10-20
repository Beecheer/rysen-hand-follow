# Behavioral checks use isolated companion-script stubs; no GPU, camera or SDK starts.
$ErrorActionPreference='Stop'
$root=Split-Path (Split-Path $PSScriptRoot)
$fixture=Join-Path ([IO.Path]::GetTempPath()) ('rysen-launchers-'+[Guid]::NewGuid().ToString('N'))
function Assert-Launcher($condition,[string]$message) { if (-not $condition) { throw $message } }
try {
    foreach ($relative in @('hand-tracking','hand-tracking\profiles','wilor-backend')) {
        $null=New-Item -ItemType Directory -Path (Join-Path $fixture $relative) -Force
    }
    foreach ($name in @('Start-HandFollow.ps1','Stop-HandFollow.ps1')) {
        Copy-Item -LiteralPath (Join-Path $root $name) -Destination (Join-Path $fixture $name)
    }
    Set-Content (Join-Path $fixture 'Rysen.Common.ps1') 'function Get-RysenContainer { return @{Labels=@{"codex.purpose"="hand-follow-readonly"}} }'
    Set-Content (Join-Path $fixture 'hand-tracking\profiles\wilor-current.json') '{"schema":3,"context":{"tracker":"wilor"}}'
    Set-Content (Join-Path $fixture 'wilor-backend\Ensure-Backend.ps1') '$global:LauncherTrace.Add("ensure"); if($global:LauncherFailEnsure){throw "load failure"}'
    Set-Content (Join-Path $fixture 'hand-tracking\Start-HandFullFollow.ps1') '$global:LauncherTrace.Add("start"); if($global:LauncherFailStart){throw "camera failure"}'
    Set-Content (Join-Path $fixture 'hand-tracking\Stop-HandFullFollow.ps1') '$global:LauncherTrace.Add("sdk-stop"); if($global:LauncherFailStop){throw "SDK stop failure"}'
    Set-Content (Join-Path $fixture 'wilor-backend\Stop-Backend.ps1') '$global:LauncherTrace.Add("gpu-stop")'
    function Invoke-RestMethod {
        param($Uri,$Method,$Headers,$TimeoutSec)
        if ($Uri -like '*/health' -and $global:LauncherBackendMode) {
            return @{service=$global:LauncherBackendMode}
        }
        if ($Uri -like '*/shutdown') { $global:LauncherTrace.Add('backend-shutdown');return @{ok=$true} }
        return $global:LauncherState
    }
    $global:LauncherTrace=[System.Collections.Generic.List[string]]::new()
    $global:LauncherState=$null
    $global:LauncherFailEnsure=$global:LauncherFailStart=$global:LauncherFailStop=$false
    & (Join-Path $fixture 'Start-HandFollow.ps1') -NoBrowser
    Assert-Launcher (($global:LauncherTrace -join ',') -eq 'ensure,start') 'Cold start order is incorrect'
    $global:LauncherTrace.Clear()
    Set-Content (Join-Path $fixture 'hand-tracking\profiles\wilor-current.json') '{"schema":5,"context":{"tracker":"wilor"}}'
    $caught=$false
    try { & (Join-Path $fixture 'Start-HandFollow.ps1') -Tracker wilor-d435 -NoBrowser } catch { $caught=$_.Exception.Message -match 'paired side-sway zero' }
    Assert-Launcher ($caught -and $global:LauncherTrace.Count -eq 0) 'Hybrid started without paired calibration'
    Set-Content (Join-Path $fixture 'hand-tracking\profiles\wilor-current.json') '{"schema":5,"context":{"tracker":"wilor"},"side_sway_zero":{"index":{"depth":0,"wilor":0}}}'
    & (Join-Path $fixture 'Start-HandFollow.ps1') -Tracker wilor-d435 -NoBrowser
    Assert-Launcher (($global:LauncherTrace -join ',') -eq 'ensure,start') 'Hybrid did not ensure WiLoR backend'
    Set-Content (Join-Path $fixture 'hand-tracking\profiles\wilor-current.json') '{"schema":3,"context":{"tracker":"wilor"}}'
    $global:LauncherTrace.Clear();$global:LauncherFailStart=$true
    $caught=$false
    try { & (Join-Path $fixture 'Start-HandFollow.ps1') -NoBrowser } catch { $caught=$_.Exception.Message -match 'camera failure' }
    Assert-Launcher $caught 'Original start failure was not reported'
    Assert-Launcher (($global:LauncherTrace -join ',') -eq 'ensure,start,sdk-stop,gpu-stop') 'Start failure left services uncleaned'
    $global:LauncherTrace.Clear();$global:LauncherFailStart=$false;$global:LauncherFailEnsure=$true
    try { & (Join-Path $fixture 'Start-HandFollow.ps1') -NoBrowser } catch {}
    Assert-Launcher (($global:LauncherTrace -join ',') -eq 'ensure,sdk-stop,gpu-stop') 'GPU load failure skipped cleanup'
    $global:LauncherFailEnsure=$false;$global:LauncherFailStop=$true;$global:LauncherTrace.Clear()
    $caught=$false
    try { & (Join-Path $fixture 'Stop-HandFollow.ps1') } catch { $caught=$_.Exception.Message -match 'SDK stop failure' }
    Assert-Launcher $caught 'Stop failure was hidden'
    Assert-Launcher (($global:LauncherTrace -join ',') -eq 'sdk-stop,gpu-stop') 'GPU cleanup was skipped after SDK failure'
    $global:LauncherFailStop=$false;$global:LauncherTrace.Clear()
    $global:LauncherState=@{units='radians';joints=@{left_index_j1=0};tracker='wilor';index_follow=@{full_hand=$true};
        hardware=@{full_hand=$true;control_capable=$true;connected=$true;feedback_age_ms=1;monitor_age_ms=1;follow=@{active=$false}}}
    & (Join-Path $fixture 'Start-HandFollow.ps1') -NoBrowser
    Assert-Launcher ($global:LauncherTrace.Count -eq 0) 'Healthy session was restarted'
    & (Join-Path $fixture 'Start-HandFollow.ps1') -Restart -NoBrowser
    Assert-Launcher (($global:LauncherTrace -join ',') -eq 'ensure,start') 'Explicit restart did not deploy'
    $global:LauncherState=@{units='other'};$global:LauncherTrace.Clear();$caught=$false
    try { & (Join-Path $fixture 'Start-HandFollow.ps1') -NoBrowser } catch { $caught=$true }
    Assert-Launcher ($caught -and $global:LauncherTrace.Count -eq 0) 'Unknown service was changed'
    # Stub only the socket probe so this test also works while a real GPU
    # service is running. HTTP requests above are already isolated.
    $backendPath=Join-Path $root 'wilor-backend\Stop-Backend.ps1'
    $backendText=[IO.File]::ReadAllText($backendPath)
    $backendAst=[Management.Automation.Language.Parser]::ParseFile($backendPath,[ref]$null,[ref]$null)
    $probe=$backendAst.Find({param($node) $node -is [Management.Automation.Language.FunctionDefinitionAst] -and $node.Name -eq 'Test-BackendPort'},$true)
    $backendText=$backendText.Remove($probe.Extent.StartOffset,$probe.Extent.EndOffset-$probe.Extent.StartOffset).Insert($probe.Extent.StartOffset,'function Test-BackendPort { return $false }')
    Set-Content (Join-Path $fixture 'Stop-Backend-Real.ps1') $backendText
    $global:LauncherBackendMode='unexpected-service';$caught=$false
    try { & (Join-Path $fixture 'Stop-Backend-Real.ps1') } catch { $caught=$true }
    Assert-Launcher ($caught -and $global:LauncherTrace.Count -eq 0) 'Backend identity guard failed'
    $global:LauncherBackendMode='rysen-wilor-inference'
    & (Join-Path $fixture 'Stop-Backend-Real.ps1')
    Assert-Launcher (($global:LauncherTrace -join ',') -eq 'backend-shutdown') 'Owned backend shutdown was not requested'
    # Parse the real scripts, including the shared backend shutdown helper.
    foreach ($relative in @('Start-HandFollow.ps1','Stop-HandFollow.ps1','Stop-HandCompare.ps1',
                           'hand-tracking\Start-HandFullFollow.ps1','hand-tracking\Start-HandIndexFollow.ps1','hand-tracking\Start-HandMapping.ps1',
                           'hand-tracking\Stop-HandIndexFollow.ps1','wilor-backend\Stop-Backend.ps1')) {
        $tokens=$null;$errors=$null
        $null=[Management.Automation.Language.Parser]::ParseFile((Join-Path $root $relative),[ref]$tokens,[ref]$errors)
        Assert-Launcher ($errors.Count -eq 0) "PowerShell syntax errors in $relative"
    }
    Write-Host 'PASS: startup/reuse/restart, failure cleanup, stop error aggregation, unknown service guard and script syntax.'
} finally {
    # The fixture is the exact GUID directory created above, within the OS temp root.
    $resolved=[IO.Path]::GetFullPath($fixture)
    $tempRoot=[IO.Path]::GetFullPath([IO.Path]::GetTempPath()).TrimEnd('\')+'\'
    if ($resolved.StartsWith($tempRoot,[StringComparison]::OrdinalIgnoreCase) -and
        (Split-Path $resolved -Leaf) -match '^rysen-launchers-[0-9a-f]{32}$') {
        Remove-Item -LiteralPath $resolved -Recurse -Force
    }
    foreach ($name in @('LauncherTrace','LauncherState','LauncherFailEnsure','LauncherFailStart','LauncherFailStop','LauncherBackendMode')) {
        Remove-Variable -Name $name -Scope Global -ErrorAction SilentlyContinue
    }
}
