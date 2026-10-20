[CmdletBinding()]
param([ValidateRange(5, 120)][int]$TimeoutSeconds = 30)
$ErrorActionPreference = 'Stop'
. (Join-Path $PSScriptRoot 'Rysen.Common.ps1')
$failures = [System.Collections.Generic.List[string]]::new()

# Stop the web UI first, then the ROS backend. Repeated calls are safe.
foreach ($name in @('rysen_explorer', 'rysen_backend')) {
    try {
        $container = Get-RysenContainer $name
        if (-not $container.State.Running) {
            Write-Host "$name is already stopped."
            continue
        }
        if ($name -eq 'rysen_backend') {
            # PID 1 may be a shell that does not forward signals to ros2 launch.
            $signalScript = @'
import os, signal
from pathlib import Path
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name) in (1, os.getpid()):
        continue
    try:
        args = (p / 'cmdline').read_bytes().split(b'\0')
        if b'launch' in args and any(a.endswith(b'/ros2') for a in args):
            os.kill(int(p.name), signal.SIGINT)
            print('Sent SIGINT to ROS launch PID', p.name)
    except (FileNotFoundError, ProcessLookupError, PermissionError):
        pass
'@
            $signalScript | & wslc exec -i $name python3 -
            if ($LASTEXITCODE -ne 0) { Write-Warning 'ROS signal request failed; proceeding with container stop.' }
        }
        & wslc stop --time $TimeoutSeconds $name
        if ($LASTEXITCODE -ne 0) { throw "Cannot stop $name" }
        $container = Get-RysenContainer $name
        if ($container.State.Running) { throw "$name is still running" }
        Write-Host "$name stopped; exit code: $($container.State.ExitCode)"
        if ($container.State.ExitCode -eq 137) {
            Write-Warning "$name was forcibly terminated. Stopping software does not confirm hardware power-off."
        }
    } catch {
        $failures.Add("${name}: $($_.Exception.Message)")
        Write-Warning $failures[$failures.Count - 1]
    }
}
if ($failures.Count) { throw ($failures -join "`n") }
Write-Host 'Services stopped. Containers, image and motion files are preserved.'
