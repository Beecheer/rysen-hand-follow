[CmdletBinding()]
param()
$ErrorActionPreference='Stop'
$origin='http://127.0.0.1:8093'
function Test-BackendPort {
    $client=[Net.Sockets.TcpClient]::new()
    try { return $client.ConnectAsync('127.0.0.1',8093).Wait(1000) -and $client.Connected }
    catch { return $false }
    finally { $client.Dispose() }
}
$health=$null
try { $health=Invoke-RestMethod "$origin/health" -TimeoutSec 2 } catch {}
if (-not $health) {
    if (Test-BackendPort) { throw 'Port 8093 is occupied but its identity cannot be verified; left untouched.' }
    Write-Host 'WiLoR backend is stopped.'
    return
}
if ($health.service -ne 'rysen-wilor-inference') { throw 'Unexpected service on 8093; left untouched.' }
$null=Invoke-RestMethod "$origin/shutdown" -Method Post -Headers @{'X-Rysen-Control'='stop'} -TimeoutSec 3
$deadline=[DateTime]::UtcNow.AddSeconds(15)
do {
    if (-not (Test-BackendPort)) { Write-Host 'WiLoR backend stopped; port 8093 released.';return }
    Start-Sleep -Milliseconds 200
} while ([DateTime]::UtcNow -lt $deadline)
throw 'WiLoR exit not confirmed within 15 seconds; check wilor-backend/logs.'
