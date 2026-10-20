[CmdletBinding()]
param([ValidateRange(1024,65535)][int]$Port = 8090)
$ErrorActionPreference='Stop'
$origin="http://127.0.0.1:$Port"
try {
    $state=Invoke-RestMethod -Uri "$origin/api/status" -TimeoutSec 3
} catch {
    if ($_.Exception.InnerException -is [System.Net.Sockets.SocketException] -or $_.Exception.Message -match 'refused|拒绝') {
        Write-Host 'Mapping preview is not running.'
        return
    }
    throw
}
if ($state.units -ne 'radians' -or $state.robot_connected -ne $false -or $null -eq $state.joints.left_index_j1) {
    throw 'Unexpected service on this port; no stop request was sent.'
}
$null=Invoke-RestMethod -Uri "$origin/api/control/stop" -Method Post -Headers @{Origin=$origin} -TimeoutSec 3
$deadline=[DateTime]::UtcNow.AddSeconds(12)
do {
    try {
        $null=Invoke-WebRequest -Uri "$origin/api/status" -TimeoutSec 1 -UseBasicParsing
    } catch {
        if ($null -eq $_.Exception.Response) {
            Write-Host 'Mapping service stopped; camera worker has been asked to release the camera.'
            return
        }
    }
    Start-Sleep -Milliseconds 200
} while ([DateTime]::UtcNow -lt $deadline)
throw 'Mapping service did not exit within 12 seconds. Check the launcher terminal/logs.'
