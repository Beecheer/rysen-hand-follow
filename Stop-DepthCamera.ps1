[CmdletBinding()]
param()
$ErrorActionPreference='Stop';$origin='http://127.0.0.1:8095'
function Test-DepthPort {
 $socket=[Net.Sockets.TcpClient]::new()
 try{return $socket.ConnectAsync('127.0.0.1',8095).Wait(1000) -and $socket.Connected}catch{return $false}finally{$socket.Dispose()}
}
$state=$null;try{$state=Invoke-RestMethod "$origin/api/status" -TimeoutSec 2}catch{}
if(-not $state){if(Test-DepthPort){throw 'Cannot verify service on 8095; left untouched.'};Write-Host 'Depth camera service is stopped.';return}
if($state.service -ne 'rysen-realsense-basic'){throw 'Unexpected service on 8095; left untouched.'}
$null=Invoke-RestMethod "$origin/api/stop" -Method Post -Headers @{Origin=$origin} -ContentType 'application/json' -Body '{}' -TimeoutSec 3
$deadline=[DateTime]::UtcNow.AddSeconds(12)
do{if(-not(Test-DepthPort)){Write-Host 'Depth camera stopped; device release requested and port 8095 closed.';return};Start-Sleep -Milliseconds 200}while([DateTime]::UtcNow -lt $deadline)
throw 'Depth camera exit not confirmed; check realsense-tools/logs.'
