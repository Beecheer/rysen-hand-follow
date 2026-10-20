if (-not (Get-Command wslc -ErrorAction SilentlyContinue)) { throw 'wslc is not on PATH.' }

function Get-RysenContainer([string]$Name) {
    $raw = & wslc inspect $Name
    if ($LASTEXITCODE -ne 0) {
        throw "Cannot inspect $Name. Check the wslc session and container name. No installation or recreation was attempted."
    }
    $items = @(($raw -join "`n") | ConvertFrom-Json)
    if ($items.Count -ne 1 -or $items[0].Name -ne "/$Name") { throw "Unexpected inspect result for $Name" }
    return $items[0]
}

function Test-RysenWebSocket {
    $socket = [System.Net.WebSockets.ClientWebSocket]::new()
    $cancel = [System.Threading.CancellationTokenSource]::new(3000)
    try {
        $socket.Options.Proxy = $null
        $socket.Options.AddSubProtocol('foxglove.sdk.v1')
        $null = $socket.ConnectAsync([Uri]'ws://127.0.0.1:8765', $cancel.Token).GetAwaiter().GetResult()
        if ($socket.State -ne 'Open') { throw 'Backend WebSocket is not open.' }
    } finally {
        $socket.Dispose()
        $cancel.Dispose()
    }
}
