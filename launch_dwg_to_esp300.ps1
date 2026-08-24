$ErrorActionPreference = "Stop"

$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$hostName = "127.0.0.1"
$port = 8766
$url = "http://$hostName`:$port"

Set-Location -LiteralPath $root

$serverRunning = $false
try {
    $client = New-Object Net.Sockets.TcpClient
    $async = $client.BeginConnect($hostName, $port, $null, $null)
    $serverRunning = $async.AsyncWaitHandle.WaitOne(300, $false)
    if ($serverRunning) {
        $client.EndConnect($async)
    }
    $client.Close()
} catch {
    $serverRunning = $false
}

if (-not $serverRunning) {
    Start-Process -FilePath "py" `
        -ArgumentList @("visualizer_server.py", "--host", $hostName, "--port", "$port") `
        -WorkingDirectory $root `
        -WindowStyle Hidden
    Start-Sleep -Seconds 2
}

Start-Process $url
