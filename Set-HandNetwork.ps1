[CmdletBinding()]
param([string]$InterfaceAlias='以太网')
$ErrorActionPreference='Stop'
$resultPath=Join-Path $PSScriptRoot 'hand-tracking\logs\hand-network-result.json'
$result=@{success=$false;interface=$InterfaceAlias;address='192.168.0.1';device='192.168.0.102'}
try {
    $identity=[Security.Principal.WindowsIdentity]::GetCurrent()
    if(-not ([Security.Principal.WindowsPrincipal]::new($identity)).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)){
        throw 'Run this script from an elevated PowerShell. Windows administrator rights are required to configure Ethernet.'
    }
    $adapter=Get-NetAdapter -Name $InterfaceAlias -ErrorAction Stop
    if($adapter.InterfaceDescription -ne 'Realtek PCIe GbE Family Controller' -or $adapter.Status -ne 'Up'){
        throw 'Expected connected Realtek Ethernet adapter was not found; no changes made.'
    }
    $before=@(Get-NetIPAddress -InterfaceIndex $adapter.ifIndex -AddressFamily IPv4 -ErrorAction Stop)
    if(@($before | Where-Object {$_.IPAddress -notlike '169.254.*' -and $_.IPAddress -ne '192.168.0.1'}).Count){
        throw 'Ethernet already has another configured address; it was left untouched.'
    }
    $backup=Join-Path $PSScriptRoot ('hand-tracking\logs\hand-network-before-'+(Get-Date -Format 'yyyyMMdd-HHmmss')+'.json')
    @{interface=$InterfaceAlias;index=$adapter.ifIndex;
      addresses=@($before | Select-Object IPAddress,PrefixLength,PrefixOrigin);
      settings=@(Get-NetIPInterface -InterfaceIndex $adapter.ifIndex -AddressFamily IPv4 | Select-Object Dhcp,InterfaceMetric)} |
        ConvertTo-Json -Depth 5 | Set-Content -LiteralPath $backup -Encoding utf8
    if($before.IPAddress -notcontains '192.168.0.1'){
        $null=New-NetIPAddress -InterfaceIndex $adapter.ifIndex -IPAddress 192.168.0.1 -PrefixLength 24 -AddressFamily IPv4
    }
    $address=Get-NetIPAddress -InterfaceIndex $adapter.ifIndex -IPAddress 192.168.0.1 -ErrorAction Stop
    if($address.PrefixLength -ne 24){throw 'Unexpected prefix on existing address; no further changes made.'}
    $result.success=$true;$result.backup=$backup
} catch {
    $result.error=$_.Exception.Message
} finally {
    $result | ConvertTo-Json -Compress | Set-Content -LiteralPath $resultPath -Encoding utf8
}
if(-not $result.success){throw $result.error}
Write-Host 'Ethernet configured as 192.168.0.1/24. No default gateway or DNS was added; Wi-Fi was not changed.'
