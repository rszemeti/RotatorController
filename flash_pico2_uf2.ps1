$ErrorActionPreference = 'Stop'

$port = 'COM3'
$uf2 = 'c:\Users\Robin\Documents\GitHub\RotatorController\RotatorFirmware\.pio\build\pico2\firmware.uf2'

if (!(Test-Path $uf2)) {
    throw "UF2 not found: $uf2"
}

$sp = New-Object System.IO.Ports.SerialPort $port,1200,'None',8,'one'
$sp.DtrEnable = $true
$sp.Open()
Start-Sleep -Milliseconds 300
$sp.Close()

$drive = $null
for ($i = 0; $i -lt 30; $i++) {
    $v = Get-Volume | Where-Object { $_.FileSystemLabel -match 'RP2350|RPI-RP2|RP2|RPI' } | Select-Object -First 1
    if ($v -and $v.DriveLetter) {
        $drive = "$($v.DriveLetter):\"
        break
    }
    Start-Sleep -Milliseconds 500
}

if (-not $drive) {
    throw 'BOOT drive not detected after reset'
}

Copy-Item $uf2 $drive -Force
Write-Output "FLASH_COPIED_TO:$drive"
