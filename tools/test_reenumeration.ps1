# Regression test for the "Code 10 after host reboot" bug: repeatedly disables and
# re-enables the Beyondex USB composite device (forcing a full re-enumeration and a
# fresh SET_CONFIGURATION without power-cycling the device) and checks it recovers.
#
# Run from an ELEVATED PowerShell with the Beyondex plugged in and working:
#   powershell -ExecutionPolicy Bypass -File tools\test_reenumeration.ps1 [-Cycles 5] [-NoAudio]
#
# Pre-fix firmware (<= v1.4.2) fails on cycle 1 with problem code 10.
param([int]$Cycles = 5, [switch]$NoAudio)

$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) { Write-Host 'Run this from an elevated (Run as administrator) PowerShell.' -ForegroundColor Red; exit 1 }

# Find the first present Beyondex by VID/PID (any serial).
$dev = Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.InstanceId -like 'USB\VID_CAFE&PID_4030\*' } | Select-Object -First 1
if (-not $dev) { Write-Host 'No Beyondex found.' -ForegroundColor Red; exit 1 }
$root = $dev.InstanceId
$rev = ((Get-PnpDeviceProperty -InstanceId $root -KeyName DEVPKEY_Device_HardwareIds).Data[0] -split '&')[2]
Write-Host "Device $root ($rev), status $($dev.Status)"
if ($dev.Status -ne 'OK') { Write-Host 'Device is not working right now. Replug it and rerun.' -ForegroundColor Yellow; exit 1 }

$tone = Join-Path $env:TEMP 'bx_reenum_tone.wav'
if (-not (Test-Path $tone)) {
  $rate = 48000; $n = $rate * 1; $ms = New-Object IO.MemoryStream; $w = New-Object IO.BinaryWriter($ms)
  $w.Write([Text.Encoding]::ASCII.GetBytes('RIFF')); $w.Write([int32](36 + $n*4)); $w.Write([Text.Encoding]::ASCII.GetBytes('WAVEfmt '))
  $w.Write([int32]16); $w.Write([int16]1); $w.Write([int16]2); $w.Write([int32]$rate); $w.Write([int32]($rate*4)); $w.Write([int16]4); $w.Write([int16]16)
  $w.Write([Text.Encoding]::ASCII.GetBytes('data')); $w.Write([int32]($n*4))
  for ($i = 0; $i -lt $n; $i++) { $s = [int16](0.15*32767*[Math]::Sin(2*[Math]::PI*440*$i/$rate)); $w.Write($s); $w.Write($s) }
  [IO.File]::WriteAllBytes($tone, $ms.ToArray())
}

function Wait-Status($want, $seconds) {
  $deadline = (Get-Date).AddSeconds($seconds)
  do { $d = Get-PnpDevice -InstanceId $root -ErrorAction SilentlyContinue; if ($d -and $d.Status -eq $want) { return $true }; Start-Sleep -Milliseconds 500 } while ((Get-Date) -lt $deadline)
  return $false
}

$pass = 0
for ($c = 1; $c -le $Cycles; $c++) {
  if (-not $NoAudio) { (New-Object System.Media.SoundPlayer $tone).PlaySync() }
  pnputil /disable-device "$root" | Out-Null
  Start-Sleep -Seconds 2
  pnputil /enable-device "$root" | Out-Null
  $t0 = Get-Date
  $ok = Wait-Status 'OK' 15
  $code = (Get-PnpDeviceProperty -InstanceId $root -KeyName DEVPKEY_Device_ProblemCode -ErrorAction SilentlyContinue).Data
  # The audio endpoint (MMDEVAPI) is created a few seconds after the composite device starts.
  $audio = $false; $deadline = (Get-Date).AddSeconds(20)
  while ($ok -and -not $audio -and (Get-Date) -lt $deadline) {
    $audio = [bool](Get-PnpDevice -PresentOnly -ErrorAction SilentlyContinue | Where-Object { $_.Class -eq 'AudioEndpoint' -and $_.FriendlyName -match 'Beyondex' -and $_.Status -eq 'OK' })
    if (-not $audio) { Start-Sleep -Milliseconds 500 }
  }
  $secs = ((Get-Date) - $t0).TotalSeconds
  if ($ok -and $audio) { $pass++; Write-Host ("cycle {0}: OK (audio endpoint back after {1:N1}s)" -f $c, $secs) -ForegroundColor Green }
  else { Write-Host ("cycle {0}: FAILED (status ok={1}, problem code {2}, audio endpoint={3})" -f $c, $ok, $code, $audio) -ForegroundColor Red; break }
  Start-Sleep -Seconds 2
}
Write-Host "`nResult: $pass / $Cycles cycles passed"
if ($pass -eq $Cycles) { exit 0 } else { exit 1 }
