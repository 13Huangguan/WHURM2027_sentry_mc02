# Flash the built firmware to the target via ST-Link + STM32CubeProgrammer.
# Usage:  .\flash.ps1
$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
$hex = Join-Path $projectRoot "build\LuojiaFox_H7_Template.hex"

if (-not (Test-Path $hex)) {
    Write-Host "Firmware not found: $hex" -ForegroundColor Red
    Write-Host "Run 'make -j8' first." -ForegroundColor Yellow
    exit 1
}

$paths = @(
    $env:STM32_PROGRAMMER_CLI,
    "C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
    "C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)

$candidates = @($paths | Where-Object { $_ -and (Test-Path $_) })

if (-not $candidates) {
    Write-Host "STM32_Programmer_CLI.exe not found." -ForegroundColor Red
    Write-Host "Install STM32CubeProgrammer, or set STM32_PROGRAMMER_CLI to its full path." -ForegroundColor Yellow
    exit 1
}

$cli = $candidates[0]
Write-Host "Target : $hex"
Write-Host "CLI    : $cli"
& $cli -c port=SWD mode=UR -w $hex -v -rst
exit $LASTEXITCODE
