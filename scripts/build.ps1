$ErrorActionPreference = 'Stop'
$Fqbn = 'esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app'
$Esp32Index = 'https://espressif.github.io/arduino-esp32/package_esp32_index.json'

if (-not (Get-Command arduino-cli -ErrorAction SilentlyContinue)) {
    throw 'Nie znaleziono arduino-cli. Zainstaluj Arduino CLI i uruchom skrypt ponownie.'
}

function Invoke-ArduinoCli([string[]]$Arguments) {
    & arduino-cli @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Polecenie arduino-cli zakończyło się błędem (kod $LASTEXITCODE)."
    }
}

Invoke-ArduinoCli @('core', 'update-index', '--additional-urls', $Esp32Index)
Invoke-ArduinoCli @('core', 'install', 'esp32:esp32@3.3.12', '--additional-urls', $Esp32Index)
Invoke-ArduinoCli @('lib', 'install', 'lvgl@9.6.0')
Invoke-ArduinoCli @(
    'compile', '--fqbn', $Fqbn,
    '--build-property', 'compiler.cpp.extra_flags=-DLV_CONF_SKIP -DLV_FONT_MONTSERRAT_16=1 -DLV_FONT_MONTSERRAT_20=1 -DLV_FONT_MONTSERRAT_24=1 -DLV_FONT_MONTSERRAT_28=1',
    '--build-property', 'compiler.c.extra_flags=-DLV_CONF_SKIP -DLV_FONT_MONTSERRAT_16=1 -DLV_FONT_MONTSERRAT_20=1 -DLV_FONT_MONTSERRAT_24=1 -DLV_FONT_MONTSERRAT_28=1',
    '--warnings', 'all', '--output-dir', 'dist', 'AgroSprayer'
)

Write-Host 'Gotowe. Pliki firmware są w katalogu dist/. Skrypt nie wgrywa ich automatycznie.'
