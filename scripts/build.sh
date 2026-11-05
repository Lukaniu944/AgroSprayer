#!/usr/bin/env bash
set -euo pipefail

FQBN='esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app'
ESP32_INDEX='https://espressif.github.io/arduino-esp32/package_esp32_index.json'

if ! command -v arduino-cli >/dev/null 2>&1; then
  echo 'Nie znaleziono arduino-cli. Zainstaluj Arduino CLI i uruchom skrypt ponownie.' >&2
  exit 1
fi

arduino-cli core update-index --additional-urls "$ESP32_INDEX"
arduino-cli core install esp32:esp32@3.3.12 --additional-urls "$ESP32_INDEX"
arduino-cli lib install lvgl@9.6.0

arduino-cli compile --fqbn "$FQBN" \
  --build-property 'compiler.cpp.extra_flags=-DLV_CONF_SKIP -DLV_FONT_MONTSERRAT_16=1 -DLV_FONT_MONTSERRAT_20=1 -DLV_FONT_MONTSERRAT_24=1 -DLV_FONT_MONTSERRAT_28=1' \
  --build-property 'compiler.c.extra_flags=-DLV_CONF_SKIP -DLV_FONT_MONTSERRAT_16=1 -DLV_FONT_MONTSERRAT_20=1 -DLV_FONT_MONTSERRAT_24=1 -DLV_FONT_MONTSERRAT_28=1' \
  --warnings all --output-dir dist AgroSprayer

echo 'Gotowe. Pliki firmware są w katalogu dist/. Skrypt nie wgrywa ich automatycznie.'
