# AgroSprayer

Projekt firmware dla panelu AgroSprayer na ESP32-S3. Źródła Arduino są już zebrane w folderze `AgroSprayer/`; poniższe skrypty przygotowują zależności i budują firmware. Samo zbudowanie nie oznacza, że urządzenie jest gotowe do pracy z opryskiem — wymagane są właściwa płytka, ekran, czujniki i sprawdzenie okablowania.

## Szybki start — kompilacja

Wymagany jest zainstalowany [Arduino CLI](https://arduino.github.io/arduino-cli/). W katalogu głównym repozytorium uruchom:

- **Linux/macOS:** `bash scripts/build.sh`
- **Windows PowerShell:** `powershell -ExecutionPolicy Bypass -File .\scripts\build.ps1`

Skrypt instaluje wymagany pakiet ESP32 oraz bibliotekę LVGL w wersjach używanych przez projekt, a następnie kompiluje szkic. Pliki wynikowe pojawią się w `dist/`. Pierwsza kompilacja może potrwać kilka minut.

Ręczne ustawienia używane podczas kompilacji:

- Płytka: `esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app` (ESP32-S3, PSRAM OPI, partycja `huge_app`)
- Pakiet ESP32: `3.3.12`
- Biblioteka LVGL: `9.6.0`
- Opcje LVGL i pełne polecenie kompilacji są zapisane w `scripts/build.sh` oraz `scripts/build.ps1`.

## Wgranie na urządzenie

1. Podłącz właściwą płytkę ESP32-S3 i sprawdź jej port szeregowy (np. `COM5` w Windows albo `/dev/ttyACM0` w Linux).
2. Przed wgrywaniem odłącz/wyłącz napęd zaworu. Zgodnie z instrukcją projektu wejście ENA mostka L298N ma być przy GND podczas uruchamiania, resetowania i wgrywania.
3. Wgraj wcześniej zbudowany obraz, podstawiając swój port:

   ```sh
   arduino-cli upload --fqbn 'esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app' --port COM5 --input-dir dist AgroSprayer
   ```

   W Linux/macOS zamień `COM5` na port urządzenia. Jeśli Twoja płytka lub ekran wymaga innych ustawień, nie wgrywaj firmware, dopóki nie potwierdzisz ich z konfiguracją sprzętu.

## Konfiguracja Wi-Fi

Przed użyciem sprawdź `AgroSprayer/config.h` i ustaw własne dane Wi-Fi. Nie publikuj haseł ani prawdziwych danych dostępowych w repozytorium. Jeżeli dane, które były wcześniej w tym pliku, są prawdziwe, zmień hasło.

## Testy logiki na komputerze

Testy niezależne od ESP32 można uruchomić z katalogu głównego, mając kompilator `g++`:

```sh
g++ -std=c++17 -Wall -Wextra -Werror -pedantic tests/test_pressure_control.cpp -o /tmp/agrosprayer-pressure-test
/tmp/agrosprayer-pressure-test
g++ -std=c++17 -Wall -Wextra -Werror -pedantic tests/test_section_volume.cpp -o /tmp/agrosprayer-section-volume-test
/tmp/agrosprayer-section-volume-test
```

## Ważne ograniczenia i bezpieczeństwo

- To firmware do konkretnej konfiguracji sprzętowej, a nie program desktopowy. Do działania potrzebne jest zgodne urządzenie ESP32-S3 i osprzęt opisany w `AgroSprayer/board_config.h`.
- Kompilacja nie potwierdza poprawności montażu, czujników, kierunku zaworu ani działania opryskiwacza. Najpierw wykonaj testy na stanowisku, bez aktywnego oprysku i zgodnie z instrukcją `AgroSprayer/INSTRUKCJA.txt`.
- Automatyczny regulator ciśnienia jest funkcją sterującą napędem. Pozostaw go wyłączonego do czasu sprawdzenia ręcznie kierunku zaworu, krańcówek, STOP i awaryjnego odcięcia zasilania. Nie uruchamiaj go bez nadzoru ani na niezweryfikowanym układzie.
- Wskazania dawki i zużycia są obliczeniami z modelu ciśnienia/dysz, a nie bezpośrednim pomiarem wypływu. Sprawdź instrukcję przed poleganiem na wskazaniach.

Szczegóły obsługi i ograniczeń modelu znajdują się w `AgroSprayer/INSTRUKCJA.txt` oraz `AgroSprayer/README.txt`.
