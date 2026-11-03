# AgroSprayer — uruchomienie BENCH-ONLY na Waveshare ESP32-S3-Touch-LCD-7B

> **OSTRZEŻENIE: to jest wersja do prób na stole. Nie używać do oprysku w polu.**
> Aktualny kod steruje wejściami kierunku L298N na GPIO43/44. Firmware nie steruje niezależnie wejściem ENA. ENA musi być fizycznie utrzymane w stanie LOW/GND przez cały czas resetu, uruchamiania i programowania. Nie polegaj na samym firmware.

## 1. Pobranie właściwej gałęzi

Repozytorium: https://github.com/Lukaniu944/AgroSprayer
Gałąź testowa: `safety-interlocks-pressure-valve`

```bash
git clone https://github.com/Lukaniu944/AgroSprayer.git
cd AgroSprayer
git checkout safety-interlocks-pressure-valve
git pull
```

Otwórz `AgroSprayer/AgroSprayer.ino` w Arduino IDE. Nie kopiuj pojedynczych plików ze starszej wersji.

## 2. Przygotowanie stanowiska — przed podłączeniem USB

1. Odłącz silnik zaworu od L298N albo fizycznie odłącz zasilanie mostka.
2. Jeżeli L298N pozostaje podłączony: zewrzyj ENA do GND tak, aby mostek był sprzętowo wyłączony.
3. Nie podawaj 12 V na silnik podczas pierwszego uruchomienia ekranu.
4. Nie podłączaj jeszcze wyjść IN1/IN2 do płytki, jeżeli nie masz pewności co do wspólnej masy i poziomów napięć.
5. Użyj stabilnego zasilania USB do samej płytki. Nie zasilaj silnika z USB ani z 3,3 V płytki.

GPIO43/GPIO44 są używane przez firmware jako IN1/IN2 i na tej płytce są związane z UART0/USB-UART. Podczas resetu ROM może generować aktywność UART. Dlatego ENA musi pozostać sprzętowo wyłączone, a na pierwszy test najlepiej pozostawić silnik odłączony.

## 3. Ustawienia Arduino IDE

Wymagane środowisko zgodne z CI:
- pakiet płytek Espressif ESP32 **3.3.12**;
- biblioteka **LVGL 9.6.0**;
- konfiguracja LVGL zgodna z projektem (w CI używane jest `LV_CONF_SKIP` oraz fonty Montserrat 16/20/24/28);
- płytka: **ESP32S3 Dev Module**;
- PSRAM: **OPI PSRAM**;
- Partition Scheme: **Huge APP**.

Wybierz właściwy port COM odpowiadający CH343. Przed pierwszym wgraniem upewnij się, że kompilacja zakończyła się sukcesem. Nie wgrywaj pliku BIN z innej gałęzi ani starego commitu.

## 4. Pierwsze uruchomienie — bez napędu

1. Podłącz USB do Waveshare.
2. Wgraj firmware wyłącznie po potwierdzeniu, że pochodzi z aktualnego commitu tej gałęzi i że CI `esp32-build` zakończył się sukcesem.
3. Otwórz monitor portu szeregowego tylko wtedy, gdy konfiguracja UART nie koliduje z pinami zaworu; nie zmieniaj przełącznika UART w trakcie pracy.
4. Sprawdź, czy uruchamia się LCD, LVGL i dotyk.
5. Sprawdź przechodzenie między ekranami i ustawieniami.
6. Sprawdź odbiór AgOpenGPS dopiero po zakończeniu testu samego ekranu.
7. Nie uzbrajaj AUTO i nie uruchamiaj ręcznych impulsów podczas tego etapu.

## 5. Kolejność testów na stole

- **Etap A:** LCD, dotyk, menu i brak restartów — bez L298N i bez silnika.
- **Etap B:** czujnik ciśnienia z bezpiecznym sygnałem testowym w dozwolonym zakresie wejścia; nie podawaj 5 V bezpośrednio na wejście ADS1115.
- **Etap C:** komunikacja AgOpenGPS i sześć stanów sekcji — bez napędu.
- **Etap D:** dopiero po sprawdzeniu okablowania i niezależnego sprzętowego odcięcia ENA: test L298N z silnikiem odłączonym mechanicznie od zaworu, krótkie impulsy, łatwo dostępne odcięcie 12 V.
- **Etap E:** test zaworu i hydrauliki wymaga osobnej procedury, krańcówek, ograniczenia ciśnienia i obserwacji manometru. Nie jest objęty testem firmware.

Po każdym naciśnięciu STOP sprawdź, że nie pojawia się kolejny impuls AUTO. Każda nieoczekiwana aktywność wyjścia oznacza natychmiastowe odłączenie zasilania napędu.

## 6. Co musi być potwierdzone przed użyciem

- udana kompilacja ESP32-S3 + LVGL dla dokładnego commitu;
- sprzętowy stan bezpieczny ENA podczas resetu/bootu/programowania;
- zgodność GPIO43/44 z przełącznikiem UART i rzeczywistym okablowaniem;
- test STOP/AUTO i watchdog na stanowisku;
- test czujnika ciśnienia oraz odcięcia przy błędzie;
- poprawny kierunek zaworu, krańcówki i zachowanie hydrauliczne;
- kalibracja dysz i weryfikacja obliczeń dawki.

**Nie ma zgody na pracę polową na podstawie samego testu uruchomienia ekranu.** Dane dawki i zużycia są obliczane z ciśnienia oraz stanów sekcji, a nie mierzone przepływomierzem.