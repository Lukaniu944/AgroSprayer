# AgroSprayer

## Lokalna konfiguracja Wi-Fi

1. Skopiuj `AgroSprayer/config_local.example.h` do `AgroSprayer/config_local.h`.
2. Wpisz własne `WIFI_SSID` i `WIFI_PASSWORD` w lokalnym pliku.
3. Nie commituj `config_local.h`; plik jest wykluczony przez `.gitignore`.

Jeśli `config_local.h` nie istnieje, firmware skompiluje się z pustymi danymi Wi-Fi i nie połączy się z siecią. Artefakty CI są przeznaczone wyłącznie do testów na biurku (**BENCH-ONLY**).

## Ważne zabezpieczenie sterownika zaworu

GPIO43 i GPIO44 sterują wejściami IN1/IN2 mostka L298N, a jednocześnie są pinami UART0 TX/RX. Podczas startu, resetu i wgrywania programu na GPIO43 mogą pojawić się impulsy UART. **ENA mostka L298N musi być sprzętowo utrzymywane w stanie LOW (GND) podczas startu/resetu/flashowania**; nie polegaj wyłącznie na programowym ustawieniu GPIO. Przed podłączeniem silnika sprawdź wyjścia miernikiem i zapewnij niezależne odcięcie zasilania napędu.

## Ograniczenia testów

Zielona kompilacja i testy natywne nie zastępują testu fizycznego czujnika ciśnienia, krańcówek, polaryzacji zaworu i sześciu sekcji. Nie używaj automatycznej regulacji w polu przed tymi testami.
