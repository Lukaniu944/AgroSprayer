# AgroSprayer — próba na stole (PL)

> **BENCH-ONLY.** Ta instrukcja nie dopuszcza pracy w polu. Na obecnym stanie repozytorium nie ma potwierdzonego, świeżo zbudowanego pliku BIN dla aktualnego commitu. Najpierw kompiluj z kodu i sprawdź, czy kompilacja przechodzi.

## 1. Bezpieczny stan przed podłączeniem

1. Odłącz zasilanie silnika/zaworu 12 V od L298N. Na pierwsze uruchomienie firmware pozostaw samą płytkę ESP32-S3 z USB.
2. **ENA L298N utrzymuj fizycznie na GND** podczas resetu, bootowania i wgrywania programu. Nie polegaj na samych GPIO43/44 — są także liniami UART0, a ROM ESP32-S3 może nadawać przed `setup()`.
3. Ustaw przełącznik UART płytki zgodnie z dokumentacją Waveshare dla użycia zewnętrznego złącza UART (UART2); nie uruchamiaj monitora szeregowego na GPIO43/44.
4. Nie podłączaj jeszcze silnika ani hydrauliki. Najpierw potwierdź start UI, ciśnienie jako wartość testową/bez czujnika tylko zgodnie z procedurą programu, stan STOP i brak samoczynnego ruchu.

## 2. Kompilacja z Arduino IDE

Ustawienia zgodne z workflow projektu:

- Płytka: **ESP32S3 Dev Module**
- Pakiet Arduino-ESP32: **3.3.12**
- PSRAM: **OPI PSRAM**
- Partition Scheme: **Huge APP**
- Biblioteka LVGL: **9.6.0**

Otwórz folder projektu i plik `AgroSprayer/AgroSprayer.ino` (zachowaj folder i nazwy plików). Jeśli IDE zgłasza brak bibliotek, instaluj wymagane biblioteki wskazane przez błędy/projekt; nie zamieniaj wersji LVGL na 8.x.

W CI kompilacja wykonywana jest poleceniem Arduino CLI dla FQBN `esp32:esp32:esp32s3:PSRAM=opi,PartitionScheme=huge_app` i definiuje `LV_CONF_SKIP` oraz fonty Montserrat 16/20/24/28. Jeśli IDE oferuje te same opcje, ustaw je analogicznie.

1. Kliknij **Weryfikuj**.
2. Jeśli kompilacja przejdzie, wybierz port USB CH343 i kliknij **Wgraj**.
3. Nie wgrywaj pliku BIN z innej rewizji ani z niezweryfikowanego źródła.

## 3. Pierwszy test po wgraniu

1. Nadal trzymaj ENA na GND i odłącz zasilanie silnika.
2. Zresetuj płytkę. Sprawdź, czy ekran uruchamia się i nie restartuje.
3. Sprawdź ekran STOP / tryby. AUTO nie może uzbroić się samo po starcie.
4. Jeżeli ekran ciśnienia pokazuje błąd czujnika, nie omijaj go w celu uruchomienia zaworu.
5. Zapisz zdjęcie ekranu i wynik kompilacji oraz prześlij je do rozmowy.

## 4. Dopiero później — napęd

Nie zwalniaj ENA i nie podłączaj silnika, dopóki nie potwierdzimy:
- prawidłowego, niezależnego wyłączenia ENA podczas boot/reset/flash;
- STOP zatrzymuje aktywny impuls i blokuje następny;
- kierunek OPEN/CLOSE jest poprawny przy krótkim impulsie;
- ograniczenie czasu impulsu i czujnik ciśnienia działają;
- zasilanie silnika, masa i zabezpieczenie prądowe są poprawne.

**Uwaga:** ustawienie IN1/IN2 na LOW w firmware nie jest sprzętowym zabezpieczeniem ENA. Nie wykonuj prób z podłączonym opryskiwaczem ani w polu.
