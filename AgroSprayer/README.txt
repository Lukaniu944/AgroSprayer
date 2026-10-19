PELNY PROJEKT AgroSprayer - profil cisnieniowy 11003 / 04RSM

1. Rozpakuj archiwum. Folder AgroSprayer zawiera caly kod projektu.
2. Otworz AgroSprayer/AgroSprayer.ino w Arduino IDE.
3. Korzystaj z dotychczasowego dzialajacego ustawienia plytki ESP32S3 Dev Module, PSRAM i partycji dla Twojego Waveshare 7B.
4. Wymagana biblioteka LVGL 9.6 oraz pakiet plytek ESP32 (w projekcie dotychczas uzywany 3.3.12). Zachowaj dzialajacy plik konfiguracji lv_conf.h z Twojej instalacji LVGL; biblioteki i pakiet plytek nie sa czescia archiwum. W konfiguracji LVGL musi byc dostepna czcionka Montserrat 16.
5. Sprawdz ustawienia Wi-Fi w config.h przed wgraniem.
6. Po wgraniu wybierz koncowki i ciecz, ustaw ponownie zbiornik oraz rozpocznij nowy zabieg.

To komplet zrodel odpowiadajacy podgladowi z profilami koncowek, nie plik BIN. Nie wykonano pelnej kompilacji Arduino/ESP32 w tej sesji.
Automatyczne poruszanie zaworem wedlug cisnienia nie jest wdrozone. AUTO sekcji to osobna funkcja.
ENA mostka L298N utrzymuj przy GND podczas uruchamiania, resetowania i wgrywania. Dopiero po starcie programu wlacz naped do recznej proby.
Szczegoly modelu i obslugi: INSTRUKCJA.txt.
