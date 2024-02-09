# SBER_STROB_PCB
This repository includes Hardware and software documentation for stroboscope creation

Проект включает в себя:
1) Прошивка микроконтроллера в папке bin
2) Проект с исходным кодом для микроконтроллера в папке Sber_Stroboscope_pcb_v1
3) Проект в Altium Designer
4) Электрическую схему Schematic_Sber_Stroboscope_v1_01_001.pdf
5) Функциональную схему Functional_Schematic_v1.pdf
6) Папку Stroboscobe commands с примерами работы устройства

Инструкция по установке для Windows и Linux
1. Установите STM32CubeProgrammer для Windows или Linux по ссылке https://www.st.com/en/development-tools/stm32cubeprog.html
2. Подсоедините контакты разъема J4 (3v3,GND,SWDIO,SWCLK) к соответствующим контактам на программаторе ST-LINK
3. Нажмите Connect в программе STM32 Programmer, затем выберите файл прошивки в папке BIN и нажмите Start Programming
4. Отключите программатор от платы и компьютерв и подключитесь к компьютеру с помощью USB кабеля через разъем Micro-USB на плате
5. Посмотрите в диспетчере устройств во вкладке "Порты" номер COM port подключенной платы
6. Для Windows установите Termite 3.4 по ссылке https://termite.software.informer.com/download/#downloading Для ОС на Linux выполните команду sudo apt-get install putty, она установит аналогичную программу для работы с консолью через последовательный порт
7. Запустите программу и натсройте в ней нужный номер COM порта, а также скорость передачи 230400 бод
8. нажмите подсоединиться и нажмите кнопку перезагрузки на устройстве, появится меню работы стробоскопа в консоли

