#Requirements
1. Napisać program na wybrany mikrokontroler STM32, który ze stałą częstotliwością 11Hz,
będzie odczytywać dane z wybranego cyfrowego czujnika (innego niż przerabiany na zajęciach) i wysyłać je w postaci tekstowej na UART.
Jednocześnie powinien mrygać diodą z częstotliwością 13Hz oraz sygnalizować wciśnięcie przycisku wiadomością na ten sam UART.
Oceniany będzie wynik jak i sposób jego otrzymania.

2. Szczegóły i szczególiki:
13Hz ma mieć cały okres mrygnięcia diody
11Hz ma być printowane na UART dane z czujnika, odczyt danych z czujnika moze ale nie musi mieć częstotliwość 11Hz, nie może mieć mniej niż 11Hz żeby były wypisywane aktualne dane
program ma byc napisany uzywajac RTOS, bedą tez akceptowane opcje bez
odczytywanie danych, pisanie na uart, itp. ma być ronione przez przerwania albo dma, zakaz używania HAL_MAX_DELAY
