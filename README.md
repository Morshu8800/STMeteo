# STMeteo
  Версия 1.0
  Метеостанция на базе микроконтроллера STM32G070xx (Cortex-M0+)
# Материалы:
  - Stm32G070xx (Минимум 128КБ флеш-памяти и 6КБ оперативной памяти);
  - Дисплей на контроллере ST7567R (GMG-12864 ver 2.0);
  - EEPROM или FRAM память от 4КБит;
  - Датчик температуры и влажности SHT3x;
  - Датчик температуры и давления BMP280;
  - Модуль часов реального времени DS3231mini;
  - Зарядный модуль на TP4056 и разъём Type-C в корпусе с проводами;
  - Buck-Boost преобразователь на микросхеме hx4002-3.3;
  - Пассивный пьезодинамик от 3-12В;
  - Кнопка тактовая 4 шт;
  - Корпус (любой);
  - Аккумулятор (чем больше, тем лучше).
# Зачем нужен этот проект?
  - Для тех, кто следит за атмосферным давлением и комфортом в комнате/помещении;
  - Для моего опыта в разработке на мк от ST Microelectronics (Stm32)
# Проблемы и будущие доработки
  -Проблемы:
  - Случайное зависание прибора (может зависнуть при выходе из спящего режима с помощью кнопки, либо при проверке будильника) - Ответ: добаить кнопку жёсткого сброса, переработать/оптимизировать код/добавить Watchdog (WDT).

  -Доработки:
  - Пропуск будильника (т.е. при срабатывании будильника можно выбрать: "выключить его или оставить на следующие сутки").
  - Вывод времени срабатываемого будильника (т.е. "Будильник1" на "Будильник xx:xx", где xx - часы и минуты срабатываемого будильника).
# Пример готового проекта
![Sample by Morshu8800 ](https://github.com/Morshu8800/STMeteo/blob/main/Sample.png)
