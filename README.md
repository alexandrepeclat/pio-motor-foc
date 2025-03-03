### Hardware

NodeMCU ESP32 | board | 5V | Pinout : https://mischianti.org/esp32-nodemcu-32s-esp-32s-kit-high-resolution-pinout-datasheet-and-specs/
                        Pins description (! not same layout as mine) : https://lastminuteengineers.com/esp32-pinout-reference/
GM4108H-120T | motor | 12-20V | Buy : https://fr.aliexpress.com/item/1005008389384811.html
                       Ref : https://shop.iflight.com/gimbal-motors-cat44/ipower-motor-gm4108h-120t-brushless-gimbal-motor-pro217?
AS5048A | magnetic encoder | 3.3-5V | SPI wiring : https://community.simplefoc.com/t/ipower-motor-and-spi-wiring/323/16
                             Ref : https://shop.iflight.com/as5048a-magnetic-encoder-pro262
Simple FOC Mini DRV8313 | 8-24V | motor driver | Buy : https://fr.aliexpress.com/item/1005007181079288.html


### Sucking errors
- Problème avec arrondis target motor (pas implémenté finalement) : https://community.simplefoc.com/t/motor-vibrates-on-achieving-angle-position/6658

### Mécanique
- Courroie GT2 (2mm step entre dents)
- Poulie GT2 20 dents
- 1 tour de moteur = 1 tour de poulie = 20 dents * 2mm = 40mm