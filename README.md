# ServoPoint_675
DIY project of simple **[DCC](https://en.wikipedia.org/wiki/Digital_Command_Control) Accessory decoder** for servo controlled point.

This project is based on [ServoPoint by Paco Cańada](http://usuaris.tinet.cat/fmco/home_en.htm) (with optocoupler)

The goal of the project is to improve the device by adding 2 configuration buttons. For this purpose, instead of the PIC12F629 processor, the **Microchip  PIC12F675** processor is used, whose analog input supports additional buttons.

* Platform:  Microchip PIC12F675
* Software Tools: MPLAB X IDE (mpasmx)
* EDA software: KiCad

# Project Details


### Build Status

>success

### Versioning

version 1.0.1  (15.06.2017)  initial



## Project Usage

### Build

To build the ServoPoint_675 project:

```
$ git clone https://github.com/NeonDevel/ServoPoint_675.git
$ cd ServoPoint_675/src
$./make_12F675.BAT
```
## License

MIT License

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND.