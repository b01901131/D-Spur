
# We are using 2 arduino boards, Mega and Uno.
## Mega serves:
* 4 mecanum wheels
* Sonar system
* i2c communication with TK1

used pins:
```
PORTA: 29    28    27    26    25    24    23    22
       EN1   STEP1 EN2   STEP2 EN3   STEP3 EN4   STEP4

PORTC: 30    31    32    33    34    35    36    37
       DIR1  [     MS1    ]    DIR2  [     MS2    ]

PORTL: 42    43    44    45    46    47    48    49
       DIR3  [     MS3    ]    DIR4  [     MS4    ]

TIMER1 TIMER3 TIMER4 TIMER5

i2c: 20 21
```

## Uno serves:
* Robotic arm
* Laser emitter
* RS232 serial communication with TK1
used pins:
```

```
