
# Our arduino Mega are serving:
* 4 mecanum wheels
* Sonar system
* i2c communication with TK1
* Robotic arm
* Laser emitter

# Pin usage
mecanum wheels
``` 
PORTA: 29    28    27    26    25    24    23    22
       DIR1  STEP1 DIR2  STEP2 DIR3  STEP3 DIR4  STEP4

PORTC: 30    31    32    33    34    35    36    37
       EN1   [     MS1    ]    EN2   [     MS2    ]

PORTL: 42    43    44    45    46    47    48    49
       EN3   [     MS3    ]    EN4   [     MS4    ]

TIMER3: 2    3     5
```

wheels | 22~29, 30~37, 42~49, 2,3,5

sonar | not yet
Content in the first column | Content in the second column
