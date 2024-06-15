from enum import IntEnum

class DriveModeNum(IntEnum): 
    FOLLOWING   = 0
    STOP        = 1
    SEARCHING   = 2
    PAYMENT     = 3

class ProductNum(IntEnum):
    BRAVO = 0
    CHAMKKAERAMEN = 1
    CHAPSSALHOTTEONGMIX = 2
    CHOCOBI = 3