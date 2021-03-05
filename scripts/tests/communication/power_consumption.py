import logging
import time
from ina219 import INA219

SHUNT_OHMS = 0.1
MAX_EXPECTED_AMPS = 0.2




def read():
    ina1 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
    ina1.configure(ina1.RANGE_16V, ina1.GAIN_AUTO)
    
    #print("ici")
    
    #print("INA1 ==============")
    #print("Bus Voltage    : %.3f V" % ina1.voltage())
    #print("Bus Current    : %.3f mA" % ina1.current())
    #print("Supply Voltage : %.3f V" % ina1.supply_voltage())
    #print("Shunt voltage  : %.3f mV" % ina1.shunt_voltage())
    print("Power          : %.3f mW" % ina1.power())
    #print("")
    
    print("")
    print("")
    print("")

    
    
    """
    ina2 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x45)
    ina2.configure(ina2.RANGE_16V, ina2.GAIN_AUTO)
    print("ici")
    
    print("INA2 ==============")
    print("Bus Voltage    : %.3f V" % ina2.voltage())
    print("Bus Current    : %.3f mA" % ina2.current())
    print("Supply Voltage : %.3f V" % ina2.supply_voltage())
    print("Shunt voltage  : %.3f mV" % ina2.shunt_voltage())
    print("Power          : %.3f mW" % ina2.power())
    
    """
    
def testTemps(nbLecture):
    ina1 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
    ina1.configure(ina1.RANGE_16V, ina1.GAIN_AUTO)
    
    #ina2 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x45)
    #ina2.configure(ina2.RANGE_16V, ina2.GAIN_AUTO)
    begin = time.time() 
    
    moyennePower1 = 0
    moyennePower2 = 0
    moyenneCurrent1 = 0
    moyenneVoltage1 = 0
    moyenneCurrent2 = 0
    moyenneVoltage2 = 0
    
    for i in range(0, nbLecture):
        moyenneCurrent1 += ina1.current()
        moyenneCurrent2 += ina1.current()
        moyenneVoltage1 += ina1.voltage()
        moyenneVoltage2 += ina1.voltage()
        moyennePower1 += ina1.power()
        moyennePower2 += ina1.power()
        

    moyenneCurrent1 = moyenneCurrent1 / nbLecture
    moyenneCurrent2 = moyenneCurrent2 / nbLecture
    moyenneVoltage1 = moyenneVoltage1 / nbLecture
    moyenneVoltage2 = moyenneVoltage2 / nbLecture
    moyennePower1 = moyennePower1 / nbLecture
    moyennePower2 = moyennePower2 / nbLecture
    
    
    end = time.time()
    
    print("Moyenne Current #1 ======= ")
    print(str(moyenneCurrent1) + "mA")
    print("")
    print("Moyenne Voltage #1 ======= ")
    print(str(moyenneVoltage1) + "V")
    print("")
    print("Moyenne Power #1 ======= ")
    print(str(moyennePower1) + "mW")
    print("")
    print("Moyenne Current #2 ======= ")
    print(str(moyenneCurrent2) + "mA")
    print("")
    print("Moyenne Voltage #2 ======= ")
    print(str(moyenneVoltage2) + "V")
    print("")
    print("Moyenne Power #2 ======= ")
    print(str(moyennePower2) + "mW")
    print("")
    print("Time used =======")
    print(end - begin) 
    
    
    
    
    
    
    
def moyenne():
    ina1 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x40)
    ina1.configure(ina1.RANGE_16V, ina1.GAIN_AUTO)
    
    ina2 = INA219(SHUNT_OHMS, MAX_EXPECTED_AMPS, address=0x45)
    ina2.configure(ina2.RANGE_16V, ina2.GAIN_AUTO)
    
    moyCurrent = 0
    moyTension = 0
    moyPower = 0
    
    begin = time.time() 
    for i in range(0, 90):
        moyCurrent += ina1.current()
        moyTension += ina2.voltage()
        moyPower += ina1.power()
        
        
        
    moyCurrent = moyCurrent/90
    moyTension = moyTension/90
    moyPower = moyPower/90
    
    end = time.time() 
    print("")
    print("Moyenne Current ======= ")
    print(str(moyCurrent) + "mA")
    print("Moyenne Tension ======= ")
    print(str(moyTension) + "V")
    print("Moyenne Power ======= ")
    print(str(moyPower) + "mW")
    print("")
    print("Time used =======")
    print(end - begin)
    return 3
    
        
        
    
    


if __name__ == "__main__":
    testTemps(100)
