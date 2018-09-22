#include <EEPROM.h>               // include the EEPROM library

//////////////////////////////////////////////////
//
//
//
//////////////////////////////////////////////////

float getEEPROM() 
{
  float storedVal = 0;
 
  EEPROM.get(0, storedVal);
  Serial.print("EEPROM retrieved: ");
  
  Serial.println(storedVal);
  return storedVal;
}

//////////////////////////////////////////////////
//
//
//
//////////////////////////////////////////////////

int checkEEPROM() 
{
  int gotEEPROM = 0;
  // now check to see if CF is saved in EEPROM (addresses 0 - 3)
  for (int i = 0; i < 4; i++)
  {
    
    if (EEPROM.read(i) != 255)
    {
      
      gotEEPROM = 1;  // address not empty, so raise flag
    }
  } // end of loop checking EEPROM addresses 0-3

  return gotEEPROM;
}

//////////////////////////////////////////////////
//
//
//
//////////////////////////////////////////////////

void updateEEPROM(float pVal)
{
  Serial.print("Write ");
  Serial.println(pVal);
  EEPROM.put(0, pVal);
 //getEEPROM();
}

//////////////////////////////////////////////////
//
//
//
//////////////////////////////////////////////////

void eraseEEPROM()
{
    Serial.println("FactoryReset");
    updateEEPROM(1.000);
    // if pin set to GND factory reset the eeprom by setting non-zero values.
   // for (int i = 0 ; i < EEPROM.length() ; i++) 
   // {
   //   EEPROM.write(i, 1);
   // }
}

