#ifndef BOTBlocks_h
#define BOTBlocks_h

#include <Arduino.h>
#define IO_REG_TYPE uint8_t


//Defines the connector mode options
#define BB_UN 0
#define BB_X 1
#define BB_I 2
#define BB_A 3
#define BB_P 4
#define BB_S 5

#define BB_HEADER_1 4
#define BB_HEADER_2 2
#define BB_HEADER_3 3
#define BB_HEADER_4 1

#define BB_BLOCK_1 1
#define BB_BLOCK_2 2
#define BB_BLOCK_3 3

#define BB_SHIFT_1 0
#define BB_SHIFT_2 8
#define BB_SHIFT_3 16
#define BB_SHIFT_4 24

#define BB_SHIFT_COUNT_1 8
#define BB_SHIFT_COUNT_2 16
#define BB_SHIFT_COUNT_3 24
#define BB_SHIFT_COUNT_4 32
#define BB_SHIFT_COUNT_5 40
//Defines the board types
#define BB_UNO 0
#define BB_MEGA 1
#define BB_WEMOS 2

#define BB_UNUSED 0
#define BB_LED 1
#define BB_BUTTON 2
#define BB_ANALOG 2
#define BB_SHIFT 3
#define BB_DS18B20 4

#define BB_TEST 5
// 1 OUTPUT
// 2 INPUT

#include "Arduino.h"

class Blocks
{
public:
  Blocks();
  void begin(uint8_t mcu);
  void setconnectormode(uint8_t connector, uint8_t mode);
  void addsetmodulmode(uint8_t connecotr, uint8_t place, uint8_t pinmode);
  void pinsinit();
  void printaddsetmodulemode();

  void ledon(uint8_t connector, uint8_t place);
  void ledoff(uint8_t connector, uint8_t place);

  void ledpwm(uint8_t connector, uint8_t place , uint8_t value);
  bool readbutton(uint8_t connector, uint8_t place);
  uint16_t readanalog(uint8_t connector, uint8_t place);
  void writeBit(uint8_t bitnum, bool value);
  void setBitCount(uint8_t connector, uint8_t bitcount);


  void dispnumber(uint8_t number, uint8_t modulenum);
  void dispclear(uint8_t modulenum);

  uint8_t search(uint8_t number, uint8_t place);
  uint8_t searchpin(uint8_t number, uint8_t place);

private:
  uint8_t connector_mode [4] = {0,0,0,0};
  uint8_t mcu_type;
  uint8_t used_pines [20];

  uint8_t shifreg_pines[3];
  uint8_t byteCount;
  byte writeBuffer[16];

  typedef struct
  {
    uint8_t Connector_Num;
    uint8_t Pin_Num [7];

  } Connector_Typetable;

  Connector_Typetable Connector[4];

private:
  uint8_t bitmask;
  volatile IO_REG_TYPE *baseReg;

  unsigned char ROM_NO[8];
  uint8_t LastDiscrepancy;
  uint8_t LastFamilyDiscrepancy;
  bool LastDeviceFlag;

public:
  void startds(uint8_t pin);

  uint8_t DS_reset(void);//used
  void DS_select(const uint8_t rom[8]);//used
  void DS_write(uint8_t v, uint8_t power = 0);//used
  uint8_t DS_read(void); //used
  void DS_write_bit(uint8_t v); //usedincpp
  uint8_t DS_read_bit(void); //usedincpp
  void DS_reset_search(); //used
  bool DS_search(uint8_t *newAddr, bool search_mode = true);  //used

  float readtemps();

};
#endif
