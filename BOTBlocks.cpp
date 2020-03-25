#include "Arduino.h"
#include "BOTBlocks.h"

#define IO_REG_TYPE uint8_t
#define IO_REG_BASE_ATTR asm("r30")
#define IO_REG_MASK_ATTR

#define DIRECT_READ(base, mask)         (((*(base)) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   ((*((base)+1)) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  ((*((base)+1)) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    ((*((base)+2)) &= ~(mask))
#define DIRECT_WRITE_HIGH(base, mask)   ((*((base)+2)) |= (mask))

Blocks::Blocks(){
}

uint8_t Blocks::search(uint8_t number, uint8_t place){
  for (size_t i = 0; i <= 3; i++) {
    if (Connector[i].Connector_Num == number) {
      uint8_t hely = place-1;
      for (size_t y = 0; y <= (7 - hely+y); y++) {
        //Connector[i].Pin_Num[hely+y] != 100 && used_pines[Connector[i].Pin_Num[hely+y]] == 0
        if(Connector[i].Pin_Num[hely+y] != 100 && used_pines[Connector[i].Pin_Num[hely+y]] == 0){
          return Connector[i].Pin_Num[hely+y];
        }
      }
    }
  }
}

uint8_t Blocks::searchpin(uint8_t number, uint8_t place){
  return Connector[number-1].Pin_Num[place-1];
}

void Blocks::addsetmodulmode(uint8_t connecotr, uint8_t place, uint8_t pinmode) {
 if (pinmode == 1) {
    uint8_t pin = search(connecotr, place);
    used_pines[pin] = pinmode;
  }
  if (pinmode == 2) {
    uint8_t pin = search(connecotr, place);
    used_pines[pin] = pinmode;
  }
  if (pinmode == 4) {
    uint8_t pin = search(connecotr, place);
    used_pines[pin] = pinmode;
    startds(pin);
  }
  if (pinmode == 3) {
    uint8_t pin;
    pin = search(connecotr, 1);
    used_pines[pin] = pinmode;
    shifreg_pines [0] = pin;

    pin = search(connecotr, 2);
    used_pines[pin] = pinmode;
    shifreg_pines [1] = pin;

    pin = search(connecotr, 3);
    used_pines[pin] = pinmode;
    shifreg_pines [2] = pin;
  }
  setBitCount(connecotr, place);
}

void Blocks::printaddsetmodulemode(){
  for (size_t i = 0; i < 20; i++) {
    Serial.print(used_pines[i]);
    Serial.print("  ");
  }
}
void Blocks::pinsinit() {
  for (size_t i = 0; i < 20; i++) {
    if (used_pines[i] == 1 || used_pines[i] == 3) {
      pinMode(i, OUTPUT);///+1
    }

    if (used_pines[i] == 2) {
      pinMode(i, INPUT);///+1
    }
  }
}
void Blocks::setconnectormode(uint8_t connector, uint8_t mode) {
  connector_mode[connector-1] = mode;
}

void Blocks::ledon(uint8_t connector, uint8_t place) {
  uint8_t pin = searchpin(connector, place);
  digitalWrite(pin, HIGH);
}
void Blocks::ledoff(uint8_t connector, uint8_t place) {
  uint8_t pin = searchpin(connector, place);
  digitalWrite(pin, LOW);

}
void Blocks::ledpwm(uint8_t connector, uint8_t place , uint8_t value) {
  uint8_t pin = search(connector, place);
  analogWrite(pin, value);
}
bool Blocks::readbutton(uint8_t connector, uint8_t place) {
  uint8_t pin = searchpin(connector, place);
  Serial.print(pin);
  return digitalRead(pin);
}
uint16_t Blocks::readanalog(uint8_t connector, uint8_t place) {
  uint16_t pin = searchpin(connector, place);
  return analogRead(pin);
}

void Blocks::setBitCount(uint8_t connector, uint8_t bitcount) {
  byteCount = bitcount/8;
  for(uint8_t i = 0; i < byteCount; i++) {
    writeBuffer[i] = 0;
  }
}
void Blocks::writeBit(uint8_t bitnum, bool value) {

byte b = writeBuffer[(bitnum-1) / 8];
  bitWrite(b, (bitnum-1) % 8, value);
  writeBuffer[(bitnum-1) / 8] = b;

  digitalWrite(shifreg_pines[1], LOW);
  digitalWrite(shifreg_pines[2], LOW);

  for(uint8_t i = 0; i < byteCount; i++) {
    shiftOut(shifreg_pines[0], shifreg_pines[2], MSBFIRST, writeBuffer[i]);

  }
  digitalWrite(shifreg_pines[1], HIGH);
}

void Blocks::dispnumber(uint8_t number, uint8_t modulenum){
if (number == 0) {
  writeBit(1+modulenum, HIGH);
  writeBit(2+modulenum, HIGH);
  writeBit(3+modulenum, HIGH);
  writeBit(4+modulenum, HIGH);
  writeBit(5+modulenum, HIGH);
  writeBit(6+modulenum, HIGH);
  writeBit(7+modulenum, LOW);
  writeBit(8+modulenum, LOW);
} else if (number == 1) {
  writeBit(1+modulenum, LOW);
  writeBit(2+modulenum, HIGH);
  writeBit(3+modulenum, HIGH);
  writeBit(4+modulenum, LOW);
  writeBit(5+modulenum, LOW);
  writeBit(6+modulenum, LOW);
  writeBit(7+modulenum, LOW);
  writeBit(8+modulenum, LOW);
}else if (number == 2) {
  writeBit(1+modulenum, HIGH);
  writeBit(2+modulenum, HIGH);
  writeBit(3+modulenum, LOW);
  writeBit(4+modulenum, HIGH);
  writeBit(5+modulenum, HIGH);
  writeBit(6+modulenum, LOW);
  writeBit(7+modulenum, HIGH);
  writeBit(8+modulenum, LOW);
}else if (number == 3) {
  writeBit(1+modulenum, HIGH);
  writeBit(2+modulenum, HIGH);
  writeBit(3+modulenum, HIGH);
  writeBit(4+modulenum, HIGH);
  writeBit(5+modulenum, LOW);
  writeBit(6+modulenum, LOW);
  writeBit(7+modulenum, HIGH);
  writeBit(8+modulenum, LOW);
}else if (number == 4) {
  writeBit(1+modulenum, LOW);
  writeBit(2+modulenum, HIGH);
  writeBit(3+modulenum, HIGH);
  writeBit(4+modulenum, LOW);
  writeBit(5+modulenum, LOW);
  writeBit(6+modulenum, HIGH);
  writeBit(7+modulenum, HIGH);
  writeBit(8+modulenum, LOW);
}else if (number == 5) {
  writeBit(1+modulenum, HIGH);
  writeBit(2+modulenum, LOW);
  writeBit(3+modulenum, HIGH);
  writeBit(4+modulenum, HIGH);
  writeBit(5+modulenum, LOW);
  writeBit(6+modulenum, HIGH);
  writeBit(7+modulenum, HIGH);
  writeBit(8+modulenum, LOW);
}else if (number == 6) {
  writeBit(1+modulenum, HIGH);
  writeBit(2+modulenum, LOW);
  writeBit(3+modulenum, HIGH);
  writeBit(4+modulenum, HIGH);
  writeBit(5+modulenum, HIGH);
  writeBit(6+modulenum, HIGH);
  writeBit(7+modulenum, HIGH);
  writeBit(8+modulenum, LOW);
}else if (number == 7) {
  writeBit(1+modulenum, HIGH);
  writeBit(2+modulenum, HIGH);
  writeBit(3+modulenum, HIGH);
  writeBit(4+modulenum, LOW);
  writeBit(5+modulenum, LOW);
  writeBit(6+modulenum, LOW);
  writeBit(7+modulenum, LOW);
  writeBit(8+modulenum, LOW);
}else if (number == 8) {
  writeBit(1+modulenum, HIGH);
  writeBit(2+modulenum, HIGH);
  writeBit(3+modulenum, HIGH);
  writeBit(4+modulenum, HIGH);
  writeBit(5+modulenum, HIGH);
  writeBit(6+modulenum, HIGH);
  writeBit(7+modulenum, HIGH);
  writeBit(8+modulenum, LOW);
}else if (number == 9) {
  writeBit(1+modulenum, HIGH);
  writeBit(2+modulenum, HIGH);
  writeBit(3+modulenum, HIGH);
  writeBit(4+modulenum, HIGH);
  writeBit(5+modulenum, LOW);
  writeBit(6+modulenum, HIGH);
  writeBit(7+modulenum, HIGH);
  writeBit(8+modulenum, LOW);
}
}

void Blocks::dispclear(uint8_t modulenum){
  writeBit(1+modulenum, LOW);
  writeBit(2+modulenum, LOW);
  writeBit(3+modulenum, LOW);
  writeBit(4+modulenum, LOW);
  writeBit(5+modulenum, LOW);
  writeBit(6+modulenum, LOW);
  writeBit(7+modulenum, LOW);
  writeBit(8+modulenum, LOW);
}

void Blocks::begin(uint8_t mcu) {
mcu_type = mcu;

if (connector_mode[0] == 1) {
  Connector[0].Connector_Num = 1;
  Connector[0].Pin_Num[0] = 2;
  Connector[0].Pin_Num[1] = 4;
  Connector[0].Pin_Num[2] = 18;
  Connector[0].Pin_Num[3] = 100;
  Connector[0].Pin_Num[4] = 100;
  Connector[0].Pin_Num[5] = 100;
  Connector[0].Pin_Num[6] = 100;
}
if (connector_mode[0] == 2) {
  Connector[0].Connector_Num = 1;
  Connector[0].Pin_Num[0] = 2;
  Connector[0].Pin_Num[1] = 100;
  Connector[0].Pin_Num[2] = 100;
  Connector[0].Pin_Num[3] = 4;
  Connector[0].Pin_Num[4] = 100;
  Connector[0].Pin_Num[5] = 18;
  Connector[0].Pin_Num[6] = 19;
}

if (connector_mode[1] == 1) {
  Connector[1].Connector_Num = 2;
  Connector[1].Pin_Num[0] = 3;
  Connector[1].Pin_Num[1] = 7;
  Connector[1].Pin_Num[2] = 8;
  Connector[1].Pin_Num[3] = 100;
  Connector[1].Pin_Num[4] = 100;
  Connector[1].Pin_Num[5] = 100;
  Connector[1].Pin_Num[6] = 100;
}
if (connector_mode[1] == 5) {
  Connector[1].Connector_Num = 2;
  Connector[1].Pin_Num[0] = 3;
  Connector[1].Pin_Num[1] = 7;
  Connector[1].Pin_Num[2] = 8;
  Connector[1].Pin_Num[3] = 10;
  Connector[1].Pin_Num[4] = 11;
  Connector[1].Pin_Num[5] = 12;
  Connector[1].Pin_Num[6] = 13;
}

if (connector_mode[2] == 1) {
  Connector[2].Connector_Num = 3;
  Connector[2].Pin_Num[0] = 14;
  Connector[2].Pin_Num[1] = 15;
  Connector[2].Pin_Num[2] = 16;
  Connector[2].Pin_Num[3] = 100;
  Connector[2].Pin_Num[4] = 100;
  Connector[2].Pin_Num[5] = 100;
  Connector[2].Pin_Num[6] = 100;
}
if (connector_mode[2] == 3) {
  Connector[2].Connector_Num = 3;
  Connector[2].Pin_Num[0] = 14;
  Connector[2].Pin_Num[1] = 15;
  Connector[2].Pin_Num[2] = 16;
  Connector[2].Pin_Num[3] = 0;
  Connector[2].Pin_Num[4] = 100;
  Connector[2].Pin_Num[5] = 100;
  Connector[2].Pin_Num[6] = 100;
}

if (connector_mode[3] == 1) {
  Connector[3].Connector_Num = 4;
  Connector[3].Pin_Num[0] = 1;
  Connector[3].Pin_Num[1] = 6;
  Connector[3].Pin_Num[2] = 9;
  Connector[3].Pin_Num[3] = 100;
  Connector[3].Pin_Num[4] = 100;
  Connector[3].Pin_Num[5] = 100;
  Connector[3].Pin_Num[6] = 100;
}
if (connector_mode[3] == 4) {
  Connector[3].Connector_Num = 4;
  Connector[3].Pin_Num[0] = 1;
  Connector[3].Pin_Num[1] = 100;
  Connector[3].Pin_Num[2] = 100;
  Connector[3].Pin_Num[3] = 17;
  Connector[3].Pin_Num[4] = 5;
  Connector[3].Pin_Num[5] = 6;
  Connector[3].Pin_Num[6] = 3;
}
}

void Blocks::startds(uint8_t pin){
	pinMode(pin, INPUT);
	bitmask = (digitalPinToBitMask(pin));
	baseReg = (portInputRegister(digitalPinToPort(pin)));
	DS_reset_search();
}

uint8_t Blocks::DS_reset(void){
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t r;
	uint8_t retries = 125;

	noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);
	interrupts();
	// wait until the wire is high... just in case
	do {
		if (--retries == 0) return 0;
		delayMicroseconds(2);
	} while ( !DIRECT_READ(reg, mask));

	noInterrupts();
	DIRECT_WRITE_LOW(reg, mask);
	DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
	interrupts();
	delayMicroseconds(480);
	noInterrupts();
	DIRECT_MODE_INPUT(reg, mask);	// allow it to float
	delayMicroseconds(70);
	r = !DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(410);
	return r;
}

void Blocks::DS_write_bit(uint8_t v){
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;

	if (v & 1) {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(10);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(55);
	} else {
		noInterrupts();
		DIRECT_WRITE_LOW(reg, mask);
		DIRECT_MODE_OUTPUT(reg, mask);	// drive output low
		delayMicroseconds(65);
		DIRECT_WRITE_HIGH(reg, mask);	// drive output high
		interrupts();
		delayMicroseconds(5);
	}
}

uint8_t Blocks::DS_read_bit(void){
	IO_REG_TYPE mask IO_REG_MASK_ATTR = bitmask;
	volatile IO_REG_TYPE *reg IO_REG_BASE_ATTR = baseReg;
	uint8_t r;

	noInterrupts();
	DIRECT_MODE_OUTPUT(reg, mask);
	DIRECT_WRITE_LOW(reg, mask);
	delayMicroseconds(3);
	DIRECT_MODE_INPUT(reg, mask);	// let pin float, pull up will raise
	delayMicroseconds(10);
	r = DIRECT_READ(reg, mask);
	interrupts();
	delayMicroseconds(53);
	return r;
}

float Blocks::readtemps(){
byte present = 0;
byte data[12];
byte addr[8];
float celsius;

if ( !Blocks::DS_search(addr)) {
	Serial.println("No more addresses.");
	Blocks::DS_reset_search();
	return;
}

Blocks::DS_reset();
Blocks::DS_select(addr);
Blocks::DS_write(0x44, 1);

present = Blocks::DS_reset();
Blocks::DS_select(addr);
Blocks::DS_write(0xBE);         // Read Scratchpad

for (byte i = 0; i < 9; i++) {           // we need 9 bytes
	data[i] = Blocks::DS_read();
}

int16_t raw = (data[1] << 8) | data[0];

celsius = (float)raw / 16.0;
Serial.print("  Temperature = ");
Serial.print(celsius);
Serial.print(" Celsius");
}

void Blocks::DS_write(uint8_t v, uint8_t power /* = 0 */) {
    uint8_t bitMask;

    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	Blocks::DS_write_bit( (bitMask & v)?1:0);
    }
    if ( !power) {
	noInterrupts();
	DIRECT_MODE_INPUT(baseReg, bitmask);
	DIRECT_WRITE_LOW(baseReg, bitmask);
	interrupts();
    }
}

uint8_t Blocks::DS_read() {
    uint8_t bitMask;
    uint8_t r = 0;
    for (bitMask = 0x01; bitMask; bitMask <<= 1) {
	if ( Blocks::DS_read_bit()) r |= bitMask;
    }
    return r;
}

void Blocks::DS_select(const uint8_t rom[8]){
    uint8_t i;
    DS_write(0x55);
    for (i = 0; i < 8; i++) DS_write(rom[i]);
}

void Blocks::DS_reset_search(){
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--) {
    ROM_NO[i] = 0;
    if ( i == 0) break;
  }
}

bool Blocks::DS_search(uint8_t *newAddr, bool search_mode /* = true */){
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   bool    search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;

   if (!LastDeviceFlag) {
      if (!DS_reset()) {
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      if (search_mode == true) {
        DS_write(0xF0);   // NORMAL SEARCH
      } else {
        DS_write(0xEC);   // CONDITIONAL SEARCH
      }

      do
      {
         id_bit = DS_read_bit();
         cmp_id_bit = DS_read_bit();
         if ((id_bit == 1) && (cmp_id_bit == 1)) {
            break;
         } else {
            if (id_bit != cmp_id_bit) {
               search_direction = id_bit;  // bit write value for search
            } else {
               if (id_bit_number < LastDiscrepancy) {
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               } else {
                  search_direction = (id_bit_number == LastDiscrepancy);
               }
               if (search_direction == 0) {
                  last_zero = id_bit_number;
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;
            DS_write_bit(search_direction);
            id_bit_number++;
            rom_byte_mask <<= 1;
            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7
      if (!(id_bit_number < 65)) {
         LastDiscrepancy = last_zero;
         if (LastDiscrepancy == 0) {
            LastDeviceFlag = true;
         }
         search_result = true;
      }
   }
   if (!search_result || !ROM_NO[0]) {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
  }
