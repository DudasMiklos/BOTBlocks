#include "BOTBlocks.h"

Blocks board;

void setup()
{
  Serial.begin(9600);
  board.setconnectormode(BB_HEADER_4, BB_X);
  board.begin(BB_UNO);
  board.addsetmodulmode(BB_HEADER_4, BB_SHIFT_COUNT_1, BB_SHIFT);
  board.printaddsetmodulemode();
  board.pinsinit();
  delay(1000);

}
void loop()
{
  for (int i = 0; i < 10; i++) {
    board.dispnumber(i, BB_SHIFT_1);
    delay(500);
  }
  board.dispclear(BB_SHIFT_1);
}
