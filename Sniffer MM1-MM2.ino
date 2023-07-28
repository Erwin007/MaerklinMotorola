#include <MaerklinMotorola.h>

// input pin moet interrupt pin zijn
#define INPUT_PIN 2
MaerklinMotorola mm(INPUT_PIN);

void setup() 
{
  Serial.begin(2000000);

  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), isr, CHANGE);

}

void loop() 
{
  mm.Parse();
  MaerklinMotorolaData* Data = mm.GetData();

  if (Data) 
  {
    Serial.print(Data->tm_package_delta); Serial.print(" ");
    if (Data->IsMM1)
    {
      Serial.print(" MM1 ");
      if (Data->IsLoc) Serial.print(" Lokdecoder     ");
      if (Data->IsMagnet) Serial.print(" Magneetdecoder ");
      Serial.print(" Addr: "); Serial.print(Data->Address);
      Serial.print("- Speed: " + String(Data->Speed) + " - Func: " + String(Data->Function));
      Serial.print(" - ChangeDir: "); Serial.print(Data->ChangeDir);
      Serial.print(" - Stop: "); Serial.print(Data->Stop);
    }

    if (Data->IsMM2)
    {
      Serial.print(" MM2 ");
      if (Data->IsLoc) Serial.print(" Lokdecoder     ");
      if (Data->IsMagnet) Serial.print(" Magneetdecoder ");
      Serial.print(" Addr: "); Serial.print(Data->Address);
      if (Data->IsFunctionRecord) Serial.print(" Function Record F" + String(Data->MM2FunctionIndex) + ": " + String(Data->IsMM2FunctionOn) + " - Speed: " + String(Data->Speed) + (" - Func: ") + String(Data->Function) + "               ");
      if (Data->IsSpeedRecord) Serial.print(" Speed Record          - Speed: " + String(Data->Speed) + " - Func: " + String(Data->Function) + " - Direction: " + String(Data->MM2Direction));
    }

    Serial.print(" A: "); Serial.print((Data->BitsAG & 128) >>7);
    Serial.print(" E: "); Serial.print((Data->BitsAG & 64) >>6);
    Serial.print(" B: "); Serial.print((Data->BitsAG & 32) >>5);
    Serial.print(" F: "); Serial.print((Data->BitsAG & 16) >>4);
    Serial.print(" C: "); Serial.print((Data->BitsAG & 8) >>3);
    Serial.print(" G: "); Serial.print((Data->BitsAG & 4) >>2);
    Serial.print(" D: "); Serial.print((Data->BitsAG & 2) >>1);
    Serial.print(" H: "); Serial.print(Data->BitsAG & 1);

    for(int i=0;i<9;i++) 
    {
      Serial.print(Data->Trits[i]);
    }

//        for(int i=0;i<35;i++) 
//        {
//          Serial.print(Data->Timings[i]);
//          Serial.print(" ");
//        }
//        Serial.println();

    Serial.println();
  }
}

void isr() 
{
  mm.PinChange();
}
