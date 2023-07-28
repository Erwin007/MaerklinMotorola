#include <MaerklinMotorola.h>

#define DecoderAdress 22

// Soort decoder op 1 zetten
#define LocDecoder 1
#define WisselDecoder 0

// input pin moet interrupt pin zijn
#define INPUT_PIN 2

#define PWM_Pin 9 // Op ATmega328 
#define Forward_Direction_PIN 11
#define Reverse_Direction_PIN 12

byte Digitale_Functie_Pinnen[] = {13 , 8 , 7 , 6 , 5}; //Functie ; F0 ; F1 ; F2 ; F3

bool changed;
unsigned int Actual_Speed = 0 , Desired_Speed;
byte DirectionValue = 1 , ActualDirectionValue = 2;
byte MMChangeDirection , MMChangeDirectionOld = 0;
byte i;


MaerklinMotorola mm(INPUT_PIN);

void setup() 
{
  Serial.begin(2000000);

  for (i = 0 ; i < sizeof(Digitale_Functie_Pinnen) ; i++)
  {
    pinMode(Digitale_Functie_Pinnen[i], OUTPUT);
    digitalWrite (Digitale_Functie_Pinnen[i], LOW);
  }

  pinMode(Forward_Direction_PIN, OUTPUT);
  pinMode(Reverse_Direction_PIN, OUTPUT);
  pinMode(PWM_Pin, OUTPUT);
  
//  TCCR1A = 0b00000011; // 10bit
//  TCCR1B = 0b00000011; // x64 phase correct

  TCCR1A = 0b00000001; // 8bit
  TCCR1B = 0b00000100; // x256 phase correct

  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), isr, CHANGE);

}

void loop() 
{
  mm.Parse();
  MaerklinMotorolaData* Data = mm.GetData();


  if ((Data) && (Data->IsLoc == LocDecoder) && (Data->Address == DecoderAdress) ) 
  {

    Serial.print(Data->tm_package_delta); Serial.print(" ");
    if (Data->IsMM1)
    {
      for (int i = 1 ; i <= 4 ; i++) digitalWrite(Digitale_Functie_Pinnen[i], LOW);

      Serial.print(" MM1 ");
      if (Data->IsLoc) Serial.print(" Lokdecoder     ");
      if (Data->IsMagnet) Serial.print(" Magneetdecoder ");
      Serial.print(" Addr: "); Serial.print(Data->Address);
      Serial.print("- Speed: " + String(Data->Speed) + " - Func: " + String(Data->Function));
      Serial.print(" - ChangeDir: "); Serial.print(Data->ChangeDir);
      Serial.print(" - Stop: "); Serial.print(Data->Stop);
      Serial.print(" - Direction: "); Serial.print(DirectionValue);
      MMChangeDirection = Data->ChangeDir;
      if (MMChangeDirectionOld != MMChangeDirection)
      {
        if (MMChangeDirection == 1) DirectionValue = 3 - DirectionValue;
        MMChangeDirectionOld = MMChangeDirection ;
      }
    }

    if (Data->IsMM2)
    {
      Serial.print(" MM2 ");
      if (Data->IsLoc) Serial.print(" Lokdecoder     ");
      if (Data->IsMagnet) Serial.print(" Magneetdecoder ");
      Serial.print(" Addr: "); Serial.print(Data->Address);
      if (Data->IsFunctionRecord) Serial.print(" Function Record F" + String(Data->MM2FunctionIndex) + ": " + String(Data->IsMM2FunctionOn) + " - Speed: " + String(Data->Speed) + (" - Func: ") + String(Data->Function) + "               ");
      if (Data->IsSpeedRecord) Serial.print(" Speed Record          - Speed: " + String(Data->Speed) + " - Func: " + String(Data->Function) + " - Direction: " + String(Data->MM2Direction));

      if (Data->IsSpeedRecord)
      {
        DirectionValue = Data->MM2Direction; 
      }

      if (Data->IsFunctionRecord)
      {
        digitalWrite(Digitale_Functie_Pinnen[Data->MM2FunctionIndex], Data->IsMM2FunctionOn);
      } 
    }


    Desired_Speed = map(Data->Speed,0,14,0,255);
    
    if (ActualDirectionValue != DirectionValue) Desired_Speed = 0;

    if (Actual_Speed < Desired_Speed) Actual_Speed += max((0.04*(Desired_Speed-Actual_Speed)) , 1);
    if (Actual_Speed > Desired_Speed) Actual_Speed -= max((0.04*(Actual_Speed-Desired_Speed)) , 1);
    analogWrite(PWM_Pin,Actual_Speed);
    Serial.print("  " + String(Desired_Speed) + " " + String(Actual_Speed));

    if (Actual_Speed == 0)
    {    
      switch ( DirectionValue )
      {
        case 1:
             digitalWrite(Forward_Direction_PIN, HIGH);
             digitalWrite(Reverse_Direction_PIN, LOW);
             break;
        case 2:
             digitalWrite(Forward_Direction_PIN, LOW);
             digitalWrite(Reverse_Direction_PIN, HIGH);
             break;
        default:
             digitalWrite(Forward_Direction_PIN, LOW);
             digitalWrite(Reverse_Direction_PIN, LOW);
             break;
      }
      ActualDirectionValue = DirectionValue;
    }

    digitalWrite(Digitale_Functie_Pinnen[0], Data->Function);


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

//    for(int i=0;i<35;i++) 
//    {
//      Serial.print(Data->Timings[i]);
//      Serial.print(" ");
//    }
//    Serial.println();



    Serial.println();
  }
}

void isr() 
{
  mm.PinChange();
}
