#include <MaerklinMotorola.h>

// For a real locdecoder uncomment line 4 and replace 22 by your loc adres
//#define DecoderAdress 22

// input pin moet interrupt pin zijn
#define INPUT_PIN 2
MaerklinMotorola mm(INPUT_PIN);

#define PWM_Pin 9 // Op ATmega328 aansturen motor met PWM 
#define Forward_Direction_PIN 11
#define Reverse_Direction_PIN 12

byte Digitale_Functie_Pinnen[] = {13 , 8 , 7 , 6 , 5}; //Functie ; F0 ; F1 ; F2 ; F3

bool changed;
unsigned int Actual_Speed = 0 , Desired_Speed;
byte DirectionValue = 1 , ActualDirectionValue = 255;
byte MMChangeDirection , MMChangeDirectionOld = 0;
byte i , DecoderAdress , Logging;

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

  if (LocDecoder)
  {
    do
    {
      Serial.print ("Geef het decoder adres: ");
      while (Serial.available() == 0) {};
      DecoderAdress = Serial.parseInt(SKIP_ALL, '\n');
      if ((DecoderAdress >= 1) && (DecoderAdress <= 79)) 
      {
        Serial.println(DecoderAdress);
      }
      else Serial.println("Verkeerde invoer"); 
    } while ((DecoderAdress <= 1) && (DecoderAdress > 79));

    while (Serial.available() > 0 ) {Serial.read();}

    do
    {
      Serial.print ("Logging nodig ? ( 1 = Ja / 2 = Nee) ");
      while (Serial.available() == 0) {};
      Logging = Serial.parseInt(SKIP_ALL, '\n');
      while (Serial.available() > 0 ) {Serial.read();}
      if (Logging == 1) Serial.println("Ja");
      if (Logging == 2) Serial.println("Nee");
      if ((Logging != 1) && (Logging != 2))  Serial.println("Verkeerde invoer"); 

    } while ((Logging != 1) && (Logging != 2));
  }
  attachInterrupt(digitalPinToInterrupt(INPUT_PIN), isr, CHANGE);

}

void loop() 
{
  mm.Parse();
  MaerklinMotorolaData* Data = mm.GetData();

  if (Data) 
  {
    if ( (Data->IsLoc == 1) && (Data->Address == DecoderAdress) )
    {
/******************************************************************/
/*                                                                */
/* Printing rail information for lokdecoder (can be deleted)      */
/*                                                                */
/******************************************************************/
      if (Logging == 1)
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
          Serial.print(" - Direction: "); Serial.print(DirectionValue);
          Serial.print(" - Desired speed: " + String(Desired_Speed) + " - Actual speed: " + String(Actual_Speed));
        }

        if (Data->IsMM2)
        {
          Serial.print(" MM2 ");
          if (Data->IsLoc) Serial.print(" Lokdecoder     ");
          if (Data->IsMagnet) Serial.print(" Magneetdecoder ");
          Serial.print(" Addr: "); Serial.print(Data->Address);
          if (Data->IsFunctionRecord) Serial.print(" Function Record F" + String(Data->MM2FunctionIndex) + ": " + String(Data->IsMM2FunctionOn) + " - Speed: " + String(Data->Speed) + (" - Func: ") + String(Data->Function) + "               ");
          if (Data->IsSpeedRecord) Serial.print(" Speed Record          - Speed: " + String(Data->Speed) + " - Func: " + String(Data->Function) + " - Direction: " + String(Data->MM2Direction));
          Serial.print(" - Desired speed: " + String(Desired_Speed) + " - Actual speed: " + String(Actual_Speed));
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

        Serial.print (" ");

        for(int i=0;i<35;i++) 
        {
          Serial.print(Data->Timings[i]);
          Serial.print(" ");
        }
        Serial.println();
      }

/******************************************************************/
/*                                                                */
/* Lok Decoder software                                           */
/*                                                                */
/******************************************************************/

      if (Data->IsMM1)
      {
        for (int i = 1 ; i <= 4 ; i++) digitalWrite(Digitale_Functie_Pinnen[i], LOW);

        MMChangeDirection = Data->ChangeDir;
        if (MMChangeDirectionOld != MMChangeDirection)
        {
          if (MMChangeDirection == 1) DirectionValue = 3 - DirectionValue;
          MMChangeDirectionOld = MMChangeDirection ;
        }
      }

      if (Data->IsMM2)
      {
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
    }

    if ( (Data->IsMagnet == 1) && (Data->Address == DecoderAdress) )
    {
/******************************************************************/
/*                                                                */
/* Printing rail information for wisseldecoder (can be deleted)   */
/*                                                                */
/******************************************************************/
      if (Logging == 1)
      {
        Serial.print(Data->tm_package_delta); Serial.print(" ");
        if (Data->IsMM1)
        {
          Serial.print(" MM1 ");
          if (Data->IsLoc) Serial.print(" Lokdecoder     ");
          if (Data->IsMagnet) Serial.print(" Magneetdecoder ");
          Serial.print(" - Decoder address: "); Serial.print(Data->Address);
          Serial.print(" - Port adres: "); Serial.print(Data->SubAddress);
          Serial.print(" - Wissel adres: "); Serial.print(Data->PortAddress);
          Serial.print(" - Waarde: "); Serial.print(Data->DecoderState);

          Serial.print("- .....: " );
        }

        if (Data->IsMM2)
        {
          Serial.print(" MM2 ");
          if (Data->IsLoc) Serial.print(" Lokdecoder     ");
          if (Data->IsMagnet) Serial.print(" Magneetdecoder ");
          Serial.print(" Addr: "); Serial.print(Data->Address);
          Serial.print("- .....: " );
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

        Serial.print (" ");

        for(int i=0;i<35;i++) 
        {
          Serial.print(Data->Timings[i]);
          Serial.print(" ");
        }
        Serial.println();
      }

/******************************************************************/
/*                                                                */
/* Wissel decoder software                                        */
/*                                                                */
/******************************************************************/
 // Nog te programmeren

    }
  }
}

void isr() 
{
  mm.PinChange();
}
