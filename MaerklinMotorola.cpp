/*
  MaerklinMotorola.cpp - Library for decoding the signals from the Märklin-Motorola-protocol. 
  Created by Laserlicht, Februar 27, 2018.
  Released under BSD 2-Clause "Simplified" License.
*/

#include <MaerklinMotorola.h>

MaerklinMotorola::MaerklinMotorola(int INPUT_PIN) 
{
  pinMode (INPUT_PIN, INPUT_PULLUP);
  DataQueueWritePosition = 0;
  sync = false;
}

MaerklinMotorolaData* MaerklinMotorola::GetData() {
	for(int QueuePos=0; QueuePos<MM_QUEUE_LENGTH;QueuePos++) {
		if(DataGramState_Validated == DataQueue[QueuePos].State) {
			DataQueue[QueuePos].State = DataGramState_Finished;
			return DataQueue + QueuePos;
		}
	}
	return 0;
}

void MaerklinMotorola::Parse() 
{
  int QueuePos , BitPeriod , HalfBitPeriod , period_tmp;
  bool valid , parsed;
  byte i , counterMM1 , counterMM2 , Bits[18];
  unsigned char s;
	
  for(QueuePos=0; QueuePos<MM_QUEUE_LENGTH;QueuePos++) 
  {

    if(DataGramState_ReadyToParse == DataQueue[QueuePos].State) 
    {
	  valid = true;
	  parsed = false;

      // Motorola formaat
//      DataQueue[QueuePos].SubAddress = DataQueue[QueuePos].tm_package_delta;
      if ((( DataQueue[QueuePos].tm_package_delta > 3060) && (DataQueue[QueuePos].tm_package_delta < 3960)) ||(( DataQueue[QueuePos].tm_package_delta > 1530) && (DataQueue[QueuePos].tm_package_delta < 2980))) 
      { //MARKLIN PROTOCOL MM1 of MM2
	    valid = true;
	    parsed = false;

	    DataQueue[QueuePos].IsMM2 = false;
	    DataQueue[QueuePos].IsMM1 = false;
        DataQueue[QueuePos].IsMagnet = false;
		DataQueue[QueuePos].IsLoc = false;
        DataQueue[QueuePos].IsSpeedRecord = false;
	    DataQueue[QueuePos].IsFunctionRecord = false;
	    DataQueue[QueuePos].Address = 0;
	    DataQueue[QueuePos].SubAddress = 0;
	    DataQueue[QueuePos].Function = false;
	    DataQueue[QueuePos].Stop = false;
	    DataQueue[QueuePos].ChangeDir = false;
	    DataQueue[QueuePos].Speed = 0;
	    DataQueue[QueuePos].MagnetState = 0;
	    DataQueue[QueuePos].IsMM2FunctionOn = false;
	    DataQueue[QueuePos].MM2FunctionIndex = 0;
	    DataQueue[QueuePos].MM2Direction = MM2DirectionState_Unavailable;
	    DataQueue[QueuePos].DecoderState = MM2DecoderState_Unavailable;
	    DataQueue[QueuePos].PortAddress = 0;
	  
        //decode bits 
        BitPeriod = ((DataQueue[QueuePos].Timings[0]+DataQueue[QueuePos].Timings[1])); //calculate bit length
        
        DataQueue[QueuePos].IsMagnet = (((BitPeriod > 85) && (BitPeriod < 110)) ? true : false);  //distinction protocol (fixed-time)
        DataQueue[QueuePos].IsLoc = (((BitPeriod > 170) && (BitPeriod < 220)) ? true : false);  //distinction protocol (fixed-time)

        HalfBitPeriod = (BitPeriod >> 1); //calculate half bit length


	    for( i = 0 ; i < 35 ; i +=2 ) 
	    {     
	      Bits[i/2] = (DataQueue[QueuePos].Timings[i]>HalfBitPeriod) ? 1 : 0; //longer than half: 1

		  if(i<33) 
		  {
		    period_tmp = DataQueue[QueuePos].Timings[i] + DataQueue[QueuePos].Timings[i+1];
		    if(period_tmp > 125 && period_tmp < 175) valid = false; //MFX herausfiltern
          }

        }

        //detectie MM1 of MM2
        if (valid)
	    {
	      //DataQueue[QueuePos].IsMM2 = false;
		  //DataQueue[QueuePos].IsMM1 = false;		
		  counterMM1 = 0;
		  counterMM2 = 0;
		
		  for( i = 5 ; i < 9 ; i++) 
          { 
            if (Bits[i*2] == Bits[i*2+1] ) counterMM1++; else counterMM2++ ;
          }
		
		  if ((counterMM1 == 4) && (counterMM2 == 0)) DataQueue[QueuePos].IsMM1 = true;
		  if ((counterMM1 < 4) && (counterMM2 > 0)) DataQueue[QueuePos].IsMM2 = true;

          if (DataQueue[QueuePos].IsMM1 || DataQueue[QueuePos].IsMM2)
          {
            //The first 5 "trits" are always ternary (MM1 and MM2) - For MM2, the least 4 "trits" are quarternary
		    //decode trits from bits
		    for( i = 0 ; i < 9 ; i++) 
		    { 
		      if(Bits[i*2] == 0 && Bits[i*2+1] == 0)
		      {
		        //00
                DataQueue[QueuePos].Trits[i] = 0;
              }
              else if(Bits[i*2] == 1 && Bits[i*2+1] == 1)
              {
                //11
                DataQueue[QueuePos].Trits[i] = 1;
              }
              else if(Bits[i*2] == 1 && Bits[i*2+1] == 0)
              {
                //10
		        DataQueue[QueuePos].Trits[i] = 2; // MM2: kan zich enkel voordoen vanaf karakter 5
                  //  DataQueue[QueuePos].IsMM2 = true;
		      }
		      else
		      {
		        //01 -> MM2 only and only for trits 5...9
			    if(i<5)
			    {
			      //Pattern 01 can't occur on trits 0...4 -> invalid input
			      valid = false;
			      break;
			    }
			    else
			    {
			      //MM1 trailing "trits" only use "11" and "00" so we have MM2 here
			      DataQueue[QueuePos].Trits[i] = 3;	
                  //  DataQueue[QueuePos].IsMM2 = true;
			    }
		      }
		    }
	      }
        }

	    //Decoder
        if (valid)
	    {	
 		  DataQueue[QueuePos].BitsAG = (Bits[10]*128) + (Bits[11]*64) + (Bits[12]*32) + (Bits[13]*16) + (Bits[14]*8) + (Bits[15]*4) + (Bits[16]*2) + (Bits[17]);
	      
		  DataQueue[QueuePos].Address = DataQueue[QueuePos].Trits[3] * 27 + DataQueue[QueuePos].Trits[2] * 9 + DataQueue[QueuePos].Trits[1] * 3 + DataQueue[QueuePos].Trits[0];

		  if(DataQueue[QueuePos].IsLoc) //Loktelegramm
		  { 
		    DataQueue[QueuePos].Function = ((DataQueue[QueuePos].Trits[4] == 1) ? true : false);
			s = Bits[16] * 8 + Bits[14] * 4 + Bits[12] * 2 + Bits[10] ;
			DataQueue[QueuePos].Speed = (s==0) ?  0 : s-1;
            DataQueue[QueuePos].Stop = (s==0) ? true : false;

            if (DataQueue[QueuePos].IsMM1)
            {
			  DataQueue[QueuePos].ChangeDir = (s==1) ? true : false;
			  DataQueue[QueuePos].IsSpeedRecord = true;
			  
// TO DO direction !!
			}

			if(DataQueue[QueuePos].IsMM2)
			{
			  byte DirectionInfo = (Bits[15] * 4 + Bits[13] * 2 + Bits[11]);
			  	  	
              if (((Bits[16] != Bits[17]) ) && ((DirectionInfo == 2)||(DirectionInfo == 5))) // Record with speed and direction see www.drkoenig.de - digital - motorola.htm
              {
			    DataQueue[QueuePos].IsSpeedRecord = true;     	
		        DataQueue[QueuePos].IsFunctionRecord = false;
              	DataQueue[QueuePos].SubAddress = (DirectionInfo) ; 
                if (DirectionInfo == 5) DataQueue[QueuePos].MM2Direction = MM2DirectionState_Backward;
                if (DirectionInfo == 2) DataQueue[QueuePos].MM2Direction = MM2DirectionState_Forward;
			  }
			  
              if (DataQueue[QueuePos].IsSpeedRecord == false) 
              {
                unsigned char sMM2 = Bits[17] + Bits[15] * 2 + Bits[13] * 4 + Bits[11] * 8;
			    
	
			    switch(sMM2)
			    {
					case 2:
					case 3:
					     DataQueue[QueuePos].MM2FunctionIndex = 2;
					     DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					     DataQueue[QueuePos].IsFunctionRecord = true;
					    // DataQueue[QueuePos].IsSpeedRecord = false;
					     break;

					case 4:
					     break;
						 	
					case 5:
						 if (DataQueue[QueuePos].Speed == 10) DataQueue[QueuePos].MM2FunctionIndex = 1;
						 if (DataQueue[QueuePos].Speed == 11) DataQueue[QueuePos].MM2FunctionIndex = 2;
						 if (DataQueue[QueuePos].Speed == 13) DataQueue[QueuePos].MM2FunctionIndex = 3;
						 if (DataQueue[QueuePos].Speed == 14) DataQueue[QueuePos].MM2FunctionIndex = 4;
						 DataQueue[QueuePos].IsMM2FunctionOn = 1;
						 DataQueue[QueuePos].IsFunctionRecord = true;
 					    // DataQueue[QueuePos].IsSpeedRecord = false;
					     break;

					case 6:
					case 7:
					     DataQueue[QueuePos].MM2FunctionIndex = 3;
					     DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					     DataQueue[QueuePos].IsFunctionRecord = true;
   					    // DataQueue[QueuePos].IsSpeedRecord = false;
					     break;

					case 10:
						 if (DataQueue[QueuePos].Speed == 2) DataQueue[QueuePos].MM2FunctionIndex = 1;
						 if (DataQueue[QueuePos].Speed == 3) DataQueue[QueuePos].MM2FunctionIndex = 2;
						 if (DataQueue[QueuePos].Speed == 5) DataQueue[QueuePos].MM2FunctionIndex = 3;
						 if (DataQueue[QueuePos].Speed == 6) DataQueue[QueuePos].MM2FunctionIndex = 4;
						 DataQueue[QueuePos].IsMM2FunctionOn = 0;
						 DataQueue[QueuePos].IsFunctionRecord = true;
 					    // DataQueue[QueuePos].IsSpeedRecord = false;
						 break;
					case 11:
					     break;

					case 12:
					case 13:
					     DataQueue[QueuePos].MM2FunctionIndex = 1;
					     DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					     DataQueue[QueuePos].IsFunctionRecord = true;
   					    // DataQueue[QueuePos].IsSpeedRecord = false;
					     break;

					case 14:
					case 15:
					     DataQueue[QueuePos].MM2FunctionIndex = 4;
					     DataQueue[QueuePos].IsMM2FunctionOn = sMM2 & 1;
					     DataQueue[QueuePos].IsFunctionRecord = true;
   					    // DataQueue[QueuePos].IsSpeedRecord = false;
					     break;
					
					default:
					     break;
			    }
			  }
			}
		    parsed=true;
		  } 
			
          if(DataQueue[QueuePos].IsMagnet) //magnet telegram 
          { 
			if(DataQueue[QueuePos].Trits[4]==0) 
			{
			  unsigned char s = Bits[10] + Bits[12] * 2 + Bits[14] * 4;
			  DataQueue[QueuePos].SubAddress = s;				
			  DataQueue[QueuePos].PortAddress = (( DataQueue[QueuePos].Address - 1) * 4) + (s >> 1) + 1;
			  if (Bits[16]==1) 
			  {
			    DataQueue[QueuePos].MagnetState = true;
				DataQueue[QueuePos].DecoderState = Bits[10] ? MM2DecoderState_Green : MM2DecoderState_Red;				    
			  }
			  parsed=true;
			}
          }  
	    }   
      }
	
	  if(parsed) 
      {
	    //Get previous DataGram from Queue
		int previousDataGramPos = QueuePos > 0 ? QueuePos - 1 : MM_QUEUE_LENGTH - 1;
		DataQueue[QueuePos].State = DataGramState_Parsed;
		if(DataGramState_Parsed == DataQueue[previousDataGramPos].State) 
		{
		  //Check if previous DataGram was identical
		  if(0 == memcmp(DataQueue[QueuePos].Trits, DataQueue[previousDataGramPos].Trits, 9)) 
		  {
		    DataQueue[QueuePos].State = DataGramState_Validated;
		  }
		}
	  }
	  else 
	  {
	    //Invalid frame
		DataQueue[QueuePos].State = DataGramState_Error;
	  }
	}
  }
}

void MaerklinMotorola::PinChange() 
{
  unsigned long tm = micros();
  unsigned long tm_delta = tm - last_tm;

  if(sync) 
  { //collect bits only after syncronization
    DataQueue[DataQueueWritePosition].Timings[timings_pos] = int(tm_delta); //filing the time difference between the last edges
    timings_pos++;

	if(tm_delta>500) 
	{
		//timeout - resync
		timings_pos = 0;
		sync = true;
		sync_tm = tm;
	}
    
 //   if ((timings_pos>=2) &&((1.05*(DataQueue[DataQueueWritePosition].Timings[0]+DataQueue[DataQueueWritePosition].Timings[1])) < int(tm_delta)))
 //   {

	if(timings_pos>=35) 
	{
      DataQueue[DataQueueWritePosition].tm_package_delta = tm - sync_tm; //calculate package length
	  DataQueue[DataQueueWritePosition].State = DataGramState_ReadyToParse;
	  DataQueueWritePosition ++;
	  //Queue end - go to queue start
	  if(MM_QUEUE_LENGTH <= DataQueueWritePosition)
	  {
		 DataQueueWritePosition = 0;
	  }
	  
	  DataQueue[DataQueueWritePosition].State = DataGramState_Reading;
      sync = false;
      timings_pos = 0;
    }
  } 
  else 
  {
    if(tm_delta>500) 
	{ //protocol-specific pause time
      sync = true;
      sync_tm = tm;
    }
  }

  last_tm = tm;
}
