//Media feeder code; S. Gough, based on Alix controller and
// James Nation 05/30/2013 EmDyeInjector 1.0
//For HARVARD feeders; finalized August 8, 2015


// Steve Gough; Version 6.1 for Ft.Lewis, CO
///////////////////////////////////////////////////////////////
//Version 6.2 Singapore, for Grahill encoders that pull to ground when pressed;
//otherwise internal pullup resistors keep pin 19 HIGH or OFF
//Lots of jockeying on this March 2, 2018.
///////////////////////////////////////////////////////////////
//	THIS VERSION FOR PANASONIC GX_F12A PROX SENSORS

#include <LiquidCrystal.h>
LiquidCrystal lcd (9, 8, 4, 5, 6, 7);  // LCD 4-bit interface.

int stepNum = 0;
int pulseCount = 0;
int oldencoder = 1000; //ensures encoder != oldencoder first time through loop so if 
 //statement runs
 long encoderPause = 0;
 long lcdTimer = 0;  //LCD update timer

//Output pins to Gecko stepper driver:
int directionPin = 11;  //analog A0 on Alix board
int stepPin = 12;	//analog pin A1 (OK on Alix)
int enablePin = 13;	//analog pin A2 (Ok on Alix)
int proxPin = 2;  //digital input for proximitiy switch (free on Alix); 
					//position 9 on Alix expansion pins
int proxPintemp = 0; //for diagnostics July 12 2015

//A5 or pin 19 is encoder switch pin
//GRAYHILL optical encoder (post Jan 2017) goes LOW when PRESSED = <512

int stepPulse_micros = 870;	//the pause between each step pulse sent to Gecko 
//driver; a very critical variable with much R&D behind it!

float gramSec = 0.20;  //feed rate in grams/sec; 0.20 in the minimum rate
float gramPulse = 0; //grams per pulse
float pulseSec = 1; //pulses per second
int runFeedmode = 0;  // zero is stop; 1 = run feeder
long pulseTimer = 0; // timer for cone pulses; used for diagnosis and calibration
//long totalFeedgrams_long = 0;  //totalizes feed
float totalFeedgrams_float = 0;  //for totalizing
int gramFlag = 0; //switches total display between grams and seconds
int serialFlag = 1;  // if 1; serial prints dignostics; set to zero otherwise
/////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////
float valveClear = 0;  //number of steps for valve clearance
//this is the 0.05" the bearing moves down before contacting the stop 
//and moving the cone valve; needed to allow the spring to compress and 
//compensate for variace in cone position on closing, depending on how much
//and size of media caught in the cone valve/seat interface.

//July 6 2015 Gough incorporated this adjustment into the formulas for
//calculating steps; see July 6 spreadsheet tab.

int updatecount = 6;  //set to 6 so runs first time lcdUpdate() runs.
long feedStart_time = 0;
unsigned long feedRun_time = 0;
unsigned long feedRun_time_temp = 0;
float caliBrate = 1.0; //calibration multiplier; starts at zero; goes to -100 and plus 100.
int calibrationFlag = 0;



void conedown()  //sends cone valve down a distance determined by stepNum
	
	
{
	digitalWrite (enablePin, HIGH);  //enable stepper coils; G512X
	// DOWN(OPEN) STROKE Step for stepNum steps, moving rod and cone down 
	for (int i=0; i <= stepNum; i++)
	{


	digitalWrite (directionPin, LOW);  			//set direction down
	digitalWrite (stepPin, HIGH);				//send step pulse
	delayMicroseconds (stepPulse_micros);		//length of pulse to step 			//pin = stepPulse_micros
	digitalWrite (stepPin, LOW);
	delayMicroseconds (50);	
	}		
	
}

void coneUP()  // brings cone valve back up until the proximity sensor detects it's
	//closed.  Complex, because of start/stop, power on/off, etc., this "homes" the cone
	//each time it comes back up, e.g. on startup when it's hanging loose.
	// The Panasonic prox sensor is very precise and reliable.  If the stepper and arm vibrate
	// or behave erratically, it's usually because the arm cannot hit the home position because
	// the valve spring is too tight or, for some reason, the motor cannot return the cone to it's 
	// seat.  Debris in the cone valve (like a bolt) can cause this.
{

	digitalWrite (proxPin, INPUT_PULLUP); //set pullup resistor  pinMode(2, INPUT_PULLUP);
	while (digitalRead(proxPin) == HIGH)    //FOR PANASONIC GX-F12A NPN
		//July 12 Harvard batch goes HIGH when tripped!  Hours of troubleshooting fun!  Gough.
	{
	digitalWrite (enablePin, HIGH); 		//enable stepper motor coils
	digitalWrite (directionPin, HIGH);			//direction is up

	digitalWrite (stepPin, HIGH);				//send step pulse
	delayMicroseconds (stepPulse_micros);		//length of pulse to step pin, lots of tuning here
	digitalWrite (stepPin, LOW);				//turn off step pulse
	delayMicroseconds (50);				//turn off step pulse; wait 10 micros for things to settle	
	}
}
//////////////////////////////////////////////////////////////////////////
// Encoder handling.//////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////

// The encoder count value.
static int16_t encoder_count = 0;  //Static variable stores between function calls, but can be changed only by the function in which it's created.

// The sequence of gray codes in the increasing and decreasing directions.
static uint8_t cw_gray_codes[4] = { 2, 0, 3, 1 };
static uint8_t ccw_gray_codes[4] = { 1, 3, 0, 2 };

// The intermediate delta in encoder count (-1, 0, or +1), incremented or decremented halfway between detents.
static int8_t half_ticks = 0;

// The gray code we last read from the encoder.
static uint8_t previous_gray_code = 0;

// Reset the encoder to zero.
static void reset_encoder()
{
  encoder_count = 0;
  half_ticks = 0;
}

// Look for encoder rotation, updating encoder_count as necessary.
static void check_encoder()
{
  // Get the Gray-code state of the encoder.
  // A line is "low" if <= 512; "high" > 512 or above.
  uint8_t gray_code = ((analogRead(3) > 512) << 1) | (analogRead(4) > 512);

/*From Chris "It is a bitwise OR, so it is not limited to 0 and 1.  The <<1 is a multiply by 2, making the first operand either 0 or 2.  So it is basically looking at two signals.  Analog input 3 is the top bit and analog input 4 is the bottom bit.  The >512 is because the signals range from 0..1023, since I am using an analog input as a digital input."
*/
  
  // If the gray code has changed, adjust the intermediate delta. 
  if (gray_code != previous_gray_code) 
  {
    
      // Each transition is half a detent.
      if (gray_code == cw_gray_codes[previous_gray_code]) {
         half_ticks++;
      } 
      else if (gray_code == ccw_gray_codes[previous_gray_code]) {
         half_ticks--;
      }
      
      // If we have accumulated a full tick, update the count.
      if (half_ticks >= 2) {
        half_ticks = 0;
        encoder_count++;
      } 
      else if (half_ticks <= -2) {
        half_ticks = 0;
        encoder_count--;
      }
      
      // Remember the most recently seen code.
      previous_gray_code = gray_code;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
// Switch handling.///////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t switch_was_down = 0;

// Much of this code was used for other Alix controller processes and not
// by the media feeder 

static uint8_t switch_released()
{
  // The switch is depressed if its sense line is low (<512).
  uint8_t switch_is_down = (analogRead(19) < 512);  // Version 6.2 changed
  //for Grayhill optical encoder that is normally high and goes to ground when
  //pressed

  // The action takes place when the switch is released.
  uint8_t released = (switch_was_down && !switch_is_down);
  
  // Remember the state of the switch.
  switch_was_down = switch_is_down;
  
  // Was the switch just released?
  return released;
}

void checkswitch()
{

	coneUP();  //insures cone is up if switch pressed at odd time
	//and while runFeedmode = 0
	
	if (runFeedmode == 1)  //if switch is pressed while running; stop running
	{
		runFeedmode = 0;
		coneUP();
		feedRun_time_temp = feedRun_time;  	
		//feedRun_time_temp stores feedRun_time while runFeedmode = 0 (not feeding)
	}
		
			else 
				{
					runFeedmode = 1;  //if switch is pressed while stopped, then run
	
					feedStart_time = millis();  //begin clock when feed is running
		
					if (serialFlag == 1)    ///set serial flag other than one unless 
						//running diagnostics to serial print
					{			Serial.println(" ");
					Serial.print ("feedRun_time =  ");
					Serial.print (feedRun_time);
					Serial.println(" ");
					Serial.print ("feedStart_time =  ");	
					Serial.println(feedStart_time);	
					Serial.print ("millis =  ");
					Serial.println (millis()); 
					}

				}
		
		if (serialFlag ==1) ///set serial flag other than one unless 
				//running diagnostics to serial print
		{
			Serial.println(" ");
			Serial.print ("checkswith runs  ");
			//Serial.println (mode);
			Serial.println(" ");	
			Serial.print ("runFeedmode1_1 =  ");
			Serial.println (runFeedmode);
		}







		if (analogRead (19) < 512)  //switch still down
		{
				delay(700); //wait to see if switch held down
		if (analogRead (19) < 512)
			{
				
				feedRun_time = 0;// reset totalizer if switch held down
				lcd.clear();
				lcd.print("switch held down");
				lcd.setCursor(0,1);
				//lcd.print ("totals reset");
				//++++++++++++++++++++++++++++++++++++++++++++++
				feedRun_time = 0;//reset all counters
				//totalFeedgrams_long = 0;
				totalFeedgrams_float = 0;
				feedRun_time_temp = 0; 
				pulseCount = 0;
				runFeedmode = 0;  //stops feeding if reset is hit while feed's running
				delay (450);
				//flash message that totals are cleared
				lcd.clear();
				lcd.print(" totals");
				delay (100);
				//lcd.clear();
				lcd.setCursor(0,1);
				lcd.print("   reset");
				delay (600);
				//update_lcd();
				while (analogRead (19) < 512)
					{}  //do nothing while switch is held down
				lcd.clear();
				
				/*
				lcd.print("  hold switch ");
				lcd.setCursor(0,1);
				lcd.print ("  again to  ");
					delay (500);
				lcd.setCursor(0,1);
				lcd.print("   calibrate    ");
				delay(400);
				long millisNow = millis();
				while (millis () - millisNow < 3000)  //waits three seconds for button press
				{
					if (analogRead (19) < 512) 
					{		
						calibrate();
					}
				}
				*/
				
				update_lcd();  //prevents blank screen after running calibrate

			}
		}


}

void update_lcd()
{
	lcd.setCursor(0,0);
	lcd.print("                ");
	lcd.setCursor(0,0);	
	lcd.print (gramSec);
	

	lcd.print (" g/s ");
	stepNumCalc();  //calculates number of steps for gramSec
	lcd.print ("SN=");  //prints nunber of steps for calibration, etc.
	lcd.print(stepNum);
	
	
	updatecount = updatecount + 1 ;
		
		if (updatecount > 3)   //switches lower line of display every few seconds/strokes
			//not enough space on 16 char LCD to display full range of both all the time
	{		
		
	switch (gramFlag)   //Simply switches gramFlag variable back and forth 
		//used to alternately totalFeedgrams and seconds
		//if variable toggle = 1 it's set to zero and vice versa
		//not enough space on 16 char LCD to display full range of both all the time
	{
		case 1:
		gramFlag = 0;
			break;
		case 0:
		gramFlag = 1;
		break;
	}
			updatecount = 0;
	}		
	
	if (gramFlag == 0)	
		{
		lcd.setCursor(0,1);
		
		long total = long (totalFeedgrams_float);
		lcd.print("                "); // clears junk from bottom line
		lcd.setCursor(0,1);
		lcd.print(" ");
		lcd.print((char)0xf6);
		lcd.print(" ");
		lcd.print (total);
		lcd.print(" g");

		}
		
		if (gramFlag == 1)
		{
			lcd.setCursor(0,1);
			lcd.print("                ");
			lcd.setCursor(0,1);
			lcd.print(" ");
			lcd.print((char)0xf6);
			lcd.print(" ");
			long feedTime = feedRun_time/1000;	
			lcd.print (feedTime);
			lcd.print (" s");			
		}
}

/////////////////////////////////////////////////////////////////////////////////////////////
///////////////

void update_gramSec()
{
  check_encoder();
	int encoder = encoder_count;
		
		if (encoder != oldencoder)  //encoder knob moved
		 {

			if (encoder_count == 1 && gramSec < 9.90)//below 10 g/s; adjust by 
				// 0.20 g/sec
			{ 
				gramSec = gramSec + 0.20;		
			}
	
			if (encoder_count == -1 && gramSec < 9.90)
			{
				gramSec = gramSec - 0.20;  
			}
			
			check_encoder();
			
			if (gramSec >= 9.90)
				gramSec = gramSec + encoder_count;
			if (gramSec >= 45.00)
				gramSec = 45.00;
			
			if (gramSec < 0.20)
			gramSec = 0.20;
					
			update_lcd();
			reset_encoder();
			//delay(50);		

			oldencoder = encoder;
		
		} //close if statement
}


void calibrate()
{
	
   	while (analogRead(19)< 512)
   		{
   			lcd.clear();
   			lcd.print ("  please  ");
			lcd.setCursor(0,1);
			lcd.print(" release switch  ");
			delay (200);  //prevents lcd flicker from rapid reprint
		}

	
		lcd.clear();
		lcd.print("turn knob to set");
		lcd.setCursor(0,1);
		lcd.print("  calibration");
			delay (1500);
		lcd.clear();
		lcd.print(" press ");
		lcd.setCursor(0,1);
		lcd.print( " when finished");
		delay(1000);
		lcd.clear();
		lcd.print ("set calibration ");
		lcd.setCursor(0,1);
			lcd.print("    ");
		lcd.print(caliBrate);


	while (analogRead(19) > 512)		//while switch not pressed
									//(or until switch pressed)
		//changed for Grayhill v 6.2
		{	

		  	check_encoder();
				int encoder = encoder_count;
		
					if (encoder != oldencoder)
					{

							if (encoder_count == 1) 
								caliBrate = caliBrate + 0.01;

							if (encoder_count == -1 )
								caliBrate = caliBrate - 0.01;
		
							lcd.clear();
							lcd.print("set calibration");
		
							lcd.setCursor(0,1);
								lcd.print("    ");
							lcd.print(caliBrate);

							if (analogRead(19) < 512)  //6.2
								break;
							check_encoder();		
	
							oldencoder = encoder;
							reset_encoder();
		

		
					}
	
		} // end while analog read		
		///process calibration; see spreadsheets	
		
	lcd.clear();
	lcd.print (" calibration set ");
	lcd.setCursor(0,1);
	lcd. print ("  to ");
	lcd.print ( caliBrate);
	delay (2000);
	lcd.clear();
	lcd.print ("----------------");
	lcd.setCursor(0,1);
	lcd.print ("----------------");
	delay(300);
	lcd.clear();

	runFeedmode == 0;

		//delay prevents switch press to exit from changing runFeedmode
	 
}  /// end calibrate()


void stepNumCalc()
{
	float gramStep = 0;
	
	if (gramSec <= 11.00)
	{
		gramStep = gramSec * caliBrate;  //doing it this way applies 
		//the curve to calibration coefficient
		//stepNum = int (valveClear + (61.32* pow(gramStep, 0.4603)));  //changed 		//July 6; see spreadsheet tab
		stepNum = int (valveClear + (95.44 * pow(gramStep, 0.335)));
		//adds valveClear steps for valve clearance Jun 12 2015
		
	}
		if (gramSec > 11.00)
	{
		gramStep = gramSec * caliBrate;  //doing it this way applies 
		//the curve to calibration coefficient
		//stepNum = int (valveClear + (73.38 * pow(gramStep, 0.3776)));	
		stepNum = int (valveClear + (95.44 * pow(gramStep, 0.335)));

	}		
}

///////////////////////////////////////////////////////////////////////////////////////////////
///////Main program ///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
void setup() 
{ 

    lcd.begin(16, 2);
	digitalWrite (enablePin, HIGH);  //check enable stepper coils; G512X
	
	

	pinMode (proxPin, INPUT_PULLUP); //set pullup resistor for proxpin; very important line!
	//Note the old way was to write digital HIGH, but not with newest IDE.
	//July 12; DISABLE THIS LINE for units that goe HIGH when tripped!
	pinMode (19, INPUT_PULLUP);  //Pullup for Grayhill encoder where OFF = HIGH  Added 3/2/2018

	
	//pinMode (19, INPUT_PULLUP);
	
	//Serial.begin(9600); // only necessary for debugging
	
	pinMode(11,OUTPUT);
	  pinMode(13, OUTPUT);
	  pinMode(12, OUTPUT);
	  //pinMode(2,INPUT);
	  pinMode(19, INPUT);
	  digitalWrite(19, HIGH); //v 6.2; sets pin pullup HIGH
	 
	stepPulse_micros = 2000;		//gently sets cone closed 
  	coneUP();
	stepPulse_micros = 870;  //back to proper setting
	
  lcd.clear();
    lcd.print("  Emriver ");
  lcd.setCursor(0, 1);
  lcd.print("media feeder");  // 
  	delay(1000);
	/*
	lcd.clear();
	proxPintemp = digitalRead(proxPin);
	lcd.print("proxpin = ");
	lcd.print(proxPintemp);
*/	delay (2000);
  
  
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  
  //while(0==0)   ///endless loop for troubleshooting July 12 2015
  //{}
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  //+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


   lcd.clear();
   lcd.print("software version");
   lcd.setCursor(0, 1);
   lcd.print(" 6.2 Singapore");
   delay(1000);
   
   lcd.clear();
   lcd.print("instructions");
   lcd.setCursor(0, 1);
   lcd.print("   follow: ");
   delay(800);
   
   lcd.clear();
   lcd.print("unplug power");
   delay(500);
   lcd.clear();
   lcd.print("   anytime      ");
   lcd.setCursor(0, 1);
   delay(500);
   lcd.print("to hard reset");
   delay(1400);
     
   lcd.clear();
   lcd.print(" turn knob");
   lcd.setCursor(0, 1);
   lcd.print(" to set rate");
   delay(1500);
   
   lcd.clear();
   lcd.print("push knob to");
   lcd.setCursor(0, 1);
   lcd.print("start/stop feed");
   delay(1700);
   
   lcd.clear();
   lcd.print("HOLD down knob");

   lcd.setCursor(0, 1);
   lcd.print("to reset totals");
   delay(1300);
   
   //lcd.clear();
   //lcd.print("unplug power");
   //lcd.setCursor(0, 1);
   //lcd.print("to hard reset");
   //delay(1400);
   
   lcd.clear();
   lcd.print("SN = step number");
   lcd.setCursor(0, 1);
   lcd.print(" for calibration");
   delay(1400);
   
   /*lcd.clear();
   lcd.print("suspend desire");
   lcd.setCursor(0, 1);
   lcd.print("to be happy");
   delay(1000);
   */
   
   lcd.clear();
   lcd.print("PRESS-HOLD NOW");
   lcd.setCursor(0, 1);
   lcd.print("to calibrate");
   //long millisNow() = millis()
	   delay (1700);
    
   	while (analogRead(19)< 512)
   		{
   			lcd.clear();
   			lcd.print ("  please  ");
			lcd.setCursor(0,1);
			lcd.print(" release switch  ");
			delay (200);  //prevents lcd flicker from rapid reprint
			if (analogRead(19)> 512)//runs calibrate() when switch released
			{
   			calibrationFlag = 1;
			calibrate();
			}

			
   		}
		
		lcd.clear();
				
		update_lcd();
			
}   //end setup

//////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////


void loop()
{

	digitalWrite (enablePin, HIGH); 		//enable stepper motor coils

	update_gramSec();  // Checks encoder turned; sets g/s rate

	if (analogRead(19) < 512) //checkswitch() run if switch is down
		checkswitch();
	
	stepNumCalc();  //calculates number of steps for gramSec


while (runFeedmode == 1)
	
{

	feedRun_time = feedRun_time_temp + (millis() - feedStart_time);  //accumulate feed time total
	//timer is started in checkSwitch() when runFeedmode is set to 1.
	//feedRun_time_temp stores feedRun_time while runFeedmode = 0 (not feeding)
	

		digitalWrite (enablePin, HIGH); 		//enable stepper motor coils
	if (millis() - lcdTimer >= 500)	
	{
	update_lcd();  //corrects "push to stop on LCD"
	lcdTimer = millis();
	}

	if (analogRead(19) < 512)
		checkswitch();

	if (runFeedmode == 0)
	{
		update_lcd();
		break;   //avoids one more cycle if switched off
	}
	
    check_encoder();
  	int encoderTurn = encoder_count;
	

		
	long openTime = millis();

	check_encoder();
	int encoderTurn2 = encoder_count; // encoder turning?  Update.
	if (encoderTurn2 != encoderTurn)
		{
			runFeedmode = 0;
			update_gramSec();
			//Serial.println ("encoder turn detected1");
			//Serial.println (encoderTurn2);
			//Serial.println (encoderTurn);
			//Serial.println ("encoder turn detected");
		}
	
	if (millis() - pulseTimer >= 1000)
		
			{
				if (serialFlag ==1)
					{				Serial.print ("pulseTimer = ");  // very hand to make
				//sure pulses are the correct length
				Serial.println (millis() - pulseTimer);
					}
				pulseTimer = millis(); //reset pulsetimer
			
			
			conedown();	///////////////////////////////////////////////////
	
			if (analogRead(19) < 512) //Grayhill encoder switch goes low when pressed
				checkswitch();
			if (runFeedmode == 0)
				break;   //avoids one more cycle if switched off
	
			delay(20);
			

		
			coneUP(); /////////////////////////////////////////////////////
			
	//////////////////////////////////////////////////////////////////////////////////////////	
			
			pulseCount = pulseCount +1;	  //update feedgrams and run time
			totalFeedgrams_float = gramSec * float(pulseCount);
			
			if (serialFlag == 1)
			{
			
				Serial. print ("stepNum ");
				Serial.println (stepNum);	
				Serial.print( "pulseCount ");
				Serial.println(pulseCount);
				//Serial.print ("openTime, ms = ");
				//Serial.println (openTime);
				Serial.print( "Total feed, grams ");
				Serial.println (  totalFeedgrams_float);
				//Serial.print( " long (gramsec) ");
				//Serial.println (  long(gramSec));
				Serial.print (" caliBrate variable = ");
				Serial.println (caliBrate);
				Serial.println ("--------------------- ");
				
			}
	//////////////////////////////////////////////////////////////////////////////////////////						
			


			}		// end pulseTimer if statement
	

			check_encoder();
	encoderTurn2 = encoder_count; // encoder turning?  Update.
	if (encoderTurn2 != encoderTurn)
		{
			runFeedmode = 0;
			update_gramSec();
			//Serial.println ("encoder turn detected2");
			//Serial.println ("encoder turn detected");
		}	
} // close while statement
} // close loop





