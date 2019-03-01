/*
  Media Feeder Code v2.0
  Mason Parrone 12/13/2018
  Accurately controls media feeder cone movement for precision feeding
  Calibration routine that does math on backend to generate necessary regression curves
  Flat menu implemented
  Software factory default added
  Widened SN constrain range to beyond values in calibration
  Added confirmation for calibration and factory reset
  Changed powA and powB factory default values, pulled from controller with good calibration
  Removed diagnostic serial print commands
  Couldn't reach 40 g/s for some calibrations, swing arm would hit pushrod.
  Decreasing range to 30 g/s
  SNhigh set to 285 for MF1801, too high and it makes swing arm impact during calibration, too low and it makes calibration inaccurate because it has to estimate to much of the operating range
  To determine if it was impacting I used a strip of paper wrapped around the spring
*/

#include <EEPROM.h>

//Pin References
const uint8_t directionPin = 11;    //Stepper driver dir pin
const uint8_t stepPin = 12;         //Stepper driver step pin
const uint8_t enablePin = 13;       //Stepper driver en pin
const uint8_t proxPin = 2;          //Proximity sensor signal pin
const uint8_t encoderswitch = 7;    //Button on optical encoder pin
const uint8_t channelB = 6;         //Channel B on optical encoder pin
const uint8_t channelA = 4;         //Channel A on optical encoder pin

//Encoder
uint8_t lastpress = HIGH;                           //Last state of encoder button
bool pressed = false;                               //Signals a button press
static uint8_t cw_gray_codes[4] = { 2, 0, 3, 1 };   //The sequence of gray codes in the increasing and decreasing directions.
static uint8_t ccw_gray_codes[4] = { 1, 3, 0, 2 };  //Gray code sequence
static uint8_t previous_gray_code = 0;              //The gray code we last read from the encoder.
bool bootup = true;                                 //Used to ignore first gray code, bug fix to eliminate false increment

//Time
uint16_t wait = 50;                 //Standard wait time
uint16_t stepPulse = 870;           //Microsec of step pulse length
uint16_t textDelay = 2000;          //Delay between scrolling text screens
uint32_t startsampletime;           //Millis at sample start time
float totalsampletime = 0.0;        //Actual second length of sample dispensed
uint8_t highsampletime = 90;        //Seconds to run sample for when output is low
uint8_t lowsampletime = 20;         //Seconds to run sample for when output is high
uint8_t sampletime;                 //Seconds to run current sample
uint32_t movetime;                  //Time at start of downward movement
uint16_t moveinterval = 1000;       //How often the cone moves up and down
uint32_t lastchartime = 0;          //Last time custom char was updated
uint16_t walktime = 1000;           //Change walking character every xxx milliseconds
uint32_t pulsetime = 0;             //Used for non-blocking stepper motor code
uint32_t pulsetime2 = 0;            //Used for non-blocking stepper motor code

//Variables
bool feed;                          //Toggles start/stop
bool lastfeed = false;              //Feed updater
uint16_t SN = 0;                    //Step number
uint8_t SNlow = 70;                 //Step number for low output ~.20g/s - too low and calibration is skewed to output below .2 g/s
uint16_t SNhigh = 285;              //Step number for high output ~25g/s - too high and swing arm impacts push rod, too low and calibration isn't adequate to model operating range
int collectedmass = 0;              //Mass collected from sample
uint16_t lastmass = 0;              //Updater for collected mass
bool enter = false;                 //User is in enter mode for calibration
int address = 0;                    //EEPROM address for saving calibration
bool calibrating = false;           //Currently in calibration mode bool
float gramsec = 1.0;                //Desired grams per second output
float lastgramsec = 1.0;            //Used to update gramsec
uint8_t menu = 1;                   //Menu number for switch case
bool currentcustomchar = true;      //Current custom character to load
bool increasing = false;            //Knob rotated in increasing direction
bool decreasing = false;            //Knob rotated in decreasing direction
bool edit = false;                  //Used to enter and exit gramsec edit mode
bool lastedit = false;              //Used to update edit
bool stepnow = true;                //Used to step
uint32_t runtime = 0;                               //Used for timer
uint16_t hour;                                      //""
uint16_t minute;                                    //""
uint16_t second;                                    //""

//Math
uint8_t samplesize = 8;             //Number of samples in calibration routine *To change sample size, change size of lnxvals and lnyvals below to match
float lnxvals[8];                   //Natural log of x values captured during calibration, change size to match samplesize above
float lnyvals[8];                   //Natural log of x values captured during calibration, change size to match samplesize above
uint8_t currentsample = 0;          //Current sample number
float e = 2.718281828459;           //Euler's number
float powA = 0;                     //A value in power regression: y=Ax^B  or  (g/s)=A(SN)^B
float powB = 0;                     //B value in power regression: y=Ax^B  or  (g/s)=A(SN)^B
float powA2 = 0.00000029786;        //Saved factory default A value
float powB2 = 3.248044;             //Saved factory default B value
float Sxx = 0;                      //Used in power regression calculation, different than standard deviation
float Sxy = 0;                      //Used in power regression calculation, different than standard deviation
float quickSum = 0;                 //Used for average calculation
float quickSum2 = 0;                //Used for average calculation
float avglnx = 0;                   //Average of natural log of all x values
float avglny = 0;                   //Average of natural log of all y values
float totalGram = 0;                //Total grams output by media feeder


////////////////////////////////////////////////////////////////////////////
// LCD Functions ///////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////

//Define custom characters
//https://maxpromer.github.io/LCD-Character-Creator/

//Initialize custom characters
void definecustom()
{
  //Cone up
  Serial.write(0xFE);       //Talk to LCD display
  Serial.write(0x54);       //Define custom character
  Serial.write(0);          //Custom character address

  Serial.write(0x04);       //Custom character bitmap
  Serial.write(0x04);
  Serial.write(0x04);
  Serial.write(0x04);
  Serial.write(0x0E);
  Serial.write(0x1F);
  Serial.write(0x00);
  Serial.write(0x00);

  //Cone down
  Serial.write(0xFE);
  Serial.write(0x54);
  Serial.write(1);

  Serial.write(0x04);
  Serial.write(0x04);
  Serial.write(0x04);
  Serial.write(0x04);
  Serial.write(0x04);
  Serial.write(0x0E);
  Serial.write(0x1F);
  Serial.write(0x00);
}

// turn on display
void displayOn() {
  Serial.write(0xFE);
  Serial.write(0x41);
}

// move the cursor to the home position on line 2
void cursorLine2() {
  Serial.write(0xFE);
  Serial.write(0x45);
  Serial.write(0x40); //Hex code for row 2, column 1
}

// move the cursor to the home position on line 2
void cursorTopRight() {
  Serial.write(0xFE);
  Serial.write(0x45);
  Serial.write(0x0F); //Hex code for row 1, column 16
}

// move the cursor to the home position on line 2
void cursorBottomRight() {
  Serial.write(0xFE);
  Serial.write(0x45);
  Serial.write(0x4F); //Hex code for row 2, column 16
}

// move the cursor to the home position
void cursorHome() {
  Serial.write(0xFE);
  Serial.write(0x46);
}

// clear the LCD
void clearLCD() {
  Serial.write(0xFE);
  Serial.write(0x51);
}

// backspace and erase previous character
void backSpace(int back) {
  for (int i = 0; i < back; i++)
  {
    Serial.write(0xFE);
    Serial.write(0x4E);
  }
}

// move cursor left
void cursorLeft(int left) {
  for (int i = 0; i < left; i++)
  {
    Serial.write(0xFE);
    Serial.write(0x49);
  }
}

// move cursor right
void cursorRight(int right) {
  for (int i = 0; i < right; i++)
  {
    Serial.write(0xFE);
    Serial.write(0x4A);
  }
}

// set LCD contrast
void setContrast(int contrast) {
  Serial.write(0xFE);
  Serial.write(0x52);
  Serial.write(contrast); //Must be between 1 and 50
}

// turn on backlight
void backlightBrightness(int brightness) {
  Serial.write(0xFE);
  Serial.write(0x53);
  Serial.write(brightness); //Must be between 1 and 8
}

void checkbutton() //Check for rising and falling edge of button press
{
  if ((digitalRead(encoderswitch) == LOW) && (lastpress)) //Falling signal edge, happens when button is first pressed
  {
    lastpress = LOW; //Pressed low indicator
  }
  else if ((digitalRead(encoderswitch) == HIGH) && (!lastpress)) //Rising signal edge, happens when button is released
  {
    lastpress = HIGH; //Reset indicator
    pressed = true;   //Indicate that the button was pressed
    delay(10);
  }
}

void checkknob() //Look for encoder rotation by observing graycode on channel A & B
{
  int gray_code = ((digitalRead(channelA) == HIGH) << 1) | (digitalRead(channelB) == HIGH);
  if (gray_code != previous_gray_code)       //Encoder clicked in a direction
  {
    if (bootup) //Ignore encoder on first detent after bootup
    {
      bootup = false;
    }
    else
    {
      if (gray_code == cw_gray_codes[previous_gray_code])     //Knob twist CW
      {
        increasing = true;        //Flag CW rotation
      }
      else if (gray_code == ccw_gray_codes[previous_gray_code])     //Knob twist CCW
      {
        decreasing = true;        //Flag CCW rotation
      }
    }
  }
  previous_gray_code = gray_code; //Stores current gray code for future comparison
}

void conedown()  //Sends cone down a distance determined by stepNum
{
  if (!calibrating)
  {
    SN = round(pow((gramsec / powA), (1.0 / powB)));  //y = Ax^B, g/s = powA * SN ^ powB, solved for SN
  }
  for (int i = 0; i <= SN;)
  {
    if (((micros() - pulsetime >= wait) || (micros() < pulsetime)) && (stepnow))    //Non-blocking delay
    {
      digitalWrite (directionPin, LOW);       //Direction down
      digitalWrite (stepPin, HIGH);           //Start step
      stepnow = false;                        //Ready to stop step
      pulsetime2 = micros();                  //Mark current time
    }
    if (((micros() - pulsetime2 >= stepPulse) || (micros() < pulsetime2)) && (!stepnow))        //Non-blocking delay
    {
      digitalWrite (stepPin, LOW);            //End step
      pulsetime = micros();                   //Mark current time
      stepnow = true;                         //Ready to step again
      i++;                                    //Increase step number counter
    }
    checkbutton();
    checkknob();
    walking();
  }
}

void coneup()     //Brings cone up until prox sensor triggers
{
  while (digitalRead(proxPin) == HIGH)      //FOR PANASONIC GX-F12A
  {

    if (((micros() - pulsetime >= wait * 2) || (micros() < pulsetime)) && (stepnow))    //Non-blocking delay
    {
      digitalWrite (directionPin, HIGH);      //Direction up
      digitalWrite (stepPin, HIGH);           //Start step
      stepnow = false;                        //Ready to stop step
      pulsetime2 = micros();                  //Mark current time
    }
    if (((micros() - pulsetime2 >= stepPulse) || (micros() < pulsetime2)) && (!stepnow))        //Non-blocking delay
    {
      digitalWrite (stepPin, LOW);            //End step
      pulsetime = micros();                   //Mark current time
      stepnow = true;                         //Ready to step again
    }
    checkbutton();
    checkknob();
    walking();
  }
}

void runfeeder()    //Run media feeder
{
  if (feed) //Toggled start
  {
    if (millis() - movetime >= moveinterval)
    {
      movetime = millis();  //Start of movement
      conedown();           //Movement
      coneup();             //Movement
    }
  }
  else if (!feed && lastfeed) //Toggled stop
  {
    coneup();
    lastfeed = feed;
  }
}

void calculate()  //Calculate A and B values for power regression of calibration values
{
  //Power regression info found @ https://keisan.casio.com/exec/system/14059931777261

  quickSum = 0;
  quickSum2 = 0;

  for (int i = 0; i < samplesize; i++)
  {
    quickSum += lnxvals[i];
    quickSum2 += lnyvals[i];
  }
  avglnx = quickSum / samplesize;
  avglny = quickSum2 / samplesize;

  quickSum = 0;
  quickSum2 = 0;

  for (int i = 0; i < samplesize; i++)
  {
    quickSum += (pow(lnxvals[i], 2));
    quickSum2 += (lnxvals[i] * lnyvals[i]);
  }
  Sxx = (quickSum / samplesize) - (pow(avglnx, 2));
  Sxy = (quickSum2 / samplesize) - (avglnx * avglny);

  powB = Sxy / Sxx;
  powA = pow(e, (avglny - (powB * (avglnx))));
}

void calibrate()  //Calibrate power curve by taking weight samples at different settings
{
  currentsample = 0;        //Reset current sample number
  feed = false;             //Turn off feeder
  calibrating = true;

  clearLCD();
  cursorHome();
  Serial.print(F("Calibration"));
  cursorLine2();
  Serial.print(F("Routine"));
  delay(textDelay);

  clearLCD();
  cursorHome();
  Serial.print(F("Collect Media"));
  cursorLine2();
  Serial.print(F("and Input Mass"));
  delay(textDelay);

  while (currentsample < samplesize)      //Calibration complete when all samples have been taken
  {
    clearLCD();
    cursorHome();
    Serial.print(F("Push Button to"));
    cursorLine2();
    Serial.print(F("Start Sample: "));
    uint8_t samplenumber = currentsample + 1;   //+1 because this number is used to access zero-indexed array
    Serial.print(samplenumber);
    delay(50);

    //Generate SN and sample time lengths in intervals equally spaced based on each range and the sample size
    SN = round(SNlow + (currentsample * ((SNhigh - SNlow) / (samplesize - 1))));
    sampletime = round(highsampletime - (currentsample * ((highsampletime - lowsampletime) / (samplesize - 1))));

    while (!feed)
    {
      checkbutton();    //Check for press
      if (pressed)
      {
        pressed = false;
        feed = true;
      }
    }
    if (feed)
    {
      clearLCD();
      cursorHome();
      Serial.print(F("Sample ("));
      uint8_t samplenumber = currentsample + 1;
      Serial.print(samplenumber);
      Serial.print(F("/"));
      Serial.print(samplesize);
      Serial.print(F(")"));
      cursorLine2();
      Serial.print(F("Dispensing . . ."));
      delay(500);
      startsampletime = millis();
    }
    while (((millis() - startsampletime) / 1000) < sampletime)    //Keep moving cone until sample time is reached
    {
      runfeeder();
    }
    //Stop feeding, home cone
    feed = false;
    coneup;
    totalsampletime = (millis() - startsampletime) / 1000.0;

    clearLCD();
    cursorHome();
    Serial.print(F("Media Mass"));
    cursorLine2();
    Serial.print(F("Collected:"));
    cursorBottomRight();
    Serial.print(F("g"));
    collectedmass = 0;
    cursorBottomRight();
    cursorLeft(1);
    backSpace(3);
    Serial.print(collectedmass);
    lastmass = collectedmass;
    enter = true;

    while (enter)
    {
      checkbutton();
      checkknob();
      if (increasing)
      {
        collectedmass += 5;
        increasing = false;
      }
      else if (decreasing)
      {
        collectedmass--;
        decreasing = false;
      }
      collectedmass = constrain(collectedmass, 0, 9999);       //Constrain to 3 digits and positive number
      if (collectedmass != lastmass) //Collected mass updated by user, update display
      {
        cursorBottomRight();
        cursorLeft(1);
        backSpace(3);
        Serial.print(collectedmass);
        lastmass = collectedmass;
      }
      if (pressed)  //User pressed button to enter collected mass value
      {
        pressed = false;
        enter = false;
        uint16_t xval = SN;
        float yval = (collectedmass / totalsampletime);     //Total mass / time of deposition = g/s
        lnxvals[currentsample] = log(xval);   //Natural log of X value (Step number)
        lnyvals[currentsample] = log(yval);   //Natural log of Y value (g/s)
        delay(750);
        collectedmass = 0;
        currentsample++;
      }
    }
  }
  if (currentsample >= samplesize)
  {
    calculate();
    save();           //Save powA and powB to EEPROM
    clearLCD();
    cursorHome();
    Serial.print(F("Calibration"));
    cursorLine2();
    Serial.print(F("Saved To Memory"));
    delay(textDelay);
    calibrating = false;
    menu = 1;
  }
}

void save()   //Save calibration settings to the EEPROM
{
  address = 0;
  EEPROM.put(address, powA);  //Save powA to address 0
  address += sizeof(float);   //Increase address by size of float
  EEPROM.put(address, powB);  //Save powB to address 0
}

void fetch()  //Fetch calibration settings from the EEPROM
{
  address = 0;
  EEPROM.get(address, powA);    //Save powA to address 0
  address += (sizeof(float));   //Increase address by size of float
  EEPROM.get(address, powB);    //Save powB to next address
  if ((isnan(powA)) || (isnan(powB)))            //No value saved to address
  {
    powA = powA2;
    powB = powB2;
    clearLCD();
    cursorHome();
    Serial.print(F("Calibration"));
    cursorLine2();
    Serial.print(F("Not Found"));
    delay(textDelay);
    clearLCD();
    cursorHome();
    Serial.print(F("Loading Factory"));
    cursorLine2();
    Serial.print(F("Default Values"));
    delay(textDelay);
  }
}

void printTotals()  //Print total gram output and total run time
{
  cursorLine2();
  Serial.print(F("                "));
  cursorLine2();
  Serial.print((char)0xf6);         //Summation symbol
  Serial.print(F(" "));
  if (totalGram > 999000)       //More than 999 kg, reset to 0
  {
    totalGram = 0;              //Reset total gram
  }
  if (totalGram < 1000)
  {
    Serial.print(totalGram, 0);
    Serial.print(F("g"));
  }
  else
  {
    uint16_t totalkg = totalGram / 1000;
    Serial.print(totalkg);
    Serial.print(F("kg"));
  }
  timer();
  printtimer();
}
  
void timer()
{
  uint32_t totalSec = (millis() - runtime) / 1000;   //Time is has been powered on for convert to seconds
  hour = totalSec / 3600;           //Seconds in an hour
  uint16_t remainder = totalSec % 3600;               //Remainder
  minute = remainder / 60;           //Seconds in a minute
  remainder = remainder % 60;                 //Remainder
  second = remainder;                //Seconds

  if (hour > 99)
  {
    runtime = millis();     //Runtime is time upon powered on, timer rolls over after 99 hrs
    timer();
  }
}

void printtimer()
{
  cursorLine2();
  cursorRight(8);
  if (hour < 10)
  {
    Serial.print(F("0"));
    Serial.print(hour);
  }
  else
  {
    Serial.print(hour);
  }
  Serial.print(F(":"));
  if (minute < 10)
  {
    Serial.print(F("0"));
    Serial.print(minute);
  }
  else
  {
    Serial.print(minute);
  }
  Serial.print(F(":"));
  if (second < 10)
  {
    Serial.print(F("0"));
    Serial.print(second);
  }
  else
  {
    Serial.print(second);
  }
}

void walking()      //Walking symbol while running feeder
{
  if ((millis() - lastchartime) > walktime)   //If elapsed time has passed
  {
    if (feed)         //If feeding media
    {
      cursorTopRight();
      Serial.print(F(" "));
      cursorTopRight();
      if (currentcustomchar)  //Flip flop between custom characters
      {
        Serial.write(0);
      }
      else
      {
        Serial.write(1);
      }
      currentcustomchar = !currentcustomchar;
      totalGram += gramsec;     //Update total gram
    }
    if (menu == 1)
    {
      printTotals();
    }
    lastchartime = millis();
  }
}

void menuselect() //Flat menu switch case
{
  switch (menu)
  {
    case 1:
      clearLCD();
      cursorTopRight();
      cursorLeft(5);
      Serial.print(F("HOME"));
      cursorHome();
      Serial.print(gramsec, 1);
      Serial.print(F(" g/s"));
      printTotals();
      while (menu == 1)
      {
        checkbutton();
        checkknob();
        if (increasing)   //Increase menu #
        {
          menu++;
          increasing = false;
        }
        else if (decreasing)  //Decrease menu #
        {
          menu = 4;
          decreasing = false;
        }
        if (pressed)
        {
          pressed = false;
          feed = !feed;
          delay(10);
        }
        runfeeder();          //Run feeder
        walking();
      }
      break;


    case 2:
      clearLCD();
      cursorHome();
      Serial.print(F("Adjust Output"));
      edit = false;
      cursorLine2();
      cursorRight(1);
      Serial.print(gramsec, 1);
      Serial.print(F(" g/s"));
      while (menu == 2)
      {
        checkbutton();  //Check button to toggle edit mode
        if (pressed)
        {
          pressed = false;
          edit = !edit;
          delay(10);
        }
        if (edit && !lastedit)     //Entered edit mode, display edit symbol
        {
          cursorLine2();
          Serial.print(F(">"));
          lastedit = edit;
        }
        else if (!edit && lastedit) //Exited edit mode, erase edit symbol
        {
          cursorLine2();
          Serial.print(F(" "));
          lastedit = edit;
        }
        checkknob();
        if (edit)   //Editing gram/sec
        {
          checkbutton();
          checkknob();
          if (increasing)
          {
            gramsec += 0.2;
            increasing = false;
          }
          else if (decreasing)
          {
            gramsec -= 0.2;
            decreasing = false;
          }
          gramsec = constrain(gramsec, 0.4, 30.0);                //Constrain to working range
          if (gramsec != lastgramsec)
          {
            cursorLine2();
            cursorRight(1);
            Serial.print(F("               "));
            cursorLine2();
            cursorRight(1);
            Serial.print(gramsec, 1);
            Serial.print(F(" g/s"));
            lastgramsec = gramsec;
            feed = false;
          }
        }
        else    //Chaning current menu
        {
          if (increasing)
          {
            menu++;
            increasing = false;
          }
          else if (decreasing)
          {
            menu--;
            decreasing = false;
          }
        }
        runfeeder();
      }
      break;

    case 3:
      clearLCD();
      cursorHome();
      Serial.print(F("Press To"));
      cursorLine2();
      Serial.print(F("Calibrate"));
      while (menu == 3)
      {
        checkbutton();
        if (pressed)
        {
          pressed = false;
          feed = false;
          runfeeder();
          enter = true;
          clearLCD();
          cursorHome();
          Serial.print(F("Press To Confirm"));
          cursorLine2();
          Serial.print(F("Calibration"));
          delay(textDelay);
          clearLCD();
          cursorHome();
          Serial.print(F("Rotate To Cancel"));
          cursorLine2();
          Serial.print(F("Calibration"));
          delay(textDelay);
          clearLCD();
          cursorHome();
          Serial.print(F("   Calibrate?   "));
          cursorLine2();
          Serial.print(F("<      ()      >"));
          while (enter)
          {
            checkbutton();
            checkknob();
            if (pressed)
            {
              pressed = false;
              enter = false;
              calibrate();
            }
            if ((increasing) || (decreasing))
            {
              clearLCD();
              cursorHome();
              Serial.print(F("Calibration"));
              cursorLine2();
              Serial.print(F("Cancelled"));
              delay(textDelay);
              increasing = false;
              decreasing = false;
              menu = 1;
              enter = false;
              pressed = false;
            }
          }
        }
        runfeeder();
        checkknob();
        if (increasing)
        {
          menu++;
          increasing = false;
        }
        else if (decreasing)
        {
          menu--;
          decreasing = false;
        }
      }
      break;

    case 4:
      clearLCD();
      
      cursorHome();
      Serial.print(F("Press To"));
      cursorLine2();
      Serial.print(F("Factory Reset"));
      while (menu == 4)
      {
        checkbutton();
        if (pressed)
        {
          pressed = false;
          feed = false;
          runfeeder();
          enter = true;
          clearLCD();
          cursorHome();
          Serial.print(F("Press To Confirm"));
          cursorLine2();
          Serial.print(F("Factory Reset"));
          delay(textDelay);
          clearLCD();
          cursorHome();
          Serial.print(F("Rotate To Cancel"));
          cursorLine2();
          Serial.print(F("Factory Reset"));
          delay(textDelay);
          clearLCD();
          cursorHome();
          Serial.print(F(" Factory Reset? "));
          cursorLine2();
          Serial.print(F("<      ()      >"));
          while (enter)
          {
            checkbutton();
            checkknob();
            if (pressed)
            {
              pressed = false;
              powA = powA2;
              powB = powB2;
              save();
              clearLCD();
              cursorHome();
              Serial.print(F("Software Factory"));
              cursorLine2();
              Serial.print(F("Reset Complete"));
              delay(textDelay);
              menu = 1;
              enter = false;
              increasing = false;
              decreasing = false;
            }
            if ((increasing) || (decreasing))
            {
              clearLCD();
              cursorHome();
              Serial.print(F("Software Factory"));
              cursorLine2();
              Serial.print(F("Reset Cancelled"));
              delay(textDelay);
              increasing = false;
              decreasing = false;
              menu = 1;
              enter = false;
              pressed = false;
            }
          }
        }
        runfeeder();
        checkknob();
        if (increasing)
        {
          menu = 1;
          increasing = false;
        }
        else if (decreasing)
        {
          menu--;
          decreasing = false;
        }
      }
      break;

    //Catch all case
    default:
      menu = 1;
      break;
  }
}

void setup()
{
  //Pinmode config
  pinMode(proxPin, INPUT_PULLUP);
  pinMode(encoderswitch, INPUT_PULLUP);
  pinMode(channelA, INPUT_PULLUP);
  pinMode(channelB, INPUT_PULLUP);
  pinMode(directionPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(enablePin, OUTPUT);

  //LCD Initialization
  Serial.begin(9600);
  displayOn();
  setContrast(40);
  backlightBrightness(6);
  clearLCD();
  cursorHome();
  definecustom();

  //Stepper Initialization
  digitalWrite (enablePin, HIGH);       //Enable coils for holding
  coneup();

  //Splash Screen
  clearLCD();
  cursorHome();
  Serial.print(F("  Media Feeder  "));
  cursorLine2();
  Serial.print(F("Software v2.19.0"));
  delay(textDelay);

  //Calibration Initialization
  fetch();                              //That is so fetch!

  menu = 1;
  feed = false;
}

void loop()
{
  menuselect();
}
