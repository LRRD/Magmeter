//Mason Parrone 6/20/17 - Changed aVal and bVal after calculating linear equation following short volumetric test with Steve Gough.

#include <LiquidCrystal.h>

LiquidCrystal lcd (9, 8, 4, 5, 6, 7);  // LCD 4-bit interface.
static double set_PWM = 0; //Sets starting PWM speed to 0. 
static int encoder_count = 0;
unsigned long timeold;
int AvgFlowPrint = 0;
int magMeterPin = 2; //Pin connected to signal wire of magmeter sensor
float magFlowRate = 0; //Stores flow rate calculate din readMagmeter
//Take readings for motor rpm vs flow rate. Plot in excel. Linear regression. Ax + B = aVal and bVal
float aVal = 1.7716;  //a value in polynomial for rps v flow
float bVal = -14.0831; // b value in ml/s = clicks/sec *a +b
 float freQuency = 0;
 int released;  // trouble with switch, can't understand Alix code
 
void readMagmeter ()
{
  
  
//function reads just one pulse :
int durationLow = pulseIn(magMeterPin, LOW);  //returns period in MICROseconds between low and high pulses, 
                                            //i.e. half of square wave; waits for pin to go LOW, starts timing, and stops
                                            //when pin goes LOW again, so in this case times the widch of the square wave
                                            // "plateau." I verified that plateaus and valleys sent by the magmeter 
                                            // are exactly equal.


float durationLowfloat = (float) durationLow;

//Serial.print ("durationLowfloat after float casting ");
//Serial.println (durationLowfloat);

freQuency = 1000000.000/(durationLowfloat *2.000);  //returns Hz; 2.000 multiplier gives full wavelength (i.e. valley, too)
                                                    //operating correctly; tested with VOM, changed from older code; July 14, 2010

freQuency = abs (freQuency);     // were geting negative freq numbers (though accurate) somehow

////////////////////////////////////////////////////////////////////////////////////////////////////////////calculate flow
//magFlowRate = ((2.95 * freQuency)-17.00) ;      // from testing at LRRD early July 2011; with 1/2" pipe.
//magFlowRate = ((1.747 * freQuency)-9.1) ;  //New testing, October 28, 2016 for Ft. Louis version.
magFlowRate = ((aVal * freQuency) + bVal) ;  //Ax + B  (aVal * freQuency) + bVal)
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
}


////////////////////////////////////////////////////////////////////////////
// Encoder handling. ///////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////
// Taken From Alix controller code by Chris Alix.
// The encoder count value.
// The sequence of gray codes in the increasing and decreasing directions.
static uint8_t cw_gray_codes[4] = { 2, 0, 3, 1 };
static uint8_t ccw_gray_codes[4] = { 1, 3, 0, 2 };
// The gray code we last read from the encoder.
static uint8_t previous_gray_code = 0;
// Reset the encoder to zero.
static void reset_encoder()
{
  encoder_count = 0;
}

// Look for encoder rotation, updating encoder_count as necessary.
static void check_encoder()
{
  // Get the Gray-code state of the encoder.
  // A line is "low" if <= 512; "high" > 512 or above.
  int gray_code = ((analogRead(3) > 512) << 1) | (analogRead(4) > 512);
  
  // If the gray code has changed, adjust the intermediate delta. 
  if (gray_code != previous_gray_code) {
    
      // adding 4 units for each click for Grayhill 62A11 encoder, removed "half ticks"
      if (gray_code == cw_gray_codes[previous_gray_code]) {
         encoder_count = encoder_count +2;
      } else if (gray_code == ccw_gray_codes[previous_gray_code]) {
         encoder_count = encoder_count - 2;
      }//all half tick code here removed for Grayhill 62A
      previous_gray_code = gray_code;
  }
}

////////////////////////////////////////////////////////////////////////////
// Switch handling. ////////////////////////////////////////////////////////

static unsigned int switch_was_down = 0;
static unsigned int switch_released()
{
  // The switch is depressed if its sense line is low (<512).
  int switch_is_down = (analogRead(19) < 512);

  // The action takes place when the switch is released.
  released = (switch_was_down && !switch_is_down);
  
  // Remember the state of the switch.
  switch_was_down = switch_is_down;
  
  // Was the switch just released?
  return released;
}

///////////////////////////////////////////////////////////////////////////////////
///// Motor Speed Control  ///////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////


void set_motor_PWM(){
  if (encoder_count >= 255){
    encoder_count = 255;
  }
  
  if (encoder_count <= 0){
    encoder_count = 0;
  }
  
  for(set_PWM; set_PWM < encoder_count; set_PWM ++){
    analogWrite(3, set_PWM);  //digital pin 3, sloppy programming, should be named; gough
  }//this is pin to MOSFET gate
  //be sure to leave off pulldown resistor on board used this way; R2 I think
  
  for(set_PWM; set_PWM > encoder_count; set_PWM --){
    analogWrite(3, set_PWM);
  }
  
}

/// Screen Refresh.  /////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

long elapsed_ms; //used to refresh the lcd Screen  
static long last_display_update_ms = 0; //used to refresh the lcd Screen 

void refresh_lcd()
  {
        elapsed_ms = millis() - last_display_update_ms;
      
     if ((last_display_update_ms == 0) || (elapsed_ms >= 500))
     { 
            readMagmeter();  //works great in this timed loop
            lcd.clear ();  

                     lcd.print ("Q = ");
                     int magFlowRateInt = (int) magFlowRate;      //converts to integer so no decimals in ml reading
                                                                 //should use a real rounding function, but error             
             
             if (magFlowRateInt < 1700)
             {                        //will be tiny
                     lcd.print (magFlowRateInt);
                     lcd.print (" ml/s");
              }
              else lcd.print ("ERROR");
                     lcd.setCursor (0,1);
                     lcd.print ("freq = ");
                     lcd.print(freQuency);
                     lcd.print (" Hz");

             last_display_update_ms = millis();

             
             Serial.print(millis()/1000);
             Serial.print (" " );
             Serial.print (",");
              Serial.print (encoder_count);// gives encoder count,flow rate
              //CSV format for analysis of relationship diagnostic Oct 26 Gough
              Serial.print (",");
              Serial.print (" " );
              Serial.println (magFlowRate);             
           }           
  }

//////////////////////////////////////////////////////////////////////////////////
// Setup / Main. /////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////

void setup(){

	//Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.print("Magmeter");
  lcd.setCursor(0,1);
    lcd.print("Controller");
  delay(2500);
  
  pinMode(3, OUTPUT); // init pin 3 as output. 
  
  digitalWrite(19, HIGH); //set switch pullup for Grayhill encoder
  ////NOTE:  use "digitalWrite" to set pullup resistor even though it's an analog pin
  //analogs can behave exactly like digitals in some ways.
  reset_encoder();
}

void loop(){
  int switch_event;
  check_encoder();
  switch_event = switch_released();  //should run routine that checks switch
  //on pin 19.
  refresh_lcd();
 

  if (analogRead(19) < 512)
  {
	   reset_encoder();
	   	set_motor_PWM();  // should turn motor off
		lcd.clear();
		lcd.print("switch");
		delay (200);
	 }
   
  if (encoder_count != set_PWM){
    set_motor_PWM();
  }

}
