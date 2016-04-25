// TODO:
// Figure out how to properly read magnetometer
// Finish calcularing bearing with haversine formula
// Figure out how to clear pixels on the led array
// Measure and cut/drill enclosure
// Wire everything together


#include <Wire.h>                 // Library for I2C
#include <Adafruit_NeoPixel.h>    // Library for LED Ring

// Magnometer Defines
#define address_mag 0x1E // I2C 7bit address of HMC5883 (Magnetometer)

// LCD Defines
#define PIN_SCE   7    // LCD CS  .... Pin 3
#define PIN_RESET 6    // LCD RST .... Pin 1
#define PIN_DC    5    // LCD Dat/Com. Pin 5
#define PIN_SDIN  11   // LCD SPIDat . Pin 6
#define PIN_SCLK  13   // LCD SPIClk . Pin 4


// LCD Defines Cont.
#define LCD_C     LOW
#define LCD_D     HIGH
#define LCD_X     84
#define LCD_Y     48
#define LCD_CMD   0

// LED Ring defines
#define LED_PIN 12
#define NUM_LEDS 16
#define BRIGHTNESS 20

// Define the max number of locations the user can haversine
#define MAX_LOCATIONS 5

// Instantiate the LED ring
Adafruit_NeoPixel ring = Adafruit_NeoPixel(NUM_LEDS, LED_PIN/*, NEO_GRB + NEO_KHZ800*/);

// A structure that holds data for a user's specified location
struct location
{
	double latitude;
	double longitude;
	char locationName[10];
}

// The array of location struct to hold multiple user specified locations
struct location locations[MAX_LOCATIONS];

volatile int currentLocation;
volatile int numLocations;

// Contains constants for displaying characters on NOKIA display
static const byte ASCII[][5] =
{
   {0x00, 0x00, 0x00, 0x00, 0x00} // 20  
  ,{0x00, 0x00, 0x5f, 0x00, 0x00} // 21 !
  ,{0x00, 0x07, 0x00, 0x07, 0x00} // 22 "
  ,{0x14, 0x7f, 0x14, 0x7f, 0x14} // 23 #
  ,{0x24, 0x2a, 0x7f, 0x2a, 0x12} // 24 $
  ,{0x23, 0x13, 0x08, 0x64, 0x62} // 25 %
  ,{0x36, 0x49, 0x55, 0x22, 0x50} // 26 &
  ,{0x00, 0x05, 0x03, 0x00, 0x00} // 27 '
  ,{0x00, 0x1c, 0x22, 0x41, 0x00} // 28 (
  ,{0x00, 0x41, 0x22, 0x1c, 0x00} // 29 )
  ,{0x14, 0x08, 0x3e, 0x08, 0x14} // 2a *
  ,{0x08, 0x08, 0x3e, 0x08, 0x08} // 2b +
  ,{0x00, 0x50, 0x30, 0x00, 0x00} // 2c ,
  ,{0x08, 0x08, 0x08, 0x08, 0x08} // 2d -
  ,{0x00, 0x60, 0x60, 0x00, 0x00} // 2e .
  ,{0x20, 0x10, 0x08, 0x04, 0x02} // 2f /
  ,{0x3e, 0x51, 0x49, 0x45, 0x3e} // 30 0
  ,{0x00, 0x42, 0x7f, 0x40, 0x00} // 31 1
  ,{0x42, 0x61, 0x51, 0x49, 0x46} // 32 2
  ,{0x21, 0x41, 0x45, 0x4b, 0x31} // 33 3
  ,{0x18, 0x14, 0x12, 0x7f, 0x10} // 34 4
  ,{0x27, 0x45, 0x45, 0x45, 0x39} // 35 5
  ,{0x3c, 0x4a, 0x49, 0x49, 0x30} // 36 6
  ,{0x01, 0x71, 0x09, 0x05, 0x03} // 37 7
  ,{0x36, 0x49, 0x49, 0x49, 0x36} // 38 8
  ,{0x06, 0x49, 0x49, 0x29, 0x1e} // 39 9
  ,{0x00, 0x36, 0x36, 0x00, 0x00} // 3a :
  ,{0x00, 0x56, 0x36, 0x00, 0x00} // 3b ;
  ,{0x08, 0x14, 0x22, 0x41, 0x00} // 3c <
  ,{0x14, 0x14, 0x14, 0x14, 0x14} // 3d =
  ,{0x00, 0x41, 0x22, 0x14, 0x08} // 3e >
  ,{0x02, 0x01, 0x51, 0x09, 0x06} // 3f ?
  ,{0x32, 0x49, 0x79, 0x41, 0x3e} // 40 @
  ,{0x7e, 0x11, 0x11, 0x11, 0x7e} // 41 A
  ,{0x7f, 0x49, 0x49, 0x49, 0x36} // 42 B
  ,{0x3e, 0x41, 0x41, 0x41, 0x22} // 43 C
  ,{0x7f, 0x41, 0x41, 0x22, 0x1c} // 44 D
  ,{0x7f, 0x49, 0x49, 0x49, 0x41} // 45 E
  ,{0x7f, 0x09, 0x09, 0x09, 0x01} // 46 F
  ,{0x3e, 0x41, 0x49, 0x49, 0x7a} // 47 G
  ,{0x7f, 0x08, 0x08, 0x08, 0x7f} // 48 H
  ,{0x00, 0x41, 0x7f, 0x41, 0x00} // 49 I
  ,{0x20, 0x40, 0x41, 0x3f, 0x01} // 4a J
  ,{0x7f, 0x08, 0x14, 0x22, 0x41} // 4b K
  ,{0x7f, 0x40, 0x40, 0x40, 0x40} // 4c L
  ,{0x7f, 0x02, 0x0c, 0x02, 0x7f} // 4d M
  ,{0x7f, 0x04, 0x08, 0x10, 0x7f} // 4e N
  ,{0x3e, 0x41, 0x41, 0x41, 0x3e} // 4f O
  ,{0x7f, 0x09, 0x09, 0x09, 0x06} // 50 P
  ,{0x3e, 0x41, 0x51, 0x21, 0x5e} // 51 Q
  ,{0x7f, 0x09, 0x19, 0x29, 0x46} // 52 R
  ,{0x46, 0x49, 0x49, 0x49, 0x31} // 53 S
  ,{0x01, 0x01, 0x7f, 0x01, 0x01} // 54 T
  ,{0x3f, 0x40, 0x40, 0x40, 0x3f} // 55 U
  ,{0x1f, 0x20, 0x40, 0x20, 0x1f} // 56 V
  ,{0x3f, 0x40, 0x38, 0x40, 0x3f} // 57 W
  ,{0x63, 0x14, 0x08, 0x14, 0x63} // 58 X
  ,{0x07, 0x08, 0x70, 0x08, 0x07} // 59 Y
  ,{0x61, 0x51, 0x49, 0x45, 0x43} // 5a Z
  ,{0x00, 0x7f, 0x41, 0x41, 0x00} // 5b [
  ,{0x02, 0x04, 0x08, 0x10, 0x20} // 5c ¥
  ,{0x00, 0x41, 0x41, 0x7f, 0x00} // 5d ]
  ,{0x04, 0x02, 0x01, 0x02, 0x04} // 5e ^
  ,{0x40, 0x40, 0x40, 0x40, 0x40} // 5f _
  ,{0x00, 0x01, 0x02, 0x04, 0x00} // 60 `
  ,{0x20, 0x54, 0x54, 0x54, 0x78} // 61 a
  ,{0x7f, 0x48, 0x44, 0x44, 0x38} // 62 b
  ,{0x38, 0x44, 0x44, 0x44, 0x20} // 63 c
  ,{0x38, 0x44, 0x44, 0x48, 0x7f} // 64 d
  ,{0x38, 0x54, 0x54, 0x54, 0x18} // 65 e
  ,{0x08, 0x7e, 0x09, 0x01, 0x02} // 66 f
  ,{0x0c, 0x52, 0x52, 0x52, 0x3e} // 67 g
  ,{0x7f, 0x08, 0x04, 0x04, 0x78} // 68 h
  ,{0x00, 0x44, 0x7d, 0x40, 0x00} // 69 i
  ,{0x20, 0x40, 0x44, 0x3d, 0x00} // 6a j 
  ,{0x7f, 0x10, 0x28, 0x44, 0x00} // 6b k
  ,{0x00, 0x41, 0x7f, 0x40, 0x00} // 6c l
  ,{0x7c, 0x04, 0x18, 0x04, 0x78} // 6d m
  ,{0x7c, 0x08, 0x04, 0x04, 0x78} // 6e n
  ,{0x38, 0x44, 0x44, 0x44, 0x38} // 6f o
  ,{0x7c, 0x14, 0x14, 0x14, 0x08} // 70 p
  ,{0x08, 0x14, 0x14, 0x18, 0x7c} // 71 q
  ,{0x7c, 0x08, 0x04, 0x04, 0x08} // 72 r
  ,{0x48, 0x54, 0x54, 0x54, 0x20} // 73 s
  ,{0x04, 0x3f, 0x44, 0x40, 0x20} // 74 t
  ,{0x3c, 0x40, 0x40, 0x20, 0x7c} // 75 u
  ,{0x1c, 0x20, 0x40, 0x20, 0x1c} // 76 v
  ,{0x3c, 0x40, 0x30, 0x40, 0x3c} // 77 w
  ,{0x44, 0x28, 0x10, 0x28, 0x44} // 78 x
  ,{0x0c, 0x50, 0x50, 0x50, 0x3c} // 79 y
  ,{0x44, 0x64, 0x54, 0x4c, 0x44} // 7a z
  ,{0x00, 0x08, 0x36, 0x41, 0x00} // 7b {
  ,{0x00, 0x00, 0x7f, 0x00, 0x00} // 7c |
  ,{0x00, 0x41, 0x36, 0x08, 0x00} // 7d }
  ,{0x10, 0x08, 0x08, 0x10, 0x08} // 7e ←
  ,{0x00, 0x06, 0x09, 0x09, 0x06} // 7f →
};



// Called at startup of Arduino
// Sets up the modules 
void setup(void)
{
  // Initialize and clear nokia display
  LcdInitialise();
  LcdClear();
  
  //Initialize Serial and I2C communications
  Serial.begin(9600);
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address_mag); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
  
  // Setup the LEDs
  ring.setBrightness(BRIGHTNESS);
  ring.begin();
  ring.show();
  
  // Initialize the button for interrupts (Input A0)
  pinMode(A0, INPUT);
  digitalWrite(A0, HIGH);
  attachInterrupt(0, changeWaypoint, RISING);
  PCICR = 0x02;
  PCMSK1 = 0b00000001;
}

// Main loop
// - Reads destinations from user
// - Gets the user's heading
// - Calculates the distance and heading to a destination
// - Calculates the direction based on user and destination headings
// - Displays the direction on the LED ring
// - Displays the distance on the LCD
void loop(void)
{
  noInterrupts();
	
  int userHeading, destinationHeading, displayHeading;
  char xdirection[20];
  float distance;
  
  // Get destinations from user if changed
  readBluetoothConnection(&currentLocation, &numLocations);
  
  // Calculate the latest user heading
  getUserHeading(&userHeading);
   
  // Calculate the distance and heading from current location to destination
  calculateDistanceAndHeading(41.6611, locations[currentLocation].latitude, -91.5302, locations[currentLocation].longitude, &distance, &destinationHeading); 
   
  // Calculate the heading to get to the destination from the user's direction
  calculateHeadingFromUserHeading(userHeading, destinationHeading, &displayHeading);
   
  // Display the direction the user should go on the LED Ring
  displayDirectionOnLED(displayHeading);
  
  // Display the distance until the destination and the destination's name on the LCD
  displayDistanceAndWaypoint(distance);
  
  // Delay for a quarter second before updating anything
  // We will allow for the button interrupt here
  Interrupts();
  delay(250);
}

float calculateBearing(float x1, float y1, float x2, float y2) {
    atan2(x2-x1,y2-y1);
}

// Use haversine formula to calculate distance
void calculateDistanceAndHeading(float lat1, float lat2, float lon1, float lon2, float *distance, int *destinationHeading)
{
    // This may all need to be replaced by the TinyGPS++ library which can 
    // do distance and course, like the following
    distance = TinyGPSPlus.distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      lat2,
      lon2);
    *destinationHeading = TinyGPSPlus.courseTo(
      gps.location.lat(),
      gps.location.lng(),
      lat2,
      lon2);
      
    /*int r = 6371000; // radius of earth in meters
    float phi1 = lat1*PI/180;
    float phi2 = lat2*PI/180;
    float deltaPhi = (lat2-lat1)*PI/180;
    float deltaLambda = (lon2-lon1)*PI/180; 
    
    float a = sin(deltaPhi/2) * sin(deltaPhi/2) + cos(phi1) * cos(phi2) * sin(deltaLambda/2) * sin(deltaLambda/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    float d = r*c;
    //printf("distance = %f", d); //should be 245.139 miles or 394.5129 km*/
    float distLat = abs(lat1 - lat2) * 111194.9;
    float distLong = 111194.9 * abs(lon1 - lon2) * cos(radians((lat1 + lat2) / 2));
    float d = sqrt(pow(distLat, 2) + pow(distLong, 2));
    *distance = d;
    //Serial.println(d);
    
    /*
    float phi3 = 180*(lon1)/3.1415926535;//flon1  //also must be done in radians
    float phi4 = 180*(lon2)/3.1415926535;//x2lon  //radians duh.
    float heading = atan2(sin(phi4-phi3)*cos(phi2),cos(phi1)*sin(phi2)-sin(lat1)*cos(lat2)*cos(lon2-lon1))*2*3.1415926535;
    //float heading = 1; //place holder for testing other parts of the code
    heading = heading*180/3.1415926535;  // convert from radians to degrees
    int head = heading; //make it a integer now
    if(head<0)
    {
      heading+=360;   //if the heading is negative then add 360 to make it positive
    }
    printf("Heading = %f\n", heading); //should be 339

    *distance = d;
    *userBearing = heading;*/
}

// Reads the magnetometer through i2c protocol
// calcualtes the heading of the user based on
// the values read from the magnetometer
void getUserHeading(int *userHeading)
{
  int x, y, z;
  
  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address_mag);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address_mag, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  // Simple formula for calculating heading (asssumes flat magnetometer)
  float heading = atan2((float) y, (float) x);
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees
  *userHeading = heading * 180/M_PI;
}

// Displays a direction on the LED ring
void displayDirectionOnLED(int bearing)
{
    bearing = map(bearing, 0, 360, 0, ring.numPixels()-1);
    for(int i = 0; i < ring.numPixels(); i++)
    {
      ring.setPixelColor(i, ring.Color(0,0,0));  // clear all pixels
      delay(20);
    }
    ring.setPixelColor(bearing, ring.Color(0,255,0));  // set the specified pixel
    ring.show();
}

// Reads the bluetooth connection and
// adds any desired destinations to the
// array of destinations
void readBluetoothConnection()
{ 
  // Reset the current location to 0 if new locations are being sent
  if(Serial.available())
  {
	currentLocation = 0;
	numLocations = 0;
  }
  
  // Read in location values until there are no more or we reached max
  while (Serial.available() && numLocations < MAX_LOCATIONS) 
  {
	locations[numLocations].latitude = Serial.parseFloat();
	locations[numLocations].longitude = Serial.parseFloat();
	locations[numLocations].locationName = Serial.parse();
	numLocations++;
  }
}

// Displays the distance specified on the NOKIA
// screen and displays the units as kilometers
// also displays the waypoints name
void displayDistanceAndWaypoint(float distance)
{
  char displayDistance[20];
  
  // 6 numbers with 1 number after decimal place
  dtostrf(distance/1000, 6, 1, displayDistance); 
  
  LcdClear();
  
  gotoXY(20,2);
  LcdString(displayDistance);
  
  gotoXY(0,3);
  LcdString("kilometers");
  
  gotoXY(0,4);
  LcdString(locations[currentLocation].locationName);
}

// Calculates the direction to the destination
// from the perspective of the user's current
// heading
void calculateHeadingFromUserHeading(int userHeading, int destinationHeading, int *displayHeading) 
{
  *displayHeading = userHeading - (userHeading + destinationHeading);
}

// Interrupt service routine that changes the
// waypoint to the next one
ISR(PCINT1_vect)
{
  if(numLocations > 0)
  {
    currentLocation = (currentLocation + 1) % numLocations;
  }
  else
  {
	currentLocation = 0;
  }
}

// Display a character on the LCD
void LcdCharacter(char character)
{
  LcdWrite(LCD_D, 0x00);
  for (int index = 0; index < 5; index++)
  {
    LcdWrite(LCD_D, ASCII[character - 0x20][index]);
  }
  LcdWrite(LCD_D, 0x00);
}

// Clear the LCD
void LcdClear(void)
{
  for (int index = 0; index < LCD_X * LCD_Y / 8; index++)
  {
    LcdWrite(LCD_D, 0x00);
  }
}

// Initialize the LCD
void LcdInitialise(void)
{
  pinMode(PIN_SCE,   OUTPUT);
  pinMode(PIN_RESET, OUTPUT);
  pinMode(PIN_DC,    OUTPUT);
  pinMode(PIN_SDIN,  OUTPUT);
  pinMode(PIN_SCLK,  OUTPUT);

  digitalWrite(PIN_RESET, LOW);
  // delay(1);
  digitalWrite(PIN_RESET, HIGH);

  LcdWrite( LCD_CMD, 0x21 );  // LCD Extended Commands.
  LcdWrite( LCD_CMD, 0xBf );  // Set LCD Vop (Contrast). //B1
  LcdWrite( LCD_CMD, 0x04 );  // Set Temp coefficent. //0x04
  LcdWrite( LCD_CMD, 0x14 );  // LCD bias mode 1:48. //0x13
  LcdWrite( LCD_CMD, 0x0C );  // LCD in normal mode. 0x0d for inverse
  LcdWrite(LCD_C, 0x20);
  LcdWrite(LCD_C, 0x0C);
}

// Display a string on the LCD
void LcdString(char *characters)
{
  while (*characters)
  {
    LcdCharacter(*characters++);
  }
}

// Display a byte on the LCD
void LcdWrite(byte dc, byte data)
{
  digitalWrite(PIN_DC, dc);
  digitalWrite(PIN_SCE, LOW);
  shiftOut(PIN_SDIN, PIN_SCLK, MSBFIRST, data);
  digitalWrite(PIN_SCE, HIGH);
}

// Position cursor at an X,Y position
// x - range: 0 to 84
// y - range: 0 to 5
void gotoXY(int x, int y)
{
  LcdWrite( 0, 0x80 | x);  // Column.
  LcdWrite( 0, 0x40 | y);  // Row.  

}
