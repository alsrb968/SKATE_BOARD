#define NEOPIXEL_STYLE_1 0
#define NEOPIXEL_STYLE_2 1
#define NEOPIXEL_STYLE_3 0


#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN            6
#define NUMPIXELS      30

#define LEAD_SWITCH_PIN 3

Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

const int numReadings = 10;
uint32_t readings[numReadings];
int readIndex = 0;
int total = 0;

bool oldLeadState;
uint32_t rpm;
uint32_t startT;
uint32_t termT;
const uint32_t oneM = 60000;

void MovingAverageFilterInit(){
  int readIndex = 0;
  int total = 0;
  for(int i : readings){
    i = 0;
  }
}

unsigned long MovingAverageFilterRpm(unsigned long rpm) {
  readIndex %= numReadings;
  readings[readIndex++] = rpm;
  if(total < numReadings) total++;
  uint32_t aver = 0;
  for(uint32_t i : readings){
    aver += i;
  }
  aver /= total;
  
  return aver;
}

void setup() {
  Serial.begin(115200);
  pinMode(LEAD_SWITCH_PIN, INPUT);

  oldLeadState = 0;
  rpm = 0;
  startT = 0;
  termT = 0;
  MovingAverageFilterInit();

  pixels.begin(); // This initializes the NeoPixel library.
}

void loop() {
  int newLeadState = digitalRead(LEAD_SWITCH_PIN);
  if(oldLeadState > newLeadState) { //Falling Edge
    termT = millis() - startT;
    rpm = MovingAverageFilterRpm(oneM / termT);
    Serial.println(rpm);
    startT = millis();
  } else {
    if(millis() - startT > 2000){
      rpm = MovingAverageFilterRpm(0);
      Serial.println(rpm);
      delay(100);
    }
  }
  oldLeadState = newLeadState;

  for(int i=0;i<NUMPIXELS;i++){
#if (NEOPIXEL_STYLE_1 == 1)
    if(i < rpm/20)  pixels.setPixelColor(i, pixels.Color(255, 255, 255));
    else            pixels.setPixelColor(i, pixels.Color(0, 0, 0));
#elif (NEOPIXEL_STYLE_2 ==1)
    double dim = 255.0 / 600.0;
    int res = (int)(rpm * dim);
    pixels.setPixelColor(i, pixels.Color(res, res, res));
#elif (NEOPIXEL_STYLE_3 == 1)

#endif
    pixels.show();
  }
}
