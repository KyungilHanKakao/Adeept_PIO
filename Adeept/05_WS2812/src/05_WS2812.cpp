#include <arduino.h>
#include <Adafruit_NeoPixel.h>

#define LED_PIN    A3         // WS2812 connect to pin A3 .
#define NUM_LEDS   10         // LED number.
Adafruit_NeoPixel pixels(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  pixels.begin();         // Initialize the NeoPixel library.
  pixels.setBrightness(5); // Set WS2812 LED brightness.
}

void loop() {
  
  for(int i=0;i<=NUM_LEDS;i++){
  // for(int i=0;i<=7;i++){
      pixels.setPixelColor(i,pixels.Color(255,0,0)); // red
  }
  pixels.show();  // Send the command.
  delay(1000); // delay 0.5s

  for(int i=0;i<=NUM_LEDS;i++){
  // for(int i=0;i<=7;i++){
      pixels.setPixelColor(i,pixels.Color(0,255,0)); // green
  }
  pixels.show();  // Send the command.
  delay(1000); // delay 0.5s

  for(int i=0;i<=NUM_LEDS;i++){
  // for(int i=0;i<=7;i++){
      pixels.setPixelColor(i,pixels.Color(0,0,255)); // blue
  }
  pixels.show();  // Send the command.
  delay(1000); // delay 0.5s
  pixels.clear(); // light off LED
  pixels.show(); // Send the command.
  delay(500); 
}


void WS2812Color(int num, int R, int G, int B){
  pixels.setPixelColor(num,pixels.Color(R,G,B));
  pixels.show();    
}

void WS2812ColorAll(int R, int G, int B){
    for(int i=0; i<=NUM_LEDS; i++){
      pixels.setPixelColor(i,pixels.Color(R,G,B));
  }
  pixels.show();  
}
