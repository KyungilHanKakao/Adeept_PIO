#include <Arduino.h>
#include <IRremote.h> //Call the library corresponding to the infrared remote control.

#define RECV_PIN     2        // Infrared receiving pin

int switch_irr(int irr_data);
IRrecv irrecv(RECV_PIN);      // Create a class object used to receive class
decode_results results;       // Create a decoding results class object
unsigned long strP,strNow;

void setup()
{
  Serial.begin(115200);       // Initialize the serial port and set the baud rate to 115200
  irrecv.enableIRIn();        // Start the receiver
  Serial.print("IRrecvDemo is now running and waiting for IR message on Pin ");
  Serial.println(RECV_PIN);   //print the infrared receiving pin
}

void loop()
{
  if (irrecv.decode(&results)) {          // Waiting for decoding
    unsigned long value = results.value;
   value = switch_irr(value);
   Serial.println(value);
    // Serial.println(value, HEX);           // Print out the decoded results
    irrecv.resume();                      // Release the IRremote. Receive the next value
  }
  delay(100);
}

int switch_irr(int irr_data)
{
  switch(irr_data)
  {
  case 16750695: return 0;
  case 16753245: return 1;
  case 16736925: return 2;
  case 16769565: return 3;
  case 16720605: return 4;
  case 16712445: return 5;
  case 16761405: return 6;
  case 16769055: return 7;
  case 16754775: return 8;
  case 16748655: return 9;
  case 16738455: return 10; // *
  case 16756815: return 11; // #
  case 16718055: return 12; // up
  case 16730805: return 13; // down
  case 16716015: return 14; // left
  case 16734885: return 15; // right
  case 16726215: return 16; // ok
  }
}
