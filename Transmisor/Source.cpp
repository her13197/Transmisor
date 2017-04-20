#include <SPI.h>
#include "RF24.h"
#include "nRF24L01.h"


byte addresses[][6]{ "1Node","2Node" };
bool radioNumber{ 0 };
RF24 radio(7, 8);
bool role{ false };
float rfData;
unsigned long timeRF;


void comenzar() 
{
  
	Serial.begin(115200);
	Serial.println("Iniciando el programa");
  
  
	radio.begin();
	radio.setPALevel(RF24_PA_LOW);

	if (radioNumber) {
		radio.openWritingPipe(addresses[1]);
		radio.openReadingPipe(1, addresses[0]);
	}
	else {
		radio.openWritingPipe(addresses[0]);
		radio.openReadingPipe(1, addresses[1]);
	}

	rfData = 1.22 ;
	radio.startListening();
  
}


void ciclo() 
{
	radio.stopListening();

	Serial.println(F("now sending..."));

	unsigned long timeRf = micros();
	rfData = 1000 ;
	if (!radio.write(&rfData, sizeof(rfData))) 
	{
		Serial.println(F("Failed"));
	}
	radio.startListening();

	unsigned long started_waiting_at = micros();
	boolean timeout{ false };

	while (!radio.available()) {                           
		if (micros() - started_waiting_at > 200000) {
			timeout = true;
			break;
		}
	}

	if(timeout)
	{
		Serial.println(F("failed, response timed out."));
	}
	else {
		radio.read(&rfData, sizeof(rfData));
		unsigned long time = micros();

		Serial.print(F("Sent "));
		Serial.print(time);
		Serial.print(F(", Got response "));
		Serial.print(timeRF);
		Serial.print(F(", Round-trip delay "));
		Serial.print(time - timeRF);
		Serial.print(F(" microseconds Value "));
		Serial.println(rfData);
	}
	delay(500);

}
