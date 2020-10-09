/* This code is for a hands-free temperature checking project
   using an MLX90614,Proximity sensor, and Oled display
*/

//Libraries for Low power functions
#include <LowPower.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>

//Libraries & Parameters for SSD1306 OLED Display
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

int tempError = 0; 
unsigned long wait_time = 4000;

int thresh = 400;      // distance threshhold value
int sensorPin = A0;    // input pin for distance sensor
int proxSwitch = 10;
int distance = 0; 
bool presence = 0;
short int minDist = 5;
short int maxDist = 14;


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET); //Declaring the display name (display)
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

void setup() {
  //  Low power & CPU speed settings
  //  CLKPR = 0x80;
  //  CLKPR = 0x01;

  //ADCSRA = 0;  // disable ADC

  power_spi_disable();    // Disable the Serial Peripheral Interface module.
  //power_timer0_disable(); // Disable the Timer 0 module..
  power_timer1_disable(); // Disable the Timer 1 module.
  power_timer2_disable(); // Disable the Timer 2 module
  //power_twi_disable();    // Disable the Two Wire Interface module.
  power_usart0_disable(); // Disable the USART 0 module.

  //  Display setup
  mlx.begin();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.setRotation(2);
  display.clearDisplay();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 4);
  display.println("YSU Robotics: ");
  display.setTextSize(2);
  display.setCursor(50, 0);
  display.println(distance, 1);
  delay(1000);


  //  I/O Setup
  pinMode(sensorPin, INPUT);
  pinMode(proxSwitch, OUTPUT);
  digitalWrite(proxSwitch, LOW);
}


void loop() {
  while(presenceCheck()) {
    if(distance >= maxDist){
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(50, 17);
        display.println("Move closer");
    }
    else if (distance < minDist){
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(50, 17);
        display.println("Move back");
    }
    else {
        display.clearDisplay();
        display.setTextSize(2);
        display.setCursor(50, 17);
        display.println("Hold position...");
        delay(400);
        TempScan();

    }

  }
    digitalWrite(proxSwitch, LOW);
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);
    // TIMER2_OFF, TIMER1_OFF, TIMER0_OFF, SPI_OFF, USART0_OFF, TWI_OFF
  }

// This function runs the ADC and distance sensor
bool presenceCheck(void) {
  
  digitalWrite(proxSwitch, HIGH);
  delay(150);
  distance = analogRead(sensorPin);
  if (distance >= thresh) {
    return true;
  }
  else {
    return false;
  }
  digitalWrite(proxSwitch, LOW);
}


void TempScan(void) {
  unsigned int current_counter = millis();
  display.ssd1306_command(SSD1306_DISPLAYON);
  while (presenceCheck() && (current_counter - millis() > wait_time)) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 4);
    display.println("Analog");
    display.setTextSize(2);
    display.setCursor(50, 0);
    display.println(distance, 1);

    display.setCursor(110, 0);
    display.println("U");

    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 20);
    display.println("Target");

    display.setTextSize(2);
    display.setCursor(50, 17);
    display.println(mlx.readObjectTempF() + tempError, 1);

    display.setCursor(110, 17);
    display.println("F");
    display.display();

  }
  delay(2000); 
  display.ssd1306_command(SSD1306_DISPLAYOFF);

}
