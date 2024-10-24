#include <HardwareSerial.h>
#include <Wire.h>

// On the S3 you may define any valid GPIO
//   pins as either UART or I2C

// -=| UART |=-
// On the S3, the Serial used for debug / prog
//   the Serial1 one is for internal flash
//   so it's recommended to use Serial2
HardwareSerial HWSerial(2);
const int HW_SERIAL_TX = 4;
const int HW_SERIAL_RX = 5;

// -=| I2C |=-
const int I2C_SDA = 9;
const int I2C_SCL = 10;

void setup() {
  // Initialize UART
  HWSerial.begin(9600, SERIAL_8N1, HW_SERIAL_RX, HW_SERIAL_TX);
  Serial.begin(115200);
  
  // ESP32 boards have a delay with UART for some reason from a reset after
  // uploading new code to the development board
#if 1  // board was newly programmed
  delay(1000);
#else
  // https://www.arduino.cc/reference/en/language/functions/communication/serial/ifserial/
  while (!Serial) {
    ;  // wait for the serial port to connect. Needed for native USB
  }
#endif

  Serial.println("INFO: Serial port initialized");


  // Initialize I2c
  Wire.begin(I2C_SDA, I2C_SCL);

  // Search for all devices on the bus
  getAllI2CDevciesOnBus();
}

void loop() {
  // readFromUARTContinuously();
}

void readFromUARTContinuously() {
  while (HWSerial.available()) {
    Serial.print(char(HWSerial.read()));
  }
}

void getAllI2CDevciesOnBus() {
  byte error, address;  // Variables to store error flag and device address
  int nDevices;         // Variable to count number of devices found

  Serial.println("Scanning for devices");
  Serial.println("--------------------");

  nDevices = 0;                                  // Initialize device count to zero
  for (address = 1; address < 127; address++) {  // Scan all addresses from 1 to 126
    Wire.beginTransmission(address);             // Start transmission to the I2C device
    error = Wire.endTransmission();              // End transmission and check for error

    if (error == 0) {                                  // If no error, device found
      Serial.print("I2C device found at address 0x");  // Print device found message
      if (address < 16) Serial.print("0");             // Add leading zero for single digit addresses
      Serial.println(address, HEX);                    // Print address in hexadecimal format
      nDevices++;                                      // Increment device count
    } else if (error == 4) {                           // If error 4, unknown error
      Serial.print("Unknown error at address 0x");     // Print unknown error message
      if (address < 16) Serial.print("0");             // Add leading zero for single digit addresses
      Serial.println(address, HEX);                    // Print address in hexadecimal format
    }
  }

  if (nDevices == 0)                         // If no devices found
    Serial.println("No I2C devices found");  // Print no devices found message
  else
    Serial.printf("Found %d devices in total\n", nDevices);  // Print done message
}