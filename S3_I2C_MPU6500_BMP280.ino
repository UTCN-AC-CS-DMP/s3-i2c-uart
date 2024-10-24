#include <HardwareSerial.h>
#include <Wire.h>

// -=| UART |=-
HardwareSerial HWSerial(2);
const int HW_SERIAL_TX = 4;
const int HW_SERIAL_RX = 5;

// -=| I2C |=-
const int I2C_SDA = 9;
const int I2C_SCL = 10;

// -=| MPU-6500 |=-
const int IMU_ADDR_ON_BUS = 0x68;
const int IMU_REG_PWR_MGMT_1 = 0x6B;
const int IMU_VAL_PWR_MGMT_1_RESET = 0x80;
const int IMU_REG_WHO_AM_I = 0x75;

// -=| BMP280 |=-
const int BMP_ADDR_ON_BUS = 0x77;
const int BMP_REG_RESET = 0xE0;
const int BMP_VAL_RESET_RESET = 0xB6;
const int BMP_REG_ID = 0xD0;

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
  delay(200);

  // Search for all devices on the bus
  // getAllI2CDevciesOnBus();

  wakeUpIdentifyDevice(IMU_ADDR_ON_BUS, IMU_REG_PWR_MGMT_1, IMU_VAL_PWR_MGMT_1_RESET, IMU_REG_WHO_AM_I);

  wakeUpIdentifyDevice(BMP_ADDR_ON_BUS, BMP_REG_RESET, BMP_VAL_RESET_RESET, BMP_REG_ID);
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

void i2cWriteToRegister(const uint8_t deviceAddress, const uint8_t registerAddress, const uint8_t value) {
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t i2cReadByteFromRegister(const uint8_t deviceAddress, const uint8_t registerAddress) {
  uint8_t value = 0;
  Wire.beginTransmission(deviceAddress);
  Wire.write(registerAddress);
  Wire.endTransmission(false);
  Wire.requestFrom(deviceAddress, static_cast<uint8_t>(1));
  while (Wire.available()) {
    value = Wire.read();
  }
  return value;
}

void wakeUpDevice(const uint8_t deviceAddress, const uint8_t resetAddress, const uint8_t resetValue) {
  i2cWriteToRegister(deviceAddress, resetAddress, resetValue);
  delay(10);
}

void identifyDevice(const uint8_t deviceAddress, const uint8_t whoAmIRegAddress) {
  const uint8_t responseWhoAmI = i2cReadByteFromRegister(deviceAddress, whoAmIRegAddress);
  Serial.printf("Device address on I2C bus: 0x%x, WHO_AM_I reg val --> 0x%x\n", deviceAddress, responseWhoAmI);
}

void wakeUpIdentifyDevice(const uint8_t deviceAddress, const uint8_t resetAddress, const uint8_t resetValue, const uint8_t whoAmIRegAddress) {
  wakeUpDevice(deviceAddress, resetAddress, resetValue);
  identifyDevice(deviceAddress, whoAmIRegAddress);
}