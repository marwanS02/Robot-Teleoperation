#include "Adafruit_MLX90393.h"
#include <SparkFun_I2C_Mux_Arduino_Library.h>
#include <time.h>
#include <ArduinoEigen.h>

/**
  * TO DO LIST before running
  * 1) Check the params: NUM_SENSORS and NUM_MAGNETS in lm_library_pro.h
  * 2) If you want to use the dipole model or the cyl model check the POSE_PARAMS in lm_library_pro.h
  * 3) Modify the initial condition x0, x0_dist, x0_cyl, x0_cyl_dist based on the number of magnets in lm_library_pro.cpp
  * 4) Check the Considerdist flag to decide whether to estimate for external disturbances
  * 5) Check the Function calls in the main
  * 6) If you use the cylinder model, check also R_j, L_j and M_star
  */

#define QWIIC_MUX_ADDRESS_0     0x70   // Default i2c address of Sparkfun i2c MUX
#define BAUDRATE                921600 // Baud
#define CLOCK_I2C               400000 // Hz

#define SERIAL_HEADER           0xA2
#define SERIAL_TAIL             0xB2

#define NUM_GRID_5S             2
#define NUM_GRID_4S             2
#define NUM_SENSORS             (5*NUM_GRID_5S + 4*NUM_GRID_4S)
#define NUM_MUX_PORTS           (2*NUM_GRID_5S + 1*NUM_GRID_4S)

#define BUFF_SIZE               3*NUM_SENSORS + 2

// Possible addresses for the magnetic sensors (Adafruit Boards)
uint8_t addresses[] = { 0x18, 0x19, 0x1A, 0x1B };
uint8_t muxPorts[]  = { 1, 2, 3, 4, 5, 6 }; 
uint8_t connected2port[] = { 4, 4, 1, 4, 4, 1 };
QWIICMUX myMux_0;
uint8_t i2cAddress;
uint8_t portN;
uint8_t numSens;
uint8_t sensIdx;
Adafruit_MLX90393 sensors[NUM_SENSORS];
Eigen::MatrixXf sensorData = Eigen::MatrixXf::Zero(NUM_SENSORS, 3);

void setup() {

  /* Serial Port (USB) configuration */
  Serial.begin(BAUDRATE);
  while (!Serial)           // wait for serial on USB platforms
     delay(10);
  
  Serial.println("\n*** Parameters Setup ***");
  Serial.printf("NUM_SENSORS: %d\n", NUM_SENSORS);
  Serial.println("");
  Serial.printf("BUFF_SIZE: %d\n", BUFF_SIZE);
  Serial.println("");
  
  Serial.println("\n*** Setup ongoing ***");

  /* I2C peripheral settings */
  Serial.println("\n*** Initialize I2C Network ***");
  Wire.begin();
  Wire.setClock(CLOCK_I2C);
  Serial.println("\n*** done. ***");

  /* MUX initialization */
  if (myMux_0.begin(QWIIC_MUX_ADDRESS_0) == false){
    Serial.println("\n*** Mux 0 not detected. Program freezing... Check your Wiring. ***");
    while (1) { delay(10); }
  }
  Serial.println("\n*** Mux 0 detected. ***");

  Serial.println("\n*** Connecting to Magnetic Sensors... ***");

  sensIdx = 0;
  // scanning the MUX port
  for (uint8_t i = 0; i < NUM_MUX_PORTS; i++) {

    portN = muxPorts[i];
    myMux_0.setPort(portN);
    numSens = connected2port[i]; // number of sensors at port

    Serial.printf("Access port%d\n", portN);

    for (uint8_t j = 0; j < numSens; j++) {

      i2cAddress = addresses[j];

      Serial.printf("iteration°%d\n", sensIdx + 1);
      if (!sensors[sensIdx].begin_I2C(i2cAddress)) {
        Serial.printf("ERROR: Could not connect to magnetometer n°%d\n", sensIdx + 1);
        while (1)
          ;
      }

      // Configure MLX90393
      sensors[sensIdx].setGain(MLX90393_GAIN_1X);
      sensors[sensIdx].setResolution(MLX90393_X, MLX90393_RES_19);
      sensors[sensIdx].setResolution(MLX90393_Y, MLX90393_RES_19);
      sensors[sensIdx].setResolution(MLX90393_Z, MLX90393_RES_19);
      sensors[sensIdx].setOversampling(MLX90393_OSR_0);
      sensors[sensIdx].setFilter(MLX90393_FILTER_6);

      // Set Burst mode
      sensors[sensIdx].setBurstMode(MLX90393_AXIS_ALL, 0);

      // Start Burst mode for each sensor
      if (!sensors[sensIdx].startBurstMode()) {
        Serial.printf("Failed to start measurement for sensor %d\n", sensIdx + 1);
      }

      Serial.printf("\n*** Finished to configure sensor %d\n", sensIdx + 1, ". ***");
      sensIdx++;

    }

  }

  Serial.println("\n*** End of Program Setup. ***");

  Serial.flush();
  delay(1000);

}

void loop() {

  sensIdx = 0;
  //Read measurements from all sensors
  for (uint8_t i = 0; i < NUM_MUX_PORTS; i++) {

    portN = muxPorts[i];
    myMux_0.setPort(portN);

    numSens = connected2port[i]; // number of sensors at port

    for (uint8_t j = 0; j < numSens; j++) {

      float x, y, z;
      if (sensors[sensIdx].readBurstMeasurement(&x, &y, &z)) {
        sensorData(sensIdx, 0) = x;
        sensorData(sensIdx, 1) = y;
        sensorData(sensIdx, 2) = z;
        // Serial.print(sensorData(sensIdx, 0));
        // Serial.print(" ");
        // Serial.print(sensorData(sensIdx, 1));
        // Serial.print(" ");
        // Serial.println(sensorData(sensIdx, 2));
      } 
      else {
        Serial.printf("Unable to read XYZ data from sensor %d.\n", sensIdx + 1);
      }
      sensIdx++;
    }

  }

  streamMagTracking(sensorData);

}

// Stream to Matlab the position and orientation of the magnet
void streamMagTracking(Eigen::MatrixXf magData) {
  float to_stream[BUFF_SIZE];
  to_stream[0] = (float) SERIAL_HEADER;
  to_stream[BUFF_SIZE - 1] = (float) SERIAL_TAIL;

  for (uint i = 0; i < NUM_SENSORS; i++){
    //Serial.println(i);
    to_stream[1+3*i]   = magData(i,0);
    to_stream[1+3*i+1] = magData(i,1);
    to_stream[1+3*i+2] = magData(i,2);
  }
  
  Serial.write((uint8_t*)to_stream, sizeof(to_stream));
}