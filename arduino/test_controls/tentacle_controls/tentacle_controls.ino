#include <Arduino.h>

// Declare pins for controlling actuators
const int actuatorPins[] = {2, 3, 4, 5, 6, 7, 8, 9, 10};

// Parametrize the number of actuators
const int num_actuators_used;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < num_actuators_used; i++)
  {
    pinMode(actuatorPins[i], OUTPUT);
  }

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  if (Serial.available() >= num_actuators_used) { // Check if there's enough data available to read
    // Read incoming data
    uint8_t buffer[num_actuators_used];
    Serial.readBytes(buffer, num_actuators_used);

    // Parse received data
    char actuator_states[] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

    for (int i = 0; i < num_actuators_used; i++)
    {
      // Assign actuator states
      memcpy(&actuator_states[i], &buffer[i*sizeof(char)], sizeof(char));

      if (actuator_states[i] != 0)
      {
        // Turn actuator ON
        digitalWrite(actuatorPins[i], HIGH);

        // Trigger LED_BUILTIN
        digitalWrite(LED_BUILTIN, HIGH);
      }

      // Relay data back through Serial
      Serial.print(static_cast<char>(actuator_states[i])); 

    }
    Serial.println(); // End of message

    // // Trigger LED_BUILTIN if any actuator is switched ON
    // if (actuator_state[0] || 
    //     actuator_state[1] || 
    //     actuator_state[2] || 
    //     actuator_state[3] || 
    //     actuator_state[4] ||
    //     actuator_state[5] ||
    //     actuator_state[6] ||
    //     actuator_state[7] ||
    //     actuator_state[8])
    // {
    //   digitalWrite(LED_BUILTIN, HIGH);
    // }    

    // // Relay the data back through serial
    // Serial.print('!'); // Start character
    // for (int i = 0; i < 9; i++)
    // {
    //   Serial.print(static_cast<char>(actuator_state[i])); // Speed value as char
    // }
    // Serial.println(); // End of message
  }
}
