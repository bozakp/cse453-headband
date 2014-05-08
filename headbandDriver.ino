/*********************************************

Optical Substitution Headband - Main Driver

Phil Bozak
Ben Demmer
Sean Rosney
Devin Toth

CSE 453, Spring 2014

*********************************************/

#define N_MODULES 5
#define ACT_ON_TICKS 10
#define NO_OBJ_DELAY 259
#define REQ_DELAY_MS 10
#define OUT_PINS {0,1,2,3,4}
#define IN_PINS {7,8,9,10,11}
// Input Phase is up to 250ms
#define OUTPUT_PHASE_STEPS 1000
#define STEP_LENGTH_US 1000

#define TRUE 1
#define FALSE 0

class DistanceSensorDriver {
  int trigger_pin;
  int echo_pin;
 public:
  DistanceSensorDriver(int trigger_pin, int echo_pin) :
    trigger_pin(trigger_pin), echo_pin(echo_pin) {}

  /**
   * Gives the current distance reading from the sensor (in cm). Returns -1 if
   * the distance was outside of the expected range.  [0,517] otherwise.
   */
  int CurrentDistance() {
    digitalWrite(trigger_pin, LOW);  // make sure the trigger is low
    delayMicroseconds(2);
    digitalWrite(trigger_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigger_pin, LOW);
    unsigned long pulse_micros = pulseIn(echo_pin, HIGH, 30000);  // 30ms: max time to wait for reply
    int distanceCm = (pulse_micros < 2 || pulse_micros >= 30000) ? -1 : pulse_micros/58;
    return distanceCm;
  }
};

class ActuatorDriver {
  int base_pin;
  int additional_delay;
  int counter;
 public:
  ActuatorDriver(int base_pin) :
    base_pin(base_pin), additional_delay(NO_OBJ_DELAY), counter(0) {}

  void Extend(int extend) {
    digitalWrite(base_pin, extend ? HIGH : LOW);
  }
  void Timestep() {    
    if (additional_delay == NO_OBJ_DELAY) {
      this.Extend(FALSE);
      return;
    }
    counter++;
    this.Extend(counter <= ACT_ON_TICKS);
    if (counter > (ACT_ON_TICKS + REQ_DELAY_MS + additional_delay))
        counter = 0;
  }
  void set_additional_delay(int del) {
    additional_delay = del;
    counter = 0;
  }
};

class NoiseFilter {
  // Translate [-1,517] to [0,258] or NO_OBJ_DELAY(=259)
  // dist (cm) -> additionalDelay (ms)
 public:
  int Filter(int dist) {
    if (dist == -1)
      return NO_OBJ_DELAY;
    else
      return dist/2;
  }
};

class ModuleController {
  DistanceSensorDriver distance_sensor;
  ActuatorDriver actuator;
  NoiseFilter noise_filter;
public:
  ModuleController() :
    distance_sensor(0,0), actuator(0) {}
  ModuleController(int outputPin, int inputPin) :
    distance_sensor(outputPin, inputPin), actuator(outputPin) {}

  void Timestep() {
    actuator.Timestep();
  }
  void Extend(extend) {
    actuator.Extend(extend);
  }
  void UpdateDistanceDelay() {
    int dist_cm = distance_sensor.CurrentDistance();
    int actuator_delay = noise_filter.Filter(dist_cm);
    actuator.set_additional_delay(actuator_delay);
  }
};

class HeadbandController{
  ModuleController modules[N_MODULES];
  
  void ExtendAll(int extend) {
    for (int i=0;i<N_MODULES;i++) {
      modules[i].Extend(extend);
    }    
  }
  void InputPhase() {
    // This takes up to 250 ms
    for (int i=0; i<N_MODULES; i++) {
      modules[i].UpdateDistanceDelay();
    }
  }
  void OutputPhase() {
    // Go through each module, call timestep
    for (int steps=0; steps<OUTPUT_PHASE_STEPS; steps++) {  // output phase = 1000 steps = 1 second
      for (int i=0; i<N_MODULES; i++) {
        // these are instant
        modules[i].Timestep(); // set each module H or L, update counters
      }
      delayMicroseconds(STEP_LENGTH_US); // stay in this pos for 1ms
    }
  }
  
 public:
  HeadbandController() {}
  HeadbandController(int outputPins[], int inputPins[]) {
    for (int i=0; i<N_MODULES; i++) {
      pinMode(inputPins[i], INPUT);
      pinMode(outputPins[i], OUTPUT);
      modules[i] = ModuleController(outputPins[i], inputPins[i]);
    }
  }
  void ProcessCycle() {
    ExtendAll(FALSE);
    InputPhase();
    OutputPhase();
  }
  void CycleActuators() {
    ExtendAll(FALSE);
    delay(1000);
    ExtendAll(TRUE);
    delay(1000);
  }
};

///////// MAIN //////////////////////////////
HeadbandController controller;
void setup() {
  int outs[] = OUT_PINS;
  int ins[] = IN_PINS;
  controller = HeadbandController(outs, ins);
}
void loop() {
  controller.ProcessCycle();
  //controller.CycleActuators();
}

