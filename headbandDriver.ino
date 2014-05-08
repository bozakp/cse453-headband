/*********************************************

Optical Substitution Headband - Main Driver

Phil Bozak
Ben Demmer
Sean Rosney
Devin Toth

CSE 453, Spring 2014

*********************************************/

#define N_MODULES 5
#define ACT_ON_TICKS 1
#define NO_OBJ_DELAY 130
#define REQ_DELAY_MS 1
#define OUT_PINS {0,1,2,3,4}
#define IN_PINS {7,8,9,10,11}
#define STEP_LENGTH_US 30000

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
    unsigned long pulse_micros = pulseIn(echo_pin, HIGH, STEP_LENGTH_US);  // 30ms: max time to wait for reply
    int distanceCm = (pulse_micros < 2 || pulse_micros >= STEP_LENGTH_US) ? -1 : pulse_micros/58;
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
    if (additional_delay >= NO_OBJ_DELAY) {
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
  }
};

class NoiseFilter {
  // Translate [-1,517] to [0,129] or NO_OBJ_DELAY(=130)
  // dist (cm) -> additionalDelay (ms)
  int last_distance;
 public:
  NoiseFilter() : last_distance(0) {}

  int Filter(int dist) {
    if (dist == -1)
        return NO_OBJ_DELAY;
    int output_delay = dist/4;
    if (dist > last_distance + FILTER_THRESHOLD)
      output_delay *= 2;
    if (dist < last_distance - FILTER_THRESHOLD)
      output_delay /= 2;
    dist = last_distance;
    if (output_delay > NO_OBJ_DELAY)
      output_delay = NO_OBJ_DELAY;  // cap the delay at the max value
    return output_delay;
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
  // Returns the distance
  int UpdateDistanceDelay() {
    int dist_cm = distance_sensor.CurrentDistance();
    int actuator_delay = noise_filter.Filter(dist_cm);
    actuator.set_additional_delay(actuator_delay);
    return dist_cm;
  }
};

class HeadbandController{
  ModuleController modules[N_MODULES];
  int next_module;
  
  void ExtendAll(int extend) {
    for (int i=0;i<N_MODULES;i++) {
      modules[i].Extend(extend);
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
    next_module = 0;
  }
  void ProcessStep() {
    int dist = modules[next_module].UpdateDistanceDelay();
    // sleep for the rest of this time period
    delayMicroseconds(STEP_LENGTH_US - dist*58);
    // update all actuators
    for (int i=0; i<N_MODULES; i++) {
      modules[i].Timestep();
    }
    next_module++;
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
  controller.ProcessStep();
  //controller.CycleActuators();
}

