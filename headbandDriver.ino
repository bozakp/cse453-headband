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
// Input Phase is about 250ms
#define OUTPUT_PHASE_STEPS 1000
#define STEP_LENGTH_US 1000

class DistanceSensorDriver {
  int trigPin;
  int echoPin;
public:
  DistanceSensorDriver(int trigPin, int echoPin) :
  trigPin(trigPin), echoPin(echoPin) {
  }

  /**
   * Gives the current distance reading from the sensor (in cm). Returns -1 if
   * the distance was outside of the expected range.  [0,517] otherwise.
   */
  int getCurrentDistance() {
    digitalWrite(trigPin, LOW);  // make sure the trigger is low
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    unsigned long pulseMicros = pulseIn(echoPin, HIGH, 30000);  // 30ms: max time to wait for reply
    int distanceCm = (pulseMicros < 2 || pulseMicros >= 30000) ? -1 : pulseMicros/58;
    return distanceCm;
  }
};

class ActuatorDriver {
  int basePin;
  int additionalDelay;
  int counter;
public:
  ActuatorDriver(int basePin) :
  basePin(basePin) , additionalDelay(NO_OBJ_DELAY), counter(0){
  }
  void extend(){
    digitalWrite(basePin, HIGH);
  }
  void retract(){
    digitalWrite(basePin, LOW);
  }
  void timestep(){    
    if (additionalDelay == NO_OBJ_DELAY){
      retract();
    }
    else if (counter < ACT_ON_TICKS){
      extend();
      counter++;
    }
    else if (counter < (ACT_ON_TICKS + REQ_DELAY_MS + additionalDelay)){
      retract();
      counter++;
    }
    else{
      extend();
      counter = 0;
    }
  }
  void setAdditionalDelay(int del){
    additionalDelay = del;
    counter = 0;
  }
};

class NoiseFilter {
  // Translate [-1,517] to [0,258] or NO_OBJ_DELAY(=259)
  // distCm -> additionalDelayMs
public:
  int getFilteredValue(int dist){
    if (dist == -1)
      return NO_OBJ_DELAY;
    else
      if (dist < 80)
        return dist/2;
      return NO_OBJ_DELAY;
  }

};

class ModuleController {
  DistanceSensorDriver distanceSensor;
  ActuatorDriver actuator;
  NoiseFilter noiseFilter;
public:
  ModuleController(): 
  distanceSensor(0,0), actuator(0) {
  }
  ModuleController(int outputPin, int inputPin) :
  distanceSensor(outputPin, inputPin), actuator(outputPin) {
  }
  int getCurrentDistance(){
    return distanceSensor.getCurrentDistance();
  }
  void timestep(){
    actuator.timestep();
  }
  void retract(){
    actuator.retract();
  }
  void extend(){
    actuator.extend();
  }
  int getFilteredValue(int dist){
    return noiseFilter.getFilteredValue(dist);
  }
  void setAdditionalDelay(int del){
    actuator.setAdditionalDelay(del);
  }
};

class HeadbandController{
  ModuleController modules[N_MODULES];
  
  void retractAll(){
    for(int i=0;i<N_MODULES;i++){
      modules[i].retract();
    }    
  }
  void extendAll(){
    for(int i=0;i<N_MODULES;i++){
      modules[i].extend();
    }    
  }
  void inputPhase(){
    // Read all dists, store additionalDelays in actuator drivers
    // This takes up to 250 ms
    for(int i=0; i<N_MODULES; i++){
      int distCm = modules[i].getCurrentDistance();
      int filteredValue = modules[i].getFilteredValue(distCm);
      modules[i].setAdditionalDelay(filteredValue);
    }
  }

  void outputPhase(){
    // Go through each module, call timestep
    for(int steps=0; steps < OUTPUT_PHASE_STEPS; steps++){  // output phase = 1000 steps = 1 second
      for(int i=0; i<N_MODULES; i++){
        // these are instant
        modules[i].timestep(); // set each module H or L, update counters
      }
      delayMicroseconds(STEP_LENGTH_US); // stay in this pos for 1ms
    }
  }
  
 public:
  HeadbandController(){
  }
  HeadbandController(int outputPins[], int inputPins[]){
    for(int i=0;i<N_MODULES;i++){
      pinMode(inputPins[i], INPUT);
      pinMode(outputPins[i], OUTPUT);
      modules[i] = ModuleController(outputPins[i], inputPins[i]);
    }
  }
  void processCycle(){
    retractAll();
    inputPhase();
    outputPhase();
  }
  void cycleActuators(){
    retractAll();
    delay(1000);
    extendAll();
    delay(1000);
  }
};

///////// MAIN //////////////////////////////
HeadbandController controller;
void setup() {
  int outs[] = OUT_PINS;
  int ins[] = IN_PINS;
  controller = HeadbandController(outs,ins);
}
void loop() {
  controller.processCycle();
  //controller.cycleActuators();
}


