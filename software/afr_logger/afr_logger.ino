#define SIMULATING 1

typedef enum
{
  sensor_init = 1,
  calibrate,
  capture,
  wait
} state_t;

void initTimer(void);
int readSensorVal(int);
float calcAfr(int);
float calcTp(int);
int calcRpm();
bool isInValidRange(int, int, int);
void orderBuffer(int*, int);
void orderBuffer(unsigned long*, int);
void calibrateSensor(void);
void rpmIsr(void);


bool calFinished = false;

/*
  The sensor inputs as they are on the real AFR logger
*/

const int afrSensorPin = A0;
const int afrStatusPin = A4;
const int tpsSensorPin = A5;
const int ignTriggerPin = 2;
const int rpmLedPin = LED_BUILTIN;

/*

   Pin used for simulating the motorcycle TPS and AFR sensor
   Note that PWM pins are used to generate analog signals on the simulation board
*/
const int afrValSimPin = 11;
const int afrStatusSimPin = 10;
const int tpsSimPin = 9;
const int ignSimPin = 8;

/*

   useful board defs

*/
const int maxAdcVal = 1023;
const float maxAdcVolt = 5.0f;

/*

   AFR
   Mode of operation:
   When lambda sensor is turned on, it must warm up. While heating, the two analog channels of
   the LC-2 (value and status) are both fixed on 2.5V.

   When the status channel returns to 0V the sensor is operational. If an error occurs, the status channel outputs 5.0V.
   While the sensor is operational, the AFR values are presented on Ch1

*/
//calibration values. Can only remove bias with LS-2 lambda readout
int calBias = 0;
const int afrCalT = 10000; //wait T [ms] for calibration signal value from AFR logger

//reference values for calibration of lambda sensor output
//0.0V = AFR lambda 10
//5.0V = AFR lambda 20
const float afrMin = 10.0f;
const float afrGain = (20.0f - afrMin) / maxAdcVolt;

//while warming up, both channels output 2.5V. Channel 1 is calibrated, channel 2 is checked to see if the unit is warming up.
const int afrCalExpectedVal = (int)((2.5 * (double)maxAdcVal) / (double)5.0);
const int afrCalExpectedStatus = (int)((2.5 * (double)maxAdcVal) / (double)5.0);
//allow up to 150mV bias difference for calibration. ADC has 5mV per LSB, hence 30xLSB
const int maxAdcDelta = 45;

//status defs
const int afrStatExpectedValOperating = 0;
const int afrStatExpectedValError = maxAdcVal;

/*

   TPS
   The throttle position sensors reports voltages roughly between  0.5V (closed) and 4.0V (full throttle)
   The actual voltages must be measured on the bike and programmed to the arduino. Calibration would be possible by
   twisting throttle fully open and back to closed before starting the engine. However, there is a risk that TPS values are slightly
   different with a running engine, invalidating the calibration.

   After the values are entered here (somehow measured with running engine), the resulting percentage MUST BE CHECKED against the throttle
   opening that is reported in the power commander software. This last step can be done without running engine, as any difference (static vs running) will be same
   for this logger and the power commander. IT IS IMPORTANT that the reported percentages are exactly the same, especially for 0%, 2%, 5% and 10% throttle openings.
*/
const float tpsMinVolt = 0.583f;
const float tpsMaxVolt = 3.764f;
const int tpsMinAdcVal = (int)((tpsMinVolt / maxAdcVolt) * (float) maxAdcVal);
const int tpsMaxAdcVal = (int)((tpsMaxVolt / maxAdcVolt) * (float) maxAdcVal);
const float tpsGain = 100.0f / (tpsMaxVolt - tpsMinVolt);

/*

   RPM

*/
#define rpmFilterLen 5
unsigned long rpmBuf[rpmFilterLen];
unsigned long curRpmPulseT = 0;

/*

   Simulation

*/
const int simAfrValWarmupPwm = 128;
const int simAfrValAvgPwm = 100;
const int simAfrStatAvgPwm = 0; //indicating no error
const int simTpsValAvgPwm = 100;
const int simAdcVar = 8; //random variation around average vals
const int simRpmHighT = 10; //number in [ms] that the ign sensor pulse is high
#define simRpmLen 26
//ignition interval (=2x crankshaft rotation intervals) in ms
const int simRpmVec[simRpmLen] = {30, 32, 33, 29, 29, 32, 35, 36, 39, 41, 45, 43, 39, 38, 35, 32, 25, 21, 18, 16, 13, 11, 15, 23, 28, 31};
//const int simRpmVec[simRpmLen] = {20};
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.print("Lambda sensor readout\n");
  pinMode(afrStatusPin, INPUT);

  /*
      LOW to trigger the interrupt whenever the pin is low,
      CHANGE to trigger the interrupt whenever the pin changes value
      RISING to trigger when the pin goes from low to high,
      FALLING for when the pin goes from high to low.
  */
  pinMode(ignTriggerPin, INPUT);
  pinMode(rpmLedPin, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ignTriggerPin), rpmIsr, RISING);

  if (SIMULATING)
  {
    initTimer();
    pinMode(ignSimPin, OUTPUT);
    randomSeed(analogRead(2));
  }

  // initialize buffers
  for (int i = 0; i < rpmFilterLen; i++)
    rpmBuf[i] = 999999999;
}

void loop() {
  // put your main code here, to run repeatedly:
  static state_t state = sensor_init;
  bool afrSwitchedToHeating;
  static bool prevAfrWasHeating = true;

  /*

    Functionality normal operation

  */

  if (state == sensor_init)
  {

  }
  else if (state == calibrate)
  {
    calibrateSensor();
  }
  else if (state == capture)
  {
    /*

       Readout of RPM

    */
    int rpmVal = calcRpm();
    Serial.print("RPM=");
    Serial.print(rpmVal);
    
    
    /*

       readout of AFR value

    */
    int adcVal = readSensorVal(afrSensorPin);

    //convert to AFR
    float afrVal = calcAfr(adcVal);

    //read status pin. 0V = normal, 2.5V = warming up, 5.0V = error
    int afrStatusVal = analogRead(afrStatusPin);
    bool afrOperational = isInValidRange(afrStatusVal, afrStatExpectedValOperating, maxAdcDelta);
    bool afrHeating = isInValidRange(afrStatusVal, afrCalExpectedVal, maxAdcDelta);
    bool afrError = isInValidRange(afrStatusVal, afrStatExpectedValError, maxAdcDelta);

    //print, Note that Arduino's sprintf does not support floats so have to print it 'the ugly way'
    Serial.print(",\tAFR=");
    Serial.print(afrVal);
    if (afrOperational)
      Serial.print(",\tSTATUS=OK");
    else if (afrHeating)
      Serial.print(",\tSTATUS=HEATING_SENSOR");
    else if (afrError)
      Serial.print(",\tSTATUS=ERROR");
    else
    {
      Serial.print("\tSTATUS=INVALID-VAL=");
      float statVoltage = ((float) afrStatusVal / (float) maxAdcVal) * (float) maxAdcVolt;
      Serial.print(statVoltage);
    }

    //detection if lambda controller has reset and started a heating cycle again
    //This happens either when a fault occurs or when power has switched on/off by the driver, which can be used to mark events in the logfile
    //initialize to current value to avoid false detection
    afrSwitchedToHeating = (prevAfrWasHeating == false) & (afrHeating == true);
    prevAfrWasHeating = afrHeating;

    /*

       Readout of TPS

    */
    adcVal = readSensorVal(tpsSensorPin);

    //convert to throttle opening percentage
    float TpVal = calcTp(adcVal);
    Serial.print(",\t TP=");
    Serial.print(TpVal);

    Serial.println();

  }
  else if (state == wait)
  {
    delay(10);
    if (SIMULATING)
      delay(1000);
      // delay(0);
  }


  /*

    Functionality simulation

  */
  if(SIMULATING)
  {
    if (state == sensor_init)
    {
      //write the expected 2.5V to both afr channels, this happens at power-on
      analogWrite(afrValSimPin, simAfrValWarmupPwm);
      analogWrite(afrStatusSimPin, simAfrValWarmupPwm);
      //wait for RC filter output to stabilize
      delay(250);
    }
    else if (state == calibrate)
    {

    }
    else if (state == capture)
    {
      //generate and write AFR values, bound in range 0 - 255
      int simAfrVal = random(simAfrValAvgPwm - simAdcVar, simAfrValAvgPwm + simAdcVar);
      simAfrVal = max(0, min(simAfrVal, 255));
      int simAfrStat = random(simAfrStatAvgPwm - simAdcVar, simAfrStatAvgPwm + simAdcVar);
      simAfrStat = max(0, min(simAfrStat, 255));

      //once every while, simulate the lambda controller has restarted
      long randNum = random(0,50);
      bool pretendAfrRestart = (randNum < 1) |  afrSwitchedToHeating;
      if(pretendAfrRestart)
      {
        //happens in 2% of cases. We will need one whole iteration of state == capture for the
        //algorithm to pickup this event, in which case (afrSwitchedToHeating) we want to keep simulating the sensor is heating
        //write the expected 2.5V to both afr channels
        simAfrVal = simAfrValWarmupPwm;
        simAfrStat = simAfrValWarmupPwm;
      }
      //write to outputs
      analogWrite(afrValSimPin, simAfrVal);
      analogWrite(afrStatusSimPin, simAfrStat);

      if(pretendAfrRestart)
      {
        //wait for RC filter output to stabilize
        delay(250);
      }

      //generate and write TPS values
      int simTpsVal = random(simTpsValAvgPwm - simAdcVar, simTpsValAvgPwm + simAdcVar);
      simTpsVal = max(0, min(simTpsVal, 255));
      analogWrite(tpsSimPin, simTpsVal);

      //generate RPM
      static bool rpmSig = false;
      rpmSig = !rpmSig;
      digitalWrite(ignSimPin, rpmSig);
    }
    else if (state == wait)
    {

    }
  }

  /*

    Statemachine transitions

  */

  if (state == sensor_init)
  {
    state = calibrate;
  }
  else if (state == calibrate)
  {
    if (calFinished);
    state = capture;
  }
  else if (state == capture)
  {
    //If we detect lambda controller has started a heating cycle again, either there was a fault or the user has switched power to the unit on/off
    //In either case it is a good idea to restart a calibration cycle.
    //
    //This code is specifically designed around the following sequence of events:
    //An onboard motorcycle unit that logs output of the AFR-logger (for ex a raspberry pi) is booting, while this arduino is already powered and
    //transmitting data. In this situation, the initial calibration output will not be seen by the motorcycle unit, so we cannot verify if the calibration
    //values make sense. 
    //If the motorcycle user switches power on/off after a while when the motorcycle unit has booted up, 
    //it will restart the lambda controller. This will trigger a re-calibration after which
    //the logfile will also contain the new calibration data that can be seen after the ride

    //This mechanism also solves the problem when the motorcycle unit resets the arduino by initializing serial communication
    //In this case the arduino will not see a heating lambda controller and skips the calibration (may lead to large offsets of +/-0.5AFR in the log)
    //The user can switch power on/off after a while to ensure the arduino will get to see a calibration at least once after
    //the motorcycle unit has started to log.

    //Optionally the user can decide to use this mechanism as a "this is where my logs starts marker" and throw away prior log values while analyzing.
    if(afrSwitchedToHeating)
    {
      Serial.println("Detected a restart of lambda controller, perform re-calibration");
      state = calibrate;
    }
    else
    {
      //everything is normal, go to wait state to avoid printing a extremely high rates
      state = wait;
    }
  }
  else if (state == wait)
  {
    state = capture;
  }
}

void initTimer(void)
{

  // Changing any of the timer settings will affect analogWrite and millis() so will steer clear of that..
  // Timer 0 is set to 250KHz, counting to 255 by default. Overflow occurs at 980Hz
  //enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
}

int readSensorVal(int pin)
{
  //take 11 measurements and take median value to filter out electrical noise
  const int adcLen = 11;
  int adcBuf[adcLen];

  for (int i = 0; i < adcLen; ++i)
  {
    adcBuf[i] = analogRead(pin);
    //To avoid taking all adc measurements while a digital signal in the system may be causing electrical noise:
    //wait a little bit.
    delay(1);
  }
  orderBuffer(adcBuf, adcLen);
  //report median value in center of ordered values
  return adcBuf[adcLen / 2];
}

float calcAfr(int adcVal)
{
  //correct ADC value
  adcVal = adcVal - calBias;

  //convert to voltage from 10-bit value
  float sensVoltage = ((float) adcVal / (float) maxAdcVal) * (float) maxAdcVolt;

  //convert voltage to AFR
  //0V - 5V correspond to lambda ratios of 10 - 20
  float afrVal = afrMin + afrGain * sensVoltage;

  return afrVal;
}

float calcTp(int adcVal)
{
  //convert to voltage from 10-bit value
  float sensVoltage = ((float) adcVal / (float) maxAdcVal) * (float) maxAdcVolt;

  //convert voltage to throttle opening. Subtract the minimum base voltage, then scale.
  sensVoltage = sensVoltage - tpsMinVolt;
  float tp = sensVoltage * tpsGain;

  //limit between 0% and 100%
  //  if(tp > 100.0f)
  //    tp = 100.0f;
  //  if(tp < 0.0f)
  //    tp = 0.0f;
  return tp;
}

//calculates a median filtered value for the RPM
int calcRpm(void)
{
  //track time between invokation of this function and use it to see if engine is actually running
  unsigned long currentTime = micros();
  unsigned long deltaT = currentTime - curRpmPulseT;
  
  //check if last pulse is more than 250ms ago, at lowest rpm (idle) we would expect to see one pulse every 100ms
  //this function will report any rpm lower than 1/250ms[events/s]*2[revs/event]*60[s/min] = 480 rpm as 0 rpm
  if(deltaT > 250000UL)
  {
      return 0;
  }
  
  //buffer to copy timing values in that are stored through the ISR
  unsigned long localRpmBuf[rpmFilterLen];

  //disable interrupts: the unsigned long could be updated while copying, invalidating the numbers
  noInterrupts();
  memcpy(localRpmBuf, rpmBuf, rpmFilterLen*sizeof(unsigned long));
  //return to normal
  interrupts();

  //order the buffer and extract the median
  orderBuffer(localRpmBuf, rpmFilterLen);
  //in us
  unsigned long medT = localRpmBuf[rpmFilterLen / 2];
  //convert to RPM from [arduino-us] interval. 2 = 2 cycles per ignition for 4-stroke, 60 to RPM
  int rpm = (int)(((unsigned long)1000000 * (unsigned long)2 * (unsigned long)60) / medT);
  return rpm;
}

void orderBuffer(int * buf, int len)
{
  //debug, print buffer
  //    for(int i=0; i<len; ++i)
  //    {
  //      Serial.print("Value ");
  //      Serial.print(i);
  //      Serial.print(" is ");
  //      Serial.println(buf[i]);
  //    }

  //iterate through buffer until ordering is complete
  int unordered;
  do {
    unordered = 0;

    for (int i = 0; i < len - 1; ++i)
    {
      //move largest elements to end
      if (buf[i] > buf[i + 1])
      {
        unordered = 1;
        int tempVal = buf[i + 1];
        buf[i + 1] = buf[i];
        buf[i] = tempVal;
      }
    }
  } while (unordered);

  //debug, print buffer
  //  for(int i=0; i<len; ++i)
  //  {
  //    Serial.print("Value after ordering ");
  //    Serial.print(i);
  //    Serial.print(" is ");
  //    Serial.println(buf[i]);
  //  }
}

void orderBuffer(unsigned long * buf, int len)
{
   //iterate through buffer until ordering is complete
  int unordered;
  do {
    unordered = 0;

    for (int i = 0; i < len - 1; ++i)
    {
      //move largest elements to end
      if (buf[i] > buf[i + 1])
      {
        unordered = 1;
        unsigned long tempVal = buf[i + 1];
        buf[i + 1] = buf[i];
        buf[i] = tempVal;
      }
    }
  } while (unordered);
}

bool isInValidRange(int val, int expect, int delta)
{
  return ((val <= expect + delta) && (val >= expect - delta));
}

void calibrateSensor(void)
{
  bool calibrationWorked = false;
  int afrBiasVal = 0;

  //wait until adc bias is estimated for max T [s]
  int timeOut = millis() + afrCalT;

  if (SIMULATING)
    timeOut = millis() + 3000;

  while ((!calibrationWorked) && (millis() < timeOut))
  {
    //are we in warming up mode?
    int ch1Val = analogRead(afrSensorPin);
    int ch2Val = analogRead(afrStatusPin);
    //both values have to be approx 2.5V to indicate a warmup
    //if(ch1Val >= calRef1l && ch1Val <= calRef1l && ch2Val >= warmingStatusRefL && ch2Val <= warmingStatusRefH)
    if (isInValidRange(ch1Val, afrCalExpectedVal, maxAdcDelta) && isInValidRange(ch2Val, afrCalExpectedStatus, maxAdcDelta))
    {
      //wait for IO transition effects to subsize (up to a few seconds the bias changes when measured..)
      delay(6000);

      //re-read ch1 value using a filtered ADC read
      ch1Val = readSensorVal(afrSensorPin);
      afrBiasVal = ch1Val - afrCalExpectedVal;

      //check out-of-bound conditions
      if (isInValidRange(afrBiasVal, 0, maxAdcDelta))

      //check if the sensor is still in warming up mode while we waited
      int ch2Val = analogRead(afrStatusPin);
      if(isInValidRange(ch2Val, afrCalExpectedStatus, maxAdcDelta))
        calibrationWorked = true;
    }
  }

  if (calibrationWorked)
  {
    calBias = afrBiasVal;
    Serial.println("calibration was performed.");
    Serial.print("AFR sensor Vbias=");
    float afrBiasVolt = ((float)afrBiasVal * maxAdcVolt) / maxAdcVal;
    Serial.print(afrBiasVolt);
    Serial.println(" [V]");
  }
  else
    Serial.println("calibration was NOT performed.");

  calFinished = true;

}

void rpmIsr(void)
{
  unsigned long currentTime = micros();
  static unsigned long prevTime = currentTime;
  static int bufIdx = 0;

  //calculate difference since last interrupt
  unsigned long delta = currentTime - prevTime;
  //Serial.print("delta T=");
  //Serial.println(delta);

  //store value in buffer
  bufIdx = ++bufIdx % rpmFilterLen;
  rpmBuf[bufIdx] = delta;

//  for(int i=0; i<5; i++)
//  {
//    Serial.print("buf[");
//    Serial.print(i);
//    Serial.print("]=");
//    Serial.println(rpmBuf[i]);
//  }

  //reset var for next round, report we found an RPM pulse
  prevTime = currentTime;
  curRpmPulseT = currentTime;
}

ISR(TIMER0_COMPA_vect) { //timer0 interrupt 1000Hz toggles pin 13 (LED)
  static int rpmIdx = 0, rpmCnt = 0;
  static bool pulseState = false;
  //interrupt is fired at f=980Hz, but will pretend it does at 1KHz and all calculations below are in ms

  //this code iterates over the RPM buffer to simulate the ignition signal
  if (rpmCnt-- <= 0)
  {
    //time to change signal flank
    if (pulseState)
    {
      //pulse was high. Write low and move to next ignition cycle
      pulseState = false;
      rpmIdx = ++rpmIdx % simRpmLen;

      //read total crank rotation interval and subtract high-pulse-time to calc the time before next ignition event
      rpmCnt = simRpmVec[rpmIdx] - simRpmHighT - 1;
    }
    else
    {
      //pulse was low. Simulate ignition event for 10ms
      pulseState = true;

      //stay high for pulse duration
      rpmCnt = simRpmHighT - 1;
    }

    digitalWrite(ignSimPin, pulseState);
    digitalWrite(rpmLedPin, pulseState);
  }
}
