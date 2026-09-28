#include "stepper.h"

// stepper engine init
FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepper = NULL;


void stepperSetup() {
  engine.init();
  stepper = engine.stepperConnectToPin(stepPin);
  if (stepper) {
    stepper->setDirectionPin(dirPin);
    stepper->setEnablePin(enablePin, true);
    stepper->setAutoEnable(true);
    stepper->setSpeedInUs(1000);    // needs better version
    stepper->setAcceleration(400000); // needs better version
  }
  // Serial.println("Stepper setup complete");
}

int calculateRPM(float speedFactor) {
    int rpm = static_cast<int>(static_cast<float>(motorMaxRPM-(motorMinRPM+1)) * speedFactor)+motorMinRPM + 1;
    return rpm;
}

void disableMotor() {
  if (stepper) {
    //while (stepper->getCurrentPosition() > 400 ) {
    //  stepper->runForward();
    //}
    //stepper->moveTo(400);
    //if (stepper->isQueueEmpty()) {
      stepper->stopMove();
      stepper->disableOutputs();
    //}
  }
}

void enableMotorMin() {
  if (stepper) {
    enableMotor(motorMinRPM);
  }
}

long returnCurrentSpeed() {
  if (stepper) {
    return stepper->getCurrentSpeedInUs();
  }
  return 0;
}

long returnCurrentPosition(){
  return stepper->getCurrentPosition();
}

void setCurrentPosition(int position) {
  stepper->setCurrentPosition(position);
}

bool isMotorRunning(){
  return stepper->isRunning();
}

void enableMotor(int rpm) {
  if (stepper) {
    // Convert RPM to microseconds per step
    float stepsPerMinute = rpm * motorStepsPerRevolution;
    float stepsPerSecond = stepsPerMinute / 60.0;
    stepper->setSpeedInHz(static_cast<uint32_t>(stepsPerSecond));
    // Serial.println(static_cast<uint32_t>(stepsPerSecond));
    stepper->applySpeedAcceleration();
    if(!(stepper->isRunning())) {
      stepper->enableOutputs();
      stepper->runForward();
    }
    // Serial.print("Motor enabled at RPM: ");
    // Serial.println(rpm);
  }

}