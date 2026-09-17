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
    stepper->setAcceleration(1000); // needs better version
  }
  Serial.println("Stepper setup complete");
}

int calculateRPM(float speedFactor) {
    int rpm = static_cast<int>(static_cast<float>(motorMaxRPM-motorMinRPM) * speedFactor)+motorMinRPM;
    return rpm;
}

void disableMotor() {
  if (stepper) {
    stepper->disableOutputs();
  }
}

void enableMotorMin() {
  if (stepper) {
    enableMotor(motorMinRPM);
  }
}

void enableMotor(int rpm) {
  if (stepper) {
    stepper->enableOutputs();
    // Convert RPM to microseconds per step
    float stepsPerMinute = rpm * motorStepsPerRevolution;
    float stepsPerSecond = stepsPerMinute / 60.0;
    float usPerStep = 1000000.0 / stepsPerSecond;
    stepper->setSpeedInUs(static_cast<uint32_t>(usPerStep));
    Serial.print("Motor enabled at RPM: ");
    Serial.println(rpm);
  }

}