
volatile int pwm_start_time[4];
uint8_t latest_interrupted_pin;

void rx_initialize() {
  pinMode(PIN_RX_ROLL, INPUT); digitalWrite(PIN_RX_ROLL, HIGH);         // Channel 1
  PCintPort::attachInterrupt(PIN_RX_ROLL, &rising, RISING);
  
  pinMode(PIN_RX_PITCH, INPUT); digitalWrite(PIN_RX_PITCH, HIGH);       // Channel 2
  PCintPort::attachInterrupt(PIN_RX_PITCH, &rising, RISING);
  
  pinMode(PIN_RX_THROTTLE, INPUT); digitalWrite(PIN_RX_THROTTLE, HIGH); // Channel 3
  PCintPort::attachInterrupt(PIN_RX_THROTTLE, &rising, RISING);
  
  pinMode(PIN_RX_YAW, INPUT); digitalWrite(PIN_RX_YAW, HIGH);           // Channel 4
  PCintPort::attachInterrupt(PIN_RX_YAW, &rising, RISING);
}

void rising()
{
  latest_interrupted_pin=PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &falling, FALLING);
  pwm_start_time[latest_interrupted_pin-RX_PINS_OFFSET] = micros();
}

void falling() {
  latest_interrupted_pin=PCintPort::arduinoPin;
  PCintPort::attachInterrupt(latest_interrupted_pin, &rising, RISING);
  rx_values[latest_interrupted_pin-RX_PINS_OFFSET] = micros()-pwm_start_time[latest_interrupted_pin-RX_PINS_OFFSET];
}

