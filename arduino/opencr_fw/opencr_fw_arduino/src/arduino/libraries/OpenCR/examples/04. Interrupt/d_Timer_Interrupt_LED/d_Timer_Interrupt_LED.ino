#define BOARD_LED_PIN 13
#define LED_RATE 1000000                // in microseconds; should give 1Hz toggles

HardwareTimer Timer(TIMER_CH1);

void setup() {
  pinMode(BOARD_LED_PIN, OUTPUT);

  Timer.stop();
  Timer.setPeriod(LED_RATE);           // in microseconds
  Timer.attachInterrupt(handler_led);
  Timer.start();
}

void loop() {
  // Nothing! It's all in the handler_led() interrupt:
}

void handler_led(void) {
  static uint8_t flag = 0;

  digitalWrite(BOARD_LED_PIN, flag);
  flag ^= 1;
}
