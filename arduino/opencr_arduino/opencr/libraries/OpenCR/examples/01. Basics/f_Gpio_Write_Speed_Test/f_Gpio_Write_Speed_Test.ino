/* Author: KurtE
 * Modified by : Kei
 */

GPIO_TypeDef *pin0_port;
uint32_t  pin0_set_mask;
uint32_t  pin0_clear_mask;

void setup() {
  pinMode (0, OUTPUT);
  
  // For register access directly
  pin0_port = g_Pin2PortMapArray[0].GPIOx_Port;
  pin0_set_mask = g_Pin2PortMapArray[0].Pin_abstraction;
  pin0_clear_mask = (uint32_t)g_Pin2PortMapArray[0].Pin_abstraction << 16;
}

void loop() {
  // Write Method1 : digitalWrite
  digitalWrite(0, HIGH);
  digitalWrite(0, LOW);
  digitalWrite(0, HIGH);
  digitalWrite(0, LOW);
  digitalWrite(0, HIGH);
  digitalWrite(0, LOW);

  // Write Method2 : Access register directly
  pin0_port->BSRR = pin0_set_mask;
  pin0_port->BSRR = pin0_clear_mask;
  pin0_port->BSRR = pin0_set_mask;
  pin0_port->BSRR = pin0_clear_mask;
  pin0_port->BSRR = pin0_set_mask;
  pin0_port->BSRR = pin0_clear_mask;

  // Write Method3 : digitalWriteFast
  digitalWriteFast(0, HIGH);
  digitalWriteFast(0, LOW);
  digitalWriteFast(0, HIGH);
  digitalWriteFast(0, LOW);
  digitalWriteFast(0, HIGH);
  digitalWriteFast(0, LOW);

  delayMicroseconds(100);
}
