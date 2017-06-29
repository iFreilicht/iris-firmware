//#define TESTING
#define DEBUG

bool int_switch_closed;

void delayCycles(uint16_t cycles){
  for(volatile uint16_t i = 0; i < cycles; i++){
    _NOP();
  }
}

struct pin_pair{
  pin_pair(iris_pin pin0, iris_pin pin1) :
    pin0(pin0),
    pin1(pin1)
  {
    this->updateStatus();
  }

  const iris_pin pin0; //has to support iris_pin_mode::output_220R
  const iris_pin pin1;

  //check whether led is connected that can be powered
  inline bool ledConnected()   { return m_led_connected;     }
  inline int readLedHeader()   { return m_ext_led_active ? HIGH : LOW; }
  inline bool switchClosed()   { return m_switch_closed;     }
  inline bool flowFrom0to1()   { return m_flow_from_0_to_1;  }
  inline bool flowFrom1to0()   { return m_flow_from_1_to_0;  }

  //Turn LED on or off if one is connected
  void writeLed(uint8_t value){
    if (!ledConnected()) return;

    pinMode(pin0, iris_pin_mode::output_220R);
    pinMode(pin1, iris_pin_mode::output);

    if (flowFrom0to1()){
      digitalWrite(pin0, value);
      digitalWrite(pin1, LOW);
    } else {
      #ifdef DEBUG
        //Make sure there's no error in the logic of updateStatus()
        if (!flowFrom1to0()) {
          SerialUSB.write("LOGIC ERROR! LED considered connected but no current flow in either direction!");
        }
      #endif
      digitalWrite(pin0, LOW);
      digitalWrite(pin1, value);
    }

    m_led_on = value == HIGH;
  }

  void updateStatus(){
    static const uint16_t led_threshold = 700;
    //Update current flow from pin0 to pin1
    //Analog read is necessary to detect LEDs, some have a very high voltage drop
    pinMode(pin0, iris_pin_mode::input_pullup);
    pinMode(pin1, iris_pin_mode::output);

    digitalWrite(pin1, LOW);
    int start = micros();
    bool low0_if_low1 = analogRead(pin0) < led_threshold;
    int end = micros();
    #ifdef DEBUG
      SerialUSB.print("Measure-time (us): "); //~112 with analog, ~4 with digital
      SerialUSB.print(end - start, DEC);
      SerialUSB.print(" pin0: ");
      SerialUSB.print(digitalRead(pin0), DEC);
      SerialUSB.print(" ");
      SerialUSB.print(analogRead(pin0), DEC);
      SerialUSB.print(" ");
    #endif

    digitalWrite(pin1, HIGH);
    bool low0_if_high1 = analogRead(pin0) < led_threshold;
    #ifdef DEBUG
      SerialUSB.print("pin0: ");
      SerialUSB.print(digitalRead(pin0), DEC);
      SerialUSB.print(" ");
      SerialUSB.print(analogRead(pin0), DEC);
      SerialUSB.print(" ");
    #endif

    m_flow_from_0_to_1 = low0_if_low1 && !low0_if_high1;

    //Update current flow from pin1 to pin0
    pinMode(pin1, iris_pin_mode::input_pullup);
    pinMode(pin0, iris_pin_mode::output);

    digitalWrite(pin0, LOW);
    bool low1_if_low0 = analogRead(pin1) < led_threshold;
    #ifdef DEBUG
      SerialUSB.print("pin1: ");
      SerialUSB.print(digitalRead(pin1), DEC);
      SerialUSB.print(" ");
      SerialUSB.print(analogRead(pin1), DEC);
      SerialUSB.print(" ");
    #endif

    digitalWrite(pin0, HIGH);
    bool low1_if_high0 = analogRead(pin1) < led_threshold;
    #ifdef DEBUG
      SerialUSB.print("pin1: ");
      SerialUSB.print(digitalRead(pin1), DEC);
      SerialUSB.print(" ");
      SerialUSB.print(analogRead(pin1), DEC);
      SerialUSB.print(" ");
    #endif

    m_flow_from_1_to_0 = low1_if_low0 && !low1_if_high0;

    //If the external LED is active, the pins have to read out different levels
    pinMode(pin0, iris_pin_mode::input_pullup);
    pinMode(pin1, iris_pin_mode::input_pullup);
    m_ext_led_active = digitalRead(pin0) != digitalRead(pin1);
    #ifdef DEBUG
      SerialUSB.print("pin0: ");
      SerialUSB.print(digitalRead(pin0), DEC);
      SerialUSB.print(" ");
      SerialUSB.print(analogRead(pin0), DEC);
      SerialUSB.print(" ");

      SerialUSB.print("pin1: ");
      SerialUSB.print(digitalRead(pin1), DEC);
      SerialUSB.print(" ");
      SerialUSB.print(analogRead(pin1), DEC);
      SerialUSB.print("\n");
    #endif

    //If current flows in both directions, a switch must be connected and closed
    m_switch_closed = flowFrom0to1() && flowFrom1to0(); 

    //The internal switch is always connected to P0 and P1
    if (pin0 == P0 || pin1 == P0) int_switch_closed = switchClosed();

    m_led_connected = !m_ext_led_active && (flowFrom0to1() != flowFrom1to0());

    if (m_led_on){
      //Make sure we don't try to source current if no LED is connected
      if (ledConnected()){
        writeLed(HIGH);
      } else {
        writeLed(LOW);
      }
    }
  }

  private:
    bool m_ext_led_active;
    bool m_switch_closed;

    //true if a diode is connected
    bool m_led_connected;

    //Denote directions current is flowing in
    bool m_flow_from_0_to_1;
    bool m_flow_from_1_to_0;

    //Store LED status
    bool m_led_on;
};

//Either P0 and P1 are connected to the mainboards PWR_SW header,
//In which case P2 is not used
//Or, P0 alone is connected to the mainboards PWR_SW_P pin,
//In which case P1 and P2 form a pair of GPIO pins that either power an LED
//and read out an external switch
bool is_P2_unused;
pin_pair P0P1 = pin_pair(P1, P0);
pin_pair P1P2 = pin_pair(P1, P2);
pin_pair P3P4 = pin_pair(P3, P4);
pin_pair P5P6 = pin_pair(P5, P6);

void setup() {
  SerialUSB.begin(9600);

  is_P2_unused = false;

#ifndef TESTING
  
#else
  //Test library
  pinMode(P0, iris_pin_mode::input_pullup);

#if 1
  pinMode(P1, iris_pin_mode::input_pullup);
#else
  pinMode(P1, iris_pin_mode::output);
  digitalWrite(P1, LOW);
#endif

  pinMode(P2, iris_pin_mode::input_pullup);
  pinMode(P3, iris_pin_mode::input_pullup);
  pinMode(P4, iris_pin_mode::input_pullup);
  pinMode(P5, iris_pin_mode::input_pullup);
  pinMode(P6, iris_pin_mode::input_pullup);
#endif
}

void loop() {

  char output[200];

#ifndef TESTING
  if(is_P2_unused){
    pinMode(P2, iris_pin_mode::input);
    P0P1.updateStatus();
    if (P0P1.ledConnected()){
      P0P1.writeLed(HIGH);
    }
  }
  else{
    pinMode(P0, iris_pin_mode::input_pullup);
    int_switch_closed = digitalRead(P0) == LOW;
    P1P2.updateStatus();
    if (P1P2.ledConnected()){
      P1P2.writeLed(HIGH);
    }
  }

  P3P4.updateStatus();
  if (P3P4.ledConnected()){
    P3P4.writeLed(HIGH);
  }

  P5P6.updateStatus();
  if (P5P6.ledConnected()){
    P5P6.writeLed(HIGH);
  }

#ifdef DEBUG
  SerialUSB.write("====================================\n");
  
  sprintf(output, 
    "int_switch_closed: %i, is_P2_unused: %i\n",
    int_switch_closed,
    is_P2_unused
  );
  SerialUSB.write(output);

  if (is_P2_unused){
    sprintf(output, 
      "P0P1: (ledConnected: %i, readLedHeader: %i, switchClosed: %i, flowFrom0to1: %i, flowFrom1to0: %i)\n",
       P0P1.ledConnected(),
       P0P1.readLedHeader(),
       P0P1.switchClosed(),
       P0P1.flowFrom0to1(),
       P0P1.flowFrom1to0()
    );
    SerialUSB.write(output);
  } else {
    sprintf(output, 
      "P1P2: (ledConnected: %i, readLedHeader: %i, switchClosed: %i, flowFrom0to1: %i, flowFrom1to0: %i)\n",
       P1P2.ledConnected(),
       P1P2.readLedHeader(),
       P1P2.switchClosed(),
       P1P2.flowFrom0to1(),
       P1P2.flowFrom1to0()
    );
    SerialUSB.write(output);
  }

  sprintf(output, 
    "P3P4: (ledConnected: %i, readLedHeader: %i, switchClosed: %i, flowFrom0to1: %i, flowFrom1to0: %i)\n",
     P3P4.ledConnected(),
     P3P4.readLedHeader(),
     P3P4.switchClosed(),
     P3P4.flowFrom0to1(),
     P3P4.flowFrom1to0()
  );
  SerialUSB.write(output);

  sprintf(output, 
    "P5P6: (ledConnected: %i, readLedHeader: %i, switchClosed: %i, flowFrom0to1: %i, flowFrom1to0: %i)\n",
     P5P6.ledConnected(),
     P5P6.readLedHeader(),
     P5P6.switchClosed(),
     P5P6.flowFrom0to1(),
     P5P6.flowFrom1to0()
  );
  SerialUSB.write(output);
#endif
#else //ifdef TESTING
  sprintf(output,
    "P0: %i %i, P1: %i %i, P2: %i %i, P3: %i %i, P4: %i %i, P5: %i %i, P6: %i %i\n",
    digitalRead(P0), analogRead(P0),
    digitalRead(P1), analogRead(P1),
    digitalRead(P2), analogRead(P2),
    digitalRead(P3), analogRead(P3),
    digitalRead(P4), analogRead(P4),
    digitalRead(P5), analogRead(P5),
    digitalRead(P6), analogRead(P6)
  );
  SerialUSB.write(output);
#endif

  delay(500);
}