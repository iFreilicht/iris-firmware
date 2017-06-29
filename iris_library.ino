#define PB 2
#define PC 3
#define PD 4
#define PE 5
#define PF 6

enum class iris_pin_mode : uint8_t{
  input = INPUT,
  output = OUTPUT,
  input_pullup = INPUT_PULLUP,
  serial_spi,
  serial_uart,
  jtag
};

enum class iris_pin : uint8_t{
      //Atmega32u4 Pin(s): | Arduino Micro Pin(s):
  P0, //PE6                | 7
  P1, //PF0, PD2           | A5, 0/RX
  P2, //PF1, PD3           | A4, 1/TX
  P3, //PF4, PB0, PD5      | A3, RXLED/SS, TXLED
  P4, //PF5, PB1           | A2, SCK
  P5, //PF6, PB2           | A1, MOSI
  P6  //PF7, PB3           | A0, MISO
};

static const iris_pin P0 = iris_pin::P0;
static const iris_pin P1 = iris_pin::P1;
static const iris_pin P2 = iris_pin::P2;
static const iris_pin P3 = iris_pin::P3;
static const iris_pin P4 = iris_pin::P4;
static const iris_pin P5 = iris_pin::P5;
static const iris_pin P6 = iris_pin::P6;

//bitfields to store which pins can be in which modes
//format: 0b06543210, where 6-0 are 1 or 0 for pins P6-P0, respectively
//1=mode available, 0=mode not available
const uint8_t PROGMEM iris_pin_mode_availability_PGM[] = {
  //                                  6543210
  [iris_pin_mode::input]         = 0b01111111,
  [iris_pin_mode::output]        = 0b01111111,
  [iris_pin_mode::input_pullup]  = 0b01111111,
  [iris_pin_mode::serial_spi]    = 0b01111000,
  [iris_pin_mode::serial_uart]   = 0b00001110,
  [iris_pin_mode::jtag]          = 0b01111000
};

#define MAX_PINS_PER_IRIS_PIN 3

//a safe memory location that can be written over with garbage
//think of it like piping to /dev/null
static volatile uint8_t _safe_value;
#define SAFE_ADDRESS &_safe_value

//Array to map pin names to array of pin bitmasks
//First entry is the default pin to use for input readouts
//  it is always an analog pin, except for P0
//Second entry is a digital pin with support for interrupts or NOT_A_PIN
//If fhe iris_pin supports JTAG, the corresponding pin is the first entry
//If the iris_pin supports SPI or UART, the corresponding pin is the second entry, 
//  except for P3s secondary CTS which is the third entry
const uint8_t PROGMEM iris_pin_to_bitmasks_PGM[][MAX_PINS_PER_IRIS_PIN] = {
  [iris_pin::P0] = {
    _BV(6),   //PE6
    NOT_A_PIN,
    NOT_A_PIN
  },
  [iris_pin::P1] = {
    _BV(0),   //PF0
    _BV(2),   //PD2
    NOT_A_PIN
  },
  [iris_pin::P2] = {
    _BV(1),   //PF1
    _BV(3),   //PD3
    NOT_A_PIN
  },
  [iris_pin::P3] = {
    _BV(4),   //PF4
    _BV(0),   //PB0
    _BV(5)    //PD5
  },
  [iris_pin::P4] = {
    _BV(5),   //PF5
    _BV(1),   //PB1
    NOT_A_PIN
  },
  [iris_pin::P5] = {
    _BV(6),   //PF6
    _BV(2),   //PB2
    NOT_A_PIN
  },
  [iris_pin::P6] = {
    _BV(7),   //PF7
    _BV(3),   //PB3
    NOT_A_PIN
  }
};

//Array to map pin names to array of port indices
//Entry explanation above iris_pin_to_bitmasks_PGM
const uint8_t PROGMEM iris_pin_to_ports_PGM[][MAX_PINS_PER_IRIS_PIN] = {
  [iris_pin::P0] = {
    PE,   //PE6
    NOT_A_PORT,
    NOT_A_PORT
  },
  [iris_pin::P1] = {
    PF,   //PF0
    PD,   //PD2
    NOT_A_PORT
  },
  [iris_pin::P2] = {
    PF,   //PF1
    PD,   //PD3
    NOT_A_PORT
  },
  [iris_pin::P3] = {
    PF,   //PF4
    PB,   //PB0
    PD    //PD5
  },
  [iris_pin::P4] = {
    PF,   //PF5
    PB,   //PB1
    NOT_A_PORT
  },
  [iris_pin::P5] = {
    PF,   //PF6
    PB,   //PB2
    NOT_A_PORT
  },
  [iris_pin::P6] = {
    PF,   //PF7
    PB,   //PB3
    NOT_A_PORT
  }
};

//bitmask can only be 5 bits long, NOT_ANALOG is an invalid value
#define NOT_ANALOG 0b1000000 
const uint8_t PROGMEM iris_pin_to_analog_bitmask_PGM[] = {
  [iris_pin::P0] = NOT_ANALOG,
  [iris_pin::P1] = 0,  //ADC0
  [iris_pin::P2] = 1,  //ADC1
  [iris_pin::P3] = 4,  //ADC4
  [iris_pin::P4] = 5,  //ADC5
  [iris_pin::P5] = 6,  //ADC6
  [iris_pin::P6] = 7,  //ADC7
};

enum class led_pin : uint8_t{
      //Atmega32u4 Pin(s): | Arduino Micro Pin(s):
  N0, //PD0                | 3/SCL
  N1, //PB6                | 10
  N2, //PB5                | 9
  N3, //PC6                | 5
  N4, //PC7                | 13
  N5, //PB7                | 11
  N6  //PD6, PD7           | 12, 6
};

//Get bitmask for one internal pin
inline uint8_t pinToBitMask(iris_pin pin, uint8_t index){
  return pgm_read_byte(
    &iris_pin_to_bitmasks_PGM[(uint8_t)pin][index % MAX_PINS_PER_IRIS_PIN]
  );
}

//Get port for one internal pin
inline uint8_t pinToPort(iris_pin pin, uint8_t index){
  return pgm_read_byte(
    &iris_pin_to_ports_PGM[(uint8_t)pin][index % MAX_PINS_PER_IRIS_PIN]
  );
}

//Get ADC channel bitmask for one internal pin
inline uint8_t pinToAnalogChannelBitMask(iris_pin pin){
  return pgm_read_byte(
    &iris_pin_to_analog_bitmask_PGM[(uint8_t)pin]
  );
}

//Get address of mode register. Returns SAFE_ADDRESS if port is invalid
inline volatile uint8_t* portToModeRegister(uint8_t port){
  return port == NOT_A_PORT ? SAFE_ADDRESS : portModeRegister(port);
}

//Get address of output register. Returns SAFE_ADDRESS if port is invalid
inline volatile uint8_t* portToOutputRegister(uint8_t port){
  return port == NOT_A_PORT ? SAFE_ADDRESS : portOutputRegister(port);
}

static uint8_t _sreg_storage;
//Always call this function before modifiying the external state
//i.e. when directly writing to registers that change the state of a pin
inline void suspendInterrupts(){
  //store current state of interrupts
  _sreg_storage = SREG;
  //disable interrupts if they weren't already
  cli(); 
}

//Always call this function after modifiying the external state
inline void resumeInterrupts(){
  //restore previous state of interrupts
  SREG = _sreg_storage;
}

//Check whether a pin can take on the specified mode
inline bool pinCanBeMode(iris_pin pin, iris_pin_mode mode){
  return bitRead( pgm_read_byte( iris_pin_mode_availability_PGM + static_cast<uint8_t>( mode ) ), static_cast<uint8_t>( pin ) );
}

//Change mode of pin
void pinMode(iris_pin pin, iris_pin_mode mode){
  if( pinCanBeMode(pin, mode) ){
    //Setup temporary storage for all bitMasks and addresses required to change mode
    uint8_t bitMasks[MAX_PINS_PER_IRIS_PIN];
    uint8_t ports[MAX_PINS_PER_IRIS_PIN];
    volatile uint8_t* modeRegisters[MAX_PINS_PER_IRIS_PIN];
    volatile uint8_t* outputRegisters[MAX_PINS_PER_IRIS_PIN];

    //Populate storage
    for(int i = 0; i < MAX_PINS_PER_IRIS_PIN; ++i){
      bitMasks[i]         = pinToBitMask(pin, i);         //Bitmask to write to data register
      ports[i]            = pinToPort(pin, i);            //General Port index
      modeRegisters[i]    = portToModeRegister(ports[i]);   //DDRx  - Data Direction Register
      outputRegisters[i]  = portToOutputRegister(ports[i]); //PORTx - Data Register
    }

    switch( mode ){
      default:
      case iris_pin_mode::input:
        //All connected pins are set as inputs
        for(int i = 0; i < MAX_PINS_PER_IRIS_PIN; ++i){
          suspendInterrupts();
          *modeRegisters[i] &= ~bitMasks[i];
          *outputRegisters[i] &= ~bitMasks[i];
          resumeInterrupts();
        }
      break;

      case iris_pin_mode::input_pullup:
        //All connected pins are set as inputs
        //and all pull-ups are activated
        //Maybe it would be better to only turn on a single pull-up?
        for(int i = 0; i < MAX_PINS_PER_IRIS_PIN; ++i){
          suspendInterrupts();
          *modeRegisters[i] &= ~bitMasks[i];
          *outputRegisters[i] |= bitMasks[i];
          resumeInterrupts();
        }
      break;

      case iris_pin_mode::output:
        //All connected pins are set as outputs
        //This maximises sink and source current
        for(int i = 0; i < MAX_PINS_PER_IRIS_PIN; ++i){
          suspendInterrupts();
          *modeRegisters[i] |= bitMasks[i];
          resumeInterrupts();
        }
      break;

      case iris_pin_mode::serial_spi:
      //All SPI pins are on index 0
      break;

      case iris_pin_mode::serial_uart:
      //RX and TX are on index 0, ~CTS is on index 2
      break;

      case iris_pin_mode::jtag:
      //All JTAG pins are on index 1
      break;
    }

    //Disconnect secondary pins to get more accurate analog readouts
    //and higher current on output
    //TODO: This needs to be implemented per mode, not globally
    suspendInterrupts();
    *modeRegisters[1] &= ~bitMasks[1];
    *outputRegisters[1] &= ~bitMasks[1];
    resumeInterrupts();
    //---END TODO

  }
  #ifdef DEBUG
  //TODO: Send Error message via serial
  #endif
}

//Read digital value from pin
int digitalRead(iris_pin pin){
  //First internal pin is the standard, see comment above iris_pin_to_bitmasks_PGM
  uint8_t bitMask = pinToBitMask(pin, 0);
  uint8_t port = pinToPort(pin, 0);

  //Original Arduino implementation check for timers here.
  //As none of the external iris 16 pins are timer-capable, we leave this out.

  if( *portInputRegister(port) & bitMask ) return HIGH;
  else return LOW;
}

//Write HIGH or LOW to pin
//It is recommended to only use this function when the pin is in output mode
//If you want to (de)activate the internal pull-ups, use pinMode
void digitalWrite(iris_pin pin, uint8_t value){
  //First internal pin is the standard, see comment above iris_pin_to_bitmasks_PGM
  uint8_t bitMask = pinToBitMask(pin, 0);
  uint8_t port = pinToPort(pin, 0);
  volatile uint8_t* outputRegister = portToOutputRegister(port);

  //Original Arduino implementation check for timers here.
  //As none of the external iris 16 pins are timer-capable, we leave this out.

  suspendInterrupts();
  if (value == LOW){
    *outputRegister &= ~bitMask;
  } else {
    *outputRegister |= bitMask;
  }
  resumeInterrupts();
}

//Read analog value from pin
int analogRead(iris_pin pin){
  uint8_t low, high;
  uint8_t bitMask = pinToAnalogChannelBitMask(pin);

  //Check whether pin is actually capable of analog input
  if (bitMask == NOT_ANALOG) return 0;

  //Select to read from channels 0 to 7 (MUX5 in ADCSRB low)
  //channels 8-15 are not connected to the external pins
  bitClear(ADCSRB, MUX5);

  // set the analog reference (high two bits of ADMUX) and select the
  // channel (low 4 bits).  this also sets ADLAR (left-adjust result)
  // to 0 (the default).
  ADMUX = (DEFAULT << 6) | bitMask;

  //start conversion
  bitSet(ADCSRA, ADSC);

  // ADSC is cleared when the conversion finishes
  while (bit_is_set(ADCSRA, ADSC));

  // we have to read ADCL first; doing so locks both ADCL
  // and ADCH until ADCH is read.  reading ADCL second would
  // cause the results of each conversion to be discarded,
  // as ADCL and ADCH would be locked when it completed.
  low  = ADCL;
  high = ADCH;

  //combine the two bytes
  return (high << 8) | low;
}