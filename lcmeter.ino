//LC-метр v 071 rev B
//original           https://ra4nal.ontvtime.ru/lc_arduino.shtml
//LiquidCrystal_I2C  https://github.com/johnrickman/LiquidCrystal_I2C
//fastRead function  https://alexgyver.ru/arduino-algorithms/
//_edKotinsky        https://vk.com/dudewhodied

//  |   Range  | frequency | max value
//  |   1(0)   | 1,000,000 |  892       
//  |   2(1)   | 250,000   |  3,568
//  |   3(2)   | 62,500    |  14,272 
//  |   4(3)   | 15,625    |  57,088
//  |   5(4)   | 3,906     |  228,352
//  |   6(5)   | 976       |  913,408
//  |   7(6)   | 244       |  3,653,632
//  |   8(7)   | 61        |  14,614,528
//  |   9(8)   | 15        |  58,458,112

//  диапазоны считаются от нуля до 8 [0; 8]
//  на каждом диапазоне частота уменьшается в 4 раза  freq = 1,000,000 / 4^diap
//  диапазон измерения увеличивается соответственно в 4 раза
//  истинное измеряемое значение value = resultADC * 4^diap
//  последний разряд результата - 0.1, т.е 892 = 89.2 пФ

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

//  выбрать дефайн, соответствующий частоте ядра
//#define CPU_FREQ_8MHZ
#define CPU_FREQ_16MHZ

#define C_BTN       2       //  вывод для кнопки калибровки
#define M_SW        3       //  вывод для переключателя режимов

#define NDIAP       8       //  количество диапазонов
#define STAB_TIME   500     //  время между заданием частоты таймеру и чтением АЦП
#define DMIN        192     //  минимальное значение для переключения диапазона
#define DMAX        892     //  максимальное значение для переключения диапазона
#define MAX_CALIB   192     //  максимальное значение при калибровке

#define MODE_C      0x0
#define MODE_L      0x1

LiquidCrystal_I2C lcd(0x27, 16, 2);

void lcdcl (uint8_t row = 0) {
  lcd.clear();
  lcd.setCursor(0, row);
}

void lcdline (uint8_t row = 0) {
  lcd.setCursor(0, row);
  lcd.print(F("                "));
}

bool fastRead(const uint8_t pin) {
  if (pin < 8) {
    return bitRead(PIND, pin);
  } else if (pin < 14) {
    return bitRead(PINB, pin - 8);
  } else if (pin < 20) {
    return bitRead(PINC, pin - 14);
  } else {
    return 0;
  }
}

  //mode switch condition
bool modeSwitch (uint8_t pin) {
  return fastRead(pin);     //0 - C mode, 1 - L mode
}

  //calibration button handler
bool button (const uint8_t pin, uint32_t debounce) {
  static uint32_t prevMillis = 0;
  static bool flag = 0;

  bool btnState = !fastRead(pin);
  
  if (btnState && !flag && millis() - prevMillis > debounce) {
    flag = 1;
    prevMillis = millis();
  }
  if (!btnState && flag &&millis() - prevMillis > debounce) {
    flag = 0;
    prevMillis = millis();
  }

  return flag;
}

  //configure timer1, wait for stabilization time, start ADC convertion and return result
int16_t freqAndRead (uint8_t diap, bool& flag) {
  //                    fclk_I/O
  //    fOC1A = -----------------------
  //            2 · presc · (1 + OCR1A)

  static uint32_t  prevMillis = 0;
  int16_t result  = 0;
  uint16_t top    = 2;

  #if defined CPU_FREQ_8MHZ 
    uint8_t pwr = 2 * diap + 2;
  #elif defined CPU_FREQ_16MHZ
    uint8_t pwr = 2 * diap + 3;
  #endif
  
  ADCL = 0x0;
  ADCH = 0x0;

  //configure timer1
  TCCR1A =   0b01000000;            //COM1A0  CTC mode with top on OCR1A
  if (diap < 3) {
    TCCR1B = 0b00001001;            //WGM12 | CS10  presc = 1
  } else {
    TCCR1B = 0b00001100;            //WGM12 | CS12  presc = 256
    
    #if defined CPU_FREQ_8MHZ 
      pwr = pwr - 7;
    #elif defined CPU_FREQ_16MHZ
      pwr = pwr - 8;
    #endif
  }

  if (pwr != 0) {
    for (uint8_t i = 1; i < pwr; ++i) {
      top = top << 1;               //top^pwr
    }
  } else {
    top = 1;
  }
  
  if (--top != OCR1A) {             //if top != previous top, range changed 
    TCNT1H = 0x0;                   //so reset timer counter
    TCNT1L = 0x0;   
    OCR1A = top;                    //write new top value to OCR
    prevMillis = millis();          //and reload delay
  }

  //wait 0.5 second to stabilize voltage on the module output
  if (millis() -  prevMillis > STAB_TIME) {              
    while (!(PINB & 0b00000010));   //wait two signal changes
    while (PINB & 0b00000010);      //and 6 mS to get value at the 
    delay(6);                       //particular moment
    ADCSRA |= (1 << ADSC);          //start adc convertion
    do {                            //while convertion is in process  
      SMCR = 0b00000011;            //SM0 | SE  enable ADC noise reduction sleep mode
    } while (ADCSRA & (1 << ADSC)); //wait until convertion complete
    flag = 1;                       //then set the result ready flag
    result = ADC;                   //read adc result register
    prevMillis = millis();          //and set current millis value to prevTime
  }

  return result;
}

  //print the calibration state
void calibrationMsg (bool error) {
  lcd.print(F("Calibration "));
  if (error) {
    lcd.print(F("err"));
  }
}

  //print range and ADC value to display
void printRL (uint8_t diap, int16_t value) {
  lcd.setCursor(0, 1);
  lcd.print(F("range "));
  lcd.print(diap);
  lcd.print(F(" ADC "));
  lcd.print(value);
}

uint8_t getCountOfDigits (int32_t number) {
  int count = (number == 0) ? 1 : 0;
  while (number) {
    ++count;
    number /= 10;
  }

  return count;
}

  //interpretate and print result
uint8_t convertAndPrint (int32_t result, uint8_t diap, uint8_t startPos) {
  uint8_t commaNum    = 1;
  uint8_t totalLength = 0;
  uint8_t numCounter  = 0;
  uint8_t num         = 0;
  bool roundCont      = 0;
  uint8_t resLen      = 0;
  
  while (diap) {
    --diap;                               //result = resultADC * 4^diap
    result = result << 2;                 //result * 4 while diap != 0
  }

  resLen = getCountOfDigits(result);
  if (resLen < 2) ++resLen;
  
  if (resLen >= 5) {                      //number of the decimal separators
    commaNum = 2;                         //depends of number of result symbols
    if (resLen > 7) {                     //1,000.0   - 5 digit, 2 decimal separators
      commaNum = 3;                       //100,000.0 - 7 digit, 3 decimal separators
    }                                     //100.0     - 4 digit, 1 decimal separators
  }

                                          //totalLength is a column, on which result printing ends
  totalLength = resLen + startPos + commaNum - 1;
                                          //loop recursively goes by number from the least significant digits and prints result from end 
  for (uint8_t i = totalLength; i >= startPos; --i) {
    lcd.setCursor(i, 0);
    if (i == totalLength - 1) {           //legth - 1 - is a decimal
      lcd.print('.');                     //they are separated by the dot
      numCounter = 0;                     //when it happens, digit counter is reset
    } else if (numCounter == 3) {         //every three digits
      lcd.print(',');                     //print the comma
      numCounter = 0;                     //and reset digit counter to zero
    } else {                              //if not a decimal separator, then print the digit
      num = result % 10;                  //get the number from current digit
      if (i > startPos + 3) {             //if loop do not goes the three high significant digits
        if (num > 5) {                    //if the number of current digit is > 5
          roundCont = 1;                  //set the rounding flag
        }                                 //and reset number of current digit to zero
        num = 0;
      }
      lcd.print(num);
      result /= 10;                       //discard the current digit
      if (roundCont) {                    //if in the discarded digit was a rounding to high 
        result += 1;                      //add 1 to the current digit number
        roundCont = 0;                    //and reset rounding flag
      }
      if (result) ++numCounter;           //count printing digits
    }
  }

  return totalLength;    //totalLength is a column, on which result printing ends
}

void printResult (uint8_t diap, int16_t value, bool mode) {
  uint8_t printCol = 2;
  lcd.setCursor(0, 0);
  if (mode == MODE_C) lcd.print(F("C"));
  else                lcd.print(F("L"));
  printCol += convertAndPrint(value, diap, printCol);
  lcd.setCursor(printCol, 0);
  if (mode == MODE_C) lcd.print(F("pF"));
  else                lcd.print(F("uH"));
}

void setup() {
  //ADC setup
  ADMUX  = 0b01000000;      //REFS0 | MUX3:0 = 0, input ADC0
  
  #if defined CPU_FREQ_16MHZ
    ADCSRA = 0b10000111;    //ADEN | ADPS2:0 prescaler = 128
  #elif defined CPU_FREQ_8MHZ
    ADCSRA = 0b10000110;    //ADEN | ADPS2:1 prescaler = 64
  #endif
  
  //first ADC convertion to initialize ADC
  ADCSRA |= (1 << ADSC);

  //pin modes
  pinMode(9, OUTPUT);           //FRQ output
  pinMode(C_BTN, INPUT_PULLUP); //calibration button
  pinMode(M_SW, INPUT_PULLUP);  //mode switch

  //lc display
  lcd.begin(16, 2);
  lcd.backlight();
}

void loop() {
  static uint8_t diap = NDIAP;
  static bool flag        = 0;
  static bool calibration = 0;
  static bool mode        = 0;

  int16_t result    = 0;
  uint16_t eeAddr   = 0;
  bool resultReady  = 0;
  bool error        = 0;
  
  if (mode != modeSwitch(M_SW)) {         //mode switch state
    if (calibration) {          
      calibration = 0;
      diap = NDIAP;
    }
    mode = !mode;
  }
  if (!calibration && button(C_BTN, 100)) { //if calibration
    diap = NDIAP;                         //reset range
    calibration = 1;
  }
  
  result = freqAndRead(diap, resultReady);
  
  if (resultReady) {                      //once a 0.5 second
    lcdcl();

    if (mode == MODE_C) eeAddr = diap;    //calibration values: c measurement mode EEPROM [0; 8]
    else  eeAddr = eeAddr + NDIAP + 1;    //                    l measurement mode EEPROM [9; 17]

    if (calibration) {                    //  CALIBRATION
      if (result < MAX_CALIB) {
        EEPROM.update(eeAddr, lowByte(result));
      } else {
        error = 1;
      }

      calibrationMsg(error);
      printRL(diap, result);      

      if (diap) {                         //calibration lasts up to range == 0
        --diap;                           //diap e [0; 8]
      } else {
        calibration = 0;      
      }
    } else {                              //  MEASURIMENT
      bool diapChanged = 0;
      
      if (result > DMAX && diap < NDIAP) {
        printRL(diap, result);
        ++diap;
        flag = 1;
        diapChanged = 1;
      }
      if (result < DMIN && diap) {
        printRL(diap, result);
        --diap;
        flag = 1;
        diapChanged = 1;
      }

      if (flag) {                         //if the range has been changed
        if (!diapChanged){                //and now range is not changed
          printRL(diap, result);          //print current range
          flag = 0;                       //and clear change flag
        } else 
          lcdline();                      //clear the first line
      } 
      
      if (!flag ){                        //if the range hasnt been changed
        result = result - EEPROM.read(eeAddr);  //calculate and print result value
        result = constrain(result, 0, 1023);
        printResult(diap, result, mode);
      }
    }
  }
}
