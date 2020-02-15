#include <stdio.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define K_OUT 18
#define K_IN  19

bool is_initialized = false;

void serial_rx_on() 
{
//  UCSR0B |= _BV(RXEN0);  //enable UART RX
  Serial1.begin(12700);    //setting enable bit didn't work, so do beginSerial
}

void serial_rx_off() 
{
  UCSR1B &= ~(_BV(RXEN1));  //disable UART RX
}

void serial_tx_off() 
{
   UCSR1B &= ~(_BV(TXEN1));  //disable UART TX
   delay(20);                 //allow time for buffers to flush
}

int iso_read_byte()
{
  int b;
  byte t=0;
  while(t!=125  && (b=Serial1.read())==-1) {
    delay(1);
    t++;
  }
  if (t>=125) {
    b = 0;
  }
  return b;
}

void iso_write_byte(byte b)
{
  serial_rx_off();
  Serial1.write(b);
  delay(10);    // ISO requires 5-20 ms delay between bytes.
  serial_rx_on();
}

bool iso_init(void)
{
  byte b;
  byte kw1, kw2;
  serial_tx_off(); //disable UART so we can "bit-Bang" the slow init.
  serial_rx_off();
  delay(3000); //k line should be free of traffic for at least two secconds.
  // drive K line high for 300ms
  digitalWrite(K_OUT, HIGH);
  delay(300);

  // send 0x10 at 5 bauds
  // start bit
  digitalWrite(K_OUT, LOW);
  delay(200);
  // data
  b=0x10;
  for (byte mask = 0x01; mask; mask <<= 1)
  {
    if (b & mask) // choose bit
      digitalWrite(K_OUT, HIGH); // send 1
    else
      digitalWrite(K_OUT, LOW); // send 0
    delay(200);
  }
  // stop bit + 60 ms delay
  digitalWrite(K_OUT, HIGH);
  delay(260);

  // switch now to 12700 bauds
  Serial1.begin(12700);

  // wait for 0x55 from the ECU (up to 300ms)
  //since our time out for reading is 125ms, we will try it three times
  for(int i=0; i<3; i++) 
  {
    b=iso_read_byte(); 
    Serial.print("ECU ACKNOWLEDGE : ");
    Serial.println(b, HEX);
    if(b!=0)
      break;
  }
  
  if(b!=0x55)
    return false;

  // wait for kw1 and kw2
  kw1=iso_read_byte();
  Serial.print("kw1 : ");
  Serial.println(kw1,HEX);

  kw2=iso_read_byte();
  Serial.print("kw2 : ");
  Serial.println(kw2,HEX);  
  delay(250);

  // sent ~kw2 (invert of last keyword)
  iso_write_byte(~kw2);
  delay(250);

  // ECU answer by 0xAB
  b=iso_read_byte();
  Serial.print("ANSWER : ");
  Serial.println(b,HEX);
  if(b != 0xAB)
    return false;

  // init OK!
  return true;
}

void setup() 
{
  // init pinouts
  pinMode(K_OUT, OUTPUT);
  pinMode(K_IN, INPUT);
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("VOLVO 850 DIAG STARTING");
}

void loop() 
{
  if (!is_initialized)
  {
    // put your main code here, to run repeatedly:
    is_initialized = iso_init();  
    if (is_initialized)
    {
      Serial.println("ECU INITIALIZED");
    }
    else
    {
      Serial.println("NOT INITIALIZED\n");    
    }
  }
  delay(1000);
}
