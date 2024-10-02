#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>

/* nRF24L01+ module connections

   module   Arduino
   1 GND ---- GND
   2 VCC ---- 3.3V  Note: 5V on VCC will destroy module (but other pins are 5V tolerant)
   3 CE ----- D9
   4 CSN ---- D10
   5 SCK ---- D13 (SCK)
   6 MOSI --- D11 (MOSI)
   7 MISO --- D12 (MISO)
   8 IRQ ---- not connected
*/

// the nRF24L01+ can tune to 128 channels with 1 MHz spacing from 2.400 GHz to 2.527 GHz.
#define CHANNELS 127
#define STARTCHANNEL 0

// SPI definitions and macros
#define CE_pin    9
#define CS_pin   10
#define MOSI_pin 11
#define MISO_pin 12
#define SCK_pin  13

#define TFT_CS         3
#define TFT_RST       -1 // Or set to -1 and connect to Arduino RESET pin
#define TFT_DC         2
#define TFT_SCL        19
#define TFT_SDA        18 

#define  MODE_CHANGE   4
#define  BAT_CHE       14
#define  BAT_LOW       5

#define  CE_on    PORTB |= 0x02
#define  CE_off   PORTB &= 0xFD
#define  CS_on    PORTB |= 0x04
#define  CS_off   PORTB &= 0xFB
#define  MOSI_on  PORTB |= 0x08
#define  MOSI_off PORTB &= 0xF7
#define  MISO_on  (PINB & 0x10)  // input
#define  SCK_on   PORTB |= 0x20
#define  SCK_off  PORTB &= 0xDF

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_SDA, TFT_SCL, TFT_RST);

// nRF24 Register map
enum {
  NRF24L01_00_CONFIG      = 0x00,
  NRF24L01_01_EN_AA       = 0x01,
  NRF24L01_02_EN_RXADDR   = 0x02,
  NRF24L01_03_SETUP_AW    = 0x03,
  NRF24L01_04_SETUP_RETR  = 0x04,
  NRF24L01_05_RF_CH       = 0x05,
  NRF24L01_06_RF_SETUP    = 0x06,
  NRF24L01_07_STATUS      = 0x07,
  NRF24L01_08_OBSERVE_TX  = 0x08,
  NRF24L01_09_CD          = 0x09,
  NRF24L01_0A_RX_ADDR_P0  = 0x0A,
  NRF24L01_0B_RX_ADDR_P1  = 0x0B,
  NRF24L01_0C_RX_ADDR_P2  = 0x0C,
  NRF24L01_0D_RX_ADDR_P3  = 0x0D,
  NRF24L01_0E_RX_ADDR_P4  = 0x0E,
  NRF24L01_0F_RX_ADDR_P5  = 0x0F,
  NRF24L01_10_TX_ADDR     = 0x10,
  NRF24L01_11_RX_PW_P0    = 0x11,
  NRF24L01_12_RX_PW_P1    = 0x12,
  NRF24L01_13_RX_PW_P2    = 0x13,
  NRF24L01_14_RX_PW_P3    = 0x14,
  NRF24L01_15_RX_PW_P4    = 0x15,
  NRF24L01_16_RX_PW_P5    = 0x16,
  NRF24L01_17_FIFO_STATUS = 0x17,
  NRF24L01_1C_DYNPD       = 0x1C,
  NRF24L01_1D_FEATURE     = 0x1D,
  //Instructions
  NRF24L01_61_RX_PAYLOAD  = 0x61,
  NRF24L01_A0_TX_PAYLOAD  = 0xA0,
  NRF24L01_E1_FLUSH_TX    = 0xE1,
  NRF24L01_E2_FLUSH_RX    = 0xE2,
  NRF24L01_E3_REUSE_TX_PL = 0xE3,
  NRF24L01_50_ACTIVATE    = 0x50,
  NRF24L01_60_R_RX_PL_WID = 0x60,
  NRF24L01_B0_TX_PYLD_NOACK = 0xB0,
  NRF24L01_FF_NOP         = 0xFF,
  NRF24L01_A8_W_ACK_PAYLOAD0 = 0xA8,
  NRF24L01_A8_W_ACK_PAYLOAD1 = 0xA9,
  NRF24L01_A8_W_ACK_PAYLOAD2 = 0xAA,
  NRF24L01_A8_W_ACK_PAYLOAD3 = 0xAB,
  NRF24L01_A8_W_ACK_PAYLOAD4 = 0xAC,
  NRF24L01_A8_W_ACK_PAYLOAD5 = 0xAD,
};

// Bit mnemonics
enum {
  NRF24L01_00_MASK_RX_DR  = 6,
  NRF24L01_00_MASK_TX_DS  = 5,
  NRF24L01_00_MASK_MAX_RT = 4,
  NRF24L01_00_EN_CRC      = 3,
  NRF24L01_00_CRCO        = 2,
  NRF24L01_00_PWR_UP      = 1,
  NRF24L01_00_PRIM_RX     = 0,

  NRF24L01_07_RX_DR       = 6,
  NRF24L01_07_TX_DS       = 5,
  NRF24L01_07_MAX_RT      = 4,

  NRF2401_1D_EN_DYN_ACK   = 0,
  NRF2401_1D_EN_ACK_PAY   = 1,
  NRF2401_1D_EN_DPL       = 2,
};

enum TXRX_State {
  TXRX_OFF,
  TX_EN,
  RX_EN,
};

const uint8_t gImage[512] = {
0X00,0X00,0X3F,0XFF,0XFF,0XF8,0X00,0X00,0X00,0X00,0X7F,0XFF,0XFF,0XFE,0X00,0X00,
0X00,0X00,0XFF,0XFF,0XFF,0XFE,0X00,0X00,0X00,0X00,0XF0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE3,0XFF,0XFF,0X8F,0X00,0X00,0X00,0X00,0XE3,0XFF,0XFF,0X8F,0X00,0X00,
0X00,0X00,0XE3,0XFF,0XFF,0X8F,0X00,0X00,0X00,0X00,0XE3,0XFF,0XFF,0X8F,0X00,0X00,
0X00,0X00,0XE3,0XFF,0XFF,0X8F,0X00,0X00,0X00,0X00,0XE3,0XFF,0XFF,0X8F,0X00,0X00,
0X00,0X00,0XE3,0XFF,0XFF,0X8F,0X00,0X00,0X00,0X00,0XE3,0XFF,0XFF,0X8F,0X00,0X00,
0X00,0X00,0XE3,0XFF,0XFF,0X8F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XE0,0X00,0X00,0X0F,0X00,0X00,0X00,0X00,0XF0,0X00,0X00,0X0F,0X00,0X00,
0X00,0X00,0XFF,0XFF,0XFF,0XFE,0X00,0X00,0X00,0X00,0X7F,0XFF,0XFF,0XFE,0X00,0X00,
0X00,0X00,0X3F,0XFF,0XFF,0XF8,0X00,0X00,0X00,0X00,0X00,0X1F,0XF0,0X00,0X00,0X00,
0X00,0X00,0X00,0X1F,0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XF0,0X00,0X00,0X00,
0X00,0X00,0X00,0X1F,0XF0,0X00,0X00,0X00,0X00,0X00,0X00,0X1F,0XF0,0X00,0X00,0X00,
};

uint8_t MHz = STARTCHANNEL;
uint16_t signalStrength[128];   // smooths signal strength with numerical range 0 - 0x7FFF
uint8_t prevStrength[128];      // save signal strength displayed on OLED for comparison with actual value
uint8_t column = STARTCHANNEL;
uint16_t strength;
uint8_t row = 0;
uint8_t b = 0;
const uint8_t ff[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
int mode = 1;
float LOW_PRO = 2.20;
int open=1;

void setup() {
  //1.44" TFT:
  tft.initR(INITR_144GREENTAB); // Init ST7735R chip, green tab
  tft.fillScreen(ST77XX_BLACK);
  delay(10);

  pinMode(BAT_CHE,INPUT);
  pinMode(MODE_CHANGE, INPUT);
  pinMode(MOSI_pin, OUTPUT);
  pinMode(SCK_pin, OUTPUT);
  pinMode(CS_pin, OUTPUT);
  pinMode(CE_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  pinMode(BAT_LOW,OUTPUT);
  CS_on;
  CE_on;
  MOSI_on;
  SCK_on;
  delay(10);
  CS_off;
  CE_off;
  MOSI_off;
  SCK_off;
  delay(10);
  CS_on;

  NRF24L01_Reset();
  delay(10);

  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);     // switch off Shockburst mode
  NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x0F);  // write default value to setup register
  NRF24L01_SetTxRxMode(RX_EN);                    // switch to receive mode

  delay(50); // start up message

  int sensorValue = analogRead(BAT_CHE);
  float voltage = sensorValue * (5.0/1023.0);
  if(voltage<LOW_PRO) {
    digitalWrite(BAT_LOW,HIGH);
    displayImage(gImage);
    delay(5000);
  }
  else{
    drawFrequencyMarkersAndLabels();

    //setup Timer1 for NRF scanning
    cli();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1  = 0;
    OCR1A = 45;                           // cca 6kHz
    TCCR1B |= (1 << WGM12);               // turn on CTC mode
    TCCR1B |= (1 << CS11) | (1 << CS10);  // Set 64 prescaler
    TIMSK1 |= (1 << OCIE1A);              // enable timer compare interrupt

    NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);
    CE_on;        // start receiving
    sei();        // allow interrupts
  }
}

ISR(TIMER1_COMPA_vect) {                                        //Timer2 intterupt vector, time to get data from nrf
  CE_off;                                                       // stop receiving - one bit is now set if received power was > -64 dBm at that instant
  if (NRF24L01_ReadReg(NRF24L01_09_CD)) {                       // signal detected so increase signalStrength unless already maxed out
    signalStrength[MHz] += (0x7FFF - signalStrength[MHz]) >> 6; // increase rapidly when previous value was low, with increase reducing exponentially as value approaches maximum
  } else {                                                      // no signal detected so reduce signalStrength unless already at minimum
    signalStrength[MHz] -= signalStrength[MHz] >> 6;            // decrease rapidly when previous value was high, with decrease reducing exponentially as value approaches zero
  }

  MHz++;
  if (MHz == CHANNELS + STARTCHANNEL) MHz = STARTCHANNEL;

  NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);                    // Set new freqency for scan
  CE_on;                                                        // start receiving
  TCNT1  = 0;                                                   // clear timer counter
  OCR1A = random(35, 55);                                       // make the measuring slightly random in time
}

void loop() {
  uint16_t prevTotal[13] = {0};
  int sensorValue = analogRead(BAT_CHE);
  float voltage = sensorValue * (5.0/1023.0);

  if(voltage>=LOW_PRO){
    if(open==1){
      if(voltage<=LOW_PRO+0.2){
        digitalWrite(BAT_LOW,HIGH);
      }
      else if(voltage>2.30){
        digitalWrite(BAT_LOW,LOW);
      }

      if(digitalRead(MODE_CHANGE)== 0){
        mode=-mode;
        tft.fillScreen(ST77XX_BLACK);
        if(mode==1){
          drawFrequencyMarkersAndLabels();
          int loop = 128;
          while(loop){
            strength = 0;
            uint8_t prevStrengthValue = 110;
            prevStrength[column] = strength;
            column++;
            if (column == CHANNELS + STARTCHANNEL) column = STARTCHANNEL;
              loop--;
            }
          }
        else{
            tft.setTextSize(1);
            tft.setTextColor(ST77XX_WHITE);
            tft.setCursor(0,0);
            tft.print("2.400-2.409GHz");
            tft.print(": ");
            tft.setCursor(0,10);
            tft.print("2.410-2.419GHz");
            tft.print(": ");
            tft.setCursor(0,20);
            tft.print("2.420-2.429GHz");
            tft.print(": ");
            tft.setCursor(0,30);
            tft.print("2.430-2.439GHz");
            tft.print(": ");
            tft.setCursor(0,40);
            tft.print("2.440-2.449GHz");
            tft.print(": ");
            tft.setCursor(0,50);
            tft.print("2.450-2.459GHz");
            tft.print(": ");
            tft.setCursor(0,60);
            tft.print("2.460-2.469GHz");
            tft.print(": ");
            tft.setCursor(0,70);
            tft.print("2.470-2.479GHz");
            tft.print(": ");
            tft.setCursor(0,80);
            tft.print("2.480-2.489GHz");
            tft.print(": ");
            tft.setCursor(0,90);
            tft.print("2.490-2.499GHz");
            tft.print(": ");
            tft.setCursor(0,100);
            tft.print("2.500-2.509GHz");
            tft.print(": ");
            tft.setCursor(0,110);
            tft.print("2.510-2.519GHz");
            tft.print(": ");
            tft.setCursor(0,120);
            tft.print("2.520-2.527GHz");
            tft.print(": ");
          displayChannelAndStrength();
        }
      }

      //波形显示
      if(mode==1){
        strength = (signalStrength[column] + 0x0040) >> 7;
        strength*=4;
        if (strength > 110) strength = 110;  // 限制最大高度为显示屏的可显示范围
        uint8_t prevStrengthValue = prevStrength[column];
        uint8_t row = 14 - (strength / 8);  // 计算信号强度的起始行
        if (strength % 8) row--;

        if (strength > prevStrengthValue) {
          // 信号变强，向上加
          uint8_t yStart = 110 - prevStrengthValue;
          uint8_t yEnd = 110 - strength + 1;
          tft.drawLine(column, yStart, column, yEnd, ST77XX_WHITE);  // 绘制上升的信号强度
        } 
        else if (strength < prevStrengthValue) {
          // 信号变弱，向下减
          uint8_t yStart = 110 - strength;
          uint8_t yEnd = 110 - prevStrengthValue + 1;
          tft.drawLine(column, yStart, column, yEnd, ST77XX_BLACK);  // 清除下降的信号强度
        }

        // 更新 prevStrength 并移动到下一列
        prevStrength[column] = strength;
        column++;
        if (column == CHANNELS + STARTCHANNEL) column = STARTCHANNEL;
      }

      //数值显示
      if(mode==-1){
        for (uint8_t c = 0; c < 13; c++) {  // 计算每个信道的总信号强度
          uint16_t total = 0;
          if(c==12){
            for (int i = 0; i < 8; i++) total += (signalStrength[i + c * 10] + 0x0040) >> 7;
          }
          else{
          for (int i = 0; i < 10; i++) total += (signalStrength[i + c * 10] + 0x0040) >> 7;
          }

          // 只有当总信号强度的变化大于8时才更新显示
          if (abs(total - prevTotal[c]) > 8) {
            // 先清除旧数据
            tft.fillRect(96, c * 10, 18, 10, ST77XX_BLACK);

            // 更新显示
            tft.setCursor(96, c * 10);
            tft.print(total);

            // 存储新的总信号强度值
            prevTotal[c] = total;
            if (digitalRead(MODE_CHANGE) == 0) break;
            delay(100);
            if (digitalRead(MODE_CHANGE) == 0) break;
            delay(100);
            if (digitalRead(MODE_CHANGE) == 0) break;
            delay(100);
            if (digitalRead(MODE_CHANGE) == 0) break;
            delay(100);
            if (digitalRead(MODE_CHANGE) == 0) break;
            delay(100);
            if (digitalRead(MODE_CHANGE) == 0) break;
            delay(100);
            if (digitalRead(MODE_CHANGE) == 0) break;
            delay(100);
            if (digitalRead(MODE_CHANGE) == 0) break;
            delay(100);
          }
        }
      }
    }
    else{
      digitalWrite(BAT_LOW,HIGH);
      delay(100);
      digitalWrite(BAT_LOW,LOW);
      delay(100);
    }
  }
  else{
    if(open==1){
      tft.fillScreen(ST77XX_BLACK);
      open=0;
    }
    digitalWrite(BAT_LOW,HIGH);
    delay(100);
    digitalWrite(BAT_LOW,LOW);
    delay(100);
  }
}

uint8_t _spi_write(uint8_t command)
{
  uint8_t result = 0;
  uint8_t n = 8;
  SCK_off;
  MOSI_off;
  while (n--) {
    if (command & 0x80)
      MOSI_on;
    else
      MOSI_off;
    if (MISO_on)
      result |= 0x01;
    SCK_on;
    _NOP();
    SCK_off;
    command = command << 1;
    result = result << 1;
  }
  MOSI_on;
  return result;
}

void _spi_write_address(uint8_t address, uint8_t data)
{
  CS_off;
  _spi_write(address);
  _NOP();
  _spi_write(data);
  CS_on;
}

uint8_t _spi_read()
{
  uint8_t result = 0;
  uint8_t i;
  MOSI_off;
  _NOP();
  for (i = 0; i < 8; i++) {
    if (MISO_on) // if MISO is HIGH
      result = (result << 1) | 0x01;
    else
      result = result << 1;
    SCK_on;
    _NOP();
    SCK_off;
    _NOP();
  }
  return result;
}

uint8_t _spi_read_address(uint8_t address)
{
  uint8_t result;
  CS_off;
  _spi_write(address);
  result = _spi_read();
  CS_on;
  return (result);
}

/* Instruction Mnemonics */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define REGISTER_MASK 0x1F
#define ACTIVATE      0x50
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define W_TX_PAYLOAD  0xA0
#define W_ACK_PAYLOAD 0xA8
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define NOP           0xFF

uint8_t NRF24L01_WriteReg(uint8_t address, uint8_t data)
{
  CS_off;
  _spi_write_address(address | W_REGISTER, data);
  CS_on;
  return 1;
}

uint8_t NRF24L01_FlushTx()
{
  return Strobe(FLUSH_TX);
}

uint8_t NRF24L01_FlushRx()
{
  return Strobe(FLUSH_RX);
}

static uint8_t Strobe(uint8_t state)
{
  uint8_t result;
  CS_off;
  result = _spi_write(state);
  CS_on;
  return result;
}

uint8_t NRF24L01_ReadReg(uint8_t reg)
{
  CS_off;
  uint8_t data = _spi_read_address(reg);
  CS_on;
  return data;
}

void NRF24L01_SetTxRxMode(uint8_t mode)
{
  if (mode == TX_EN) {
    CE_off;
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR)    // reset the flag(s)
                      | (1 << NRF24L01_07_TX_DS)
                      | (1 << NRF24L01_07_MAX_RT));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                      (1 << NRF24L01_00_EN_CRC)   // switch to TX mode
                      | (1 << NRF24L01_00_CRCO)
                      | (1 << NRF24L01_00_PWR_UP));
    delayMicroseconds(130);
    CE_on;
  } else if (mode == RX_EN) {
    CE_off;
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // reset the flag(s)
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);        // switch to RX mode
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR)    //reset the flag(s)
                      | (1 << NRF24L01_07_TX_DS)
                      | (1 << NRF24L01_07_MAX_RT));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                      (1 << NRF24L01_00_EN_CRC)   // switch to RX mode
                      | (1 << NRF24L01_00_CRCO)
                      | (1 << NRF24L01_00_PWR_UP)
                      | (1 << NRF24L01_00_PRIM_RX));
    delayMicroseconds(130);
    CE_on;
  } else {
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC)); // PowerDown
    CE_off;
  }
}

uint8_t NRF24L01_Reset()
{
  NRF24L01_FlushTx();
  NRF24L01_FlushRx();
  uint8_t status1 = Strobe(0xFF); // NOP
  uint8_t status2 = NRF24L01_ReadReg(0x07);
  NRF24L01_SetTxRxMode(TXRX_OFF);
  return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}

// 显示图片函数
void displayImage(const uint8_t *image) {
  tft.setRotation(-1);
  int width = 96; // 设置图像宽度
  int height = 96; // 设置图像高度
  int idx = 0; // 图像数组索引

  for (int y = 32; y < height; y++) {
    for (int x = 32; x < width; x++) {
      // 计算当前像素点在数组中的位置
      uint8_t byte = image[idx / 8];
      uint8_t bit = 7 - (idx % 8);

      // 根据位图数据设置像素颜色
      if (byte & (1 << bit)) {
        tft.drawPixel(x, y, ST77XX_WHITE); // 绘制白色像素
      } else {
        tft.drawPixel(x, y, ST77XX_BLACK); // 绘制黑色像素
      }
      idx++;
    }
  }
}

void displayChannelAndStrength() {
  for (uint8_t c = 0; c < 13; c++) {
    uint16_t totalStrength = 0;
    uint8_t maxIndex = (c == 12) ? 8 : 10;  // 判断是最后一个信道
    for (uint8_t i = 0; i < maxIndex; i++) {
      totalStrength += (signalStrength[i + c * 10] + 0x0040) >> 7;
    }
    tft.setCursor(96, c * 10);
    tft.print(totalStrength);  // 显示信号强度值
  }
}

void drawFrequencyMarkersAndLabels() {
  // 绘制频率刻度线
  for (int x = 0; x < 128; x++) {
    uint8_t b = 0x01;  // baseline
    if (!(x % 10)) {
      b |= 0x0F;  // 每10MHz刻度
    }
    if (x == 10 || x == 60 || x == 110) {
      b |= 0xF8;  // 在2.41, 2.46, 和 2.51 GHz处的刻度标记
    }

    // 在屏幕上绘制垂直的刻度线
    for (int i = 0; i < 9; i++) {
      if (b & (1 << i)) {
        tft.drawPixel(x, 111 + i, ST77XX_WHITE);
      }
    }
  }

  // 显示频率标记
  tft.setCursor(0, 121);
  tft.print(F("2.41"));

  tft.setCursor(50, 121);
  tft.print(F("2.46"));

  tft.setCursor(100, 121);
  tft.print(F("2.51"));

  // 清空其他的文本行
  for (int y = 1; y <= 5; y += 2) {
    tft.setCursor(0, y * 8);
    tft.print(F("                     "));
  }
}