#include <Adafruit_ST7735.h> // ST7735硬件库
//TO DO change Adafruit_ST7735.h to PDQ_ST7735.h

/*
  nRF24L01+模块连接说明：
    模块引脚  Arduino引脚
    1 GND ---- GND
    2 VCC ---- 3.3V
    3 CE ----- D9
    4 CSN ---- D10
    5 SCK ---- D13 (SCK)
    6 MOSI --- D11 (MOSI)
    7 MISO --- D12 (MISO)
    8 IRQ ---- 未连接
*/

// nRF24信道参数
#define CHANNELS 128       // 总信道数
#define STARTCHANNEL 0     // 起始信道

// SPI相关引脚定义
#define CE_pin    9
#define CS_pin   10
#define MOSI_pin 11
#define MISO_pin 12
#define SCK_pin  13

// TFT显示屏引脚定义
#define TFT_CS         3
#define TFT_RST       -1  // 连接到Arduino复位或不连接
#define TFT_DC         2
#define TFT_SCL        19
#define TFT_SDA        18 

#define MODE_CHANGE    4  // 模式切换按键

// 通过端口操作控制引脚（比digitalWrite快）
#define CE_on    PORTB |= 0x02
#define CE_off   PORTB &= 0xFD
#define CS_on    PORTB |= 0x04
#define CS_off   PORTB &= 0xFB
#define MOSI_on  PORTB |= 0x08
#define MOSI_off PORTB &= 0xF7
#define MISO_on  (PINB & 0x10)  // 输入读取
#define SCK_on   PORTB |= 0x20
#define SCK_off  PORTB &= 0xDF

// 初始化显示屏对象，使用软件SPI
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_SDA, TFT_SCL, TFT_RST);

// nRF24寄存器地址定义
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
  // 指令
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

// nRF24寄存器位掩码定义
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

// 发送/接收模式枚举
enum TXRX_State {
  TXRX_OFF,
  TX_EN,
  RX_EN,
};

// 全局变量定义
uint8_t MHz = STARTCHANNEL;                  // 当前信道索引
uint16_t signalStrength[129];                // 信号强度平滑值，范围0-0x7FFF
uint8_t prevStrength[129];                   // 上一次绘制的信号强度，用于对比
uint8_t column = STARTCHANNEL;                // 当前绘制列
uint16_t strength;
uint8_t row = 0;
uint8_t b = 0;
const uint8_t ff[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}; // 用于广播地址
int flag = 1;                                 // 模式标志
uint16_t extotal[16]={0};                      // 通道强度缓存，用于避免重复绘制

// Timer1 比较匹配中断服务程序，用于周期性扫描频率并更新信号强度
ISR(TIMER1_COMPA_vect) {
  CE_off;  // 停止接收，准备更换信道

  // 读取当前信道信号检测寄存器(CD)
  if (NRF24L01_ReadReg(NRF24L01_09_CD)) {
    // 有信号，指数平滑增加信号强度值
    signalStrength[MHz] += (0x7FFF - signalStrength[MHz]) >> 6;
  } else {
    // 无信号，指数平滑递减信号强度值
    signalStrength[MHz] -= signalStrength[MHz] >> 6;
  }

  // 切换到下一个信道
  MHz++;
  if (MHz == CHANNELS + STARTCHANNEL) MHz = STARTCHANNEL;

  // 设置nRF24工作在新信道
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);
  CE_on;   // 开始接收新信道数据

  // 重置计数器，OCR1A计数值稍作随机，避免周期完全固定
  TCNT1  = 0;
  OCR1A = random(35, 55);
}

// 初始化函数，完成显示屏及模块配置，定时器启动等
void setup(){
  // 初始化显示屏，黑色背景，旋转方向设置
  tft.initR(INITR_18BLACKTAB);
  tft.fillScreen(ST77XX_BLACK);
  tft.setRotation(2);

  // 设置引脚模式
  pinMode(MODE_CHANGE, INPUT);
  pinMode(MOSI_pin, OUTPUT);
  pinMode(SCK_pin, OUTPUT);
  pinMode(CS_pin, OUTPUT);
  pinMode(CE_pin, OUTPUT);
  pinMode(MISO_pin, INPUT);
  pinMode(5,OUTPUT);
  digitalWrite(5,HIGH);

  // 初始化nRF24模块
  NRF24L01_Reset();
  delay(10);

  // 关闭Shockburst模式，默认射频设置
  NRF24L01_WriteReg(NRF24L01_01_EN_AA, 0x00);
  NRF24L01_WriteReg(NRF24L01_06_RF_SETUP, 0x0F);
  NRF24L01_SetTxRxMode(RX_EN);

  delay(10);

  // 绘制频率刻度和标签
  drawFrequencyMarkersAndLabels();

  // 配置Timer1用于定时扫描信道
  cli(); // 禁用中断
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  OCR1A = 45;               // 大约6kHz中断频率
  TCCR1B |= (1 << WGM12);   // CTC模式
  TCCR1B |= (1 << CS11) | (1 << CS10);  // 64分频
  TIMSK1 |= (1 << OCIE1A);  // 允许比较匹配中断
  sei(); // 使能中断

  // 设置nRF24初始信道并开启接收
  NRF24L01_WriteReg(NRF24L01_05_RF_CH, MHz);
  CE_on;
}

// 主循环函数，根据模式标志选择不同显示逻辑
void loop(){
  if(digitalRead(MODE_CHANGE) == 0) {
    flag = -flag;              // 模式切换
    tft.fillScreen(ST77XX_BLACK);
    if(flag == 1){
      drawFrequencyMarkersAndLabels();
      int loop = 129;
      // 清除上一屏数据
      while(loop){
        strength = 0;
        prevStrength[column] = strength;
        column++;
        if (column == CHANNELS + STARTCHANNEL) column = STARTCHANNEL;
        loop--;
      }
    }
    else{
      // 显示通道标签
      for (int d = 0; d < 16; d++) {
        tft.setTextSize(1);
        tft.setTextColor(ST77XX_WHITE);
        tft.setCursor(0, d * 10);
        tft.print("CH");
        tft.print(d);
        tft.print(d >= 10 ? " :  " : "  :  ");
      }
    }
  }

  if(flag == 1){
    // 计算当前信号强度，右移并限制最大值
    strength = (signalStrength[column] + 0x0040) >> 7;
    if (strength >= 140) strength = 140;

    uint8_t prevStrengthValue = prevStrength[column];
    uint8_t row = 14 - (strength / 8);  // 信号强度起始行
    if (strength % 8) row--;

    if (strength > prevStrengthValue) {
      // 信号增强，绘制白线
      uint8_t yStart = 139 - prevStrengthValue;
      uint8_t yEnd = 139 - strength + 1;
      tft.drawLine(column, yStart, column, yEnd, ST77XX_WHITE);
    } else if (strength < prevStrengthValue) {
      // 信号减弱，绘制黑线清除
      uint8_t yStart = 139 - strength;
      uint8_t yEnd = 139 - prevStrengthValue + 1;
      tft.drawLine(column, yStart, column, yEnd, ST77XX_BLACK);
    }

    // 更新缓存并移动绘制列
    prevStrength[column] = strength;
    column++;
    if (column == CHANNELS + STARTCHANNEL) column = STARTCHANNEL;
  }

  if(flag == -1){
    // 固定显示指定通道强度
    updateChannelStrength(95,0);
    updateChannelStrength(73,1);
    updateChannelStrength(75,2);
    updateChannelStrength(77,3);
    updateChannelStrength(79,4);
    updateChannelStrength(81,5);
    updateChannelStrength(85,6);
    updateChannelStrength(89,7);
    updateChannelStrength(101,8);
    updateChannelStrength(103,9);
    updateChannelStrength(93,10);
    updateChannelStrength(105,11);
    updateChannelStrength(109,12);
    updateChannelStrength(113,13);
    updateChannelStrength(117,14);
    updateChannelStrength(121,15);
  }
}

// 软件SPI写入一个字节函数，使用位操作控制引脚
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
    _NOP();  // 小延时，确保时钟稳定
    SCK_off;
    command = command << 1;
    result = result << 1;
  }
  MOSI_on;
  return result;
}

// 软件SPI写寄存器函数，写地址和数据
void _spi_write_address(uint8_t address, uint8_t data)
{
  CS_off;
  _spi_write(address);
  _NOP();
  _spi_write(data);
  CS_on;
}

// 软件SPI读一个字节函数
uint8_t _spi_read()
{
  uint8_t result = 0;
  uint8_t i;
  MOSI_off;
  _NOP();
  for (i = 0; i < 8; i++) {
    if (MISO_on) // MISO高
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

// 软件SPI读寄存器函数，读指定地址
uint8_t _spi_read_address(uint8_t address)
{
  uint8_t result;
  CS_off;
  _spi_write(address);
  result = _spi_read();
  CS_on;
  return result;
}

// nRF24寄存器读写相关指令定义
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

// 写寄存器
uint8_t NRF24L01_WriteReg(uint8_t address, uint8_t data)
{
  CS_off;
  _spi_write_address(address | W_REGISTER, data);
  CS_on;
  return 1;
}

// 清空发送缓冲区
uint8_t NRF24L01_FlushTx()
{
  return Strobe(FLUSH_TX);
}

// 清空接收缓冲区
uint8_t NRF24L01_FlushRx()
{
  return Strobe(FLUSH_RX);
}

// 发送一个命令字节
static uint8_t Strobe(uint8_t state)
{
  uint8_t result;
  CS_off;
  result = _spi_write(state);
  CS_on;
  return result;
}

// 读取寄存器数据
uint8_t NRF24L01_ReadReg(uint8_t reg)
{
  CS_off;
  uint8_t data = _spi_read_address(reg);
  CS_on;
  return data;
}

// 设置nRF24工作模式：发射、接收或关闭
void NRF24L01_SetTxRxMode(uint8_t mode)
{
  if (mode == TX_EN) {
    CE_off;
    // 清除状态寄存器中断标志
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR) |
                      (1 << NRF24L01_07_TX_DS) |
                      (1 << NRF24L01_07_MAX_RT));
    // 配置为发射模式
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                      (1 << NRF24L01_00_EN_CRC) |
                      (1 << NRF24L01_00_CRCO) |
                      (1 << NRF24L01_00_PWR_UP));
    delayMicroseconds(130);
    CE_on;
  } else if (mode == RX_EN) {
    CE_off;
    NRF24L01_WriteReg(NRF24L01_07_STATUS, 0x70);        // 重置状态寄存器
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, 0x0F);        // 设置为接收模式基本配置
    NRF24L01_WriteReg(NRF24L01_07_STATUS,
                      (1 << NRF24L01_07_RX_DR) |
                      (1 << NRF24L01_07_TX_DS) |
                      (1 << NRF24L01_07_MAX_RT));
    NRF24L01_WriteReg(NRF24L01_00_CONFIG,
                      (1 << NRF24L01_00_EN_CRC) |
                      (1 << NRF24L01_00_CRCO) |
                      (1 << NRF24L01_00_PWR_UP) |
                      (1 << NRF24L01_00_PRIM_RX));
    delayMicroseconds(130);
    CE_on;
  } else {
    // 关闭nRF24模块，仅保留CRC使能
    NRF24L01_WriteReg(NRF24L01_00_CONFIG, (1 << NRF24L01_00_EN_CRC));
    CE_off;
  }
}

// 重置nRF24模块
uint8_t NRF24L01_Reset()
{
  NRF24L01_FlushTx();
  NRF24L01_FlushRx();
  uint8_t status1 = Strobe(0xFF); // 发送NOP命令，读取状态
  uint8_t status2 = NRF24L01_ReadReg(0x07);
  NRF24L01_SetTxRxMode(TXRX_OFF);
  // 返回模块是否正常（状态匹配）
  return (status1 == status2 && (status1 & 0x0f) == 0x0e);
}

// 绘制频率刻度和文字标记
void drawFrequencyMarkersAndLabels() {
  for (int x = 0; x < 128; x++) {
    uint8_t b = 0x01;  // 基线刻度
    if (!(x % 10)) {
      b |= 0x0F;  // 每10MHz长刻度
    }
    if (x == 10 || x == 60 || x == 110) {
      b |= 0xF8;  // 2.41, 2.46, 2.51 GHz处刻度加粗
    }

    // 垂直绘制刻度线
    for (int i = 0; i < 9; i++) {
      if (b & (1 << i)) {
        tft.drawPixel(x, 140 + i, ST77XX_WHITE);
      }
    }
  }

  // 显示文字频率标签
  tft.setCursor(1, 150);
  tft.print(F("2.41"));
  tft.setCursor(50, 150);
  tft.print(F("2.46"));
  tft.setCursor(100, 150);
  tft.print(F("2.51"));

  // 清除多余文本行
  for (int y = 1; y <= 5; y += 2) {
    tft.setCursor(0, y * 8);
    tft.print(F("                     "));
  }
}

// 更新指定通道强度显示，防止重复绘制
void updateChannelStrength(uint8_t data, uint8_t CH) {
  if (digitalRead(MODE_CHANGE) == LOW)
  return;

  float total = 0;
  float data_c = 0;
  float data_c1 = 0;
  float data_c2 = 0;
  // 平滑计算当前通道信号强度，加权主信道和邻近信道
  for (int i = data - 2; i <= data + 2; i++) {
    if (i == data) {
      data_c = ((signalStrength[i]) >> 5) * 0.7;
      total += data_c;
    }
    if (i == data - 1 || i == data + 1) {
      data_c1 = ((signalStrength[i]) >> 5) * 0.1;
      total += data_c1;
    }
    if (i == data - 2 || i == data + 2) {
      data_c2 = ((signalStrength[i]) >> 5) * 0.05;
      total += data_c2;
    }
  }

  if (total == extotal[CH]) {
    // 信号强度绘制
    tft.setCursor(48, CH * 10);
    tft.print((uint16_t)total);
  } 
  else {
    // 信号强度变化，清除旧区域，重新绘制
    tft.fillRect(48, CH * 10, 18, 10, ST77XX_BLACK);
    tft.setCursor(48, CH * 10);
    tft.print((uint16_t)total);
  }
  extotal[CH] = total;
  
  // 模式切换按键被按下时，提前退出避免冲突
  if (digitalRead(MODE_CHANGE) == LOW)
  return;
}