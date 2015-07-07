// The program is tested on Arduino Leonardo R3.

#include <SoftwareSerial.h>
#include <PS3BT.h>
#include <PS3USB.h>
#include <usbhub.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#include <SPI.h>
#endif

#define PS3_UP        0x0001
#define PS3_DOWN      0x0002
#define PS3_LEFT      0x0004
#define PS3_RIGHT     0x0008
#define PS3_START     0x0010
#define PS3_SELECT    0x0020
#define PS3_L3        0x0040
#define PS3_R3        0x0080
#define PS3_L1        0x0100
#define PS3_R1        0x0200
#define PS3_PS        0x0400
#define PS3_CROSS     0x1000
#define PS3_CIRCLE    0x2000
#define PS3_SQUARE    0x4000
#define PS3_TRIANGLE  0x8000

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

#define GamepadSerial Serial1

unsigned short gamepad_digital;
unsigned char package_msg[8];
unsigned char package_crc[2];

static const unsigned short crc16table[256] = {
  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

bool crc16(unsigned char* buffer, const unsigned char* message, int msg_l) {
  // taken from http://www.menie.org/georges/embedded/crc16.html
  int counter;
  unsigned short crc = 0;
  for (counter = 0; counter < msg_l; counter++) {
    crc = (crc << 8) ^ crc16table[((crc >> 8) ^ *(char *)message++) & 0x00FF];
  }
  buffer[0] = (unsigned char)(crc >> 8);
  buffer[1] = (unsigned char)crc;
  return true;
}

short scale_up_u8_s16(unsigned char input) {
  if (input == 128)
    return 0;
  return (((long)input << 8)) - 32768 | input;
}

void setup() {
  if (Usb.Init() == -1) {
    while (1);  // halt
  }
  GamepadSerial.begin(115200);
}

void loop() {
  Usb.Task();

  if (PS3.PS3Connected) {
    // Digital buttons
    gamepad_digital = 0x0000; // 16-bit unsigned integer.
    if (PS3.getButtonPress(UP))
      gamepad_digital |= PS3_UP;
    if (PS3.getButtonPress(DOWN))
      gamepad_digital |= PS3_DOWN;
    if (PS3.getButtonPress(LEFT))
      gamepad_digital |= PS3_LEFT;
    if (PS3.getButtonPress(RIGHT))
      gamepad_digital |= PS3_RIGHT;
    if (PS3.getButtonPress(START))
      gamepad_digital |= PS3_START;
    if (PS3.getButtonPress(SELECT))
      gamepad_digital |= PS3_SELECT;
    if (PS3.getButtonPress(L3))
      gamepad_digital |= PS3_L3;
    if (PS3.getButtonPress(R3))
      gamepad_digital |= PS3_R3;
    if (PS3.getButtonPress(L1))
      gamepad_digital |= PS3_L1;
    if (PS3.getButtonPress(R1))
      gamepad_digital |= PS3_R1;
    if (PS3.getButtonPress(PS))
      gamepad_digital |= PS3_PS;
    if (PS3.getButtonPress(CROSS))
      gamepad_digital |= PS3_CROSS;
    if (PS3.getButtonPress(CIRCLE))
      gamepad_digital |= PS3_CIRCLE;
    if (PS3.getButtonPress(SQUARE))
      gamepad_digital |= PS3_SQUARE;
    if (PS3.getButtonPress(TRIANGLE))
      gamepad_digital |= PS3_TRIANGLE;

    // L2, R2 triggers
    unsigned char gamepad_left_trigger = PS3.getAnalogButton(L2);
    unsigned char gamepad_right_trigger = PS3.getAnalogButton(R2);

    // Left & right analog joystick value
    short gamepad_left_joy_x = scale_up_u8_s16(PS3.getAnalogHat(LeftHatX));
    short gamepad_left_joy_y = scale_up_u8_s16(255 - PS3.getAnalogHat(LeftHatY));
    short gamepad_right_joy_x = scale_up_u8_s16(PS3.getAnalogHat(RightHatX));
    short gamepad_right_joy_y = scale_up_u8_s16(255 - PS3.getAnalogHat(RightHatY));

    // Send data (part 1)
    package_msg[0] = gamepad_digital;
    package_msg[1] = gamepad_digital >> 8;
    package_msg[2] = gamepad_left_trigger;
    package_msg[3] = gamepad_right_trigger;
    package_msg[4] = gamepad_left_joy_x;
    package_msg[5] = gamepad_left_joy_x >> 8;
    package_msg[6] = gamepad_left_joy_y;
    package_msg[7] = gamepad_left_joy_y >> 8;
    crc16(package_crc, package_msg, 8);

    GamepadSerial.write(0x12); // soh
    GamepadSerial.write(0x90); // id
    GamepadSerial.write(0x08); // data_length
    GamepadSerial.write(package_msg[0]);
    GamepadSerial.write(package_msg[1]);
    GamepadSerial.write(package_msg[2]);
    GamepadSerial.write(package_msg[3]);
    GamepadSerial.write(package_msg[4]);
    GamepadSerial.write(package_msg[5]);
    GamepadSerial.write(package_msg[6]);
    GamepadSerial.write(package_msg[7]);
    GamepadSerial.write(0x90); // id
    GamepadSerial.write(package_crc[0]);
    GamepadSerial.write(package_crc[1]);
    GamepadSerial.write(0x34); // eot

    // Send data (part 2)
    package_msg[0] = gamepad_right_joy_x;
    package_msg[1] = gamepad_right_joy_x >> 8;
    package_msg[2] = gamepad_right_joy_y;
    package_msg[3] = gamepad_right_joy_y >> 8;
    crc16(package_crc, package_msg, 4);

    GamepadSerial.write(0x12); // soh
    GamepadSerial.write(0x91); // id
    GamepadSerial.write(0x04); // data_length
    GamepadSerial.write(package_msg[0]);
    GamepadSerial.write(package_msg[1]);
    GamepadSerial.write(package_msg[2]);
    GamepadSerial.write(package_msg[3]);
    GamepadSerial.write(0x91); // id
    GamepadSerial.write(package_crc[0]);
    GamepadSerial.write(package_crc[1]);
    GamepadSerial.write(0x34); // eot
  }
}
