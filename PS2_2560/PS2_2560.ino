// NOTE: requires the Encoder library.
// 1) open Tools -> Manage Libraries...
// 2) install "Encoder" by Paul Stoffregen v1.4.1
#include <Encoder.h>

// NOTE: Requires the PS2X_lib installed.
// 1) open Sketch -> Include Library -> Add .ZIP Library
// 2) select "PS2X_lib.zip"
#include <PS2X_lib.h>
 
//电机引脚
#define PWMA 12    //A电机转速
#define DIRA1 34 
#define DIRA2 35  //A电机方向
#define PWMB 8    //B电机转速  
#define DIRB1 37 
#define DIRB2 36  //B电机方向
#define PWMC 9   //C电机转速
#define DIRC1 43 
#define DIRC2 42  //C电机方向
#define PWMD 5    //D电机转速
#define DIRD1 26  
#define DIRD2 27  //D电机方向

//PS2
#define PS2_DAT        52  //14    
#define PS2_CMD        51  //15
#define PS2_SEL        53  //16
#define PS2_CLK        50  //17

char speed;
// #define pressures   true
#define pressures   false
// #define rumble      true
#define rumble      false
PS2X ps2x; // create PS2 Controller Class

int error = 0;
byte type = 0;
byte vibrate = 0;

void (* resetFunc) (void) = 0;

#define SERIAL_SDK  Serial

#define LOG_DEBUG

#ifdef LOG_DEBUG
#define M_LOG SERIAL_SDK.print
#else
#define M_LOG 
#endif

#define MAX_PWM   200
#define MIN_PWM   130
int Motor_PWM = 130;

#define NEW_MOTOR
#ifdef NEW_MOTOR

// --- SPD Motor ---

class SPDMotor {
  public:
  SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 );

  /// Set the PWM speed and direction pins.
  /// pwm = 0, stop (no active control)
  /// pwm = 1 to 255, proportion of CCW rotation
  /// pwm = -1 to -255, proportion of CW rotation
  void speed( int pwm );

  /// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
  void hardStop();

  /// Get the current speed.
  int getSpeed();

  /// Get the current rotation position from the encoder.
  long getEncoderPosition();

  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;

    // Current speed setting.
    int _speed;
};

SPDMotor::SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 ) {
  _encoder = new Encoder(encoderA, encoderB);
  _encoderReversed = encoderReversed;

  _motorPWM = motorPWM;
  pinMode( _motorPWM, OUTPUT );
  _motorDir1 = motorDir1;
  pinMode( _motorDir1, OUTPUT );
  _motorDir2 = motorDir2;
  pinMode( _motorDir2, OUTPUT );
}

/// Set the PWM speed and direction pins.
/// pwm = 0, stop (no active control)
/// pwm = 1 to 255, proportion of CCW rotation
/// pwm = -1 to -255, proportion of CW rotation
void SPDMotor::speed( int speedPWM ) {
  _speed = speedPWM;
  if( speedPWM == 0 ) {
    digitalWrite(_motorDir1,LOW);
    digitalWrite(_motorDir2,LOW);
    analogWrite( _motorPWM, 255);
  } else if( speedPWM > 0 ) {
    digitalWrite(_motorDir1, LOW );
    digitalWrite(_motorDir2, HIGH );
    analogWrite( _motorPWM, speedPWM < 255 ? speedPWM : 255);
  } else if( speedPWM < 0 ) {
    digitalWrite(_motorDir1, HIGH );
    digitalWrite(_motorDir2, LOW );
    analogWrite( _motorPWM, (-speedPWM) < 255 ? (-speedPWM): 255);
  }
}

/// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
void SPDMotor::hardStop() {
    _speed = 0;
    digitalWrite(_motorDir1,HIGH);
    digitalWrite(_motorDir2,HIGH);
    analogWrite( _motorPWM, 0);
}

/// Get the current speed.
int SPDMotor::getSpeed() {
    return _speed;
}

/// Get the current rotation position from the encoder.
long SPDMotor::getEncoderPosition() {
  long position = _encoder->read();
  return _encoderReversed ? -position : position;
}

SPDMotor *motorLF = new SPDMotor(18, 31, true, 12, 34, 35); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, false, 8, 36, 37); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor *motorLR = new SPDMotor( 3, 49, true,  9, 43, 42); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor( 2, 23, false, 5, 27, 26); // <- NOTE: Motor Dir pins reversed for opposite operation

#define MOTORA_FORWARD(pwm)    {motorLF->speed(pwm);}
#define MOTORA_STOP(x)         {motorLF->speed(0);}
#define MOTORA_BACKWARD(pwm)   {motorLF->speed(-pwm);}

#define MOTORB_FORWARD(pwm)    {motorRF->speed(pwm);}
#define MOTORB_STOP(x)         {motorRF->speed(0);}
#define MOTORB_BACKWARD(pwm)   {motorRF->speed(-pwm);}

#define MOTORC_FORWARD(pwm)    {motorLR->speed(pwm);}
#define MOTORC_STOP(x)         {motorLR->speed(0);}
#define MOTORC_BACKWARD(pwm)   {motorLR->speed(-pwm);}

#define MOTORD_FORWARD(pwm)    {motorRR->speed(pwm);}
#define MOTORD_STOP(x)         {motorRR->speed(0);}
#define MOTORD_BACKWARD(pwm)   {motorRR->speed(-pwm);}

#else 

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKWARD(pwm)   do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,HIGH); digitalWrite(DIRB2,LOW);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKWARD(pwm)   do{digitalWrite(DIRB1,LOW);digitalWrite(DIRB2,HIGH); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKWARD(pwm)   do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,HIGH); digitalWrite(DIRD2,LOW);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKWARD(pwm)   do{digitalWrite(DIRD1,LOW);digitalWrite(DIRD2,HIGH); analogWrite(PWMD,pwm);}while(0)

#endif

//控制电机运动    宏定义


//    ↑A-----B↑   
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_FORWARD(Motor_PWM);    
  MOTORC_FORWARD(Motor_PWM);MOTORD_FORWARD(Motor_PWM);    
}

//    ↓A-----B↓   
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void BACK()
{
  MOTORA_BACKWARD(Motor_PWM);MOTORB_BACKWARD(Motor_PWM);
  MOTORC_BACKWARD(Motor_PWM);MOTORD_BACKWARD(Motor_PWM);
}
//    =A-----B↑   
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑   
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void LEFT_2()
{
  MOTORA_BACKWARD(Motor_PWM);MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);MOTORD_BACKWARD(Motor_PWM);
}
//    ↓A-----B=   
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_BACKWARD(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_BACKWARD(Motor_PWM);
}
//    ↑A-----B=   
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓   
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM);MOTORB_BACKWARD(Motor_PWM);
  MOTORC_BACKWARD(Motor_PWM);MOTORD_FORWARD(Motor_PWM);
}
//    =A-----B↓   
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM);MOTORB_BACKWARD(Motor_PWM);
  MOTORC_BACKWARD(Motor_PWM);MOTORD_STOP(Motor_PWM);
}
//    =A-----B=  
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);MOTORD_STOP(Motor_PWM);
}
void UART_Control()
{
  char Uart_Date=0;
 if(SERIAL_SDK.available())
  {
   Uart_Date = SERIAL_SDK.read();
  }
  switch(Uart_Date)
  {
     case 'A':  ADVANCE(); M_LOG("Run!\r\n");        break;
     case 'B':  RIGHT_1();  M_LOG("Right up!\r\n");     break;
     case 'C':  RIGHT_2();  M_LOG("Right!\r\n");        break;
     case 'D':  RIGHT_3();  M_LOG("Right down!\r\n");   break;
     case 'E':  BACK();     M_LOG("Run!\r\n");        break;
     case 'F':  LEFT_3();   M_LOG("Left down!\r\n");    break;
     case 'G':  LEFT_2();   M_LOG("Left!\r\n");       break;
     case 'H':  LEFT_1();   M_LOG("Left up!\r\n");  break;
     case 'Z':  STOP();     M_LOG("Stop!\r\n");       break;
     case 'L':  Motor_PWM = 240;                      break;
     case 'M':  Motor_PWM = 130;                       break;
   }
}
void IO_init()
{
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);
  STOP();
}
void setup()
{
  Serial.begin(9600);
  delay(300) ;//added delay to give wireless ps2 module some time to startup, before configuring it
  //CHANGES for v1.6 HERE!!! **************PAY ATTENTION*************

  //setup pins and settings: GamePad(clock, command, attention, data, Pressures?, Rumble?) check for error
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, pressures, rumble);

  if (error == 0) {
    Serial.print("Found Controller, configured successful ");
    Serial.print("pressures = ");
    if (pressures)
      Serial.println("true ");
    else
      Serial.println("false");
    Serial.print("rumble = ");
    if (rumble)
      Serial.println("true)");
    else
      Serial.println("false");
    Serial.println("Try out all the buttons, X will vibrate the controller, faster as you press harder;");
    Serial.println("holding L1 or R1 will print out the analog stick values.");
    Serial.println("Note: Go to www.billporter.info for updates and to report bugs.");
  }
  else if (error == 1)
  {
    Serial.println("No controller found, check wiring, see readme.txt to enable debug. visit www.billporter.info for troubleshooting tips");
    resetFunc();
    
  }

  else if (error == 2)
    Serial.println("Controller found but not accepting commands. see readme.txt to enable debug. Visit www.billporter.info for troubleshooting tips");

  else if (error == 3)
    Serial.println("Controller refusing to enter Pressures mode, may not support it. ");

//  Serial.print(ps2x.Analog(1), HEX);

  type = ps2x.readType();
  switch (type) {
  case 0:
    Serial.print("Unknown Controller type found ");
    break;
  case 1:
    Serial.print("DualShock Controller found ");
    break;
  case 2:
    Serial.print("GuitarHero Controller found ");
    break;
  case 3:
    Serial.print("Wireless Sony DualShock Controller found ");
    break;
  }
  IO_init();
  
//  SERIAL_SDK.print("Start");
}


void loop() {
  /* You must Read Gamepad to get new values and set vibration values
    ps2x.read_gamepad(small motor on/off, larger motor strenght from 0-255)
    if you don't enable the rumble, use ps2x.read_gamepad(); with no values
    You should call this at least once a second
  */

  UART_Control();//串口接收处理 
  if (error == 1) //skip loop if no controller found
    return;

  if (type == 2) { //Guitar Hero Controller
    return;
  }
  else  { //DualShock Controller
    ps2x.read_gamepad(false, vibrate); //read controller and set large motor to spin at 'vibrate' speed

#ifndef NEW_MOTOR
//start 开始运行，电机初PWM为120；
    if (ps2x.Button(PSB_START))  {
      Serial.println("Start is being held");
     Motor_PWM = 90;
      ADVANCE();


    }
// 电机正转；
    if (ps2x.Button(PSB_PAD_UP)) {
      Serial.println("Up held this hard: ");
      Motor_PWM = 120;
     ADVANCE();
    }

// 电机反转；
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Serial.println("Down held this hard: ");
       Motor_PWM = 120;
      BACK();
    }

//左转；
    if (ps2x.Button(PSB_PAD_LEFT)) {
      Serial.println("turn left ");
        Motor_PWM = 120;//200
      LEFT_1();
    }

//右转；
    if (ps2x.Button(PSB_PAD_RIGHT)) {
      Serial.println("turn right");
        Motor_PWM = 120;//200
      RIGHT_1();
    }
// Stop
    if (ps2x.Button(PSB_SELECT)) {
      Serial.println("stop");
      STOP();
    }
// 左平移
    if (ps2x.Button(PSB_PINK)) {
      Serial.println("motor_pmove_left");
      LEFT_2();
    }
// 右平移
    if (ps2x.Button(PSB_RED)) {
      Serial.println("motor_pmove_right");
      RIGHT_2();
    }
    delay(20);
#endif // #ifndef NEW_MOTOR
  }

  if (ps2x.Button(PSB_L1) || ps2x.Button(PSB_R1)) {
#ifdef NEW_MOTOR
    int LY = ps2x.Analog(PSS_LY);
    int LX = ps2x.Analog(PSS_LX);
    int RX = ps2x.Analog(PSS_RX);
    float forwardNormalized = (float)(-LY + 128)/127.f;
    forwardNormalized = constrain( forwardNormalized, -1.f, 1.f );
    float multiplier = (ps2x.Button(PSB_L1) && ps2x.Button(PSB_R1)) ? 255.f : 80.f;
    int forward = (int)(pow(forwardNormalized, 2.0) * multiplier);

    // Preserve the direction of movement.
    if( forwardNormalized < 0 ) {
      forward = -forward;
    }
 
    int right = -RX + 127;
    int ccwTurn = (LX - 127)/2;
    Serial.print( "fwd:" );
    Serial.print( forward );
    Serial.print( " r:" );
    Serial.print( right );
    Serial.print( " ∆°:" );
    Serial.print( ccwTurn );
    Serial.print( " LF:" );
    Serial.print( motorLF->getEncoderPosition() );
    Serial.print( " RF:" );
    Serial.print( motorRF->getEncoderPosition() );
    Serial.print( " LR:" );
    Serial.print( motorLR->getEncoderPosition() );
    Serial.print( " RR:" );
    Serial.println( motorRR->getEncoderPosition() );

    motorLF->speed(forward + ccwTurn - right); motorRF->speed(forward - ccwTurn + right);
    motorLR->speed(forward - ccwTurn - right); motorRR->speed(forward + ccwTurn + right);
#else
    Serial.print("Stick Values:");
    Serial.print(ps2x.Analog(PSS_LY), DEC); //Left stick, Y axis. Other options: LX, RY, RX
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_LX), DEC);
    Serial.print(",");
    Serial.print(ps2x.Analog(PSS_RY), DEC);
    Serial.print(",");
    Serial.println(ps2x.Analog(PSS_RX), DEC);

    int LY = ps2x.Analog(PSS_LY);
    int LX = ps2x.Analog(PSS_LX);
    int RY = ps2x.Analog(PSS_RY);
    int RX = ps2x.Analog(PSS_RX);

    if (LY < 127) //前进
    {

     Motor_PWM = 1.5 * (127 - LY);
      ADVANCE();
      delay(2);
    }
    //后退
    if (LY > 127)
    {
      Motor_PWM = 1.5 * (LY - 128);
      BACK();
      delay(2);
    }
    //左转
    if (LX < 128)
    {
      Motor_PWM = 1.5 * (127 - LX);
       LEFT_1();
      delay(2);
    }
    //右转
    if (LX > 128)
    {
      Motor_PWM = 1.5 * (LX - 128);
      RIGHT_3();
      delay(2);
    }
    //如果摇杆居中
    if (LY >= 128 && LY <= 128 && LX >= 128 && LX <= 128)
    {
      STOP();
      delay(2);
    }
#endif

  } else {
#ifdef NEW_MOTOR
  if( motorLF->getSpeed() != 0
  || motorRF->getSpeed() != 0
  || motorLR->getSpeed() != 0
  || motorRR->getSpeed() != 0 )
  {
      motorLF->hardStop(); motorRF->hardStop();
      motorLR->hardStop(); motorRR->hardStop();
      delay(500);
      motorLF->speed(0); motorRF->speed(0);
      motorLR->speed(0); motorRR->speed(0);
  }
#endif
  }
}
