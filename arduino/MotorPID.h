

class MotorPID {
  public:
    int goalSpeed;

    MotorPID(
      float kp, float ki, float kd,
      int plusePerRev, int gearRatio, int deltaTime
    );
    void setGoalSpeed(int motorGoalSpeed);
    void setPIDGain(float kp, float ki, float kd);

    void encoderAInterrupt(bool encoderAValue, bool encoderBValue);
    void encoderBInterrupt(bool encoderAValue, bool encoderBValue);

    void control(int arr[]);

    float getCurrentSpeed();

    long getEncoderPosition();

  private:
    float currentSpeed;
    float kp, ki, kd;

    float errorSpeed;
    float prevErrorSpeed;
    float errorSpeedSum;

    long encoderPosition;
    long prevEncoderPosition;
    int dletaEncoderPosition;

    int plusePerRev;
    int gearRatio;
    int dletaTime;
};