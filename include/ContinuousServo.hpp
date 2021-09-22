
class ContinuousServo : public ServoBase{
    public:
        ContinuousServo(ServoDriver driver, int channel);
        ~ContinuousServo();
}