
class Servo : public ServoBase{
    public:
        Servo(ServoDriver driver, int channel);
        ~Servo();

        void setLimits(float minimum_limit, float maximum_limit);
    
    private:
        float minimum_limit = 0;
        float maximum_limit = 0;
}