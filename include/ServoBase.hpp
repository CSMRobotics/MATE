
class ServoBase{
    public:
        virtual void setSetpoint(float setpoint) = 0;
        float getSetpoint(){
            return this->setpoint;
        }


    protected:
        int channel;
        ServoDriver driver;
        float setpoint = 0.0f;
}