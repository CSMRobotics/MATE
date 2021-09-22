
ServoDriver::ServoDriver(){
    this.setPWMFrequency(60.0f);
}

Servo* ServoDriver::addServo(int channel){
    if(channel < 0 || channel > 15) throw "Channel number " + std::to_string(channel) + " is not in range [0, 15]";
    if(this->servos[channel] != 0) throw "Channel number" + std::to_string(channel) + " is in use";

    Servo* new_servo = new Servo(this);
    this->servos[channel] = new_servo;
    return new_servo;
}

ContinuousServo ServoDriver::addContinuousServo(int channel){
    if(channel < 0 || channel > 15) throw "Channel number " + std::to_string(channel) + " is not in range [0, 15]";
    if(this->servos[channel] != 0) throw "Channel number" + std::to_string(channel) + " is in use";

    Servo* new_servo = new ContinuousServo(this);
    this->servos[channel] = new_servo;
    return new_servo;
}

ServoBase* ServoDriver::get(int channel){
    if(channel < 0 || channel > 15) throw "Channel number " + std::to_string(channel) + " is not in range [0, 15]";
    if(this->servos[channel] == 0) throw "Channel number" + std::to_string(channel) + " is not in use";

    return this->servos[channel];
}

void ServoDriver::setPWM(int channel, int value){
    if(channel < 0 || channel > 15) throw "Channel number " + std::to_string(channel) + " is not in range [0, 15]";
    if(this->servos[channel] == 0) throw "Channel number" + std::to_string(channel) + " is not in use";

    this->driver_board.setPWM(channel, 0, value);
}

void ServoDriver::setPWMFrequency(float frequency){
    this->driver_board.setPWMFrequency(frequency);
}