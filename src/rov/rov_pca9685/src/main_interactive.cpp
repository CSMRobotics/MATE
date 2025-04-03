#include <iostream>
#include <map>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

#include "pca9685/ServoDriver.hpp"

int main([[maybe_unused]] int argc, [[maybe_unused]] char** argv) {
    std::string userInput;
    std::map<int, bool> isContinuous;

    ServoDriver servoDriver;

    std::cout << "Load Default Config?\n";
    std::getline(std::cin, userInput);
    boost::algorithm::to_lower(userInput);
    if(userInput.at(0) == 'y') {
        std::cout << "Continuous C or non continuous N\n";
        std::getline(std::cin, userInput);
        boost::algorithm::to_lower(userInput);
        if(userInput.at(0) == 'c') {
            // load continuous servo defaults
            for(int i=0; i<16; i++) {
                servoDriver.registerServo(i, ServoType::CONTINUOUS);
                isContinuous[i]=true;
                servoDriver.setUSBounds(i, 1100, 1900);
            }
        } else {
            // load non continuous servo defaults
            for(int i=0; i<16; i++) {
                servoDriver.registerServo(i, ServoType::POSITIONAL);
                isContinuous[i]=false;
                servoDriver.setUSBounds(i, 500, 2500);
            }
        }
    } else {
        std::cout << "For all 16 pins, C for continuous N for non continuous\n";
        for(int i = 0; i < 16; i++) {
            std::cout << "Pin " << i << ": ";
            std::getline(std::cin, userInput);
            std::cout << '\n';
            boost::algorithm::to_lower(userInput);
            try {
                if(userInput.at(0) == 'c') {
                    servoDriver.registerServo(i, ServoType::CONTINUOUS);
                    isContinuous[i] = true;
                } else {
                    // default to non continuous
                    servoDriver.registerServo(i, ServoType::POSITIONAL);
                    isContinuous[i] = false;
                }
            } catch(std::out_of_range &) {
                servoDriver.registerServo(i, ServoType::POSITIONAL);
                isContinuous[i] = false;
            }
        }

        std::cout << "\nFor all 16 pins, enter US range separated by spaces (newline for default 500-2500)\n";
        for(int i = 0; i < 16; i++) {
            std::cout << "Pin " << i << ": ";
            std::getline(std::cin, userInput);
            boost::algorithm::to_lower(userInput);
            size_t idx = userInput.find(' ');
            int lower_bound = 500;
            int upper_bound = 2500;
            if(idx != 0 && idx != std::string::npos) {
                lower_bound = boost::lexical_cast<int>(userInput.substr(0, idx));
                upper_bound = boost::lexical_cast<int>(userInput.substr(idx+1));
            }
            servoDriver.setUSBounds(i, lower_bound, upper_bound);
        }
    }

    std::cout << "Configuration Done.\nInput expected: <pin> <pwm_or_angle> <optional:is_pwm>";

    while(true) {
        // get user input
        std::getline(std::cin, userInput);
        boost::algorithm::to_lower(userInput);

        // process exit code
        if(userInput.size() == 0) continue;
        if(userInput.at(0) == 'q') break;

        try {
            size_t split = userInput.find(' ')+1;
            size_t split2 = userInput.substr(split).find(' ')+1;
            if(split == std::string::npos) {
                std::cout << "Could not find a space in your string, try again\n";
            }
            std::string pinNumber = userInput.substr(0, split-1);
            std::cout << "pinNumber: '" << pinNumber << "'\n";
            std::string PWM_or_angle = userInput.substr(split, split2-1);
            std::cout << "PWM_or_angle: '" << PWM_or_angle << "'\n";
            bool is_pwm;
            if(split2 != std::string::npos && split2 != 0) {
                is_pwm = userInput.substr(split2+2).size() > 0;
            } else {
                is_pwm = false;
            }
            int pin = boost::lexical_cast<int>(pinNumber);
            float pwm_or_angle = boost::lexical_cast<float>(PWM_or_angle);

            if(is_pwm) {
                std::cout << "Setting pin " << pin << " to pwm: " << pwm_or_angle << '\n';
                servoDriver.setDuty(pin, pwm_or_angle);
            } else {
                if(isContinuous[pin]) {
                    servoDriver.setThrottle(pin, pwm_or_angle);
                } else {
                    servoDriver.setAngle(pin, pwm_or_angle);
                }
            }
        } catch(boost::bad_lexical_cast &) {
            std::cout << "Bad format\nUse the format: 'pinNumber PWM_or_angle <PWM_TRUE>'\n";
            std::cout << "If PWM_TRUE is not input, PWM_or_angle will be interpreted as an angle\n";
        } catch(std::out_of_range &) {
            std::cout << "substr threw out of range exception\n";
        }
    }
}