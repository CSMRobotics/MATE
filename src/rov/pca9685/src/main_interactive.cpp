#include <iostream>

#include <rclcpp/rclcpp.hpp>

#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

int main(int argc, char** argv) {
    std::string userInput;

    while(true) {
        // get user input
        std::getline(std::cin, userInput);
        boost::algorithm::to_lower(userInput);

        // process exit code
        if(userInput.size() == 0) continue;
        if(userInput.at(0) == 'q') break;

        try {
            int split = userInput.find(' ')+1;
            int split2 = userInput.substr(split).find(' ')+1;
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
            int pwm_or_angle = boost::lexical_cast<int>(PWM_or_angle);

            if(is_pwm) {
                std::cout << "Setting pin " << pin << " to pwm: " << pwm_or_angle << '\n';
            } else {
                std::cout << "Setting pin " << pin << " to angle: " << pwm_or_angle << '\n';
            }
        } catch(boost::bad_lexical_cast &) {
            std::cout << "Bad format\nUse the format: 'pinNumber PWM_or_angle <PWM_TRUE>'\n";
            std::cout << "If PWM_TRUE is not input, PWM_or_angle will be interpreted as an angle\n";
        } catch(std::out_of_range &) {
            std::cout << "substr threw out of range exception\n";
        }
    }
}