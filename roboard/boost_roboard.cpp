#include <boost/python.hpp>
#include <iostream>

#include <roboard.h>
#include <pwm.h>

class RoboardManager{
    private:
        unsigned long period;
    public:
        RoboardManager();
        ~RoboardManager();
        bool initialize();
        void close_servos();
        void send_pwm_to_channel(int channel, unsigned long duty, unsigned long milliseconds);
        void send_continuous_pwm_to_channel(int channel, unsigned long duty);
        void display_last_error();
};
RoboardManager::RoboardManager(){
    period = 2000L;
}
RoboardManager::~RoboardManager(){
    this->close_servos();
}

void RoboardManager::send_pwm_to_channel(int channel, unsigned long duty, unsigned long milliseconds){
    rcservo_SendPWM(channel, period, duty, milliseconds);
}

void RoboardManager::send_continuous_pwm_to_channel(int channel, unsigned long duty){
    rcservo_SendCPWM(channel, period, duty);
}

bool RoboardManager::initialize(){
    unsigned long used_channels = RCSERVO_USECHANNEL0
        + RCSERVO_USECHANNEL1
        + RCSERVO_USECHANNEL2
        + RCSERVO_USECHANNEL3
        + RCSERVO_USECHANNEL4
        + RCSERVO_USECHANNEL5
        + RCSERVO_USECHANNEL6
        + RCSERVO_USECHANNEL7
        + RCSERVO_USECHANNEL8
        + RCSERVO_USECHANNEL9
        + RCSERVO_USECHANNEL10
        + RCSERVO_USECHANNEL11
        + RCSERVO_USECHANNEL12
        + RCSERVO_USECHANNEL13
        + RCSERVO_USECHANNEL14
        + RCSERVO_USECHANNEL15;
    roboio_SetRBVer(RB_110);
    bool result = rcservo_Initialize(used_channels);
    rcservo_EnterPWMMode();
    return result;
}
void RoboardManager::close_servos(){
    rcservo_Close();
}
void RoboardManager::display_last_error(){
    std::cout<<roboio_GetErrMsg()<<"\n";
}

using namespace boost::python;
BOOST_PYTHON_MODULE(boost_roboard)
{
    class_<RoboardManager>("RoboardManager")
        .def("initialize", &RoboardManager::initialize)
        .def("close_servos", &RoboardManager::close_servos)
        .def("send_pwm_to_channel", &RoboardManager::send_pwm_to_channel)
        .def("send_continuous_pwm_to_channel", &RoboardManager::send_continuous_pwm_to_channel)
        .def("display_last_error", &RoboardManager::display_last_error);
}
