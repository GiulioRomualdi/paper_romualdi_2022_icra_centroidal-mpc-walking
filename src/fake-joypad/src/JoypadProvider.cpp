#include <BipedalLocomotion/System/SharedResource.h>
#include <FakeJoypad/JoypadProvider.h>

#include <yarp/os/BufferedPort.h>
#include <yarp/sig/Vector.h>

using namespace CentroidalMPCWalking;

struct JoypadProvider::Impl
{
    yarp::os::BufferedPort<yarp::sig::Vector> port;
    JoypadSignal joypad;

    Impl()
    {
        port.open("/fakeJoypad:o");
    }

    ~Impl()
    {
        port.close();
    }
};

JoypadProvider::JoypadProvider()
{
    m_pimpl = std::make_unique<Impl>();
}

JoypadProvider::~JoypadProvider() = default;

bool JoypadProvider::advance()
{
    auto& output = m_pimpl->port.prepare();
    output.resize(4);
    output[0] = m_pimpl->joypad.leftAnalogX;
    output[1] = m_pimpl->joypad.leftAnalogY;
    output[2] = m_pimpl->joypad.rightAnalogX;
    output[3] = m_pimpl->joypad.rightAnalogY;
    m_pimpl->port.write();

    return true;
}


bool JoypadProvider::setInput(const JoypadSignal& input)
{
    m_pimpl->joypad = input;
    return true;
}
