/**
 * @file FakeJoystick.h
 * @author Giulio Romualdi
 * @copyright BSD 3-clause
 */

#ifndef JOYPAD_PROVIDER_H
#define JOYPAD_PROVIDER_H

#include <BipedalLocomotion/System/Sink.h>
#include <FakeJoypad/FakeJoypad.h>
#include <memory>

namespace CentroidalMPCWalking
{

class JoypadProvider : public BipedalLocomotion::System::Sink<JoypadSignal>
{
public:
    JoypadProvider();
    ~JoypadProvider();

    bool advance() override;
    bool setInput(const JoypadSignal& input) override;
private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};
} // namespace CentroidalMPCWalking

#endif
