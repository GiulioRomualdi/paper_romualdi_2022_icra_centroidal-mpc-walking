/**
 * @file FakeJoystick.h
 * @author Giulio Romualdi
 * @copyright BSD 3-clause
 */

#ifndef FAKE_JOYSTICK_H
#define FAKE_JOYSTICK_H

#include <BipedalLocomotion/System/Source.h>
#include <memory>

namespace CentroidalMPCWalking
{

struct JoypadSignal
{
    double leftAnalogX = 0.0;
    double leftAnalogY = 0.0;
    double rightAnalogX = 0.0;
    double rightAnalogY = 0.0;
};

class FakeJoypad : public BipedalLocomotion::System::Source<JoypadSignal>
{
public:
    FakeJoypad();
    ~FakeJoypad();

    bool advance() override;
    const JoypadSignal& getOutput() const override;
    bool isOutputValid() const override; 

private:
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};
} // namespace CentroidalMPCWalking

#endif
