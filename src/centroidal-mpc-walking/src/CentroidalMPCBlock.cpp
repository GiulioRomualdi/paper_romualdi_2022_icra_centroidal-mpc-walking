/**
 * @file CentroidalMPCBlock.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#include <manif/manif.h>

#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/Contacts/ContactListJsonParser.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/ModelIO/ModelLoader.h>

#include <CentroidalMPCWalking/CentroidalMPCBlock.h>

#include <iDynTree/Core/EigenHelpers.h>

using namespace CentroidalMPCWalking;
using namespace BipedalLocomotion::ParametersHandler;

Eigen::Vector3d CoM0;

void updateContactPhaseList(
    const std::map<std::string, BipedalLocomotion::Contacts::PlannedContact>& nextPlannedContacts,
    BipedalLocomotion::Contacts::ContactPhaseList& phaseList)
{
    auto newList = phaseList.lists();
    for (const auto& [key, contact] : nextPlannedContacts)
    {
        auto it = newList.at(key).getPresentContact(contact.activationTime);
        newList.at(key).editContact(it, contact);
    }

    phaseList.setLists(newList);
}

bool CentroidalMPCBlock::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[CentroidalMPCBlock::initialize]";
    using BipedalLocomotion::log;
    using namespace std::chrono_literals;

    auto getParameter = [logPrefix](std::weak_ptr<const IParametersHandler> handler,
                                    const std::string& paramName,
                                    auto& param) -> bool {
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            log()->error("{} Invalid parameter handler.", logPrefix);
            return false;
        }

        if (!ptr->getParameter(paramName, param))
        {
            log()->error("{} Unable to find the parameter named {}.", logPrefix, paramName);
            return false;
        }

        return true;
    };

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    if (!m_controller.initialize(ptr->getGroup("CENTROIDAL_MPC")))
    {
        log()->error("{} Unable to initialize the MPC.", logPrefix);
        return false;
    }

    std::string modelPath
        = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName("model.urdf");
    BipedalLocomotion::log()->info("{} Model path: {}", logPrefix, modelPath);

    std::vector<std::string> jointsList;
    if (!getParameter(ptr->getGroup("MANN"), "joints_list", jointsList))
    {
        log()->error("{} Unable to get the parameter required by the MANN network.", logPrefix);
        return false;
    }

    if (!getParameter(ptr->getGroup("CENTROIDAL_MPC"), "sampling_time", m_dT))
    {
        log()->error("{} Unable to get the sampling_time.", logPrefix);
        return false;
    }

    iDynTree::ModelLoader ml;
    ml.loadReducedModelFromFile(modelPath, jointsList);
    if (!m_generator.setRobotModel(ml.model()))
    {
        log()->error("{} Unable to set the robot model for MANN.", logPrefix);
        return false;
    }

    m_robotMass = ml.model().getTotalMass();

    if (!m_generator.initialize(ptr->getGroup("MANN")))
    {
        log()->error("{} Unable to initialize the MANN trajectory generator class.", logPrefix);
        return false;
    }

    if (!m_generatorInputBuilder.initialize(ptr->getGroup("MANN")))
    {
        log()->error("{} Unable to initialize the MANN trajectory generator input builder class.",
                     logPrefix);
        return false;
    }

    // set the initial state of mann trajectory generator
    Eigen::VectorXd jointPositions(26);
    jointPositions << -0.10922017141063572, 0.05081325960010118, 0.06581966291990003,
        -0.0898053099824925, -0.09324922528169599, -0.05110058859172172, // left leg
        -0.11021232812838086, 0.054291515925228385, 0.0735575862560208, -0.09509332143185895,
        -0.09833823347493076, -0.05367281245082792, // right leg
        0.1531558711397399, -0.001030634273454133, 0.0006584764419034815, // torso
        -0.0016821925351926288, -0.004284529460797688, 0.030389771690123243, // head
        -0.040592118429752494, -0.1695472679986807, -0.20799422095574033,
        0.045397975984119654, // left arm
        -0.03946672931050908, -0.16795588539580256, -0.20911090583076936,
        0.0419854257806720; // right
                            // arm

    iDynTree::KinDynComputations kinDyn;
    kinDyn.loadRobotModel(ml.model());

    // get the frame associated to l_sole
    auto lSoleFrame = kinDyn.model().getFrameIndex("l_sole");
    if (lSoleFrame == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Unable to find the frame named l_sole.", logPrefix);
        return false;
    }
    auto linkBaseIndex = kinDyn.model().getFrameLink(lSoleFrame);
    auto linkFrameName = kinDyn.model().getLinkName(linkBaseIndex);

    auto l_foot_H_l_sole = kinDyn.getRelativeTransform(lSoleFrame, linkBaseIndex);

    // set kindybase
    kinDyn.setFloatingBase(linkFrameName);

    iDynTree::VectorDynSize dummyVel, jointPositionsiDyn;
    dummyVel.resize(jointPositions.size());
    jointPositionsiDyn.resize(jointPositions.size());
    iDynTree::toEigen(jointPositionsiDyn) = jointPositions;
    dummyVel.zero();

    iDynTree::Vector3 dummyGravity;
    dummyGravity.zero();
    kinDyn.setRobotState(l_foot_H_l_sole,
                         jointPositionsiDyn,
                         iDynTree::Twist::Zero(),
                         dummyVel,
                         dummyGravity);

    manif::SE3d leftFootPose = BipedalLocomotion::Conversions::toManifPose(
        kinDyn.getWorldTransform(ml.model().getFrameIndex("l_sole")));
    manif::SE3d rightFootPose = BipedalLocomotion::Conversions::toManifPose(
        kinDyn.getWorldTransform(ml.model().getFrameIndex("r_sole")));

    manif::SE3d basePose = BipedalLocomotion::Conversions::toManifPose(
        kinDyn.getWorldTransform(ml.model().getFrameIndex("root_link")));

    auto leftFootPoseNewPosition = leftFootPose.translation();
    leftFootPoseNewPosition[0] = -basePose.translation()[0] + leftFootPose.translation()[0];
    leftFootPoseNewPosition[1] = -basePose.translation()[1] + leftFootPose.translation()[1];
    leftFootPoseNewPosition[2] = 0;
    leftFootPose = manif::SE3d(leftFootPoseNewPosition, manif::SO3d::Identity());

    auto rightFootPoseNewPosition = rightFootPose.translation();
    rightFootPoseNewPosition[0] = -basePose.translation()[0] + rightFootPose.translation()[0];
    rightFootPoseNewPosition[1] = -basePose.translation()[1] + rightFootPose.translation()[1];
    rightFootPoseNewPosition[2] = 0;
    rightFootPose = manif::SE3d(rightFootPoseNewPosition, manif::SO3d::Identity());

    auto newBasePosition = basePose.translation();
    newBasePosition[0] = 0;
    newBasePosition[1] = 0;

    basePose = manif::SE3d(newBasePosition, basePose.quat());

    BipedalLocomotion::Contacts::EstimatedContact leftFoot, rightFoot;

    leftFoot.isActive = true;
    leftFoot.name = "left_foot";
    leftFoot.index = ml.model().getFrameIndex("l_sole");
    leftFoot.switchTime = 0s;
    leftFoot.pose = leftFootPose;

    rightFoot.isActive = true;
    rightFoot.name = "right_foot";
    rightFoot.index = ml.model().getFrameIndex("r_sole");
    rightFoot.switchTime = 0s;
    rightFoot.pose = rightFootPose;

    CoM0 = iDynTree::toEigen(kinDyn.getCenterOfMassPosition());
    m_generator.setInitialState(jointPositions, leftFoot, rightFoot, basePose, 0s);

    m_joypadPort.open("/centroidal-mpc/joystick:i");

    m_directionalInput.motionDirection.setZero();
    m_directionalInput.motionDirection[0] = 1;
    m_directionalInput.facingDirection[0] = 1;

    m_directionalInput.facingDirection.setZero();

    return true;
}

const CentroidalMPCBlock::Output& CentroidalMPCBlock::getOutput() const
{
    return m_output;
}

bool CentroidalMPCBlock::setInput(const Input& input)
{
    if (!input.isValid)
    {
        return true;
    }

    m_inputValid = input.isValid;

    // the angular momentum needs to be scaled by the robot mass
    Input scaledInput = input;
    scaledInput.angularMomentum = input.angularMomentum;

    return m_controller.setState(scaledInput.com,
                                 scaledInput.dcom,
                                 scaledInput.angularMomentum,
                                 scaledInput.totalExternalWrench);
}

bool CentroidalMPCBlock::advance()
{
    namespace blf = ::BipedalLocomotion;
    using BipedalLocomotion::log;

    m_output.isValid = false;

    constexpr auto logPrefix = "[CentroidalMPCBlock::advance]";

    if (!m_inputValid)
    {
        log()->warn("{} Input is not valid. Skipping.", logPrefix);
    }

    // MANN trajectory generator does not require any input. Indeed we need the output of the
    // trajectory generator to compute the first step of the whole body controller.
    yarp::sig::Vector* tmp = m_joypadPort.read(false);
    if (tmp != nullptr && tmp->size() == 4)
    {
        BipedalLocomotion::log()->warn("joypad {}", tmp->toString());

        m_directionalInput.motionDirection << tmp->operator()(0), tmp->operator()(1);
        m_directionalInput.facingDirection << tmp->operator()(2), tmp->operator()(3);
    }

    if (!m_generatorInputBuilder.setInput(m_directionalInput))
    {
        log()->error("{} Unable to set the input to MANN generator.", logPrefix);
        return false;
    }
    if (!m_generatorInputBuilder.advance())
    {
        log()->error("{} Unable to compute one step of the MANN generator builder.", logPrefix);
        return false;
    }

    BipedalLocomotion::ML::MANNTrajectoryGeneratorInput generatorInput;
    generatorInput.mergePointIndex = 0;
    if (m_inputValid && !m_isFirstRun)
    {
        generatorInput.mergePointIndex = 1;
    }
    generatorInput.desiredFutureBaseTrajectory
        = m_generatorInputBuilder.getOutput().desiredFutureBaseTrajectory;
    generatorInput.desiredFutureBaseVelocities
        = m_generatorInputBuilder.getOutput().desiredFutureBaseVelocities;
    generatorInput.desiredFutureFacingDirections
        = m_generatorInputBuilder.getOutput().desiredFutureFacingDirections;

    if (!m_generator.setInput(generatorInput))
    {
        log()->error("{} Unable to set the input to MANN generator.", logPrefix);
        return false;
    }
    if (!m_generator.advance())
    {
        log()->error("{} Unable to compute one step of the MANN generator.", logPrefix);
        return false;
    }

    // get the contact phase list from MANN generator
    const auto& MANNGeneratorOutput = m_generator.getOutput();
    m_output.contactPhaseList = MANNGeneratorOutput.phaseList;

    // for the next steps we need a valid input
    if (!m_inputValid)
    {
        return true;
    }

    // the feedback has been already set in setInput
    auto scaledAngularMomentum = MANNGeneratorOutput.angularMomentumTrajectory;
    for (auto& t : scaledAngularMomentum)
    {
        t = t / m_robotMass;
    }

    if (!m_controller.setReferenceTrajectory(MANNGeneratorOutput.comTrajectory,
                                             scaledAngularMomentum))
    {
        log()->error("{} Unable to set the reference trajectory of the MPC.", logPrefix);
        return false;
    }

    if (!m_controller.setContactPhaseList(m_output.contactPhaseList))
    {
        log()->error("{} Unable to set the contact list in the MPC.", logPrefix);
        return false;
    }

    if (!m_controller.advance())
    {
        log()->error("{} Unable to evaluate the output of the MPC.", logPrefix);
        return false;
    }

    // set the controller output
    m_output.controllerOutput = m_controller.getOutput();
    m_output.isValid = true;
    m_output.currentTime = m_absoluteTime;

    m_isFirstRun = false;

    m_absoluteTime += m_dT;

    return true;
}

bool CentroidalMPCBlock::isOutputValid() const
{
    // for the time beeing we assume that the output is always valid
    return true;
}
