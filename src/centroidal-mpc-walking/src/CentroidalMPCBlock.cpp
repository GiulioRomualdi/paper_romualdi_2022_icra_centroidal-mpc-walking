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
        -0.0898053099824925, -0.09324922528169599, -0.05110058859172172, -0.11021232812838086,
        0.054291515925228385, 0.0735575862560208, -0.09509332143185895, -0.09833823347493076,
        -0.05367281245082792, 0.1531558711397399, -0.001030634273454133, 0.0006584764419034815,
        -0.0016821925351926288, -0.004284529460797688, 0.030389771690123243, -0.040592118429752494,
        -0.1695472679986807, -0.20799422095574033, 0.045397975984119654, -0.03946672931050908,
        -0.16795588539580256, -0.20911090583076936, 0.0419854257806720;

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

    BipedalLocomotion::Contacts::EstimatedContact leftFoot, rightFoot;

    leftFoot.isActive = true;
    leftFoot.name = "left_foot";
    leftFoot.index = ml.model().getFrameIndex("l_sole");
    leftFoot.switchTime = 0s;
    leftFoot.pose = manif::SE3d(Eigen::Vector3d{0, 0.0, 0}, manif::SO3d::Identity());

    rightFoot.isActive = true;
    rightFoot.name = "right_foot";
    rightFoot.index = ml.model().getFrameIndex("r_sole");
    rightFoot.switchTime = 0s;
    rightFoot.pose
        = BipedalLocomotion::Conversions::toManifPose(kinDyn.getWorldTransform(rightFoot.index));

    const manif::SE3d basePose
        = BipedalLocomotion::Conversions::toManifPose(kinDyn.getWorldTransform("root_link"));

    CoM0 = iDynTree::toEigen(kinDyn.getCenterOfMassPosition());
    BipedalLocomotion::log()->info("{} CoM0: {}", logPrefix, CoM0.transpose());

    m_generator.setInitialState(jointPositions, leftFoot, rightFoot, basePose, 0s);

    m_joypadPort.open("/centroidal-mpc/joystick:i");


    BipedalLocomotion::Contacts::ContactListMap contactListMap;
    BipedalLocomotion::Contacts::PlannedContact tmp1;
    tmp1.name = "left_foot";
    tmp1.pose = leftFoot.pose;
    tmp1.activationTime = std::chrono::nanoseconds::zero();
    tmp1.deactivationTime = std::chrono::nanoseconds::max();
    tmp1.index = leftFoot.index;
    contactListMap["left_foot"].addContact(tmp1);
    tmp1.name = "right_foot";
    tmp1.pose = rightFoot.pose;
    tmp1.activationTime = std::chrono::nanoseconds::zero();
    tmp1.deactivationTime = std::chrono::nanoseconds::max();
    tmp1.index = rightFoot.index;
    contactListMap["right_foot"].addContact(tmp1);
    m_output.contactPhaseList.setLists(contactListMap);


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
    scaledInput.angularMomentum = input.angularMomentum / m_robotMass;

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

    // MANN trajectory generator does not require any input. Indeed we need the output of the
    // trajectory generator to compute the first step of the whole body controller.
    yarp::sig::Vector* tmp = m_joypadPort.read(false);
    if (tmp != nullptr && tmp->size() == 4)
    {
        m_directionalInput.motionDirection << tmp->operator()(0), tmp->operator()(1);
        m_directionalInput.facingDirection << tmp->operator()(2), tmp->operator()(3);
    }

    m_generatorInputBuilder.setInput(m_directionalInput);
    m_generatorInputBuilder.advance();

    BipedalLocomotion::ML::MANNTrajectoryGeneratorInput generatorInput;
    generatorInput.mergePointIndex = 1;
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
    // m_output.contactPhaseList = MANNGeneratorOutput.phaseList;

    // for (const auto& lists : m_output.contactPhaseList.lists())
    // {
    //     for (const auto& contact : lists.second)
    //     {
    //         BipedalLocomotion::log()->info("{}, position {}",
    //                                        contact.name,
    //                                        contact.pose.translation().transpose());
    //     }

    //     // print the position in each phase
    // }

    // for the next steps we need a valid input
    if (!m_inputValid)
    {
        return true;
    }

    // if the input is valid, we can assume that this is the first run of this advanceable
    if (m_isFirstRun)
    {
        generatorInput.mergePointIndex = 0;
        m_isFirstRun = false;
    }

    // the feedback has been already set in setInput
    auto scaledAngularMomentum = MANNGeneratorOutput.angularMomentumTrajectory;
    for (auto& t : scaledAngularMomentum)
    {
        t = t / m_robotMass * 0;
    }

    auto comTemp = MANNGeneratorOutput.comTrajectory;
    for (auto& t : comTemp)
    {
        t = CoM0;
    }



    if (!m_controller.setReferenceTrajectory(comTemp,
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

    return true;
}

bool CentroidalMPCBlock::isOutputValid() const
{
    // for the time beeing we assume that the output is always valid
    return true;
}
