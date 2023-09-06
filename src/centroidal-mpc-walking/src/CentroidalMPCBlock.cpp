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

double comz0mann = 0;

BipedalLocomotion::Contacts::ContactPhaseList
updateContactPhaseList(const std::chrono::nanoseconds& currentTime,
                       const BipedalLocomotion::Contacts::ContactPhaseList& mannPhaseList,
                       const BipedalLocomotion::Contacts::ContactPhaseList& mpcPhaseList)
{
    BipedalLocomotion::Contacts::ContactPhaseList contactPhaseList;
    BipedalLocomotion::Contacts::ContactListMap contactListMap;

    // auto printContactList = [](const BipedalLocomotion::Contacts::ContactList& contactList) {
    //     for (const auto& contact : contactList)
    //     {
    //         BipedalLocomotion::log()->info("name: {}, activation time: {}, deactivation time: {},
    //         "
    //                                        "pose {}",
    //                                        contact.name,
    //                                        std::chrono::duration_cast<std::chrono::milliseconds>(
    //                                            contact.activationTime),
    //                                        std::chrono::duration_cast<std::chrono::milliseconds>(
    //                                            contact.deactivationTime),
    //                                        contact.pose.coeffs().transpose());
    //     }
    // };

    // BipedalLocomotion::log()->warn("MANN phase list. time {}",
    //                                std::chrono::duration_cast<std::chrono::milliseconds>(
    //                                    currentTime));
    // for (const auto& [name, contactList] : mannPhaseList.lists())
    // {
    //     printContactList(contactList);
    // }

    // BipedalLocomotion::log()->warn("MPC phase list. time {}",
    //                                std::chrono::duration_cast<std::chrono::milliseconds>(
    //                                    currentTime));
    // for (const auto& [name, contactList] : mpcPhaseList.lists())
    // {
    //     printContactList(contactList);
    // }

    for (const auto& [name, contactList] : mannPhaseList.lists())
    {
        // get the index of the current contact in the mann phase list
        auto mannPresentContact = contactList.getPresentContact(currentTime);
        for (auto it = std::next(mannPresentContact); it != contactList.cend(); ++it)
        {
            contactListMap[name].addContact(*it);
        }

        // get the index of the current contact in the mpc phase list
        const auto& mpcList = mpcPhaseList.lists().at(name);
        auto mpcPresentContact = mpcList.getPresentContact(currentTime);

        for (auto it = mpcList.begin(); it != mpcPresentContact; ++it)
        {
            contactListMap[name].addContact(*it);
        }

        // for the current we take the time of mann and the pose of the MPC
        // this is required since the deactivation time may changed
        if (mpcPresentContact != mpcList.cend())
        {
            auto contact = *mpcPresentContact;
            contact.activationTime = (*mannPresentContact).activationTime;
            contact.deactivationTime = (*mannPresentContact).deactivationTime;
            contactListMap[name].addContact(contact);
        }
    }

    contactPhaseList.setLists(contactListMap);

    // BipedalLocomotion::log()->warn("Updated phase list. time {}",
    //                                std::chrono::duration_cast<std::chrono::milliseconds>(
    //                                    currentTime));
    // for (const auto& [name, contactList] : contactPhaseList.lists())
    // {
    //     printContactList(contactList);
    // }

    return contactPhaseList;
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
    // jointPositions << -0.09935663695085467, 0.025701155059457442, 0.06534184984184833,
    //     -0.08227673039926048, -0.07768761174843312, -0.024315843233342373, -0.11525704616838933,
    //     0.02478877256772321, 0.05046307803997472, -0.08762109045807909, -0.10031852516392248,
    //     -0.025565627238000203, 0.16487824979762272, 0.0030093422556509227, -0.010453910438339882,
    //     -0.003975671198541142, -0.004306264052024712, 0.02768477434088547, -0.05625823587699814,
    //     0.09999973925655095, -0.21065705479250768, 0.015695286263621544, -0.0647162232623409,
    //     0.10000197803987905, -0.21257567597757412, -0.003282499516478188;

    // jointPositions << -0.1083405028191962, 0.01464940967892403, 0.07062119719508,
    //     -0.10289331748217925, -0.1012132490914801, -0.0173038389436336, -0.11275167823891682,
    //     0.03667259663921431, 0.05493886524271771, -0.09055099895926208, -0.09278315719945165,
    //     -0.03478220710451874, 0.15267533871086064, 0.003451022449904091, -0.017515644863920408,
    //     -0.0020115451190194367, 0.0014601683787918676, 0.011565481804382874, -0.04759409393252947,
    //     0.10000590836596246, -0.19197253684934223, 0.05394598490420893, -0.04704542789138096,
    //     0.09997832901610988, -0.20382346126939987, -0.008043248437166084;

    // jointPositions << -0.1046405016025761, 0.017001975000216298, 0.05860553979299023,
    //     -0.10861521376740862, -0.10469989126931348, -0.017162796031807485, -0.10555041888916736,
    //     0.027253879470372736, 0.05886380122392785, -0.09220682518818205, -0.08923763315828329,
    //     -0.026626646132658194, 0.16448252889206036, 0.0008976397176696274, -0.004168695848377414,
    //     -0.003024068405504912, -0.003051667477044264, 0.02620836750592551, -0.05091593006812735,
    //     0.09999862602765555, -0.20693304218868638, 0.0284961484128734, -0.0524330311572014,
    //     0.09999598464167686, -0.2124976706557769, 0.00869636757561082;

    // jointPositions << -0.10922017141063572, 0.05081325960010118, 0.06581966291990003,
    //     -0.0898053099824925, -0.09324922528169599, -0.05110058859172172, // left leg
    //     -0.11021232812838086, 0.054291515925228385, 0.0735575862560208, -0.09509332143185895,
    //     -0.09833823347493076, -0.05367281245082792, // right leg
    //     0.1531558711397399, -0.001030634273454133, 0.0006584764419034815, // torso
    //     -0.0016821925351926288, -0.004284529460797688, 0.030389771690123243, // head
    //     -0.040592118429752494, -0.1695472679986807, -0.20799422095574033,
    //     0.045397975984119654, // left arm
    //     -0.03946672931050908, -0.16795588539580256, -0.20911090583076936,
    //     0.0419854257806720; // right
    //                         // arm

    jointPositions <<
        -0.10704676769729708,
        0.012925973405522994, 0.071895284039525, -0.0996134196581665, -0.09861216989767335,
        -0.014251179274100766, -0.11118618014578055, 0.035651653099700815, 0.057847116296006404,
        -0.09680096080425257, -0.0990365301429598, -0.033878120609673684, 0.15568026047513014,
        0.004317234521230275, -0.02030554515076784, -0.0021306312005093236, 0.0029481809738139265,
        0.009979402224284993, -0.05175542996772139, 0.10000318973045413, -0.19553431552870545,
        0.04983171308345671, -0.053410414924022265, 0.09998072151822349, -0.20769041030905094,
        -0.006947678797282503;

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

    auto l_sole_H_l_foot = kinDyn.getRelativeTransform(lSoleFrame, linkBaseIndex);

    // set kindybase
    kinDyn.setFloatingBase(linkFrameName);

    iDynTree::VectorDynSize dummyVel, jointPositionsiDyn;
    dummyVel.resize(jointPositions.size());
    jointPositionsiDyn.resize(jointPositions.size());
    iDynTree::toEigen(jointPositionsiDyn) = jointPositions;
    dummyVel.zero();

    iDynTree::Vector3 dummyGravity;
    dummyGravity.zero();
    kinDyn.setRobotState(l_sole_H_l_foot,
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

    iDynTree::Transform leftFootTransformIdyn;
    leftFootTransformIdyn.setPosition(iDynTree::Position(leftFoot.pose.translation()));
    leftFootTransformIdyn.setRotation(iDynTree::Rotation(leftFoot.pose.rotation()));
    kinDyn.setRobotState(leftFootTransformIdyn * l_sole_H_l_foot,
                         jointPositionsiDyn,
                         iDynTree::Twist::Zero(),
                         dummyVel,
                         dummyGravity);

    m_generator.setInitialState(jointPositions,
                                leftFoot,
                                rightFoot,
                                basePose,
                                iDynTree::toEigen(kinDyn.getCenterOfMassPosition()),
                                0s);

    m_joypadPort.open("/centroidal-mpc/joystick:i");

    m_directionalInput.motionDirection.setZero();
    m_directionalInput.facingDirection.setZero();

    BipedalLocomotion::log()->info("{} Right foot pose {}.",
                                   logPrefix,
                                   rightFoot.pose.coeffs().transpose());
    BipedalLocomotion::log()->info("{} Left foot pose {}.",
                                   logPrefix,
                                   leftFoot.pose.coeffs().transpose());
    BipedalLocomotion::log()->info("{} Initialization completed.", logPrefix);

    return true;
}

const CentroidalMPCBlock::Output& CentroidalMPCBlock::getOutput() const
{
    return m_output;
}

bool CentroidalMPCBlock::setInput(const Input& input)
{
    m_inputValid = input.isValid;

    if (!input.isValid)
    {
        return true;
    }

    // Here we assume that the total external wrench and the angular momentum are scaled by the
    // robot mass
    return m_controller.setState(input.com,
                                 input.dcom,
                                 input.angularMomentum,
                                 input.totalExternalWrench);
}

bool CentroidalMPCBlock::advance()
{
    namespace blf = ::BipedalLocomotion;
    using BipedalLocomotion::log;

    auto tic = BipedalLocomotion::clock().now();

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

    if (m_isFirstRun)
    {
        m_output.contactPhaseList = MANNGeneratorOutput.phaseList;
        comz0mann = MANNGeneratorOutput.comTrajectory.front()[2];
    }

    m_output.regularizedJoints = MANNGeneratorOutput.jointPositions.front();
    // for the next steps we need a valid input
    if (!m_inputValid)
    {
        return true;
    }

    // the feedback has been already set in setInput
    auto scaledAngularMomentum = MANNGeneratorOutput.angularMomentumTrajectory;
    for (auto& t : scaledAngularMomentum)
    {
        t = t / m_robotMass / 5;
    }

    auto reducedHeightCoM = MANNGeneratorOutput.comTrajectory;
    for (auto& t : reducedHeightCoM)
    {
        t[2] = 0.7 + t[2] - comz0mann;
    }

    m_output.comMANN = reducedHeightCoM.front();
    m_output.angularMomentumMann = scaledAngularMomentum.front();

    auto toc = BipedalLocomotion::clock().now();
    m_output.adherentComputationTime = toc - tic;
    tic = toc;

    if (!m_controller.setReferenceTrajectory(reducedHeightCoM, scaledAngularMomentum))
    {
        log()->error("{} Unable to set the reference trajectory of the MPC.", logPrefix);
        return false;
    }

    BipedalLocomotion::Contacts::ContactPhaseList contactPhaseList;
    if (!m_isFirstRun)
    {
        contactPhaseList = updateContactPhaseList(m_absoluteTime,
                                                  MANNGeneratorOutput.phaseList,
                                                  m_controller.getOutput().contactPhaseList);
    } else
    {
        contactPhaseList = MANNGeneratorOutput.phaseList;
    }

    if (!m_controller.setContactPhaseList(contactPhaseList))
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
    m_output.mpcComputationTime = BipedalLocomotion::clock().now() - tic;
    m_output.contactPhaseList = m_controller.getOutput().contactPhaseList;
    m_output.mannContactPhaseList = MANNGeneratorOutput.phaseList;

    m_isFirstRun = false;

    m_absoluteTime += m_dT;

    return true;
}

bool CentroidalMPCBlock::isOutputValid() const
{
    // for the time beeing we assume that the output is always valid
    return true;
}
