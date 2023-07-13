/**
 * @file WholeBodyQPBlock.cpp
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>

#include <Eigen/Dense>

#include <manif/SE3.h>

#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/Contacts/ContactListJsonParser.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <CentroidalMPCWalking/WholeBodyQPBlock.h>

using namespace CentroidalMPCWalking;
using namespace BipedalLocomotion::ParametersHandler;
using namespace std::chrono_literals;

bool WholeBodyQPBlock::createKinDyn(const std::string& modelPath,
                                    const std::vector<std::string>& jointLists)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::createKinDyn]";

    iDynTree::ModelLoader ml;
    if (!ml.loadReducedModelFromFile(modelPath, jointLists))
    {
        BipedalLocomotion::log()->error("{} Unable to load the reduced model located in: {}.",
                                        errorPrefix,
                                        modelPath);
        return false;
    }

    m_kinDynWithDesired = std::make_shared<iDynTree::KinDynComputations>();
    m_kinDynWithMeasured = std::make_shared<iDynTree::KinDynComputations>();

    if (!m_kinDynWithDesired->loadRobotModel(ml.model())
        || !m_kinDynWithMeasured->loadRobotModel(ml.model()))
    {
        BipedalLocomotion::log()->error("{} Unable to load a KinDynComputation object",
                                        errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateLeggedOdometry(std::shared_ptr<const IParametersHandler> handler,
                                                 const std::string& modelPath,
                                                 const std::vector<std::string>& jointLists)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::instantiateLeggedOdometry]";

    iDynTree::ModelLoader ml;
    if (!ml.loadReducedModelFromFile(modelPath, jointLists))
    {
        BipedalLocomotion::log()->error("{} Unable to load the reduced model located in: {}.",
                                        logPrefix,
                                        modelPath);
        return false;
    }

    auto tmpKinDyn = std::make_shared<iDynTree::KinDynComputations>();
    if (!tmpKinDyn->loadRobotModel(ml.model()))
    {
        BipedalLocomotion::log()->error("{} Unable to load a KinDynComputation object", logPrefix);
        return false;
    }

    if (!m_floatingBaseEstimator.initialize(handler->getGroup("FLOATING_BASE_ESTIMATOR"),
                                            tmpKinDyn))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the legged odometry.", logPrefix);
        return false;
    }

    if (!m_fixedFootDetector.initialize(handler->getGroup("FIXED_FOOT_DETECTOR")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the fixed foot detector.",
                                        logPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateIK(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::instantiateIK]";

    auto getTask = [this, logPrefix](const std::string& taskName, auto task) -> bool {
        auto ptr = m_IKandTasks.ikProblem.ik->getTask(taskName).lock();
        if (ptr == nullptr)
        {
            BipedalLocomotion::log()->error("{} Unable to get the task named {}.",
                                            logPrefix,
                                            taskName);
            return false;
        }

        task = std::dynamic_pointer_cast<typename decltype(task)::element_type>(ptr);
        if (task == nullptr)
        {
            BipedalLocomotion::log()->error("{} Unable to cast the task named {} to the expected "
                                            "type.",
                                            logPrefix,
                                            taskName);
            return false;
        }
        return true;
    };

    m_IKandTasks.ikProblem
        = BipedalLocomotion::IK::QPInverseKinematics::build(handler, m_kinDynWithDesired);
    if (!m_IKandTasks.ikProblem.isValid())
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the IK.", logPrefix);
        return false;
    }

    return getTask("LEFT_FOOT", m_IKandTasks.leftFootTask)
           && getTask("RIGHT_FOOT", m_IKandTasks.rightFootTask)
           && getTask("COM", m_IKandTasks.comTask) //
           && getTask("CHEST", m_IKandTasks.chestTask)
           && getTask("JOINT_REGULARIZATION", m_IKandTasks.regularizationTask);
}

bool WholeBodyQPBlock::initializeRobotControl(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::initializeRobotControl]";

    if (!m_robotControl.initialize(handler->getGroup("ROBOT_CONTROL")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the robot control.", errorPrefix);
        return false;
    }
    if (!m_robotControl.setDriver(m_controlBoard.poly))
    {
        BipedalLocomotion::log()->error("{} Unable to set the polydriver.", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateSensorBridge(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::instantiateSensorBridge]";

    if (!m_sensorBridge.initialize(handler->getGroup("SENSOR_BRIDGE")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the sensor bridge.", errorPrefix);
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(m_controlBoard.poly.get(), m_controlBoard.key.c_str());
    for (auto& [key, driver] : m_leftFootContacWrenches)
    {
        list.push(driver.polyDriverDescriptor.poly.get(), key.c_str());
    }

    for (auto& [key, driver] : m_rightFootContacWrenches)
    {
        list.push(driver.polyDriverDescriptor.poly.get(), key.c_str());
    }

    for (auto& [key, driver] : m_externalContactWrenches)
    {
        list.push(driver.polyDriverDescriptor.poly.get(), key.c_str());
    }

    if (!m_sensorBridge.setDriversList(list))
    {
        BipedalLocomotion::log()->error("{} Unable to set the driver list.", errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::instantiateSwingFootPlanner(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::instantiateSwingFootPlanner]";
    auto ptr = handler->getGroup("SWING_FOOT_PLANNER").lock();

    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Unable to find the group 'SWING_FOOT_PLANNER'.",
                                        errorPrefix);
        return false;
    }

    auto tmp = ptr->clone();
    tmp->setParameter("sampling_time", m_dT);
    if (!m_leftFootPlanner.initialize(tmp))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the left foot planner.",
                                        errorPrefix);
        return false;
    }

    if (!m_rightFootPlanner.initialize(tmp))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the right foot planner.",
                                        errorPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::updateFloatingBase()
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::updateFloatingBase]";

    // set the contact phase list in the fixed foot detector
    // this function assumes that the contact phase list is not empty
    m_fixedFootDetector.setContactPhaseList(m_input.contactPhaseList);

    if (!m_floatingBaseEstimator.setKinematics(m_currentJointPos, m_currentJointVel))
    {
        BipedalLocomotion::log()->error("{} Unable to set the kinematics in the base estimator.",
                                        logPrefix);
        return false;
    }

    if (!m_fixedFootDetector.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to update the fixed foot detector.", logPrefix);
        return false;
    }

    for (const auto& [key, foot] : m_fixedFootDetector.getOutput())
    {
        // TODO Please change the signature of setContactStatus
        const auto frameName = m_kinDynWithDesired->model().getFrameName(foot.index);
        if (!m_floatingBaseEstimator.setContactStatus(frameName,
                                                      foot.isActive,
                                                      foot.switchTime,
                                                      foot.lastUpdateTime))
        {
            BipedalLocomotion::log()->error("{} Unable to set the contact status in the base "
                                            "estimator.",
                                            logPrefix);
            return false;
        }
    }

    if (!m_floatingBaseEstimator.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to update the floating base estimator.",
                                        logPrefix);
        return false;
    }

    if (!m_floatingBaseEstimator
             .changeFixedFrame(m_fixedFootDetector.getFixedFoot().index,
                               m_fixedFootDetector.getFixedFoot().pose.quat(),
                               m_fixedFootDetector.getFixedFoot().pose.translation()))
    {
        BipedalLocomotion::log()->error("{} Unable to change the fixed frame in the base "
                                        "estimator.",
                                        logPrefix);
        return false;
    }

    return true;
}

bool WholeBodyQPBlock::createPolydriver(std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::createPolydriver]";

    std::string name;
    if (!handler->getParameter("name", name))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", errorPrefix);
        return false;
    }

    auto ptr = handler->getGroup("ROBOT_INTERFACE").lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Robot interface options is empty.", errorPrefix);
        return false;
    }
    ptr->setParameter("local_prefix", name);
    m_controlBoard = BipedalLocomotion::RobotInterface::constructRemoteControlBoardRemapper(ptr);
    if (!m_controlBoard.isValid())
    {
        BipedalLocomotion::log()->error("{} the robot polydriver has not been constructed.",
                                        errorPrefix);
        return false;
    }

    return true;
}

BipedalLocomotion::RobotInterface::PolyDriverDescriptor
WholeBodyQPBlock::createContactWrenchDriver(std::weak_ptr<const IParametersHandler> handler,
                                            const std::string& local)
{
    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        return BipedalLocomotion::RobotInterface::PolyDriverDescriptor();
    }
    auto tmp = ptr->clone();
    tmp->setParameter("local_prefix", local);
    return BipedalLocomotion::RobotInterface::constructGenericSensorClient(tmp);
}

bool WholeBodyQPBlock::createAllContactWrenchesDriver(
    std::shared_ptr<const IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::createGenericSensorClient]";

    std::string localPrefix;
    if (!handler->getParameter("name", localPrefix))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", errorPrefix);
        return false;
    }

    auto ptr = handler->getGroup("CONTACT_WRENCHES").lock();
    if (ptr == nullptr)
    {
        BipedalLocomotion::log()->error("{} Robot interface options is empty.", errorPrefix);
        return false;
    }

    std::vector<std::string> contactWrenchClients;
    if (!ptr->getParameter("left_contact_wrenches_group", contactWrenchClients))
    {
        BipedalLocomotion::log()->error("{} Unable to find the left contact wrench group.",
                                        errorPrefix);
        return false;
    }

    for (const auto& wrench : contactWrenchClients)
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor descriptor
            = this->createContactWrenchDriver(ptr->getGroup(wrench), localPrefix);

        if (!descriptor.isValid())
        {
            BipedalLocomotion::log()->error("{} The generic sensor client for the wrench named {} "
                                            "cannot be opened.",
                                            errorPrefix,
                                            wrench);
            return false;
        }

        m_leftFootContacWrenches[descriptor.key].polyDriverDescriptor = descriptor;
    }

    if (!ptr->getParameter("right_contact_wrenches_group", contactWrenchClients))
    {
        BipedalLocomotion::log()->error("{} Unable to find the right contact wrench group.",
                                        errorPrefix);
        return false;
    }

    for (const auto& wrench : contactWrenchClients)
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor descriptor
            = this->createContactWrenchDriver(ptr->getGroup(wrench), localPrefix);

        if (!descriptor.isValid())
        {
            BipedalLocomotion::log()->error("{} The generic sensor client for the wrench named {} "
                                            "cannot be opened.",
                                            errorPrefix,
                                            wrench);
            return false;
        }

        m_rightFootContacWrenches[descriptor.key].polyDriverDescriptor = descriptor;
    }

    if (!ptr->getParameter("external_contact_wrenches_group", contactWrenchClients))
    {
        BipedalLocomotion::log()->error("{} Unable to find the right contact wrench group.",
                                        errorPrefix);
        return false;
    }

    for (const auto& wrench : contactWrenchClients)
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor descriptor
            = this->createContactWrenchDriver(ptr->getGroup(wrench), localPrefix);

        if (!descriptor.isValid())
        {
            BipedalLocomotion::log()->error("{} The generic sensor client for the wrench named {} "
                                            "cannot be opened.",
                                            errorPrefix,
                                            wrench);
            return false;
        }

        m_externalContactWrenches[descriptor.key].polyDriverDescriptor = descriptor;
    }

    return true;
}

bool WholeBodyQPBlock::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[WholeBodyQPBlock::initialize]";

    auto parametersHandler = handler.lock();

    BipedalLocomotion::log()->info("{} Create the polydriver.", logPrefix);
    if (!this->createPolydriver(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to create the polydriver.", logPrefix);
        return false;
    }
    if (!this->createAllContactWrenchesDriver(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to create the contact wrench drivers.",
                                        logPrefix);
        return false;
    }

    // This sleep is required to let the robot interface to be ready
    BipedalLocomotion::clock().sleepFor(1s);

    BipedalLocomotion::log()->info("{} Create the robot control helper.", logPrefix);
    if (!this->initializeRobotControl(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the robotControl interface.",
                                        logPrefix);
        return false;
    }

    BipedalLocomotion::log()->info("{} Create the sensor bridge.", logPrefix);
    if (!this->instantiateSensorBridge(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the sensor bridge.", logPrefix);
        return false;
    }

    std::string name;
    if (!parametersHandler->getParameter("name", name))
    {
        BipedalLocomotion::log()->error("{} Unable to find the name.", logPrefix);
        return false;
    }

    if (!this->createKinDyn(yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(
                                "model.urdf"),
                            m_robotControl.getJointList()))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the kinDyn.", logPrefix);
        return false;
    }

    m_kinDynWithDesired->setFloatingBase("root_link");
    m_kinDynWithMeasured->setFloatingBase("root_link");

    if (!this->instantiateIK(parametersHandler->getGroup("IK").lock()))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the IK.", logPrefix);
        return false;
    }

    if (!this->instantiateLeggedOdometry(parametersHandler,
                                         yarp::os::ResourceFinder::getResourceFinderSingleton()
                                             .findFileByName("model.urdf"),
                                         m_robotControl.getJointList()))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the legged odometry.", logPrefix);
        return false;
    }

    if (!m_CoMZMPController.initialize(parametersHandler->getGroup("COM_ZMP_CONTROLLER")))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the CoM-ZMP Controller.",
                                        logPrefix);
        return false;
    }

    if (!this->instantiateSwingFootPlanner(parametersHandler))
    {
        BipedalLocomotion::log()->error("{} Unable to initialize the swing foot planner.",
                                        logPrefix);
        return false;
    }

    // resize the vectors
    const std::size_t numberOfJoints = m_robotControl.getJointList().size();
    m_currentJointPos.resize(numberOfJoints);
    m_currentJointVel.resize(numberOfJoints);
    m_currentJointVel.setZero();
    m_desJointPos.resize(numberOfJoints);
    m_desJointVel.resize(numberOfJoints);
    m_desJointVel.setZero();

    if (!m_sensorBridge.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to get the robot state.", logPrefix);
        return false;
    }

    // initialize the joint positions
    if (!m_sensorBridge.getJointPositions(m_currentJointPos)
        || !m_sensorBridge.getJointPositions(m_desJointPos))
    {
        BipedalLocomotion::log()->error("{} Unable to get the joint positions.", logPrefix);
        return false;
    }

    // instantiate the system integrators
    using namespace BipedalLocomotion::ContinuousDynamicalSystem;
    m_floatingBaseSystem.dynamics = std::make_shared<FloatingBaseSystemKinematics>();
    m_floatingBaseSystem.integrator
        = std::make_shared<ForwardEuler<FloatingBaseSystemKinematics>>();
    m_floatingBaseSystem.integrator->setIntegrationStep(m_dT);
    m_floatingBaseSystem.integrator->setDynamicalSystem(m_floatingBaseSystem.dynamics);

    m_centroidalSystem.dynamics = std::make_shared<CentroidalDynamics>();
    m_centroidalSystem.integrator = std::make_shared<ForwardEuler<CentroidalDynamics>>();
    m_centroidalSystem.integrator->setIntegrationStep(m_dT);
    m_centroidalSystem.integrator->setDynamicalSystem(m_centroidalSystem.dynamics);

    m_comSystem.dynamics = std::make_shared<LinearTimeInvariantSystem>();
    m_comSystem.dynamics->setSystemMatrices(Eigen::Matrix2d::Zero(), Eigen::Matrix2d::Identity());
    m_comSystem.integrator = std::make_shared<ForwardEuler<LinearTimeInvariantSystem>>();
    m_comSystem.integrator->setIntegrationStep(m_dT);
    m_comSystem.integrator->setDynamicalSystem(m_comSystem.dynamics);

    BipedalLocomotion::log()->info("{} The WholeBodyQPBlock has been configured.", logPrefix);

    return false;
}

const WholeBodyQPBlock::Output& WholeBodyQPBlock::getOutput() const
{
    return m_output;
}

bool WholeBodyQPBlock::setInput(const Input& input)
{
    m_input = input;
    return true;
}

bool WholeBodyQPBlock::evaluateZMP(Eigen::Ref<Eigen::Vector2d> zmp)
{
    using namespace BipedalLocomotion;
    Eigen::Vector3d zmpRight, zmpLeft;
    zmpLeft.setZero();
    zmpRight.setZero();
    double totalZ = 0;

    Math::Wrenchd leftWrench = Math::Wrenchd::Zero();
    for (const auto& [key, value] : m_leftFootContacWrenches)
    {
        leftWrench += value.wrench;
    }
    Math::Wrenchd rightWrench = Math::Wrenchd::Zero();
    for (const auto& [key, value] : m_rightFootContacWrenches)
    {
        rightWrench += value.wrench;
    }

    double zmpLeftDefined = 0.0, zmpRightDefined = 0.0;
    if (rightWrench.force()(2) < 0.001)
        zmpRightDefined = 0.0;
    else
    {
        zmpRight(0) = -rightWrench.torque()(1) / rightWrench.force()(2);
        zmpRight(1) = rightWrench.torque()(0) / rightWrench.force()(2);
        zmpRight(2) = 0.0;
        zmpRightDefined = 1.0;
        totalZ += rightWrench.force()(2);
    }

    if (leftWrench.force()(2) < 0.001)
        zmpLeftDefined = 0.0;
    else
    {
        zmpLeft(0) = -leftWrench.torque()(1) / leftWrench.force()(2);
        zmpLeft(1) = leftWrench.torque()(0) / leftWrench.force()(2);
        zmpLeft(2) = 0.0;
        zmpLeftDefined = 1.0;
        totalZ += leftWrench.force()(2);
    }

    if (totalZ < 0.1)
    {
        BipedalLocomotion::log()->error("[WholeBodyQPBlock::evaluateZMP] The total z-component of "
                                        "contact wrenches is too low.");
        return false;
    }

    manif::SE3d I_H_lf
        = BipedalLocomotion::Conversions::toManifPose(m_kinDynWithMeasured->getWorldTransform("l_"
                                                                                              "sol"
                                                                                              "e"));
    manif::SE3d I_H_rf
        = BipedalLocomotion::Conversions::toManifPose(m_kinDynWithMeasured->getWorldTransform("r_"
                                                                                              "sol"
                                                                                              "e"));

    zmpLeft = I_H_lf.act(zmpLeft);
    zmpRight = I_H_rf.act(zmpRight);

    // the global zmp is given by a weighted average
    zmp = ((leftWrench.force()(2) * zmpLeftDefined) / totalZ) * zmpLeft.head<2>()
          + ((rightWrench.force()(2) * zmpRightDefined) / totalZ) * zmpRight.head<2>();

    return true;
}

Eigen::Vector2d WholeBodyQPBlock::computeDesiredZMP(
    const std::map<std::string, BipedalLocomotion::Contacts::DiscreteGeometryContact>& contacts)
{
    Eigen::Vector2d zmp = Eigen::Vector2d::Zero();
    double totalZ = 0;
    Eigen::Vector3d localZMP;

    for (const auto& [key, contact] : contacts)
    {
        BipedalLocomotion::Math::Wrenchd totalWrench = BipedalLocomotion::Math::Wrenchd::Zero();
        for (const auto& corner : contact.corners)
        {
            totalWrench.force() = corner.force;
            totalWrench.torque() += corner.position.cross(contact.pose.asSO3().act(corner.force));
        }
        if (totalWrench.force()(2) > 0.001)
        {
            totalZ += totalWrench.force()(2);
            localZMP(0) = -totalWrench.torque()(1) / totalWrench.force()(2);
            localZMP(1) = totalWrench.torque()(0) / totalWrench.force()(2);
            localZMP(2) = 0.0;

            // the wrench is already expressed in mixed we have just to translate it
            // if left
            if (key == "left_foot")
            {
                manif::SE3d temptransofrm = BipedalLocomotion::Conversions::toManifPose(
                    m_kinDynWithDesired->getWorldTransform("l_sole"));
                zmp += totalWrench.force()(2) * temptransofrm.act(localZMP).head<2>();
            }
            // if left
            else if (key == "right_foot")
            {
                manif::SE3d temptransofrm = BipedalLocomotion::Conversions::toManifPose(
                    m_kinDynWithDesired->getWorldTransform("r_sole"));
                zmp += totalWrench.force()(2) * temptransofrm.act(localZMP).head<2>();
            } else
            {
                BipedalLocomotion::log()->error("Problem in evaluated the desired zmp");
            }
        }
    }

    zmp = zmp / totalZ;

    return zmp;
}

bool WholeBodyQPBlock::advance()
{
    constexpr auto errorPrefix = "[WholeBodyQPBlock::advance]";

    bool shouldAdvance = false;
    m_output.isValid = false;

    // check if the block should advance
    // in this case we need the contact phase list in order to compute the base position and
    // evaluate the robot state
    if (m_input.contactPhaseList.size() == 0)
    {
        return true;
    }

    // from now one we assume that the contact phase list contains a set of contacts

    Eigen::Vector2d desiredZMP = Eigen::Vector2d::Zero();
    Eigen::Vector2d measuredZMP = Eigen::Vector2d::Zero();

    // get the Feedback from the robot
    if (!m_sensorBridge.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to get the robot state.", errorPrefix);
        return false;
    }

    if (!m_sensorBridge.getJointPositions(m_currentJointPos)
        || !m_sensorBridge.getJointVelocities(m_currentJointVel))
    {
        BipedalLocomotion::log()->error("{} Unable to get the joint positions and velocities.",
                                        errorPrefix);
        return false;
    }

    // get the cartesian wrenches associated to the left foot
    for (auto& [key, value] : m_leftFootContacWrenches)
    {
        if (!m_sensorBridge.getCartesianWrench(key, value.wrench))
        {
            BipedalLocomotion::log()->error("{} Unable to get the left wrench named {}.",
                                            errorPrefix,
                                            key);
            return false;
        }
    }

    // get the cartesian wrenches associated to the right foot
    for (auto& [key, value] : m_rightFootContacWrenches)
    {
        if (!m_sensorBridge.getCartesianWrench(key, value.wrench))
        {
            BipedalLocomotion::log()->error("{} Unable to get the right wrench named {}.",
                                            errorPrefix,
                                            key);
            return false;
        }
    }

    Eigen::Vector3d gravity;
    gravity.setZero();
    gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    // update the floating base
    if (!this->updateFloatingBase())
    {
        BipedalLocomotion::log()->error("{} Unable to update the floating base.", errorPrefix);
        return false;
    }
    m_baseTransform = m_floatingBaseEstimator.getOutput().basePose;
    m_baseVelocity = m_floatingBaseEstimator.getOutput().baseTwist;

    /////// update kinDyn
    if (!m_kinDynWithMeasured->setRobotState(m_baseTransform.transform(),
                                             m_currentJointPos,
                                             iDynTree::make_span(m_baseVelocity.data(),
                                                                 manif::SE3d::Tangent::DoF),
                                             m_currentJointVel,
                                             gravity))
    {
        BipedalLocomotion::log()->error("{} Unable to set the robot state in the kinDyn object.",
                                        errorPrefix);
        return false;
    }

    if (!m_kinDynWithDesired->setRobotState(m_baseTransform.transform(),
                                            m_desJointPos,
                                            iDynTree::make_span(m_baseVelocity.data(),
                                                                manif::SE3d::Tangent::DoF),
                                            m_desJointVel,
                                            gravity))
    {
        BipedalLocomotion::log()->error("{} Unable to set the robot state in the kinDyn object.",
                                        errorPrefix);
        return false;
    }

    // prepare the output for the MPC
    for (auto& [key, value] : m_externalContactWrenches)
    {
        if (!m_sensorBridge.getCartesianWrench(key, value.wrench))
        {
            BipedalLocomotion::log()->error("{} Unable to get the left wrench named {}.",
                                            errorPrefix,
                                            key);
            return false;
        }

        const manif::SO3d rotation = BipedalLocomotion::Conversions::toManifRot(
            m_kinDynWithMeasured->getWorldTransform("root_link").getRotation());
        m_output.totalExternalWrench += rotation * value.wrench;
    }

    // TODO (Giulio): here we added a threshold probably it should be removed or better set as a
    // configuration parameter
    if (m_output.totalExternalWrench.force().norm() < 30)
    {
        m_output.totalExternalWrench.setZero();
    }

    m_output.com = iDynTree::toEigen(m_kinDynWithDesired->getCenterOfMassPosition());
    m_output.dcom = iDynTree::toEigen(m_kinDynWithDesired->getCenterOfMassVelocity());
    m_output.angularMomentum
        = iDynTree::toEigen(m_kinDynWithDesired->getCentroidalTotalMomentum().getAngularVec3());
    m_output.isValid = true;

    // this is the case in which the input provided by the MPC is not valid so we cannot proceed
    // further
    if (!m_input.isValid)
    {
        return true;
    }

    // if this is the first iteration we need to initialize some quantities
    if (m_firstIteration)
    {
        if (!m_centroidalSystem.dynamics->setState(
                {m_output.com, m_output.dcom, m_output.angularMomentum}))
        {
            BipedalLocomotion::log()->error("{} Unable to set the state for the centroidal "
                                            "dynamics.",
                                            errorPrefix);
            return false;
        }

        // take the x and y components of the CoM
        if (!m_comSystem.dynamics->setState({m_output.com.head<2>()}))
        {
            BipedalLocomotion::log()->error("{} Unable to set the state for the CoM dynamics.",
                                            errorPrefix);
            return false;
        }

        if (!m_floatingBaseSystem.dynamics->setState(
                {m_baseTransform.translation(), m_baseTransform.asSO3(), m_currentJointPos}))
        {
            BipedalLocomotion::log()->error("{} Unable to set the state for the floating base "
                                            "dynamics.",
                                            errorPrefix);
            return false;
        }

        // TODO this can be provided by MANN
        if (!m_IKandTasks.regularizationTask->setSetPoint(m_currentJointPos))
        {
            BipedalLocomotion::log()->error("{} Unable to set the set point for the "
                                            "regularization task.",
                                            errorPrefix);
            return false;
        }

        m_firstIteration = false;
    }

    // the input is now valid so we can update the centroidal dynamics
    if (!m_centroidalSystem.dynamics->setControlInput(
            {m_input.controllerOutput.contacts, m_output.totalExternalWrench}))
    {
        BipedalLocomotion::log()->error("{} Unable to set the control input for the centroidal "
                                        "dynamics.",
                                        errorPrefix);
    }

    // we need also to update the swing feet planners
    if (!m_leftFootPlanner.setContactList(m_input.contactPhaseList.lists().at("left_foot")))
    {
        BipedalLocomotion::log()->error("{} Unable to set the contact list for the left foot "
                                        "planner.",
                                        errorPrefix);
        return false;
    }
    if (!m_rightFootPlanner.setContactList(m_input.contactPhaseList.lists().at("right_foot")))
    {
        BipedalLocomotion::log()->error("{} Unable to set the contact list for the right foot "
                                        "planner.",
                                        errorPrefix);
        return false;
    }

    // advance the feet planners
    if (!m_leftFootPlanner.advance() || !m_rightFootPlanner.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to advance the feet planners.", errorPrefix);
        return false;
    }

    desiredZMP = this->computeDesiredZMP(m_input.controllerOutput.contacts);

    if (!this->evaluateZMP(measuredZMP))
    {
        BipedalLocomotion::log()->error("{} Unable to evaluate the measured zmp.", errorPrefix);
        return false;
    }

    // if (!m_leftFootPlanner.getOutput().mixedVelocity.coeffs().isZero())
    // {
    //     m_IKandTasks.leftFootTask->enableControl();
    // } else
    // {
    //     m_IKandTasks.leftFootTask->disableControl();
    // }

    // if (!m_rightFootPlanner.getOutput().mixedVelocity.coeffs().isZero())
    // {
    //     m_IKandTasks.rightFootTask->enableControl();
    // } else
    // {
    //     m_IKandTasks.rightFootTask->disableControl();
    // }

    // ZMP-COM controller
    using namespace BipedalLocomotion::GenericContainer::literals;
    Eigen::Vector3d comdes = m_centroidalSystem.dynamics->getState().get_from_hash<"com_pos"_h>();
    Eigen::Vector3d dcomdes = m_centroidalSystem.dynamics->getState().get_from_hash<"com_vel"_h>();

    // the X and Y position of the CoM is corrected by the CoM-ZMP controller
    m_CoMZMPController.setSetPoint(dcomdes.head<2>(), comdes.head<2>(), desiredZMP);

    // TODO (Giulio): the angle should be computed from the orientation of the base
    const double angle = 0;
    m_CoMZMPController
        .setFeedback(iDynTree::toEigen(m_kinDynWithMeasured->getCenterOfMassPosition()).head<2>(),
                     measuredZMP,
                     angle);
    if (!m_CoMZMPController.advance())
    {
        BipedalLocomotion::log()->error("{} Unable to advance the ZMP-COM controller.",
                                        errorPrefix);
        return false;
    }

    dcomdes.head<2>() = m_CoMZMPController.getOutput();
    m_comSystem.dynamics->setControlInput({dcomdes.head<2>()});
    m_comSystem.integrator->integrate(0s, m_dT);
    comdes.head<2>() = std::get<0>(m_comSystem.integrator->getSolution());

    // set the set points for the IK tasks
    m_IKandTasks.comTask->setSetPoint(comdes, dcomdes);
    m_IKandTasks.leftFootTask->setSetPoint(m_leftFootPlanner.getOutput().transform,
                                           m_leftFootPlanner.getOutput().mixedVelocity);

    m_IKandTasks.rightFootTask->setSetPoint(m_rightFootPlanner.getOutput().transform,
                                            m_rightFootPlanner.getOutput().mixedVelocity);

    m_IKandTasks.chestTask->setSetPoint(manif::SO3d::Identity(), manif::SO3d::Tangent::Zero());

    // evaluate the IK problem
    if (!m_IKandTasks.ikProblem.ik->advance())
    {
        BipedalLocomotion::log()->error("{} Unable to solve the IK problem", errorPrefix);
        return false;
    }

    // integrate the joint velocity and the base velocity
    m_floatingBaseSystem.dynamics->setControlInput(
        {m_IKandTasks.ikProblem.ik->getOutput().baseVelocity.coeffs(),
         m_IKandTasks.ikProblem.ik->getOutput().jointVelocity});
    m_floatingBaseSystem.integrator->integrate(0s, m_dT);

    const auto& [basePosition, baseRotation, jointPosition]
        = m_floatingBaseSystem.integrator->getSolution();

    m_desJointPos = jointPosition;
    m_desJointVel = m_IKandTasks.ikProblem.ik->getOutput().jointVelocity;

    if (!m_robotControl.setReferences(jointPosition,
                                      BipedalLocomotion::RobotInterface::IRobotControl::
                                          ControlMode::PositionDirect))
    {
        BipedalLocomotion::log()->error("{} Unable to set the reference", errorPrefix);
        return false;
    }

    // advance the time
    m_absoluteTime += m_dT;

    return true;
}

bool WholeBodyQPBlock::isOutputValid() const
{
    return true;
}

bool WholeBodyQPBlock::close()
{
    return true;
}
