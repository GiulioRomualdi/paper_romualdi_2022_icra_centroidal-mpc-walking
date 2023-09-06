/**
 * @file WholeBodyQPBlock.h
 * @authors Giulio Romualdi
 * @copyright This software may be modified and distributed under the terms of the GNU Lesser
 * General Public License v2.1 or any later version.
 */

#ifndef CENTROIDAL_MCP_WALKING_WHOLE_BODY_QP_BLOCK_H
#define CENTROIDAL_MCP_WALKING_WHOLE_BODY_QP_BLOCK_H

#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>

#include <BipedalLocomotion/ContactDetectors/FixedFootDetector.h>
#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/CentroidalDynamics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/FloatingBaseSystemKinematics.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/LinearTimeInvariantSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/RK4.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>

#include <BipedalLocomotion/IK/CoMTask.h>
#include <BipedalLocomotion/IK/IntegrationBasedIK.h>
#include <BipedalLocomotion/IK/JointTrackingTask.h>
#include <BipedalLocomotion/IK/QPInverseKinematics.h>
#include <BipedalLocomotion/IK/SE3Task.h>
#include <BipedalLocomotion/IK/SO3Task.h>
#include <BipedalLocomotion/IK/R3Task.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/SimplifiedModelControllers/CoMZMPController.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>

#include <yarp/os/BufferedPort.h>

#include <CentroidalMPCWalking/CentroidalMPCBlock.h>

namespace CentroidalMPCWalking
{
class WholeBodyQPBlock
    : public BipedalLocomotion::System::Advanceable<CentroidalMPCOutput, CentroidalMPCInput>
{

    typename WholeBodyQPBlock::Output m_output;
    typename WholeBodyQPBlock::Input m_input;
    bool m_firstIteration{true};
    double m_robotMass;
    /*     std::string m_robot; /**< Robot name. */

    Eigen::VectorXd m_currentJointPos; /**< Current joint positions. */
    Eigen::VectorXd m_currentJointVel; /**< Current joint velocities. */
    Eigen::VectorXd m_desJointPos; /**< Current joint positions. */
    Eigen::VectorXd m_desJointVel; /**< Current joint velocities. */

    yarp::os::BufferedPort<BipedalLocomotion::YarpUtilities::VectorsCollection> m_logDataPort;

    manif::SE3d m_baseTransform;
    manif::SE3d::Tangent m_baseVelocity;

    BipedalLocomotion::RobotInterface::PolyDriverDescriptor m_controlBoard; /**< Control board
                                                                               remapper. */

    struct ContactWrenchHandler
    {
        BipedalLocomotion::RobotInterface::PolyDriverDescriptor polyDriverDescriptor;
        BipedalLocomotion::Math::Wrenchd wrench;
    };

    std::unordered_map<std::string, ContactWrenchHandler> m_leftFootContacWrenches;
    std::unordered_map<std::string, ContactWrenchHandler> m_rightFootContacWrenches;
    std::unordered_map<std::string, ContactWrenchHandler> m_externalContactWrenches;

    BipedalLocomotion::RobotInterface::YarpRobotControl m_robotControl; /**< Robot control object.
                                                                         */
    BipedalLocomotion::RobotInterface::YarpSensorBridge m_sensorBridge; /**< Sensor bridge object.
                                                                         */

    BipedalLocomotion::Estimators::LeggedOdometry m_floatingBaseEstimator;
    BipedalLocomotion::Contacts::FixedFootDetector m_fixedFootDetector;

    BipedalLocomotion::Planners::SwingFootPlanner m_leftFootPlanner;
    BipedalLocomotion::Planners::SwingFootPlanner m_rightFootPlanner;

    BipedalLocomotion::SimplifiedModelControllers::CoMZMPController m_CoMZMPController;
    struct IKProblemAndTask
    {
        BipedalLocomotion::IK::IntegrationBasedIKProblem ikProblem;
        std::shared_ptr<BipedalLocomotion::IK::SE3Task> leftFootTask;
        std::shared_ptr<BipedalLocomotion::IK::SE3Task> rightFootTask;
        std::shared_ptr<BipedalLocomotion::IK::CoMTask> comTask;
        std::shared_ptr<BipedalLocomotion::IK::SO3Task> chestTask;
        std::shared_ptr<BipedalLocomotion::IK::R3Task> rootTask;
        std::shared_ptr<BipedalLocomotion::IK::JointTrackingTask> regularizationTask;
    };
    IKProblemAndTask m_IKandTasks;
    Eigen::Vector3d m_rootLinkOffset;
    Eigen::VectorXd m_jointPosRegularize;

    template <typename _Dynamics, typename _Integrator> struct DynamicsAndIntegrator
    {
        std::shared_ptr<_Integrator> integrator;
        std::shared_ptr<_Dynamics> dynamics;
    };

    DynamicsAndIntegrator<BipedalLocomotion::ContinuousDynamicalSystem::CentroidalDynamics,
                          BipedalLocomotion::ContinuousDynamicalSystem::RK4<
                              BipedalLocomotion::ContinuousDynamicalSystem::CentroidalDynamics>>
        m_centroidalSystem;

    DynamicsAndIntegrator<
        BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem,
        BipedalLocomotion::ContinuousDynamicalSystem::RK4<
            BipedalLocomotion::ContinuousDynamicalSystem::LinearTimeInvariantSystem>>
        m_comSystem;

    DynamicsAndIntegrator<
        BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics,
        BipedalLocomotion::ContinuousDynamicalSystem::ForwardEuler<
            BipedalLocomotion::ContinuousDynamicalSystem::FloatingBaseSystemKinematics>>
        m_floatingBaseSystem;

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynWithDesired;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynWithMeasured;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynWithRegularization;

    std::chrono::nanoseconds m_dT;
    std::chrono::nanoseconds m_absoluteTime{std::chrono::nanoseconds::zero()};

    bool createPolydriver(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool initializeRobotControl(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool instantiateSensorBridge(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool instantiateIK(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool instantiateLeggedOdometry(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
        const std::string& modelPath,
        const std::vector<std::string>& jointLists);

    bool instantiateSwingFootPlanner(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool createKinDyn(const std::string& modelPath, const std::vector<std::string>& jointLists);

    bool updateFloatingBase();

    bool createAllContactWrenchesDriver(
        std::shared_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    BipedalLocomotion::RobotInterface::PolyDriverDescriptor createContactWrenchDriver(
        std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
        const std::string& local);

    bool evaluateZMP(Eigen::Ref<Eigen::Vector2d> zmp);

    bool computeDesiredZMP(
        const std::map<std::string, BipedalLocomotion::Contacts::DiscreteGeometryContact>& contacts,
        Eigen::Ref<Eigen::Vector2d> zmp);

public:
    bool initialize(std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler>
                        handler) override;

    const Output& getOutput() const override;

    bool setInput(const Input& input) override;

    bool advance() override;

    bool isOutputValid() const override;

    bool close() override;
};
} // namespace CentroidalMPCWalking

#endif // CENTROIDAL_MCP_WALKING_WHOLE_BODY_QP_BLOCK_H
