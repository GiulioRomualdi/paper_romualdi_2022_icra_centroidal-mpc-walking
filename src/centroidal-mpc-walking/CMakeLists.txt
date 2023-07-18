# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi

add_centroidal_mpc_walking_application(
  NAME            walking
  SOURCES         src/Main.cpp src/CentroidalMPCBlock.cpp src/WholeBodyQPBlock.cpp
  HEADERS         include/CentroidalMPCWalking/CentroidalMPCBlock.h include/CentroidalMPCWalking/WholeBodyQPBlock.h
  LINK_LIBRARIES  BipedalLocomotion::ParametersHandlerYarpImplementation
                  BipedalLocomotion::RobotInterfaceYarpImplementation
                  BipedalLocomotion::FloatingBaseEstimators
                  BipedalLocomotion::IK
                  BipedalLocomotion::ContinuousDynamicalSystem
                  BipedalLocomotion::System
                  BipedalLocomotion::SystemYarpImplementation
                  BipedalLocomotion::ReducedModelControllers
                  BipedalLocomotion::Planners
                  BipedalLocomotion::ManifConversions
                  BipedalLocomotion::ContactDetectors
                  BipedalLocomotion::SimplifiedModelControllers
                  BipedalLocomotion::ML
                  BipedalLocomotion::Math
                  BipedalLocomotion::GenericContainer
                  BipedalLocomotion::VectorsCollection
  )

install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)

