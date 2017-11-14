/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#include "g2o_driver.hpp"
#include "utils/macros.hpp"
#include "types_sba_extension.hpp"

// G2O Headers
#include <g2o/config.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/icp/types_icp.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

G2ODriver::G2ODriver(
  const Eigen::Vector2d& focal_length,
  const Eigen::Vector2d& principal_point,
  const double stereo_baseline
)
  : next_vertex_id_( 0 ), optimizer_abort_request_( false ), optimizer_abort_request_copy_( false )
  , focal_length_( focal_length ), principal_point_( principal_point ), baseline_( stereo_baseline )
{
  // stereo baseline (must be a possitive value)
  assert( 0 < baseline_ );

    /// SETUP G2O ///

  optimizer_.setVerbose( false );

  optimizer_.setForceStopFlag( &optimizer_abort_request_ );

  // Higher converge velocity
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver
    = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();

  // Higher confident (better than CHOLMOD see paper: "3-D MappingWith an RGB-D Camera")
  //  g2o::BlockSolver_6_3::LinearSolverType* linearSolver
  //    = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();

  // Higher confident
  //  g2o::BlockSolver_6_3::LinearSolverType* linearSolver
  //    = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3( linearSolver );

  // choosing the method to use by optimizer
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );

  optimizer_.setAlgorithm(solver);

  // Set terminate thresholds
  gainTerminateThreshold_ = 1e-6;

  // Set Convergence Criterion
  g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction();
  terminateAction->setGainThreshold(gainTerminateThreshold_);
  optimizer_.addPostIterationAction(terminateAction);

  // Set Robust cost Function (Huber) delta
  robust_cost_function_delta_ = std::sqrt(5.991); // %95 of confident

}

bool G2ODriver::Adjust(int maxIterations)
{
  optimizer_.initializeOptimization();

  // TODO: pass as argument
  optimizer_.optimize( maxIterations );

  // set abortRequest flag false (TODO: I am not sure if it goes here) -> I think not =) <--- discussion between developers
  bool aborted = optimizer_abort_request_copy_;
  optimizer_abort_request_ = false;
  optimizer_abort_request_copy_ = optimizer_abort_request_;

  return not aborted; // aborted = false, occurs when the method converge or when it hits the max iteration number
}

void G2ODriver::Break()
{
  optimizer_abort_request_ = true;
  optimizer_abort_request_copy_ = optimizer_abort_request_;
}

void G2ODriver::Clear()
{
  next_vertex_id_ = 0;

  // freeing the graph memory
  optimizer_.clear();
}

G2ODriver::Vertex* G2ODriver::AddVertex(const CameraPose& cameraPose, const bool isFixed, VertexData* userData)
{
  // the position and the orientation (as quaternion is computed) in the expected format of g2o
  const Eigen::Vector3d& position = cameraPose.GetPosition();
  const Eigen::Quaterniond& orientation = cameraPose.GetOrientationQuaternion();

  // set up initial camera estimate
  g2o::SBACam sbacam(orientation, position);
  sbacam.setKcam(
    focal_length_[0], focal_length_[1],
    principal_point_[0], principal_point_[1],
    baseline_
  );

  g2o::VertexCam* v_se3 = new g2o::VertexCam();

  v_se3->setId( next_vertex_id_++ );
  v_se3->setEstimate( sbacam );
  v_se3->setFixed( isFixed );
  v_se3->setUserData( userData );

  optimizer_.addVertex( v_se3 );

  return v_se3;
}

G2ODriver::Vertex* G2ODriver::AddVertex(const MapPoint& mapPoint, const bool marginalize ,const bool isFixed, VertexData* userData)
{
  g2o::VertexSBAPointXYZ* v_p = new g2o::VertexSBAPointXYZ();

  v_p->setId( next_vertex_id_++ );
  v_p->setMarginalized( marginalize );
  v_p->setEstimate( mapPoint.GetPosition() );
  v_p->setFixed( isFixed );
  v_p->setUserData( userData );

  optimizer_.addVertex( v_p );

  return v_p;
}

void G2ODriver::AddEdge(int edgeId, Vertex* point, Vertex* keyFrame, const Measurement& meas)
{
  Edge* e = nullptr;
  switch ( meas.GetType() )
  {
    case Measurement::LEFT :
      e = BuildMonoEdge( meas.GetProjection() );
      break;
    case Measurement::STEREO :
      e = BuildStereoEdge( meas.GetProjection() );
      break;
    case Measurement::RIGHT :
      e = BuildMonoEdgeRight( meas.GetProjection() );
      break;
    default:
      // TODO tirar una excepcion como la gente (tfischer)
      assert( false );
      break;
  }
  
  e->vertices()[0] = point;
  e->vertices()[1] = keyFrame;

  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  e->setRobustKernel(rk);
  rk->setDelta(robust_cost_function_delta_);

  e->setId( edgeId );

  optimizer_.addEdge( e );
}

Eigen::Vector3d G2ODriver::GetPoint( g2o::HyperGraph::Vertex& vertex )
{
  g2o::VertexSBAPointXYZ& point_vertex = dynamic_cast<g2o::VertexSBAPointXYZ&>( vertex );
  return point_vertex.estimate();
}

CameraPose G2ODriver::GetPose( g2o::HyperGraph::Vertex& vertex, const Eigen::Matrix6d& covariance )
{
  g2o::VertexCam& vertex_view = dynamic_cast<g2o::VertexCam&>( vertex );

  // apparently in g2o translation means position.
  Eigen::Vector3d position = vertex_view.estimate().translation();
  const Eigen::Quaterniond& orientation = vertex_view.estimate().rotation();

  return CameraPose(position, orientation, covariance);
}

G2ODriver::Edge* G2ODriver::BuildMonoEdge(const std::vector<double>& projection)
{
  g2o::EdgeProjectP2MC* e = new g2o::EdgeProjectP2MC();
  e->setMeasurement( Eigen::Vector2d(projection[0], projection[1]) );
  e->information() = Eigen::Matrix2d::Identity();
  return e;
}

G2ODriver::Edge* G2ODriver::BuildMonoEdgeRight(const std::vector<double>& projection)
{
  g2o::EdgeProjectP2MCRight* e = new g2o::EdgeProjectP2MCRight();
  e->setMeasurement( Eigen::Vector2d(projection[0], projection[1]) );
  e->information() = Eigen::Matrix2d::Identity();
  return e;
}

G2ODriver::Edge* G2ODriver::BuildStereoEdge(const std::vector<double>& projection)
{
  g2o::EdgeProjectP2SC*e = new g2o::EdgeProjectP2SC();
  e->setMeasurement( Eigen::Vector3d(projection[0] , projection[1], projection[2]) );
  e->information() = Eigen::Matrix3d::Identity();
  return e;
}
