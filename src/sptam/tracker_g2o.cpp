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

#include "tracker_g2o.hpp"
#include "types_sba_extension.hpp"

#include "utils/macros.hpp"
#include "utils/pose_covariance.hpp"
#include "utils/projective_math.hpp"
#include "utils/projection_derivatives.hpp"

#include <Eigen/Eigenvalues>

#ifdef SHOW_PROFILING
#include "utils/log/Profiler.hpp"
#endif // SHOW_PROFILING

namespace g2o
{
  typedef EdgeProjectP2MC LeftMeasurementEdge;
  typedef EdgeProjectP2MCRight RightMeasurementEdge;
  typedef EdgeProjectP2SC StereoMeasurementEdge;
} // g2o

template <typename DATA_TYPE, int SIZE>
bool isDefinitePositive( const Eigen::Matrix<DATA_TYPE, SIZE, SIZE>& M )
{
  Eigen::SelfAdjointEigenSolver< Eigen::Matrix<DATA_TYPE, SIZE, SIZE> > eigensolver( M );

  if (eigensolver.info () != Eigen::Success)
  {
    std::cerr << "failed to compute eigen vectors/values. Is the covariance matrix correct?" << std::endl;
    return false;
  }

  Eigen::Matrix<DATA_TYPE, SIZE, 1> eigenValues = eigensolver.eigenvalues();
  //~ Eigen::Matrix<DATA_TYPE, SIZE, SIZE> eigenVectors = eigensolver.eigenvectors();

  forn(i, SIZE)
    if ( eigenValues[i] < 0 )
    {
      std::cerr << eigenValues[i] << " < 0" << std::endl;
      return false;
    }

  return true;
}

template <typename T>
inline Eigen::Matrix6d computeEdgeInformation( const T& edge )
{
  const g2o::SBACam& cam = dynamic_cast<g2o::VertexCam*>( edge.vertices()[1] )->estimate();
  const Eigen::Vector3d& point = dynamic_cast<g2o::VertexSBAPointXYZ*>( edge.vertices()[0] )->estimate();

  Eigen::Matrix<double,2,6> j = jacobianXj(cam.w2n, cam.Kcam, point);

  return j.transpose() * j;
}

template <typename T>
inline double computeEdgeSquaredError( const T& edge )
{
  return edge.error().transpose() * edge.error();
}

template <int SIZE>
inline double sumSquareInverse( const Eigen::Matrix<double, SIZE, 1>& v )
{
  double ret = 0;
  forn(i, SIZE)
    ret += 1.0 / (v[i]*v[i]);
  return ret;
}

// constructor of class Tracker_g2o
tracker_g2o::tracker_g2o(const Eigen::Vector2d& focal_length, const Eigen::Vector2d& principal_point, double stereo_baseline)
  : g2o_driver_(focal_length, principal_point, stereo_baseline)
  , fu_( focal_length[0] ), fv_( focal_length[1] ), u0_( principal_point[0] ), v0_( principal_point[1] )
{}

bool tracker_g2o::RefineCameraPose(const CameraPose& estimatedCameraPose, CameraPose& refinedCameraPose, const std::list<Match>& measurements)
{
  size_t measurements_num = measurements.size();

  // We need at least 4 measurements to get a solution.
  // Make sure there are a bunch more just to be robust.
  if( measurements_num < 10 ) {
    std::cerr << std::endl << std::endl << "WARNING: Not enough points for tracking." << std::endl << std::endl;

    #ifdef SHOW_PROFILING
      WriteToLog("WARNING: Not enough points for tracking. Measurements: ", measurements_num);
    #endif // SHOW_PROFILING

    refinedCameraPose = estimatedCameraPose;
    return false;
  }

  // freeing the graph memory
  g2o_driver_.Clear();

  G2ODriver::Vertex* pose_vertex = g2o_driver_.AddVertex(estimatedCameraPose, false);

  // Add the points' 3D position

  for( const auto& match : measurements )
  {
    G2ODriver::Vertex* point_vertex = g2o_driver_.AddVertex(*match.mapPoint, false, true);

    // trivial edge id, since we won't need it for anything
    g2o_driver_.AddEdge(0, point_vertex, pose_vertex, match.measurement);
  }

  g2o_driver_.Adjust( 10 );

  refinedCameraPose = G2ODriver::GetPose( *pose_vertex, GetPoseCovariance(*pose_vertex) );
  return true;
}

void getRPY(const Eigen::Matrix3d& R, double& roll, double& pitch, double& yaw, unsigned int solution_number = 1)
{
	struct Euler
	{
		double yaw;
		double pitch;
		double roll;
	};

	Euler euler_out;
	Euler euler_out2; //second solution
	//get the pointer to the raw data

	// Check that pitch is not at a singularity
	// Check that pitch is not at a singularity
	if (abs(R(2, 0)) >= 1)
	{
		euler_out.yaw = 0;
		euler_out2.yaw = 0;

		// From difference of angles formula
		if (R(2, 0) < 0)  //gimbal locked down
		{
			double delta = atan2(R(0, 1),R(0, 2));
			euler_out.pitch = M_PI / double(2.0);
			euler_out2.pitch = M_PI / double(2.0);
			euler_out.roll = delta;
			euler_out2.roll = delta;
		}
		else // gimbal locked up
		{
			double delta = atan2(-R(0, 1),-R(0, 2));
			euler_out.pitch = -M_PI / double(2.0);
			euler_out2.pitch = -M_PI / double(2.0);
			euler_out.roll = delta;
			euler_out2.roll = delta;
		}
	}
	else
	{
		euler_out.pitch = - asin(R(2, 0));
		euler_out2.pitch = M_PI - euler_out.pitch;

		euler_out.roll = atan2(R(2, 1)/cos(euler_out.pitch), 
		R(2, 2)/cos(euler_out.pitch));
		euler_out2.roll = atan2(R(2, 1)/cos(euler_out2.pitch), 
		R(2, 2)/cos(euler_out2.pitch));

		euler_out.yaw = atan2(R(1, 0)/cos(euler_out.pitch), 
		R(0, 0)/cos(euler_out.pitch));
		euler_out2.yaw = atan2(R(1, 0)/cos(euler_out2.pitch), 
		R(0, 0)/cos(euler_out2.pitch));
	}

	if (solution_number == 1)
	{ 
		yaw = euler_out.yaw; 
		pitch = euler_out.pitch;
		roll = euler_out.roll;
	}
	else
	{ 
		yaw = euler_out2.yaw; 
		pitch = euler_out2.pitch;
		roll = euler_out2.roll;
	}
}

Eigen::Matrix6d tracker_g2o::GetPoseCovariance(g2o::HyperGraph::Vertex& pose_vertex)
{
  Eigen::Matrix2d cov_z = Eigen::Matrix2d::Identity();

  Eigen::Matrix3d K = Eigen::Matrix3d::Zero();
  K(0, 0) = fu_;
  K(1, 1) = fv_;
  K(0, 2) = u0_;
  K(1, 2) = v0_;

  const g2o::VertexCam& vertex_view = dynamic_cast<const g2o::VertexCam&>( pose_vertex );
  const g2o::SBACam& current_camera = dynamic_cast<const g2o::SBACam&>( vertex_view.estimate() );

  // apparently in g2o translation means position.
  Eigen::Vector3d position = current_camera.translation();
  const Eigen::Quaterniond& orientation_q = current_camera.rotation();
  Eigen::Matrix3d orientation( orientation_q );

  // transform from world to camera coordinates
  Eigen::Matrix<double,3,4> w2c = current_camera.w2n;

  double mu_roll, mu_pitch, mu_yaw;
  getRPY(orientation, mu_roll, mu_pitch, mu_yaw);

  // sum individual covariance for each edge

  Eigen::Matrix6d pose_covariance = Eigen::Matrix6d::Zero();
  for ( const auto& edge : g2o_driver_.activeEdges() )
  {
    // threshold for a 2D gaussian confidence interval of 95%
    if ( 5.991 < edge->chi2() ) {
      //~ std::cerr << "ignoring outlayer" << std::endl;
      continue;
    }

    // If it is a stereo measurement
    if(const g2o::StereoMeasurementEdge* e = dynamic_cast<const g2o::StereoMeasurementEdge*>( edge ))
    {
      // get point in world coordinates
      const Eigen::Vector3d& xw = dynamic_cast<g2o::VertexSBAPointXYZ*>( e->vertices()[0] )->estimate();
      const Eigen::Vector3d xc = w2c * Eigen::Vector4d(xw[0], xw[1], xw[2], 1);

      Eigen::Vector3d z3 = e->measurement();
      Eigen::Vector2d z = Eigen::Vector2d(z3[0], z3[1]);

      pose_covariance += computeMeasurementCovariance(position[0], position[1], position[2], mu_roll, mu_pitch, mu_yaw, xw, xc, z, K, cov_z);
    }

    // If it is a left monocular measurement
    else if(const g2o::LeftMeasurementEdge* e = dynamic_cast<const g2o::LeftMeasurementEdge*>( edge ))
    {
      // get point in world coordinates
      const Eigen::Vector3d& xw = dynamic_cast<g2o::VertexSBAPointXYZ*>( e->vertices()[0] )->estimate();
      const Eigen::Vector3d xc = w2c * Eigen::Vector4d(xw[0], xw[1], xw[2], 1);

      Eigen::Vector2d z = e->measurement();

      pose_covariance += computeMeasurementCovariance(position[0], position[1], position[2], mu_roll, mu_pitch, mu_yaw, xw, xc, z, K, cov_z);
    }

    // If it is a right monocular measurement
    else if(/*const g2o::RightMeasurementEdge* e =*/ dynamic_cast<const g2o::RightMeasurementEdge*>( edge ))
    {
    }

    else
      std::cerr << "tried to get information for unsupported measurement type" << std::endl;
  }

  return pose_covariance;
}
