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

#include "Map.hpp"
#include "StereoFrame.hpp"
#include "RowMatcher.hpp"
#include "utils/cv2eigen.hpp"
#include "utils/projective_math.hpp"

#include <opencv2/calib3d/calib3d.hpp>  // cv::triangulatePoints

#include <opencv2/highgui/highgui.hpp>  // for debugging

StereoFrame::StereoFrame(
  const CameraPose& cameraPose, const CameraParameters& calibrationLeft,
  const double stereo_baseline, const CameraParameters& calibrationRight,
  const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight,
  bool bFixed
)
  : frameLeft_(Camera(cameraPose, calibrationLeft), imageFeaturesLeft)
  , frameRight_(Camera(ComputeRightCameraPose(cameraPose, stereo_baseline), calibrationRight), imageFeaturesRight)
  , stereo_baseline_( stereo_baseline )
  , bFixed_( bFixed ), id_( -1 )
{}


/* When a keyframe is added to the Map, has to be copyed, but the shared_mutex does not have a
 * a default copy constructor. Thats why we have to specify that the new points will have a separate mutex */
StereoFrame::StereoFrame(const StereoFrame& stereoFrame)
: stereo_frames_mutex_() // The copy has a diferent mutex!
, frameLeft_( stereoFrame.frameLeft_ ) // TODO: check if Frame copy constructor is working properly.
, frameRight_( stereoFrame.frameRight_ ) // TODO: check if Frame copy constructor is working properly.
, stereo_baseline_( stereoFrame.stereo_baseline_ )
, bFixed_( stereoFrame.bFixed_ )
, id_( stereoFrame.id_ )
{}




void StereoFrame::FindMatches(
  const std::vector<cv::Point3d>& points,
  const std::vector<cv::Mat>& descriptors,
  const cv::DescriptorMatcher& descriptorMatcher,
  const size_t matchingNeighborhoodThreshold, const double matchingDistanceThreshold,
  std::map<size_t, MEAS>& measurementsLeft,
  std::map<size_t, MEAS>& measurementsRight
) const
{
  boost::unique_lock<boost::shared_mutex> lock(stereo_frames_mutex_);

  // TODO: maybe is useful to pass right descriptors too
  frameLeft_.FindMatches(points, descriptors, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold, measurementsLeft);
  frameRight_.FindMatches(points, descriptors, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold, measurementsRight);
}

void StereoFrame::TriangulatePoints(
  const RowMatcher& matcher,
  std::vector<cv::Point3d>& points,
  std::vector<cv::Point2d>& featuresLeft, std::vector<cv::Mat>& descriptorsLeft,
  std::vector<cv::Point2d>& featuresRight, std::vector<cv::Mat>& descriptorsRight
)/* const*/
{

  boost::unique_lock<boost::shared_mutex> lock(stereo_frames_mutex_);

  // TODO ImageFeatures podria guardar internamente el vector
  // de los unmatched, y descartar los matched, total nunca más los uso.
  // Esto permitiría recibir acá una referencia

  // retrieve unmatched features from both frames.

  std::vector<cv::KeyPoint> unmatchedKeypointsLeft, unmatchedKeypointsRight;
  cv::Mat unmatchedDescriptorsLeft, unmatchedDescriptorsRight;
  std::vector<size_t> indexesLeft, indexesRight;

  // Get descritptors for both images

  frameLeft_.GetUnmatchedKeyPoints(unmatchedKeypointsLeft, unmatchedDescriptorsLeft, indexesLeft);

  frameRight_.GetUnmatchedKeyPoints(unmatchedKeypointsRight, unmatchedDescriptorsRight, indexesRight);

  std::list<std::pair<size_t, size_t>> matches;

  CreatePoints(
    matcher,
    unmatchedKeypointsLeft, unmatchedDescriptorsLeft,
    unmatchedKeypointsRight, unmatchedDescriptorsRight,
    points, matches
  );

  featuresLeft.reserve( matches.size() );
  descriptorsLeft.reserve( matches.size() );
  featuresRight.reserve( matches.size() );
  descriptorsRight.reserve( matches.size() );

  for ( auto pair : matches )
  {
    size_t idxLeft = pair.first;
    size_t idxRight = pair.second;

    featuresLeft.push_back( unmatchedKeypointsLeft[ idxLeft ].pt );
    descriptorsLeft.push_back( unmatchedDescriptorsLeft.row( idxLeft ) );

    featuresRight.push_back( unmatchedKeypointsRight[ idxRight ].pt );
    descriptorsRight.push_back( unmatchedDescriptorsRight.row( idxRight ) );

    frameLeft_.SetMatchedKeyPoint( indexesLeft[ idxLeft ] );
    frameRight_.SetMatchedKeyPoint( indexesRight[ idxRight] );
  }
}

void StereoFrame::CreatePoints(
  const RowMatcher& matcher,
  const std::vector<cv::KeyPoint>& keypointsLeft, const cv::Mat& allDescriptorsLeft,
  const std::vector<cv::KeyPoint>& keypointsRight, const cv::Mat& allDescriptorsRight,
  std::vector<cv::Point3d>& points, std::list<std::pair<size_t, size_t>>& matches
)
{

  // per-row matcher for stereo rectify images
  std::vector<cv::DMatch> cvMatches;
  matcher.match(keypointsLeft, allDescriptorsLeft, keypointsRight, allDescriptorsRight, cvMatches);

  // Return if no matches were found
  if ( cvMatches.empty() ) {
    std::cerr << "No matches found for triangulation" << std::endl;
    return;
  }

  const Camera& cameraLeft = frameLeft_.GetCamera();
  const Camera& cameraRight = frameRight_.GetCamera();

  // Compute Projections matrices
  cv::Matx34d projectionLeft = cameraLeft.GetProjection();
  cv::Matx34d projectionRight = cameraRight.GetProjection();

  // NO hace falta filtrar con la F dado que si luego de que los puntos son creados
  // estos se proyectan nuevamente sobre la imagen  ambos metodos tienen los mismos resultados
  // Compute Fundamental Matrix
//  cv::Matx33d fundamentalMatrix = ComputeFundamentalMat(projectionLeft, projectionRight);

//  std::vector<cv::DMatch> goodMatches = FilterMatchesByF( fundamentalMatrix, cvMatches,
//                                                          keypointsLeft, keypointsRight,
//                                                          epipolarDistanceThreshold, matchingDistanceThreshold );
  const std::vector<cv::DMatch>& goodMatches = cvMatches;
//  std::cout << "F: Correct (by F) / Total matches: " << goodMatches.size() << " / " << cvMatches.size() << std::endl;


  std::vector<cv::Point2d> matchedPointsLeft;
  std::vector<cv::Point2d> matchedPointsRight;
  matchedPointsLeft.reserve( goodMatches.size() );
  matchedPointsRight.reserve( goodMatches.size() );

  // initialize the points
  for ( auto match : goodMatches ) {
    matchedPointsLeft.push_back( keypointsLeft[ match.queryIdx ].pt );
    matchedPointsRight.push_back( keypointsRight[ match.trainIdx ].pt );
  }

  // Triangulate 3D points from both cameras
  cv::Mat point3DHomos;
  cv::triangulatePoints(
    projectionLeft, projectionRight, matchedPointsLeft, matchedPointsRight, point3DHomos
  );

  // Filter some more by viewing frustum and fill the return parameters
  for(int i = 0; i < point3DHomos.cols; ++i)
  {
    cv::Point3d point = toInHomo(cv::Vec4d( point3DHomos.col(i) ));

    // if the point is not in range of any camera it is discarded
    if( not cameraLeft.CanView( cv2eigen( point ) ) || not cameraRight.CanView( cv2eigen( point ) ) )
      continue;

    // this indexes are for the original unmatched collections
    size_t idxLeft = goodMatches[i].queryIdx;
    size_t idxRight = goodMatches[i].trainIdx;

    points.push_back( point );

    matches.push_back( std::pair<size_t, size_t>(idxLeft, idxRight) );
  }
}

void StereoFrame::UpdateCameraPose(const CameraPose& cameraPose)
{

  boost::unique_lock<boost::shared_mutex> lock(stereo_frames_mutex_);

  frameLeft_.UpdateCameraPose( cameraPose );
  frameRight_.UpdateCameraPose( ComputeRightCameraPose(cameraPose, stereo_baseline_) );
}

bool StereoFrame::canView(const MapPoint& mapPoint) const
{

  boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);

  const Eigen::Vector3d& point = mapPoint.GetPosition();

  // compute only once, since both cameras have the same orientation.
  bool similar_angle = false;
  {
    Eigen::Vector3d currentNormal = point - frameLeft_.GetPosition();
    currentNormal.normalize();

    // angle is in radians
    double angle = std::acos( ( mapPoint.GetNormal() ).dot( currentNormal ) );

    // Discard Points which were created from a greater 45 degrees pint of view.
    // TODO: pass this threshold as a parameter.
    similar_angle = angle < (M_PI / 4.0);
  }

  return similar_angle and ( frameLeft_.GetCamera().CanView( point ) or frameRight_.GetCamera().CanView( point ) );
}

  /*** private functions ***/

CameraPose StereoFrame::ComputeRightCameraPose(const CameraPose& leftCameraPose, const double stereo_baseline)
{
  // The position is the baseline converted to world coordinates.
  // The orientation is the same as the left camera.
  return CameraPose(leftCameraPose.ToWorld( Eigen::Vector3d(stereo_baseline, 0, 0) ), leftCameraPose.GetOrientationQuaternion(), leftCameraPose.covariance());
}

  /*** other functions ***/

//std::ostream& operator << ( std::ostream& os, const StereoFrame& keyFrame)
//{
//  return os << "id: " << keyFrame.GetId() << " pose: " << keyFrame.GetCameraLeft();
//}
