
#include "sptam/msg_mapDiff.h"           
#include <opencv2/opencv.hpp>           //cv types
#include "../CameraPose.hpp"            // typedef for Matrix6d
#include "../Map.hpp"                   // cv::KeyPoint
#include "../utils/macros.hpp"          // forn


namespace dsptam{

inline std::vector<float> descriptor2Vector(const cv::Mat& m)
{
  std::vector<float> v; 
  for (int j = 0; j < m.size().width ; ++j)
    {
      v.push_back(m.at<uchar>(0,j));
    }
    return v;
}

inline void fromMSG_Geometry_msgsPoint2EigenVector3d(const geometry_msgs::Point& msg, Eigen::Vector3d& out)
{
   out.x() = msg.x;
   out.y() = msg.y;
   out.z() = msg.z;
}

inline Eigen::Vector3d geometry_msgsPoint2EigenVector3d(const geometry_msgs::Point& msg)
{
   Eigen::Vector3d out;
   out.x() = msg.x;
   out.y() = msg.y;
   out.z() = msg.z;

   return out;
}




/////====================================================================================
/////============================        msg creatiom             =======================
/////====================================================================================


inline sptam::msg_kp createMsgKeyPoint(const cv::KeyPoint& keyPoint){
  sptam::msg_kp msg_kp;

  msg_kp.x = keyPoint.pt.x;
    msg_kp.y = keyPoint.pt.y;
    msg_kp.size = keyPoint.size;
    msg_kp.angle = keyPoint.angle;
    msg_kp.response = keyPoint.response;
    msg_kp.octave = keyPoint.octave;
    msg_kp.class_id = keyPoint.class_id;

    return msg_kp;
}







inline sptam::msg_feature createMsgFeature(const ImageFeatures& iF, size_t n)
{

  // ImageFeatures::GetKeypoints() devuelve std::vector<cv::KeyPoint>&
  // ImageFeatures::GetDescriptors() devuelve cv::Mat&
  // supongo que la fila i de cv::Mat contiene los descriptores del kp de la posicion i del vector
  sptam::msg_feature msg_feature;

  //genero un msg_kp a partir del n-ésimo kp de iF
  msg_feature.keypoint = createMsgKeyPoint(iF.GetKeypoint(n));

  // Itero por cantidad de columnas de las matriz(vector de tamaño 1xj )
  for (int j = 0; j < (iF.GetDescriptor(n)).size().width ; ++j)
    {
      msg_feature.descriptor.push_back((iF.GetDescriptor(n)).at<uchar>(0,j));
    }

    return msg_feature;
}

// Creo un vector de msg_feature 
inline std::vector<sptam::msg_feature> ImageFeatures2Vector(const ImageFeatures& iF)
{
  std::vector<sptam::msg_feature> vector_msg_feature;

  for (unsigned int i = 0; i < (iF.GetKeypoints()).size(); ++i)
  {
    vector_msg_feature.push_back(createMsgFeature(iF, i));    
  }

  return vector_msg_feature;
}  



inline geometry_msgs::PoseWithCovariance createMsgCameraPose(const CameraPose cp){

  geometry_msgs::PoseWithCovariance msg_CameraPose;

  const Eigen::Matrix6d& covariance = cp.covariance();

  msg_CameraPose.pose.position.x = cp.GetPosition().x();
  msg_CameraPose.pose.position.y = cp.GetPosition().y();
  msg_CameraPose.pose.position.z = cp.GetPosition().z();


  msg_CameraPose.pose.orientation.x = cp.GetOrientationQuaternion().x();
  msg_CameraPose.pose.orientation.y = cp.GetOrientationQuaternion().y();
  msg_CameraPose.pose.orientation.z = cp.GetOrientationQuaternion().z();
  msg_CameraPose.pose.orientation.w = cp.GetOrientationQuaternion().w();

  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  forn(i, 6) forn(j, 6)
  {
    msg_CameraPose.covariance[6*i + j] = covariance(i, j);
  }

  return msg_CameraPose;
}


inline CameraPose fromMsgCameraPose(geometry_msgs::PoseWithCovariance msg_cameraPose){

  Eigen::Quaterniond quat = Eigen::Quaterniond(
    msg_cameraPose.pose.orientation.w,   //const Scalar& w,
    msg_cameraPose.pose.orientation.x,   //const Scalar& x,
    msg_cameraPose.pose.orientation.y,   //const Scalar& y,
    msg_cameraPose.pose.orientation.z   //const Scalar& z,
  );   

  Eigen::Vector3d vector3d;
  dsptam::fromMSG_Geometry_msgsPoint2EigenVector3d(msg_cameraPose.pose.position, vector3d);

  Eigen::Matrix6d matr;
  matr = Eigen::Matrix6d(6,6);
  for(int i = 0; i < 6; i++){
    for(int j = 0; j < 6; j++){
      matr(i,j) = msg_cameraPose.covariance[6*i+j];
    }
  }


  //Create object
  return CameraPose(
      vector3d, //const Eigen::Vector3d& position, 
      quat,     //const Eigen::Quaterniond& orientation, 
      matr      //const Eigen::Matrix6d& covariance
      );
}



inline sptam::msg_KeyFrame createMsgKeyFrame(const sptam::Map::KeyFrame& kf)
{
  sptam::msg_KeyFrame msg_kf;

  msg_kf.id_KeyFrame = kf.GetId();

  msg_kf.features_left = ImageFeatures2Vector((kf.GetFrameLeft()).GetFeatures());
  msg_kf.features_right = ImageFeatures2Vector((kf.GetFrameRight()).GetFeatures());
  
  msg_kf.cameraPose = dsptam::createMsgCameraPose(kf.GetCameraPose());
  msg_kf.bFixed = kf.isFixed();

  return msg_kf;
}

inline sptam::msg_KeyFrame createMsgKeyFrameToUpdate(const sptam::Map::KeyFrame& kf)
{
  // Only CameraPose will be updated 
  sptam::msg_KeyFrame msg_kf;

  msg_kf.id_KeyFrame = kf.GetId();

  msg_kf.cameraPose = dsptam::createMsgCameraPose(kf.GetCameraPose());

  return msg_kf;
}



inline sptam::msg_MapPoint createMsgMapPoint(const sptam::Map::Point& mp)
{
  sptam::msg_MapPoint msg_mp;

  msg_mp.id_MapPoint = mp.getMapPointId();
  
  // GetPosition returns Eigen::Vector3d
  msg_mp.position.x = mp.GetPosition().x();
  msg_mp.position.y = mp.GetPosition().y();
  msg_mp.position.z = mp.GetPosition().z();

  // GetNormal returns Eigen::Vector3d
  msg_mp.normal.x = mp.GetNormal().x();
  msg_mp.normal.y = mp.GetNormal().y();
  msg_mp.normal.z = mp.GetNormal().z();
  
  // GetDescriptor returns cv::Mat&
  msg_mp.descriptor = dsptam::descriptor2Vector(mp.GetDescriptor()); 

  // covariance returns  Eigen::Matrix3d& 
  Eigen::Matrix3d covariance = mp.covariance();
  forn(i, 3) forn(j, 3)
  {
    msg_mp.covariance.push_back(covariance(i, j));
  }

  return msg_mp;
}





inline sptam::msg_MapPoint createMsgMapPointToUpdate(const sptam::Map::Point& mp)
{
  // // Only Position will be updated  
  sptam::msg_MapPoint msg_mp;

  msg_mp.id_MapPoint = mp.getMapPointId();
    
  msg_mp.position.x = mp.GetPosition().x();
  msg_mp.position.y = mp.GetPosition().y();
  msg_mp.position.z = mp.GetPosition().z();

  return msg_mp;
}



inline sptam::msg_Meas_id createMsgMeasurementToDelete(const std::pair<size_t, size_t>& idPairToDelete)
{
  sptam::msg_Meas_id msg_meas;

  msg_meas.idKF = std::get<0>(idPairToDelete);
  msg_meas.idMP = std::get<1>(idPairToDelete);

  return msg_meas;
}


inline sptam::msg_Measurement createMsgMeasurement(const Measurement& m, const sptam::Map::KeyFrame& kf, const sptam::Map::Point& mp)
{
    sptam::msg_Measurement msg_meas;

    msg_meas.source = m.GetSource();        
    msg_meas.type = m.GetType();   

    msg_meas.id_kf = kf.GetId();                    
    msg_meas.id_mp = mp.getMapPointId();                    
   
    if ( msg_meas.type == 0 ) //STEREO
    // el metodo se llama GetProjection pero en realidad devuelve (x,y) de feature extraido de la img 
    //           projection_({ featureLeft.x, featureLeft.y, featureRight.x })
    {
      msg_meas.coord_feat_left.x = (m.GetProjection()).at(0);    
      msg_meas.coord_feat_left.y = (m.GetProjection()).at(1);    
      msg_meas.coord_feat_right.x = (m.GetProjection()).at(2);    
      msg_meas.coord_feat_right.y = msg_meas.coord_feat_left.y;
    }
    else if ( msg_meas.type ==1 ) //LEFT
    {
      msg_meas.coord_feat_left.x = (m.GetProjection()).at(0);    
      msg_meas.coord_feat_left.y = (m.GetProjection()).at(1);    
    }
    else //RIGHT
    {
      msg_meas.coord_feat_right.x = (m.GetProjection()).at(0);    
      msg_meas.coord_feat_right.y = (m.GetProjection()).at(1);    
    }



    return msg_meas;
}





inline sptam::msg_mapDiff createMsgMapDiff(   std::list< std::tuple<sptam::Map::SharedKeyFrame, sptam::Map::SharedPoint, Measurement *>  >& l)
{
  sptam::msg_mapDiff msg;
  std::list<std::size_t> id_kf_added;

  for (auto& tuple : l)
  {
    if ( (std::get<2>(tuple))->GetSource() == 0 ) //SRC_TRIANGULATION -> en este caso debo agregar KF y MP
    {
      if ( not (std::find(id_kf_added.begin(), id_kf_added.end(), (std::get<0>(tuple))->GetId()) != id_kf_added.end()) )
      { //el kf no fue agregado al msg todavia
        msg.KeyFramesToAddorUpdate.push_back(createMsgKeyFrame(*(std::get<0>(tuple))));
        id_kf_added.push_back((std::get<0>(tuple))->GetId());
      }
      msg.MapPointsToAddorUpdate.push_back(createMsgMapPoint(*(std::get<1>(tuple))));  // agrego el msg de cada MapPoint al vector
    }
    
    msg.MeasurementsToAddorUpdate.push_back(createMsgMeasurement(*(std::get<2>(tuple)),*(std::get<0>(tuple)), *(std::get<1>(tuple))));
  }    

 // Borro los measurements creados con new
  for (auto& tuple : l)
    delete std::get<2>(tuple);

  // limpio la lista
  l.clear();
  return msg;
}



  // MSG de vuelta
inline sptam::msg_mapDiff createMsgMapRefined(const std::list< sptam::Map::SharedKeyFrame >& list_kf, const std::list< sptam::Map::SharedPoint >& list_mp, std::list<std::pair<size_t, size_t>>& idMeasToDelete )
{
  sptam::msg_mapDiff msg;
  for (auto& kf : list_kf)
    msg.KeyFramesToAddorUpdate.push_back(createMsgKeyFrameToUpdate(*kf));   // agrego el msg de cada KeyFrame al vector solo con los datos modificados
    
  for (auto& mp : list_mp)
    msg.MapPointsToAddorUpdate.push_back(createMsgMapPointToUpdate(*mp));  // agrego el msg de cada MapPoint correspondiente al KF al vector solo con los datos modificados

  
  for (auto& p : idMeasToDelete)
    {msg.MeasurementsToDelete.push_back(createMsgMeasurementToDelete(p));
     // std::cout << "Measurement entre KF= " << msg.MeasurementsToDelete[0].idKF << " y MP=" << msg.MeasurementsToDelete[1].idMP << std::endl;
    }

  idMeasToDelete.clear();

  return msg;
}


/////====================================================================================
/////============================        object creation        =========================
/////====================================================================================

inline cv::KeyPoint fromMsgKeyPoint(const sptam::msg_kp &msg_kp){
  
  cv::KeyPoint keyPoint;
  keyPoint.pt.x = msg_kp.x;
  keyPoint.pt.y = msg_kp.y;
  keyPoint.size = msg_kp.size;
  keyPoint.angle = msg_kp.angle;
  keyPoint.response = msg_kp.response;
  keyPoint.octave = msg_kp.octave;
  keyPoint.class_id = msg_kp.class_id;

  return keyPoint;
}


inline ImageFeatures fromMsgImageFeatures(const std::vector<sptam::msg_feature> msg_feature, /*size_t*/ int MatchingCellSize, const cv::Size& image_size){



  std::vector<cv::KeyPoint> kp;
  cv::Mat descriptors;     //Cada fila corresponde al descriptor de un kp
  cv::Mat descriptors2; 
 
  bool first_row = true;
  
  for ( const auto feat : msg_feature )
  {
    kp.push_back(fromMsgKeyPoint(feat.keypoint));
    cv::Mat aux(feat.descriptor);

    if (first_row){
      descriptors2 =  aux.t();
      first_row = false;
    }
    else
    {
      vconcat(descriptors2,aux.t(),descriptors2);
    }

  }  

  return ImageFeatures(
    image_size,         //const cv::Size& image_size,
    kp,                 //const std::vector<cv::KeyPoint> keyPoints,
    descriptors2,       //const cv::Mat descriptors,
    MatchingCellSize    //const size_t MatchingCellSize
  );

}



inline MapPoint fromMsgMapPoint(sptam::msg_MapPoint msg_MapPoint)
{
  cv::Mat descriptor;
  // Convierto el std::vector en un cv::Mat
  descriptor.push_back(cv::Mat(msg_MapPoint.descriptor));
  

  Eigen::Matrix3d covariance;
  forn(i, 3) forn(j, 3)
  {
    covariance(i, j) = msg_MapPoint.covariance[3*i + j];
  }
  return MapPoint(
          dsptam::geometry_msgsPoint2EigenVector3d(msg_MapPoint.position), 
          dsptam::geometry_msgsPoint2EigenVector3d(msg_MapPoint.normal), 
          descriptor, 
          covariance,
          msg_MapPoint.id_MapPoint);
}




inline Measurement fromMsgMeasurement(const sptam::msg_Measurement msg_meas, const sptam::Map::SharedKeyFrame& kf, const sptam::Map::SharedPoint& mp)
{
  std::vector<double> projection;
  if (msg_meas.type == 0)       //STEREO
    projection = {msg_meas.coord_feat_left.x, msg_meas.coord_feat_left.y, msg_meas.coord_feat_right.x};
  else if (msg_meas.type == 1)  //LEFT
    projection = {msg_meas.coord_feat_left.x, msg_meas.coord_feat_left.y};
  else                          //RIGHT
    projection = {msg_meas.coord_feat_right.x, msg_meas.coord_feat_right.y};

  return Measurement( (Measurement::Type) msg_meas.type, 
                      (Measurement::Source) msg_meas.source, 
                      projection,
                      mp->GetDescriptor()
                     );

}



/////====================================================================================
/////============================     end object creation        =========================
/////====================================================================================



/////====================================================================================
/////============================            Debug                =======================
/////====================================================================================
      
  const std::string red("\033[0;31m"); 
  const std::string reset("\033[0m");
  const std::string green("\033[0;32m");   
  const std::string yellow("\033[0;33m");   
  const std::string blue("\033[0;34m");   
  const std::string cyan("\033[0;36m");   
  inline void print_keyFrame(sptam::Map::KeyFrame &kf, int source){
    
    switch ( source )
    {
    case 1:
      std::cout << dsptam::green << "[Tracker]" 
      << " -  last keyFrame \n   CameraPose = " << kf.GetCameraPose() 
      << "\n   Size keypoints left = " << kf.GetFrameLeft().GetFeatures().GetKeypoints().size() 
      << " descriptors = " << kf.GetFrameLeft().GetFeatures().GetDescriptors().size()
      << "\n   Size keypoints right = " << kf.GetFrameRight().GetFeatures().GetKeypoints().size() 
      << " descriptors = " << kf.GetFrameRight().GetFeatures().GetDescriptors().size()
      << dsptam::reset << std::endl; 
      break;

    case 2:
      std::cout << dsptam::cyan << "[MapNode]" 
      << " -  last keyFrame \n   CameraPose = " << kf.GetCameraPose() 
      << "\n   Size keypoints left = " << kf.GetFrameLeft().GetFeatures().GetKeypoints().size() 
      << " descriptors = " << kf.GetFrameLeft().GetFeatures().GetDescriptors().size()
      << "\n   Size keypoints right = " << kf.GetFrameRight().GetFeatures().GetKeypoints().size() 
      << " descriptors = " << kf.GetFrameRight().GetFeatures().GetDescriptors().size()
      << dsptam::reset << std::endl; 
      break;
    }


  }




   
} //namespace dsptam