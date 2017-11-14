#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include "SPTAMInterface.hpp"

namespace sptam
{
  class SPTAMNodelet : public nodelet::Nodelet
  {
    public:

      void onInit() {
        NODELET_DEBUG("Initializing sptam nodelet...");

        sptam_interface_.reset( new sptam::SPTAMInterface( getNodeHandle(), getPrivateNodeHandle() ) );
      }

    private:

      std::unique_ptr<sptam::SPTAMInterface> sptam_interface_;
  };
}

PLUGINLIB_EXPORT_CLASS(sptam::SPTAMNodelet, nodelet::Nodelet)
