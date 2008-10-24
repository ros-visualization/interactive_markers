#include <std_msgs/PointCloud.h>
#include <ros/node.h>
#include <rosTF/rosTF.h>

using namespace std;

class SmallvTransformer : public ros::node
{
public:
  SmallvTransformer() : ros::node("smallv_transformer"), rtf_(*this, true, 30000000000ULL, 3000000000000ULL)
  {

    // -- Get transformations over ros.
    int nWaits = 0;
    cout << "Waiting for transformations from rosTF/frameServer... "; flush(cout);
    while(true) {
      try {
	rtf_.getMatrix("FRAMEID_SMALLV", "FRAMEID_TILT_BASE", ros::Time::now().to_ull());
	break;
      }
      catch (libTF::TransformReference::LookupException & ex) {
	cout << "Waiting " << nWaits++ << endl;
	usleep(500000);
      }
    }
    cout << "Done." << endl; 

    advertise<std_msgs::PointCloud>("full_cloud_smallv", 1000);
    advertise<std_msgs::PointCloud>("videre/cloud_smallv", 1000);
    
    subscribe("videre/cloud", videre_cloud_, &SmallvTransformer::videreCallback, 1);
    subscribe("full_cloud", full_cloud_, &SmallvTransformer::fullCallback, 1);

  }

private:
  rosTFClient rtf_;
  std_msgs::PointCloud full_cloud_;
  std_msgs::PointCloud videre_cloud_;
  std_msgs::PointCloud full_cloud_smallv_;
  std_msgs::PointCloud videre_cloud_smallv_;

  void fullCallback() {
    try {
      full_cloud_smallv_ = rtf_.transformPointCloud("FRAMEID_SMALLV", full_cloud_);
      publish("full_cloud_smallv", full_cloud_smallv_);
      //cout << "Sent a full_cloud_smallv" << endl;
    }
    catch (libTF::TransformReference::ExtrapolateException & ex) {
      cerr << "Extrapolation exception.  Why?   ex.what: " << ex.what() << endl;
    }
  }

  void videreCallback() {
    try {
      videre_cloud_smallv_ = rtf_.transformPointCloud("FRAMEID_SMALLV", videre_cloud_);
      publish("videre/cloud_smallv", videre_cloud_smallv_);
      //cout << "Sent a videre/cloud_smallv" << endl;
    }
    catch (libTF::TransformReference::ExtrapolateException & ex) {
      cerr << "Extrapolation exception.  Why?   ex.what: " << ex.what() << endl;
    }
  }
};


int main(int argc, char **argv) {
  ros::init(argc, argv);
  SmallvTransformer *st = new SmallvTransformer();
  st->spin();

  usleep(500000);
  ros::fini();
}
