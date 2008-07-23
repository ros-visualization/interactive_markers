#include "CameraPanelsGenerated.h"

namespace ros
{
	class node;
}

class CameraSetupDialog : public CameraSetupDialogBase
{
public:
	CameraSetupDialog( wxWindow* parent, ros::node* node, const std::string& imageSubscription, const std::string& ptzStateSubscription, 
					   const std::string& ptzControlCommand );
	~CameraSetupDialog();

	std::string GetImageSubscription();
	std::string GetPTZStateSubscription();
	std::string GetPTZControlCommand();

private:
	void OnOk( wxCommandEvent& event );
	void OnCancel( wxCommandEvent& event );

	void OnImageSubscriptionBrowse( wxCommandEvent& event );
	void OnPTZStateSubscriptionBrowse( wxCommandEvent& event );


	ros::node* m_ROSNode;
};
