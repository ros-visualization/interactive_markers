#include "rosout_setup_dialog.h"

#include "wx_topic_display/TopicDisplayDialog.h"

namespace wx_rosout
{

RosoutSetupDialog::RosoutSetupDialog( wxWindow* parent, ros::node* node, const std::string& topic, uint32_t buffer_size)
: RosoutSetupDialogBase( parent )
{
	topic_->SetValue( wxString::FromAscii( topic.c_str() ) );
	buffer_size_spinner_->SetValue( buffer_size );

	ros_node_ = node;
}

void RosoutSetupDialog::onCancel( wxCommandEvent& event )
{
	EndModal( wxCANCEL );
}

void RosoutSetupDialog::onOk( wxCommandEvent& event )
{
	EndModal( wxOK );
}

std::string RosoutSetupDialog::getTopic()
{
	return (const char*)topic_->GetValue().mb_str();
}

uint32_t RosoutSetupDialog::getBufferSize()
{
  return buffer_size_spinner_->GetValue();
}

void RosoutSetupDialog::onTopicBrowse( wxCommandEvent& event )
{
	TopicDisplayDialog dialog(this, ros_node_, false);
	if (dialog.ShowModal() == wxID_OK)
	{
		std::vector<std::string> selection;
		dialog.getSelection(selection);
		if (!selection.empty())
		{
			topic_->SetValue( wxString::FromAscii( selection[0].c_str() ) );
		}
	}
}

} // namespace wx_rosout
