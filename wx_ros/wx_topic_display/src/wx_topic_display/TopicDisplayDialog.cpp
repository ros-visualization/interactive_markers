#include "TopicDisplayDialog.h"
#include "TopicDisplay.h"

// ROS includes
#include "ros/node.h"

TopicDisplayDialog::TopicDisplayDialog(wxWindow* parent, ros::node* node, bool multiselect)
: GenTopicDisplayDialog( parent )
{
	m_TopicDisplayPanel = new TopicDisplay( m_TreePanel, node, m_TreePanel->GetSize() );
	m_TopicDisplayPanel->setMultiselectAllowed( multiselect );
}

TopicDisplayDialog::~TopicDisplayDialog()
{
}

void TopicDisplayDialog::onOK( wxCommandEvent& event )
{
	EndModal( wxID_OK );
}

void TopicDisplayDialog::onCancel( wxCommandEvent& event )
{
	EndModal( wxID_CANCEL );
}

void TopicDisplayDialog::getSelection( std::vector<std::string>& selection )
{
	selection = m_TopicDisplayPanel->getSelectedTopics();
}
