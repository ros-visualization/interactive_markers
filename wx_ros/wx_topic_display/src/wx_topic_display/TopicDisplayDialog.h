#pragma once

#include "GenTopicDisplay.h"

#include <vector>

namespace ros
{
	class node;
}

class TopicDisplay;

class TopicDisplayDialog : public GenTopicDisplayDialog
{
public:
	TopicDisplayDialog( wxWindow* parent, ros::node* node, bool multiselect );
	~TopicDisplayDialog();
	
	void getSelection( std::vector< std::string >& selection );

private:
	void onOK( wxCommandEvent& event );
	void onCancel( wxCommandEvent& event );

	TopicDisplay* m_TopicDisplayPanel;
};
