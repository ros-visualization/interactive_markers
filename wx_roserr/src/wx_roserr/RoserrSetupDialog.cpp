#include "RoserrSetupDialog.h"

#include "wx_topic_display/TopicDisplayDialog.h"

RoserrSetupDialog::RoserrSetupDialog( wxWindow* parent, ros::node* node, const std::string& subscription, bool monochrome)
:
RoserrSetupDialogBase( parent )
{
	m_rostopicL->SetValue( wxString::FromAscii( subscription.c_str() ) );
	m_MonochromeCB->SetValue(monochrome);
	m_ROSNode = node;
}

void RoserrSetupDialog::OnCancel( wxCommandEvent& event )
{
	EndModal( wxID_CANCEL );
}

void RoserrSetupDialog::OnOk( wxCommandEvent& event )
{
	EndModal( wxID_OK );
}

bool RoserrSetupDialog::GetMonochrome()
{
	return m_MonochromeCB->GetValue();
}

std::string RoserrSetupDialog::GetSubscription()
{
	return (const char*)m_rostopicL->GetValue().mb_str();
}

void RoserrSetupDialog::OnClick( wxCommandEvent& event )
{
	TopicDisplayDialog dialog(this, m_ROSNode, false);
	if (dialog.ShowModal() == wxID_OK)
	{
		std::vector<std::string> selection;
		dialog.getSelection(selection);
		if (!selection.empty())
		{
			m_rostopicL->SetValue( wxString::FromAscii( selection[0].c_str() ) );
		}
	}
}
