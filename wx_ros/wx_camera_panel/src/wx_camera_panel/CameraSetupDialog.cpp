#include "CameraSetupDialog.h"

#include "wx_topic_display/TopicDisplayDialog.h"

CameraSetupDialog::CameraSetupDialog(wxWindow* parent, ros::node* node, const std::string& imageSubscription, const std::string& ptzStateSubscription, 
									 const std::string& ptzControlCommand)
: CameraSetupDialogBase( parent )
, m_ROSNode( node )
{
	m_ImageSubscriptionText->SetValue( wxString::FromAscii( imageSubscription.c_str() ) );
	m_PTZStateSubscriptionText->SetValue( wxString::FromAscii( ptzStateSubscription.c_str() ) );
	m_PTZControlCommandText->SetValue( wxString::FromAscii( ptzControlCommand.c_str() ) );
}

CameraSetupDialog::~CameraSetupDialog()
{
}

void CameraSetupDialog::OnOk( wxCommandEvent& event )
{
	EndModal( wxID_OK );
}

void CameraSetupDialog::OnCancel( wxCommandEvent& event )
{
	EndModal( wxID_CANCEL );
}

void CameraSetupDialog::OnImageSubscriptionBrowse( wxCommandEvent& event )
{
	TopicDisplayDialog dialog(this, m_ROSNode, false);

	if (dialog.ShowModal() == wxID_OK)
	{
		std::vector<std::string> selection;
		dialog.getSelection(selection);

		if (!selection.empty())
		{
			m_ImageSubscriptionText->SetValue( wxString::FromAscii( selection[0].c_str() ) );
		}
	}
}

void CameraSetupDialog::OnPTZStateSubscriptionBrowse( wxCommandEvent& event )
{
	TopicDisplayDialog dialog(this, m_ROSNode, false);

	if (dialog.ShowModal() == wxID_OK)
	{
		std::vector<std::string> selection;
		dialog.getSelection(selection);

		if (!selection.empty())
		{
			m_PTZStateSubscriptionText->SetValue( wxString::FromAscii( selection[0].c_str() ) );
		}
	}
}

std::string CameraSetupDialog::GetImageSubscription()
{
	return (const char*)m_ImageSubscriptionText->GetValue().mb_str();
}

std::string CameraSetupDialog::GetPTZStateSubscription()
{
	return (const char*)m_PTZStateSubscriptionText->GetValue().mb_str();
}

std::string CameraSetupDialog::GetPTZControlCommand()
{
	return (const char*)m_PTZControlCommandText->GetValue().mb_str();
}
