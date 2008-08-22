#include "RoserrPanel.h"
#include "RoserrSetupDialog.h"


BEGIN_DECLARE_EVENT_TYPES()
DECLARE_EVENT_TYPE(EVT_FAKE_REFRESH, wxID_ANY)
END_DECLARE_EVENT_TYPES()

DEFINE_EVENT_TYPE(EVT_FAKE_REFRESH)

RoserrPanel::RoserrPanel( wxWindow* parent )
:
RoserrPanelBase( parent ),
m_Enabled(false),
m_Monochrome(false),
m_messageIncomplete(true),
m_subscription("/rosout")
{
	Connect( EVT_FAKE_REFRESH, wxCommandEventHandler( RoserrPanel::WriteError ), NULL, this );
	// ensure a unique node name
	static uint32_t count = 0;
	std::stringstream ss;
	ss << "RoserrPanelNode" << count++;

	m_rosNode = new ros::node( ss.str() );
}

void RoserrPanel::OnEnable( wxCommandEvent& )
{
	SetEnabled(m_EnableCB->GetValue());
}

void RoserrPanel::OnFontChange( wxFontPickerEvent& event )
{
	wxTextAttr textAttr;
	long lastPos = m_roserrTC->GetLastPosition() -1;
	m_roserrTC->GetStyle(lastPos,textAttr);
	textAttr.SetFont(event.GetFont());
	m_roserrTC->SetStyle(0,m_roserrTC->GetLastPosition(),textAttr);
	m_roserrTC->SetDefaultStyle(textAttr);
}

void RoserrPanel::OnSetup( wxCommandEvent& )
{
	RoserrSetupDialog dialog(this,m_rosNode,m_subscription, m_Monochrome);
	if (dialog.ShowModal() == wxID_OK)
	{
		wxTextAttr textAttr;
		long lastPos = m_roserrTC->GetLastPosition();
		m_roserrTC->GetStyle(lastPos,textAttr);
		m_Monochrome = dialog.GetMonochrome();

		if(m_Monochrome)
		{
			textAttr.SetTextColour(*wxBLACK);
		}
		else
		{
			textAttr.SetFont(m_FontFP->GetSelectedFont());
		}

		if(dialog.GetSubscription() != m_subscription)
		{
			if(m_Enabled)
			{
				SetEnabled(false);
				m_subscription = dialog.GetSubscription();
				SetEnabled(true);
			}
		}

		m_roserrTC->SetStyle(0,m_roserrTC->GetLastPosition(),textAttr);
		m_roserrTC->SetDefaultStyle(textAttr);
	}
}

void RoserrPanel::SetEnabled(bool enabled)
{
	if(enabled == m_Enabled)
		return;

	m_Enabled = enabled;
	if(enabled)
	{
		m_rosNode->subscribe(m_subscription,rosErrMsg, &RoserrPanel::IncomingError,this,50);
	}
	else
	{
		m_rosNode->unsubscribe(m_subscription);
	}
	m_EnableCB->SetValue( enabled );
}

void RoserrPanel::IncomingError()
{
	m_messageIncomplete = true; //because mutices are even more annoying to debug
	wxCommandEvent evt( EVT_FAKE_REFRESH);
	wxPostEvent( this, evt );
	while(m_messageIncomplete){} //wait until main thred is done with the message
}

void RoserrPanel::WriteError(wxCommandEvent&)
{
	if(m_Monochrome)
	{
		m_roserrTC->SetDefaultStyle(wxTextAttr(*wxBLACK));
	}
	else
	{
		if(rosErrMsg.level < 10) //Fatal
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxRED));
		}
		else if(rosErrMsg.level < 20)//Error
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxGREEN));
		}
		else if(rosErrMsg.level < 30)//Warn
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxCYAN));
		}
		else if(rosErrMsg.level < 100)//Debug
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxBLUE));
		}
		else//General
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxBLACK));
		}
	}

	m_roserrTC->AppendText(wxString::FromAscii(rosErrMsg.name.c_str()) + wxString::FromAscii(": ") + wxString::FromAscii(rosErrMsg.msg.c_str() ));
	m_messageIncomplete = false; //let ros callback know that it's time for a new message
}

