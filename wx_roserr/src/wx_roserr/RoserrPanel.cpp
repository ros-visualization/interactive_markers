#include "RoserrPanel.h"
#include "RoserrSetupDialog.h"

BEGIN_DECLARE_EVENT_TYPES()
DECLARE_EVENT_TYPE(EVT_FAKE_REFRESH, wxID_ANY)
END_DECLARE_EVENT_TYPES()

DEFINE_EVENT_TYPE(EVT_FAKE_REFRESH)

RoserrPanel::RoserrPanel( wxWindow* parent )
:
RoserrPanelBase( parent ),
m_Monochrome(false),
m_Wordwrap(false),
m_messageIncomplete(true)
{
	Connect( EVT_FAKE_REFRESH, wxCommandEventHandler( RoserrPanel::WriteError ), NULL, this );
	// ensure a unique node name
	static uint32_t count = 0;
	std::stringstream ss;
	ss << "RoserrPanelNode" << count++;

	m_rosNode = new ros::node( ss.str() );
}

void RoserrPanel::OnEnable( wxCommandEvent& event )
{
	SetEnabled(m_EnableCB->GetValue());
}

void RoserrPanel::OnFontChange( wxFontPickerEvent& event )
{
	std::cout << "change font\n";
	wxTextAttr textAttr;
	long lastPos = m_roserrTC->GetLastPosition();
	if(m_roserrTC->GetStyle(lastPos,textAttr))
	{
		std::cout << "moo\n";
		textAttr.SetFont(event.GetFont());
		m_roserrTC->SetStyle(0,m_roserrTC->GetLastPosition(),textAttr);
	}
}

// TODO: Put stuff in this setup dialog
void RoserrPanel::OnSetup( wxCommandEvent& event )
{
	RoserrSetupDialog dialog(this);
	if (dialog.ShowModal() == wxID_OK)
	{
		wxTextAttr textAttr;
		long lastPos = m_roserrTC->GetLastPosition();
		if(m_roserrTC->GetStyle(lastPos,textAttr))
		{
			m_Monochrome = dialog.GetMonochrome();
			if(m_Monochrome)
				textAttr.SetTextColour(*wxBLACK);
			if(m_Wordwrap != dialog.GetWordWrap())
			{
				textAttr.SetFlags(textAttr.GetFlags()^wxTE_WORDWRAP );//xor me with wordwrap bit (does this work???)
			}

			m_roserrTC->SetStyle(0,m_roserrTC->GetLastPosition(),textAttr);
		}
	}
}

void RoserrPanel::SetEnabled(bool enabled)
{
	if(enabled == m_Enabled)
		return;
	if(enabled)
	{
		m_rosNode->subscribe("/roserr",rosErrMsg, &RoserrPanel::IncomingError,this,50);
	}
	else
	{
		m_rosNode->unsubscribe("/roserr");
	}
	m_EnableCB->SetValue( enabled );
}

void RoserrPanel::IncomingError()
{
	//m_ImageMutex.lock();
	curErrMsg.level = rosErrMsg.level; //because rosErrMsg may change??? how do these buffers work?
	curErrMsg.name = rosErrMsg.name;
	curErrMsg.msg = rosErrMsg.msg;
	curErrMsg.topics = rosErrMsg.topics;
	//m_ImageMutex.unlock();
	m_messageIncomplete = true; //because mutices are even more annoying to debug
	wxCommandEvent evt( EVT_FAKE_REFRESH);
	wxPostEvent( this, evt );
	while(m_messageIncomplete){} //
	std::cout << "Done message\n";
}

void RoserrPanel::WriteError(wxCommandEvent& event)
{
	if(m_Monochrome)
	{
		m_roserrTC->SetDefaultStyle(wxTextAttr(*wxBLACK));
	}
	else
	{
		if(curErrMsg.level < 10) //Fatal
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxRED));
		}
		else if(curErrMsg.level < 20)//Error
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxGREEN));
		}
		else if(curErrMsg.level < 30)//Warn
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxCYAN));
		}
		else if(curErrMsg.level < 100)//Debug
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxBLUE));
		}
		else//General
		{
			m_roserrTC->SetDefaultStyle(wxTextAttr(*wxBLACK));
		}
	}
	wxString line;
	const char* charString=curErrMsg.msg.c_str();
	wxMBConvUTF8 *wxconv= new wxMBConvUTF8();
	line=wxString(wxconv->cMB2WC(charString),wxConvUTF8);
	delete wxconv;
	if(line.length()==0)
	line=wxString(wxString::FromAscii(curErrMsg.msg.c_str()));

	m_roserrTC->AppendText(line);
	m_messageIncomplete = false; //let ros callback that it's time for a new message
}

