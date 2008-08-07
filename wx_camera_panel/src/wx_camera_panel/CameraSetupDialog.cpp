/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */


#include "CameraSetupDialog.h"

#include "wx_topic_display/TopicDisplayDialog.h"

CameraSetupDialog::CameraSetupDialog(wxWindow* parent, ros::node* node, const std::string& imageSubscription, const std::string& ptzStateSubscription, 
									 const std::string& ptzControlCommand, float panMin, float panMax, float tiltMin, float tiltMax, float zoomMin,
									 float zoomMax)
: CameraSetupDialogBase( parent )
, m_ROSNode( node )
{
	m_ImageSubscriptionText->SetValue( wxString::FromAscii( imageSubscription.c_str() ) );
	m_PTZStateSubscriptionText->SetValue( wxString::FromAscii( ptzStateSubscription.c_str() ) );
	m_PTZControlCommandText->SetValue( wxString::FromAscii( ptzControlCommand.c_str() ) );
	
	m_PanMinSpin->SetValue((int)panMin);
	m_PanMaxSpin->SetValue((int)panMax);
	m_TiltMinSpin->SetValue((int)tiltMin);
	m_TiltMaxSpin->SetValue((int)tiltMax);
	m_ZoomMinSpin->SetValue((int)zoomMin);
	m_ZoomMaxSpin->SetValue((int)zoomMax);
	
	if(ptzStateSubscription == std::string("") && ptzControlCommand == std::string(""))
	{
		m_EnablePTZCheck->SetValue(false);
		m_PTZStateSubscriptionText->Enable(false);
		m_PTZStateSubscriptionBrowse->Enable(false);
		m_PTZControlCommandText->Enable(false);
	}
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

void CameraSetupDialog::OnPTZEnableChecked( wxCommandEvent& event)
{
	if(m_EnablePTZCheck->IsChecked())
	{
		m_PTZStateSubscriptionText->SetValue( wxString() );
		m_PTZControlCommandText->SetValue( wxString() );
		m_PTZStateSubscriptionText->Enable(true);
		m_PTZStateSubscriptionBrowse->Enable(true);
		m_PTZControlCommandText->Enable(true);
	}
	else
	{
		m_PTZStateSubscriptionText->SetValue( wxString() );
		m_PTZControlCommandText->SetValue( wxString() );
		m_PTZStateSubscriptionText->Enable(false);
		m_PTZStateSubscriptionBrowse->Enable(false);
		m_PTZControlCommandText->Enable(false);
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

float CameraSetupDialog::GetPanMin()
{
	return (float)m_PanMinSpin->GetValue();
}
float CameraSetupDialog::GetPanMax()
{
	return (float)m_PanMaxSpin->GetValue();
}
float CameraSetupDialog::GetTiltMin()
{
	return (float)m_TiltMinSpin->GetValue();
}
float CameraSetupDialog::GetTiltMax()
{
	return (float)m_TiltMaxSpin->GetValue();
}
float CameraSetupDialog::GetZoomMin()
{
	return (float)m_ZoomMinSpin->GetValue();
}
float CameraSetupDialog::GetZoomMax()
{
	return (float)m_ZoomMaxSpin->GetValue();
}
