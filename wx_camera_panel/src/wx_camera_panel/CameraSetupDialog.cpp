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

#include "rxtools/topic_display_dialog.h"

using namespace rxtools;

CameraSetupDialog::CameraSetupDialog(wxWindow* parent, ros::Node* node, const std::string& name, float panMin, float panMax,
                                     float tiltMin, float tiltMax, float zoomMin, float zoomMax, bool ptz_enabled)
: CameraSetupDialogBase( parent )
, ros_node_( node )
{
	camera_name_->SetValue( wxString::FromAscii( name.c_str() ) );

	pan_min_->SetValue((int)panMin);
	pan_max_->SetValue((int)panMax);
	tilt_min_->SetValue((int)tiltMin);
	tilt_max_->SetValue((int)tiltMax);
	zoom_min_->SetValue((int)zoomMin);
	zoom_max_->SetValue((int)zoomMax);

	if( !ptz_enabled )
	{
		enable_ptz_->SetValue(false);
	}
}

CameraSetupDialog::~CameraSetupDialog()
{
}

void CameraSetupDialog::onOk( wxCommandEvent& event )
{
	EndModal( wxOK );
}

void CameraSetupDialog::onCancel( wxCommandEvent& event )
{
	EndModal( wxCANCEL );
}

bool CameraSetupDialog::getPTZEnabled()
{
  return enable_ptz_->IsChecked();
}

std::string CameraSetupDialog::getName()
{
	return (const char*)camera_name_->GetValue().mb_str();
}

float CameraSetupDialog::getPanMin()
{
	return (float)pan_min_->GetValue();
}
float CameraSetupDialog::getPanMax()
{
	return (float)pan_max_->GetValue();
}
float CameraSetupDialog::getTiltMin()
{
	return (float)tilt_min_->GetValue();
}
float CameraSetupDialog::getTiltMax()
{
	return (float)tilt_max_->GetValue();
}
float CameraSetupDialog::getZoomMin()
{
	return (float)zoom_min_->GetValue();
}
float CameraSetupDialog::getZoomMax()
{
	return (float)zoom_max_->GetValue();
}
