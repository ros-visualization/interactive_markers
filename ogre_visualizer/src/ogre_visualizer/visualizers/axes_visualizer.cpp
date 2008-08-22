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

#include "axes_visualizer.h"
#include "../common.h"

#include "ogre_tools/axes.h"

#include <Ogre.h>
#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>

#define LENGTH_PROPERTY wxT("Length")
#define RADIUS_PROPERTY wxT("Radius")

namespace ogre_vis
{

AxesVisualizer::AxesVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
: VisualizerBase( sceneManager, node, tfClient, name, enabled )
, length_( 1.0 )
, radius_( 0.1 )
{
  axes_ = new ogre_tools::Axes( scene_manager_, NULL, length_, radius_ );

  axes_->GetSceneNode()->setVisible( IsEnabled() );

  Ogre::Quaternion orient( Ogre::Quaternion::IDENTITY );
  RobotToOgre( orient );
  axes_->SetOrientation( orient );

  if ( IsEnabled() )
  {
    OnEnable();
  }
  else
  {
    OnDisable();
  }
}

AxesVisualizer::~AxesVisualizer()
{
}

void AxesVisualizer::OnEnable()
{
  axes_->GetSceneNode()->setVisible( true );
}

void AxesVisualizer::OnDisable()
{
  axes_->GetSceneNode()->setVisible( false );
}

void AxesVisualizer::Create()
{
  axes_->Set( length_, radius_ );

  CauseRender();
}

void AxesVisualizer::Set( float length, float radius )
{
  length_ = length;
  radius_ = radius;

  Create();
}

void AxesVisualizer::FillPropertyGrid( wxPropertyGrid* propertyGrid )
{
  wxPGId prop = propertyGrid->Append( new wxFloatProperty( LENGTH_PROPERTY, wxPG_LABEL, length_ ) );
  propertyGrid->SetPropertyAttribute( prop, wxT("Min"), 0.0001 );

  prop = propertyGrid->Append( new wxFloatProperty( RADIUS_PROPERTY, wxPG_LABEL, radius_ ) );
  propertyGrid->SetPropertyAttribute( prop, wxT("Min"), 0.0001 );
}

void AxesVisualizer::PropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  const wxString& name = property->GetName();
  wxVariant value = property->GetValue();

  if ( name == LENGTH_PROPERTY )
  {
    float length = value.GetDouble();
    Set( length, radius_ );
  }
  else if ( name == RADIUS_PROPERTY )
  {
    float radius = value.GetDouble();
    Set( length_, radius );
  }
}

} // namespace ogre_vis
