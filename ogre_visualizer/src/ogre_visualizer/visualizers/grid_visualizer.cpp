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

#include "grid_visualizer.h"

#include "ogre_tools/grid.h"

#include <Ogre.h>
#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>

#define COLOR_PROPERTY wxT("Color")
#define CELLSIZE_PROPERTY wxT("CellSize")
#define CELLCOUNT_PROPERTY wxT("CellCount")

namespace ogre_vis
{

GridVisualizer::GridVisualizer( Ogre::SceneManager* sceneManager, ros::node* node, rosTFClient* tfClient, const std::string& name, bool enabled )
: VisualizerBase( sceneManager, node, tfClient, name, enabled )
, m_CellSize( 1.0f )
, m_CellCount( 10 )
, m_R( 0.5 )
, m_G( 0.5 )
, m_B( 0.5 )
{
  m_Grid = new ogre_tools::Grid( m_SceneManager, m_CellCount, m_CellSize, m_R, m_G, m_B );

  m_Grid->GetSceneNode()->setVisible( IsEnabled() );
}

GridVisualizer::~GridVisualizer()
{
}

void GridVisualizer::OnEnable()
{
  m_Grid->GetSceneNode()->setVisible( true );
}

void GridVisualizer::OnDisable()
{
  m_Grid->GetSceneNode()->setVisible( false );
}

void GridVisualizer::Create()
{
  m_Grid->Set( m_CellCount, m_CellSize, m_R, m_G, m_B );

  CauseRender();
}

void GridVisualizer::Set( uint32_t cellCount, float cellSize, float r, float g, float b )
{
  m_CellCount = cellCount;
  m_CellSize = cellSize;
  m_R = r;
  m_G = g;
  m_B = b;

  Create();
}

void GridVisualizer::SetCellSize( float size )
{
  Set( m_CellCount, size, m_R, m_G, m_B );
}

void GridVisualizer::SetCellCount( uint32_t count )
{
  Set( count, m_CellSize, m_R, m_G, m_B );
}

void GridVisualizer::SetColor( float r, float g, float b )
{
  Set( m_CellCount, m_CellSize, r, g, b );
}

void GridVisualizer::FillPropertyGrid( wxPropertyGrid* propertyGrid )
{
  wxPGId countProp = propertyGrid->Append( new wxIntProperty( CELLCOUNT_PROPERTY, wxPG_LABEL, m_CellCount ) );
  propertyGrid->SetPropertyAttribute( countProp, wxT("Min"), 1 );
  propertyGrid->SetPropertyAttribute( countProp, wxT("Step"), 1 );
  propertyGrid->SetPropertyEditor( countProp, wxPG_EDITOR(SpinCtrl) );

  wxPGId sizeProp = propertyGrid->Append( new wxFloatProperty( CELLSIZE_PROPERTY, wxPG_LABEL, m_CellSize ) );
  propertyGrid->SetPropertyAttribute( sizeProp, wxT("Min"), 0.0001 );

  propertyGrid->Append( new wxColourProperty( COLOR_PROPERTY, wxPG_LABEL, wxColour( m_R * 255, m_G * 255, m_B * 255 ) ) );
}

void GridVisualizer::PropertyChanged( wxPropertyGridEvent& event )
{
  wxPGProperty* property = event.GetProperty();

  const wxString& name = property->GetName();
  wxVariant value = property->GetValue();

  if ( name == CELLCOUNT_PROPERTY )
  {
    int cellCount = value.GetLong();
    SetCellCount( cellCount );
  }
  else if ( name == CELLSIZE_PROPERTY )
  {
    float cellSize = value.GetDouble();
    SetCellSize( cellSize );
  }
  else if ( name == COLOR_PROPERTY )
  {
    wxColour color;
    color << value;

    SetColor( color.Red() / 255.0f, color.Green() / 255.0f, color.Blue() / 255.0f );
  }
}

void GridVisualizer::GetColor( float& r, float& g, float& b )
{
  r = m_R;
  g = m_G;
  b = m_B;
}

} // namespace ogre_vis
