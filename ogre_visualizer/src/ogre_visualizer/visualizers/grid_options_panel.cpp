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
 
#include "grid_options_panel.h"
#include "grid_visualizer.h"

GridOptionsPanel::GridOptionsPanel( wxWindow* parent, GridVisualizer* visualizer )
: GridOptionsPanelGenerated( parent )
, m_Visualizer( visualizer )
{
  uint32_t cellCount = m_Visualizer->GetCellCount();
  float cellSize = m_Visualizer->GetCellSize();
  float r, g, b;
  m_Visualizer->GetColor( r, g, b );
  
  *m_CellCount << (int)cellCount;
  *m_CellSize << cellSize;
  *m_R << r;
  *m_G << g;
  *m_B << b;
}

void GridOptionsPanel::OnKillFocus( wxFocusEvent& event )
{
  OptionChanged( event.GetId() );
}

void GridOptionsPanel::OnTextEnter( wxCommandEvent& event )
{
  OptionChanged( event.GetId() );
}

void GridOptionsPanel::OptionChanged( int windowId )
{
  if ( windowId == m_CellCount->GetId() )
  {
    long cellCount;
    if ( m_CellCount->GetValue().ToLong( &cellCount ) && cellCount > 0 )
    {
      m_Visualizer->SetCellCount( cellCount );
      m_CellCount->SetBackgroundColour( wxNullColour );
    }
    else
    {
      m_CellCount->SetBackgroundColour( *wxRED );
    }
  }
  else if ( windowId == m_CellSize->GetId() )
  {
    double cellSize;
    if ( m_CellSize->GetValue().ToDouble( &cellSize ) && cellSize > 0.0 )
    {
      m_Visualizer->SetCellSize( cellSize );
      m_CellSize->SetBackgroundColour( wxNullColour );
    }
    else
    {
      m_CellSize->SetBackgroundColour( *wxRED );
    }
  }
  else if ( windowId == m_R->GetId() || windowId == m_G->GetId() || windowId == m_B->GetId() )
  {
    float origR, origG, origB;
    double r, g, b;
    
    m_Visualizer->GetColor( origR, origG, origB );
    
    // Reset our background color to the default 
    m_R->SetBackgroundColour( wxNullColour );
    m_G->SetBackgroundColour( wxNullColour );
    m_B->SetBackgroundColour( wxNullColour );
    
    if ( !m_R->GetValue().ToDouble( &r ) || r < 0.0 || r > 1.0 )
    {
      m_R->SetBackgroundColour( *wxRED );
      r = origR;
    }
    
    if ( !m_G->GetValue().ToDouble( &g ) || g < 0.0 || g > 1.0 )
    {
      m_G->SetBackgroundColour( *wxRED );
      g = origG;
    }
    
    if ( !m_B->GetValue().ToDouble( &b ) || b < 0.0 || b > 1.0 )
    {
      m_B->SetBackgroundColour( *wxRED );
      b = origB;
    }
    
    m_Visualizer->SetColor( r, g, b );
  }
  
} 
