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

#include "property.h"
#include "ros_topic_property.h"

#include <ros/node.h>

#include <wx/propgrid/advprops.h>
#include <wx/config.h>



namespace ogre_vis
{

void BoolProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( parent_, new wxBoolProperty( name_, prefix_ + name_, get() ) );
    property_->SetAttribute( wxPG_BOOL_USE_CHECKBOX, true );
  }
  else
  {
    grid_->SetPropertyValue(property_, get());
  }
}

void BoolProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( var.GetBool() );
}

void BoolProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, (int)get() );
}

void BoolProperty::loadFromConfig( wxConfigBase* config )
{
  bool val;
  config->Read( prefix_ + name_, &val, get() );

  set( val );
}

void IntProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( parent_, new wxIntProperty( name_, prefix_ + name_, get() ) );
  }
  else
  {
    grid_->SetPropertyValue(property_, (long)get());
  }
}

void IntProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( var.GetLong() );
}

void IntProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, (int)get() );
}

void IntProperty::loadFromConfig( wxConfigBase* config )
{
  long val;
  config->Read( prefix_ + name_, &val, get() );

  set( val );
}

void FloatProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( parent_, new wxFloatProperty( name_, prefix_ + name_, get() ) );
  }
  else
  {
    grid_->SetPropertyValue(property_, (double)get());
  }
}

void FloatProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( var.GetDouble() );
}

void FloatProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, (float)get() );
}

void FloatProperty::loadFromConfig( wxConfigBase* config )
{
  double val;
  config->Read( prefix_ + name_, &val, get() );

  set( val );
}

void StringProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( parent_, new wxStringProperty( name_, prefix_ + name_, wxString::FromAscii( get().c_str() ) ) );
  }
  else
  {
    grid_->SetPropertyValue(property_, wxString::FromAscii( get().c_str() ));
  }
}

void StringProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( (const char*)var.GetString().fn_str() );
}

void StringProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, wxString::FromAscii( get().c_str() ) );
}

void StringProperty::loadFromConfig( wxConfigBase* config )
{
  wxString val;
  config->Read( prefix_ + name_, &val, wxString::FromAscii( get().c_str() ) );

  set( (const char*)val.fn_str() );
}

void ROSTopicStringProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( parent_, new ROSTopicProperty( ros::node::instance(), name_, prefix_ + name_, wxString::FromAscii( get().c_str() ) ) );
  }
  else
  {
    grid_->SetPropertyValue(property_, wxString::FromAscii( get().c_str() ));
  }
}

void ColorProperty::writeToGrid()
{
  if ( !property_ )
  {
    Color c = get();
    property_ = grid_->AppendIn( parent_, new wxColourProperty( name_, prefix_ + name_, wxColour( c.r_ * 255, c.g_ * 255, c.b_ * 255 ) ) );
  }
  else
  {
    Color c = get();
    wxVariant var;
    var << wxColour( c.r_ * 255, c.g_ * 255, c.b_ * 255 );
    grid_->SetPropertyValue(property_, var);
  }
}

void ColorProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  wxColour col;
  col << var;
  set( Color( col.Red() / 255.0f, col.Green() / 255.0f, col.Blue() / 255.0f ) );
}

void ColorProperty::saveToConfig( wxConfigBase* config )
{
  Color c = get();

  config->Write( prefix_ + name_ + wxT("R"), c.r_ );
  config->Write( prefix_ + name_ + wxT("G"), c.g_ );
  config->Write( prefix_ + name_ + wxT("B"), c.b_ );
}

void ColorProperty::loadFromConfig( wxConfigBase* config )
{
  Color c = get();
  double r, g, b;
  config->Read( prefix_ + name_ + wxT("R"), &r, c.r_ );
  config->Read( prefix_ + name_ + wxT("G"), &g, c.g_ );
  config->Read( prefix_ + name_ + wxT("B"), &b, c.b_ );

  set( Color( r, g, b ) );
}

void EnumProperty::addOption( const std::string& name, int value )
{
  wxPGChoices& choices = grid_->GetPropertyChoices( property_ );
  choices.Add( wxString::FromAscii( name.c_str() ), value );

  writeToGrid();
}

void EnumProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( parent_, new wxEnumProperty( name_, prefix_ + name_ ) );
  }
  else
  {
    grid_->SetPropertyValue(property_, (long)get());
  }
}

void EnumProperty::readFromGrid()
{
  wxVariant var = property_->GetValue();
  set( var.GetLong() );
}

void EnumProperty::saveToConfig( wxConfigBase* config )
{
  config->Write( prefix_ + name_, (int)get() );
}

void EnumProperty::loadFromConfig( wxConfigBase* config )
{
  long val;
  config->Read( prefix_ + name_, &val, get() );

  set( val );
}

void CategoryProperty::writeToGrid()
{
  if ( !property_ )
  {
    if ( parent_ )
    {
      property_ = grid_->AppendIn( parent_, new wxPropertyCategory( name_ ) );
    }
    else
    {
      property_ = grid_->Append( new wxPropertyCategory( name_ ) );
    }
  }
}

}
