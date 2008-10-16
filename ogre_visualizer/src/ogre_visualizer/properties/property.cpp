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

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
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

void IntProperty::setMin( int min )
{
  property_->SetAttribute( wxT("Min"), min );
}

void IntProperty::setMax( int max )
{
  property_->SetAttribute( wxT("Max"), max );
}

void IntProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( parent_, new wxIntProperty( name_, prefix_ + name_, get() ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
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

void FloatProperty::setMin( float min )
{
  property_->SetAttribute( wxT("Min"), min );
}

void FloatProperty::setMax( float max )
{
  property_->SetAttribute( wxT("Max"), max );
}


void FloatProperty::writeToGrid()
{
  if ( !property_ )
  {
    property_ = grid_->AppendIn( parent_, new wxFloatProperty( name_, prefix_ + name_, get() ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
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

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
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

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
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

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
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

    if ( !hasSetter() )
    {
      grid_->DisableProperty( property_ );
    }
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

void Vector3Property::writeToGrid()
{
  if ( !x_ )
  {
    Ogre::Vector3 v = get();

    wxString composed_name = name_ + wxT("Composed");
    wxPGProperty* composed_parent = grid_->AppendIn( parent_, new wxStringProperty( name_, prefix_ + composed_name, wxT("<composed>")) );
    composed_parent->SetClientData( this );

    x_ = grid_->AppendIn( composed_parent, new wxFloatProperty( wxT("X"), prefix_ + name_ + wxT("X"), v.x ) );
    y_ = grid_->AppendIn( composed_parent, new wxFloatProperty( wxT("Y"), prefix_ + name_ + wxT("Y"), v.y ) );
    z_ = grid_->AppendIn( composed_parent, new wxFloatProperty( wxT("Z"), prefix_ + name_ + wxT("Z"), v.z ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( composed_parent );
      grid_->DisableProperty( x_ );
      grid_->DisableProperty( y_ );
      grid_->DisableProperty( z_ );
    }

    grid_->Collapse( composed_parent );
  }
  else
  {
    Ogre::Vector3 v = get();
    grid_->SetPropertyValue(x_, v.x);
    grid_->SetPropertyValue(y_, v.y);
    grid_->SetPropertyValue(z_, v.z);
  }
}

void Vector3Property::readFromGrid()
{
  set( Ogre::Vector3( x_->GetValue().GetDouble(), y_->GetValue().GetDouble(), z_->GetValue().GetDouble() ) );
}

void Vector3Property::saveToConfig( wxConfigBase* config )
{
  Ogre::Vector3 v = get();

  config->Write( prefix_ + name_ + wxT("X"), v.x );
  config->Write( prefix_ + name_ + wxT("Y"), v.y );
  config->Write( prefix_ + name_ + wxT("Z"), v.z );
}

void Vector3Property::loadFromConfig( wxConfigBase* config )
{
  Ogre::Vector3 v = get();
  double x, y, z;
  config->Read( prefix_ + name_ + wxT("X"), &x, v.x );
  config->Read( prefix_ + name_ + wxT("Y"), &y, v.y );
  config->Read( prefix_ + name_ + wxT("Z"), &z, v.z );

  set( Ogre::Vector3( x, y, z ) );
}

void Vector3Property::setPGClientData()
{
  x_->SetClientData( this );
  y_->SetClientData( this );
  z_->SetClientData( this );
}

void QuaternionProperty::writeToGrid()
{
  if ( !x_ )
  {
    Ogre::Quaternion q = get();

    wxString composed_name = name_ + wxT("Composed");
    wxPGProperty* composed_parent = grid_->AppendIn( parent_, new wxStringProperty( name_, prefix_ + composed_name, wxT("<composed>")) );
    composed_parent->SetClientData( this );

    x_ = grid_->AppendIn( composed_parent, new wxFloatProperty( wxT("X"), prefix_ + name_ + wxT("X"), q.x ) );
    y_ = grid_->AppendIn( composed_parent, new wxFloatProperty( wxT("Y"), prefix_ + name_ + wxT("Y"), q.y ) );
    z_ = grid_->AppendIn( composed_parent, new wxFloatProperty( wxT("Z"), prefix_ + name_ + wxT("Z"), q.z ) );
    w_ = grid_->AppendIn( composed_parent, new wxFloatProperty( wxT("W"), prefix_ + name_ + wxT("W"), q.z ) );

    if ( !hasSetter() )
    {
      grid_->DisableProperty( composed_parent );
      grid_->DisableProperty( x_ );
      grid_->DisableProperty( y_ );
      grid_->DisableProperty( z_ );
      grid_->DisableProperty( w_ );
    }

    grid_->Collapse( composed_parent );
  }
  else
  {
    Ogre::Quaternion q = get();
    grid_->SetPropertyValue(x_, q.x);
    grid_->SetPropertyValue(y_, q.y);
    grid_->SetPropertyValue(z_, q.z);
    grid_->SetPropertyValue(w_, q.w);
  }
}

void QuaternionProperty::readFromGrid()
{
  set( Ogre::Quaternion( x_->GetValue().GetDouble(), y_->GetValue().GetDouble(), z_->GetValue().GetDouble(), w_->GetValue().GetDouble() ) );
}

void QuaternionProperty::saveToConfig( wxConfigBase* config )
{
  Ogre::Quaternion q = get();

  config->Write( prefix_ + name_ + wxT("X"), q.x );
  config->Write( prefix_ + name_ + wxT("Y"), q.y );
  config->Write( prefix_ + name_ + wxT("Z"), q.z );
  config->Write( prefix_ + name_ + wxT("W"), q.w );
}

void QuaternionProperty::loadFromConfig( wxConfigBase* config )
{
  Ogre::Quaternion q = get();
  double x, y, z, w;
  config->Read( prefix_ + name_ + wxT("X"), &x, q.x );
  config->Read( prefix_ + name_ + wxT("Y"), &y, q.y );
  config->Read( prefix_ + name_ + wxT("Z"), &z, q.z );
  config->Read( prefix_ + name_ + wxT("W"), &w, q.w );

  set( Ogre::Quaternion( x, y, z, w ) );
}

void QuaternionProperty::setPGClientData()
{
  x_->SetClientData( this );
  y_->SetClientData( this );
  z_->SetClientData( this );
  w_->SetClientData( this );
}

}
