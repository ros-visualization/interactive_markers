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
#include "properties/property.h"
#include "properties/property_manager.h"

#include "ogre_tools/grid.h"

#include <Ogre.h>
#include <wx/wx.h>
#include <wx/propgrid/propgrid.h>
#include <wx/propgrid/advprops.h>
#include <wx/confbase.h>

#include <boost/bind.hpp>

namespace ogre_vis
{

GridVisualizer::GridVisualizer( Ogre::SceneManager* scene_manager, ros::node* node, rosTFClient* tf_client, const std::string& name )
: VisualizerBase( scene_manager, node, tf_client, name )
, cell_size_( 1.0f )
, cell_count_( 10 )
, color_( 0.5, 0.5, 0.5 )
, cellcount_property_( NULL )
, cellsize_property_( NULL )
, color_property_( NULL )
{
  grid_ = new ogre_tools::Grid( scene_manager_, cell_count_, cell_size_, color_.r_, color_.g_, color_.b_ );
}

GridVisualizer::~GridVisualizer()
{
  delete grid_;
}

void GridVisualizer::onEnable()
{
  grid_->getSceneNode()->setVisible( true );
}

void GridVisualizer::onDisable()
{
  grid_->getSceneNode()->setVisible( false );
}

void GridVisualizer::create()
{
  grid_->set( cell_count_, cell_size_, color_.r_, color_.g_, color_.b_ );

  causeRender();
}

void GridVisualizer::set( uint32_t cell_count, float cell_size, const Color& color )
{
  cell_count_ = cell_count;
  cell_size_ = cell_size;
  color_ = color;

  create();

  if ( cellcount_property_ )
  {
    cellcount_property_->changed();
  }

  if ( cellsize_property_ )
  {
    cellsize_property_->changed();
  }

  if ( color_property_ )
  {
    color_property_->changed();
  }
}

void GridVisualizer::setCellSize( float size )
{
  set( cell_count_, size, color_ );
}

void GridVisualizer::setCellCount( uint32_t count )
{
  set( count, cell_size_, color_ );
}

void GridVisualizer::setColor( const Color& color )
{
  set( cell_count_, cell_size_, color );
}

void GridVisualizer::createProperties()
{
  cellcount_property_ = property_manager_->createProperty<IntProperty>( "Cell Count", property_prefix_, boost::bind( &GridVisualizer::getCellCount, this ),
                                                           boost::bind( &GridVisualizer::setCellCount, this, _1 ), parent_category_, this );
  cellsize_property_ = property_manager_->createProperty<FloatProperty>( "Cell Size", property_prefix_, boost::bind( &GridVisualizer::getCellSize, this ),
                                                             boost::bind( &GridVisualizer::setCellSize, this, _1 ), parent_category_, this );

  color_property_ = property_manager_->createProperty<ColorProperty>( "Color", property_prefix_, boost::bind( &GridVisualizer::getColor, this ),
                                                                      boost::bind( &GridVisualizer::setColor, this, _1 ), parent_category_, this );
}

} // namespace ogre_vis
