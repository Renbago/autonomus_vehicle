/*
 * Software License Agreement (BSD License)
 *
 * Copyright (c) 2017, Poznan University of Technology
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
 *     * Neither the name of the Poznan University of Technology nor the names
 *       of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
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

/*
 * Author: Mateusz Przybyla
 */

#include "obstacle_detector/displays/segment_visual.h"

namespace obstacles_display
{

SegmentVisual::SegmentVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_line_ = parent_node->createChildSceneNode();
  frame_node_velocity_ = parent_node->createChildSceneNode();
  frame_node_text_ = parent_node->createChildSceneNode();

  line_.reset(new rviz::BillboardLine(scene_manager_, frame_node_line_));
  velocity_.reset(new rviz::Arrow(scene_manager_, frame_node_line_));
  const auto color = Ogre::ColourValue(0.0f, 0.5f, 1.0f);
  velocity_->setColor(color);
  center_position_ = Ogre::Vector3(0.0, 0.0, 0.0);
  text_ = new rviz::MovableText("Segment");
  text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  text_->setColor(color);
  frame_node_text_->attachObject(text_);
}

SegmentVisual::~SegmentVisual() {
  scene_manager_->destroySceneNode(frame_node_line_);
  scene_manager_->destroySceneNode(frame_node_velocity_);
  scene_manager_->destroySceneNode(frame_node_text_);
  delete text_;
}

void SegmentVisual::setData(const obstacle_detector::SegmentObstacle& segment) {
  Ogre::Vector3 p1(segment.first_point.x, segment.first_point.y, 0.0);
  Ogre::Vector3 p2(segment.last_point.x, segment.last_point.y, 0.0);
  line_->addPoint(p1);
  line_->addPoint(p2);

  center_position_ = (p1 + p2) / 2;
  const auto velocity = Ogre::Vector3(segment.first_velocity.x + segment.last_velocity.x, segment.first_velocity.y + segment.last_velocity.y, segment.first_velocity.z + segment.last_velocity.z);
  const auto speed = sqrt(pow(velocity.x, 2.0) + pow(velocity.y, 2.0));
  velocity_->setPosition(center_position_);
  velocity_->setDirection(velocity);
  velocity_->setScale(Ogre::Vector3(speed));

  //text_->setLocalTranslation(Ogre::Vector3(0.5,0,0));
  text_->setCharacterHeight(std::max(0.1, std::min(0.5, sqrt(pow(p1.x - p2.x, 2.0) + pow(p1.y - p2.y, 2.0)) / 2.0)));
  text_->setCaption(std::to_string(segment.uid));
}

void SegmentVisual::setFramePose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
  frame_node_line_->setPosition(position);
  frame_node_text_->setPosition(position + orientation * center_position_);
  frame_node_line_->setOrientation(orientation);
  //frame_node_text_->setOrientation(orientation); // Has no effect
}

void SegmentVisual::setColor(float r, float g, float b, float a) {
  line_->setColor(r, g, b, a);
}

void SegmentVisual::setWidth(float w) {
  line_->setLineWidth(w);
}

} // end namespace obstacles_display

