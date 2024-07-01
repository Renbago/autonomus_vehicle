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

#include "obstacle_detector/displays/circle_visual.h"

namespace obstacles_display
{

CircleVisual::CircleVisual(Ogre::SceneManager* scene_manager, Ogre::SceneNode* parent_node) {
  scene_manager_ = scene_manager;
  frame_node_obstacle_ = parent_node->createChildSceneNode();
  frame_node_margin_ = parent_node->createChildSceneNode();
  frame_node_velocity_ = parent_node->createChildSceneNode();
  frame_node_text_ = parent_node->createChildSceneNode();

  obstacle_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_obstacle_));
  margin_.reset(new rviz::Shape(rviz::Shape::Cylinder, scene_manager_, frame_node_margin_));
  velocity_.reset(new rviz::Arrow(scene_manager_, frame_node_margin_));
  const auto color = Ogre::ColourValue(1.0f, 0.5f, 0.0f);
  velocity_->setColor(color);
  center_position_ = Ogre::Vector3(0.0, 0.0, 0.0);
  text_ = new rviz::MovableText("Circle");
  text_->setTextAlignment(rviz::MovableText::H_CENTER, rviz::MovableText::V_CENTER);
  frame_node_text_->attachObject(text_);
}

CircleVisual::~CircleVisual() {
  scene_manager_->destroySceneNode(frame_node_obstacle_);
  scene_manager_->destroySceneNode(frame_node_margin_);
  scene_manager_->destroySceneNode(frame_node_velocity_);
  scene_manager_->destroySceneNode(frame_node_text_);
  delete text_;
}

void CircleVisual::setData(const obstacle_detector::CircleObstacle& circle) {
  center_position_ = Ogre::Vector3(circle.center.x, circle.center.y, 0.25);
  obstacle_->setPosition(center_position_);

  Ogre::Vector3 true_scale(2.0 * circle.true_radius, 0.1, 2.0 * circle.true_radius);
  obstacle_->setScale(true_scale);

  Ogre::Vector3 pos2(circle.center.x, circle.center.y, 0.1);
  margin_->setPosition(pos2);

  Ogre::Vector3 scale(2.0 * circle.radius, 0.2, 2.0 * circle.radius);
  margin_->setScale(scale);

  const auto speed = sqrt(pow(circle.velocity.x, 2.0) + pow(circle.velocity.y, 2.0));
  velocity_->setPosition(center_position_);
  velocity_->setDirection(Ogre::Vector3(circle.velocity.x, circle.velocity.y, circle.velocity.z));
  velocity_->setScale(Ogre::Vector3(speed));

  text_->setCharacterHeight(std::max(0.1, circle.true_radius * 2));
  text_->setCaption(std::to_string(circle.uid));

  Ogre::Vector3 dir(Ogre::Real(1.0), Ogre::Real(0.0), Ogre::Real(0.0));
  Ogre::Radian angle(Ogre::Real(M_PI_2));
  Ogre::Quaternion q(angle, dir);
  obstacle_->setOrientation(q);
  margin_->setOrientation(q);
}

void CircleVisual::setFramePose(const Ogre::Vector3& position, const Ogre::Quaternion& orientation) {
  frame_node_obstacle_->setPosition(position);
  frame_node_margin_->setPosition(position);
  frame_node_text_->setPosition(position + orientation * center_position_);
  frame_node_obstacle_->setOrientation(orientation);
  frame_node_margin_->setOrientation(orientation);
  //frame_node_text_->setOrientation(orientation); // Has no effect
}

void CircleVisual::setMainColor(float r, float g, float b, float a) {
  obstacle_->setColor(r, g, b, a);
}

void CircleVisual::setMarginColor(float r, float g, float b, float a) {
  margin_->setColor(r, g, b, a);
}

} // end namespace obstacles_display

