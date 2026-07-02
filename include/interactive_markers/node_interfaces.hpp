// Copyright (c) 2026, Open Source Robotics Foundation, Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#ifndef INTERACTIVE_MARKERS__NODE_INTERFACES_HPP_
#define INTERACTIVE_MARKERS__NODE_INTERFACES_HPP_

namespace interactive_markers
{

/// Concept for node-like objects usable to construct an InteractiveMarkerServer.
/**
 * Satisfied by any pointer-like handle (e.g. rclcpp::Node::SharedPtr,
 * rclcpp_lifecycle::LifecycleNode::SharedPtr, or a raw node pointer) that exposes
 * the node interface accessors the server delegates to in its constructor.
 *
 * Constraining the templated constructor with this concept turns a deep, nested
 * instantiation error into a single "constraint not satisfied" diagnostic pointing
 * at the offending accessor, and documents the requirement in-signature.
 */
template<typename NodePtr>
concept ServerNodeInterfaces = requires(NodePtr node) {
  node->get_node_base_interface();
  node->get_node_clock_interface();
  node->get_node_logging_interface();
  node->get_node_topics_interface();
  node->get_node_services_interface();
};  // NOLINT(readability/braces): trailing ';' is required for a concept definition

/// Concept for node-like objects usable to construct an InteractiveMarkerClient.
/**
 * The same as ServerNodeInterfaces, plus the node graph interface, which the client
 * additionally requires in order to query the ROS graph.  The accessors are listed
 * in full (rather than refining ServerNodeInterfaces via '&&') so the client's
 * complete requirement is visible in one place.
 */
template<typename NodePtr>
concept ClientNodeInterfaces = requires(NodePtr node) {
  node->get_node_base_interface();
  node->get_node_clock_interface();
  node->get_node_logging_interface();
  node->get_node_topics_interface();
  node->get_node_services_interface();
  node->get_node_graph_interface();
};  // NOLINT(readability/braces): trailing ';' is required for a concept definition

}  // namespace interactive_markers

#endif  // INTERACTIVE_MARKERS__NODE_INTERFACES_HPP_
