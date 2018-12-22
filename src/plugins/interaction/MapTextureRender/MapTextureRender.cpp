/*!
 * @file
 *
 * @section LICENSE
 *
 * Copyright (C) 2017 by the Georgia Tech Research Institute (GTRI)
 *
 * This file is part of SCRIMMAGE.
 *
 *   SCRIMMAGE is free software: you can redistribute it and/or modify it under
 *   the terms of the GNU Lesser General Public License as published by the
 *   Free Software Foundation, either version 3 of the License, or (at your
 *   option) any later version.
 *
 *   SCRIMMAGE is distributed in the hope that it will be useful, but WITHOUT
 *   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU Lesser General Public
 *   License for more details.
 *
 *   You should have received a copy of the GNU Lesser General Public License
 *   along with SCRIMMAGE.  If not, see <http://www.gnu.org/licenses/>.
 *
 * @author Kevin DeMarco <kevin.demarco@gtri.gatech.edu>
 * @author Eric Squires <eric.squires@gtri.gatech.edu>
 * @date 31 July 2017
 * @version 0.1.0
 * @brief Brief file description.
 * @section DESCRIPTION
 * A Long description goes here.
 *
 */

#include <scrimmage/plugins/interaction/MapTextureRender/MapTextureRender.h>

#include <scrimmage/math/Angles.h>
#include <scrimmage/common/Utilities.h>
#include <scrimmage/entity/Entity.h>
#include <scrimmage/plugin_manager/RegisterPlugin.h>
#include <scrimmage/math/State.h>
#include <scrimmage/proto/Shape.pb.h>
#include <scrimmage/proto/ProtoConversions.h>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/Constants.hpp>

#include <memory>
#include <limits>
#include <iostream>

using std::cout;
using std::endl;

namespace sc = scrimmage;

REGISTER_PLUGIN(scrimmage::EntityInteraction,
                scrimmage::interaction::MapTextureRender,
                MapTextureRender_plugin)

namespace scrimmage {
namespace interaction {

MapTextureRender::MapTextureRender() {
}

bool MapTextureRender::init(std::map<std::string, std::string> &mission_params,
                               std::map<std::string, std::string> &plugin_params) {
    const double lat = std::stod(plugin_params["lat"]);
    const double lon = std::stod(plugin_params["lon"]);
    double x = 0.0, y = 0.0, z = 0.0;
    parent()->projection()->Forward(lat, lon, 0.0, x, y, z);

    auto map_plane = std::make_shared<scrimmage_proto::Shape>();
    map_plane->set_persistent(true);
    map_plane->set_opacity(1.0);
    scrimmage::set(map_plane->mutable_color(), 255, 255, 255); // dont change this
    sc::set(map_plane->mutable_plane()->mutable_center(), x, y, std::stod(plugin_params["z"]));

    // see https://wiki.openstreetmap.org/wiki/Zoom_levels
    int zm_lvl = std::stoi(plugin_params["zm_lvl"]);
    // circumference of earth
    double C = 2 * M_PI * GeographicLib::Constants::WGS84_a();
    const double m_per_px = C * std::cos(sc::Angles::deg2rad(lat)) / (1 << (zm_lvl + 8));

    const double tex_w_px = std::stod(plugin_params["width_px"]);
    const double tex_h_px = std::stod(plugin_params["height_px"]);

    map_plane->mutable_plane()->set_x_length(tex_w_px * m_per_px);
    map_plane->mutable_plane()->set_y_length(tex_h_px * m_per_px);
    scrimmage::set(map_plane->mutable_plane()->mutable_quat(), 1, 0, 0, 0);
    map_plane->mutable_plane()->set_texture("china_lake.xml");
    draw_shape(map_plane);

    return true;
}


bool MapTextureRender::step_entity_interaction(std::list<sc::EntityPtr> &ents,
                                                  double t, double dt) {
    if (ents.empty()) {
        return true;
    }

    return true;
}
} // namespace interaction
} // namespace scrimmage
