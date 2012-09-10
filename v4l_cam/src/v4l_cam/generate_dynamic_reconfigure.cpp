/*
    <one line to give the library's name and an idea of what it does.>
    Copyright (C) 2012 Markus Bader

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA

*/

#include <v4l_cam/v4l_cam_node.h>
#include <stdio.h>
#include <string>
extern "C" {
#include "luvcview/v4l2uvc.h"
#include <libv4l2.h>
#include <linux/videodev2.h>
}
#include <boost/algorithm/string.hpp>
