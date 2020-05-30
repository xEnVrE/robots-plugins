/*
 * Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT)
 *
 * This software may be modified and distributed under the terms of the
 * GPL-2+ license. See the accompanying LICENSE file for details.
 */

#ifndef ROBOTSPLUGINS_UTILS_H
#define ROBOTSPLUGINS_UTILS_H

#include <gazebo/common/Plugin.hh>

#include <string>

namespace RobotsPlugins {
    namespace Utils {
        template<class T>
        bool load_parameter_from_sdf(sdf::ElementPtr sdf, const std::string &name, T& value);
    }
}


template<class T>
bool RobotsPlugins::Utils::load_parameter_from_sdf(sdf::ElementPtr sdf, const std::string &name, T& value)
{
    const std::string log_name = "RobotsPlugins::Utils::load_parameter_from_sdf";

    /* Check if the element exists. */
    if (!(sdf->HasElement(name)))
    {
        std::cerr << log_name + " Error: cannot find parameter " << name;
        return false;
    }

    /* Get the associated parameter. */
    sdf::ParamPtr parameter = sdf->GetElement(name)->GetValue();

    /* Check if the value can be intrepreted as a T. */
    if (!parameter->Get<T>(value))
    {
        std::cerr << log_name + " Error: parameter " << name << " is not a valid.";
        return false;
    }

    return true;
}

#endif

/* ROBOTSPLUGINS_UTILS_H */
