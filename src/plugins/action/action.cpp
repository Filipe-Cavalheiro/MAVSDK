#include "action_impl.h"
#include "plugins/action/action.h"

namespace mavsdk {

Action::Action(System& system) : PluginBase(), _impl{new ActionImpl(system)} {}

Action::~Action() {}

void Action::arm_async(const result_callback_t callback)
{
    _impl->arm_async(callback);
}

Action::Result Action::arm() const
{
    return _impl->arm();
}

void Action::disarm_async(const result_callback_t callback)
{
    _impl->disarm_async(callback);
}

Action::Result Action::disarm() const
{
    return _impl->disarm();
}

void Action::takeoff_async(const result_callback_t callback)
{
    _impl->takeoff_async(callback);
}

Action::Result Action::takeoff() const
{
    return _impl->takeoff();
}

void Action::land_async(const result_callback_t callback)
{
    _impl->land_async(callback);
}

Action::Result Action::land() const
{
    return _impl->land();
}

void Action::reboot_async(const result_callback_t callback)
{
    _impl->reboot_async(callback);
}

Action::Result Action::reboot() const
{
    return _impl->reboot();
}

void Action::shutdown_async(const result_callback_t callback)
{
    _impl->shutdown_async(callback);
}

Action::Result Action::shutdown() const
{
    return _impl->shutdown();
}

void Action::kill_async(const result_callback_t callback)
{
    _impl->kill_async(callback);
}

Action::Result Action::kill() const
{
    return _impl->kill();
}

void Action::return_to_launch_async(const result_callback_t callback)
{
    _impl->return_to_launch_async(callback);
}

Action::Result Action::return_to_launch() const
{
    return _impl->return_to_launch();
}

void Action::goto_location_async(
    double latitude_deg,
    double longitude_deg,
    float absolute_altitude_m,
    float yaw_deg,
    const result_callback_t callback)
{
    _impl->goto_location_async(latitude_deg, longitude_deg, absolute_altitude_m, yaw_deg, callback);
}

Action::Result Action::goto_location(
    double latitude_deg, double longitude_deg, float absolute_altitude_m, float yaw_deg) const
{
    return _impl->goto_location(latitude_deg, longitude_deg, absolute_altitude_m, yaw_deg);
}

void Action::transition_to_fixedwing_async(const result_callback_t callback)
{
    _impl->transition_to_fixedwing_async(callback);
}

Action::Result Action::transition_to_fixedwing() const
{
    return _impl->transition_to_fixedwing();
}

void Action::transition_to_multicopter_async(const result_callback_t callback)
{
    _impl->transition_to_multicopter_async(callback);
}

Action::Result Action::transition_to_multicopter() const
{
    return _impl->transition_to_multicopter();
}

void Action::get_takeoff_altitude_async(const altitude_callback_t callback)
{
    _impl->get_takeoff_altitude_async(callback);
}

std::pair<Action::Result, float> Action::get_takeoff_altitude() const
{
    return _impl->get_takeoff_altitude();
}

void Action::set_takeoff_altitude_async(float altitude, const result_callback_t callback)
{
    _impl->set_takeoff_altitude_async(altitude, callback);
}

Action::Result Action::set_takeoff_altitude(float altitude) const
{
    return _impl->set_takeoff_altitude(altitude);
}

void Action::get_maximum_speed_async(const speed_callback_t callback)
{
    _impl->get_maximum_speed_async(callback);
}

std::pair<Action::Result, float> Action::get_maximum_speed() const
{
    return _impl->get_maximum_speed();
}

void Action::set_maximum_speed_async(float speed, const result_callback_t callback)
{
    _impl->set_maximum_speed_async(speed, callback);
}

Action::Result Action::set_maximum_speed(float speed) const
{
    return _impl->set_maximum_speed(speed);
}

void Action::get_return_to_launch_altitude_async(const relative_altitude_m_callback_t callback)
{
    _impl->get_return_to_launch_altitude_async(callback);
}

std::pair<Action::Result, float> Action::get_return_to_launch_altitude() const
{
    return _impl->get_return_to_launch_altitude();
}

void Action::set_return_to_launch_altitude_async(
    float relative_altitude_m, const result_callback_t callback)
{
    _impl->set_return_to_launch_altitude_async(relative_altitude_m, callback);
}

Action::Result Action::set_return_to_launch_altitude(float relative_altitude_m) const
{
    return _impl->set_return_to_launch_altitude(relative_altitude_m);
}

const char* Action::result_str(Action::Result result)
{
    switch (result) {
        case Action::Result::Unknown:
            return "Unknown error";
        case Action::Result::Success:
            return "Success: the action command was accepted by the vehicle";
        case Action::Result::NoSystem:
            return "No system is connected";
        case Action::Result::ConnectionError:
            return "Connection error";
        case Action::Result::Busy:
            return "Vehicle is busy";
        case Action::Result::CommandDenied:
            return "Command refused by vehicle";
        case Action::Result::CommandDeniedLandedStateUnknown:
            return "Command refused because landed state is unknown";
        case Action::Result::CommandDeniedNotLanded:
            return "Command refused because vehicle not landed";
        case Action::Result::Timeout:
            return "Request timed out";
        case Action::Result::VtolTransitionSupportUnknown:
            return "Hybrid/VTOL transition refused because VTOL support is unknown";
        case Action::Result::NoVtolTransitionSupport:
            return "Vehicle does not support hybrid/VTOL transitions";
        case Action::Result::ParameterError:
            return "Error getting or setting parameter";
        default:
            return "Unknown";
    }
}

} // namespace mavsdk