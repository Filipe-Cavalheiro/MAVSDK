// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/winch/winch.proto)

#pragma once

#include <array>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plugin_base.h"

#include "handle.h"

namespace mavsdk {

class System;
class WinchImpl;

/**
 * @brief Allows users to send winch actions, as well as receive status information from winch
 * systems.
 */
class Winch : public PluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto winch = Winch(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit Winch(System& system); // deprecated

    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto winch = Winch(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit Winch(std::shared_ptr<System> system); // new

    /**
     * @brief Destructor (internal use only).
     */
    ~Winch() override;

    /**
     * @brief Winch Action type.
     */
    enum class WinchAction {
        Relaxed, /**< @brief Allow motor to freewheel. */
        RelativeLengthControl, /**< @brief Wind or unwind specified length of line, optionally using
                                  specified rate. */
        RateControl, /**< @brief Wind or unwind line at specified rate. */
        Lock, /**< @brief Perform the locking sequence to relieve motor while in the fully retracted
                 position. */
        Deliver, /**< @brief Sequence of drop, slow down, touch down, reel up, lock. */
        Hold, /**< @brief Engage motor and hold current position. */
        Retract, /**< @brief Return the reel to the fully retracted position. */
        LoadLine, /**< @brief Load the reel with line. The winch will calculate the total loaded
                     length and stop when the tension exceeds a threshold. */
        AbandonLine, /**< @brief Spool out the entire length of the line. */
        LoadPayload, /**< @brief Spools out just enough to present the hook to the user to load the
                        payload. */
    };

    /**
     * @brief Stream operator to print information about a `Winch::WinchAction`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Winch::WinchAction const& winch_action);

    /**
     * @brief Winch Status Flags.
     *
     * The status flags are defined in mavlink
     * https://mavlink.io/en/messages/common.html#MAV_WINCH_STATUS_FLAG.
     *
     * Multiple status fields can be set simultaneously. Mavlink does
     * not specify which states are mutually exclusive.
     */
    struct StatusFlags {
        bool healthy{}; /**< @brief Winch is healthy */
        bool fully_retracted{}; /**< @brief Winch line is fully retracted */
        bool moving{}; /**< @brief Winch motor is moving */
        bool clutch_engaged{}; /**< @brief Winch clutch is engaged allowing motor to move freely */
        bool locked{}; /**< @brief Winch is locked by locking mechanism */
        bool dropping{}; /**< @brief Winch is gravity dropping payload */
        bool arresting{}; /**< @brief Winch is arresting payload descent */
        bool ground_sense{}; /**< @brief Winch is using torque measurements to sense the ground */
        bool retracting{}; /**< @brief Winch is returning to the fully retracted position */
        bool redeliver{}; /**< @brief Winch is redelivering the payload. This is a failover state if
                             the line tension goes above a threshold during RETRACTING. */
        bool abandon_line{}; /**< @brief Winch is abandoning the line and possibly payload. Winch
                                unspools the entire calculated line length. This is a failover state
                                from REDELIVER if the number of attempts exceeds a threshold. */
        bool locking{}; /**< @brief Winch is engaging the locking mechanism */
        bool load_line{}; /**< @brief Winch is spooling on line */
        bool load_payload{}; /**< @brief Winch is loading a payload */
    };

    /**
     * @brief Equal operator to compare two `Winch::StatusFlags` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Winch::StatusFlags& lhs, const Winch::StatusFlags& rhs);

    /**
     * @brief Stream operator to print information about a `Winch::StatusFlags`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Winch::StatusFlags const& status_flags);

    /**
     * @brief Status type.
     */
    struct Status {
        uint64_t time_usec{}; /**< @brief Time in usec */
        float line_length_m{}; /**< @brief Length of the line in meters */
        float speed_m_s{}; /**< @brief Speed in meters per second */
        float tension_kg{}; /**< @brief Tension in kilograms */
        float voltage_v{}; /**< @brief Voltage in volts */
        float current_a{}; /**< @brief Current in amperes */
        int32_t temperature_c{}; /**< @brief Temperature in Celsius */
        StatusFlags status_flags{}; /**< @brief Status flags */
    };

    /**
     * @brief Equal operator to compare two `Winch::Status` objects.
     *
     * @return `true` if items are equal.
     */
    friend bool operator==(const Winch::Status& lhs, const Winch::Status& rhs);

    /**
     * @brief Stream operator to print information about a `Winch::Status`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Winch::Status const& status);

    /**
     * @brief Possible results returned for winch action requests.
     */
    enum class Result {
        Unknown, /**< @brief Unknown result. */
        Success, /**< @brief Request was successful. */
        NoSystem, /**< @brief No system is connected. */
        Busy, /**< @brief Temporarily rejected. */
        Timeout, /**< @brief Request timed out. */
        Unsupported, /**< @brief Action not supported. */
        Failed, /**< @brief Action failed. */
    };

    /**
     * @brief Stream operator to print information about a `Winch::Result`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Winch::Result const& result);

    /**
     * @brief Callback type for asynchronous Winch calls.
     */
    using ResultCallback = std::function<void(Result)>;

    /**
     * @brief Callback type for subscribe_status.
     */
    using StatusCallback = std::function<void(Status)>;

    /**
     * @brief Handle type for subscribe_status.
     */
    using StatusHandle = Handle<Status>;

    /**
     * @brief Subscribe to 'winch status' updates.
     */
    StatusHandle subscribe_status(const StatusCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_status
     */
    void unsubscribe_status(StatusHandle handle);

    /**
     * @brief Poll for 'Status' (blocking).
     *
     * @return One Status update.
     */
    Status status() const;

    /**
     * @brief Allow motor to freewheel.
     *
     * This function is non-blocking. See 'relax' for the blocking counterpart.
     */
    void relax_async(uint32_t instance, const ResultCallback callback);

    /**
     * @brief Allow motor to freewheel.
     *
     * This function is blocking. See 'relax_async' for the non-blocking counterpart.
     *

     * @return Result of request.

     */
    Result relax(uint32_t instance) const;

    /**
     * @brief Wind or unwind specified length of line, optionally using specified rate.
     *
     * This function is non-blocking. See 'relative_length_control' for the blocking counterpart.
     */
    void relative_length_control_async(
        uint32_t instance, float length_m, float rate_m_s, const ResultCallback callback);

    /**
     * @brief Wind or unwind specified length of line, optionally using specified rate.
     *
     * This function is blocking. See 'relative_length_control_async' for the non-blocking
     counterpart.
     *

     * @return Result of request.

     */
    Result relative_length_control(uint32_t instance, float length_m, float rate_m_s) const;

    /**
     * @brief Wind or unwind line at specified rate.
     *
     * This function is non-blocking. See 'rate_control' for the blocking counterpart.
     */
    void rate_control_async(uint32_t instance, float rate_m_s, const ResultCallback callback);

    /**
     * @brief Wind or unwind line at specified rate.
     *
     * This function is blocking. See 'rate_control_async' for the non-blocking counterpart.
     *

     * @return Result of request.

     */
    Result rate_control(uint32_t instance, float rate_m_s) const;

    /**
     * @brief Perform the locking sequence to relieve motor while in the fully retracted position.
     *
     * This function is non-blocking. See 'lock' for the blocking counterpart.
     */
    void lock_async(uint32_t instance, const ResultCallback callback);

    /**
     * @brief Perform the locking sequence to relieve motor while in the fully retracted position.
     *
     * This function is blocking. See 'lock_async' for the non-blocking counterpart.
     *

     * @return Result of request.

     */
    Result lock(uint32_t instance) const;

    /**
     * @brief Sequence of drop, slow down, touch down, reel up, lock.
     *
     * This function is non-blocking. See 'deliver' for the blocking counterpart.
     */
    void deliver_async(uint32_t instance, const ResultCallback callback);

    /**
     * @brief Sequence of drop, slow down, touch down, reel up, lock.
     *
     * This function is blocking. See 'deliver_async' for the non-blocking counterpart.
     *

     * @return Result of request.

     */
    Result deliver(uint32_t instance) const;

    /**
     * @brief Engage motor and hold current position.
     *
     * This function is non-blocking. See 'hold' for the blocking counterpart.
     */
    void hold_async(uint32_t instance, const ResultCallback callback);

    /**
     * @brief Engage motor and hold current position.
     *
     * This function is blocking. See 'hold_async' for the non-blocking counterpart.
     *

     * @return Result of request.

     */
    Result hold(uint32_t instance) const;

    /**
     * @brief Return the reel to the fully retracted position.
     *
     * This function is non-blocking. See 'retract' for the blocking counterpart.
     */
    void retract_async(uint32_t instance, const ResultCallback callback);

    /**
     * @brief Return the reel to the fully retracted position.
     *
     * This function is blocking. See 'retract_async' for the non-blocking counterpart.
     *

     * @return Result of request.

     */
    Result retract(uint32_t instance) const;

    /**
     * @brief Load the reel with line.
     *
     * The winch will calculate the total loaded length and stop when the tension exceeds a
     * threshold.
     *
     * This function is non-blocking. See 'load_line' for the blocking counterpart.
     */
    void load_line_async(uint32_t instance, const ResultCallback callback);

    /**
     * @brief Load the reel with line.
     *
     * The winch will calculate the total loaded length and stop when the tension exceeds a
     threshold.
     *
     * This function is blocking. See 'load_line_async' for the non-blocking counterpart.
     *

     * @return Result of request.

     */
    Result load_line(uint32_t instance) const;

    /**
     * @brief Spool out the entire length of the line.
     *
     * This function is non-blocking. See 'abandon_line' for the blocking counterpart.
     */
    void abandon_line_async(uint32_t instance, const ResultCallback callback);

    /**
     * @brief Spool out the entire length of the line.
     *
     * This function is blocking. See 'abandon_line_async' for the non-blocking counterpart.
     *

     * @return Result of request.

     */
    Result abandon_line(uint32_t instance) const;

    /**
     * @brief Spools out just enough to present the hook to the user to load the payload.
     *
     * This function is non-blocking. See 'load_payload' for the blocking counterpart.
     */
    void load_payload_async(uint32_t instance, const ResultCallback callback);

    /**
     * @brief Spools out just enough to present the hook to the user to load the payload.
     *
     * This function is blocking. See 'load_payload_async' for the non-blocking counterpart.
     *

     * @return Result of request.

     */
    Result load_payload(uint32_t instance) const;

    /**
     * @brief Copy constructor.
     */
    Winch(const Winch& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const Winch& operator=(const Winch&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<WinchImpl> _impl;
};

} // namespace mavsdk