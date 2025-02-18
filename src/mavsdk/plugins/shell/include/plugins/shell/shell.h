// WARNING: THIS FILE IS AUTOGENERATED! As such, it should not be edited.
// Edits need to be made to the proto files
// (see https://github.com/mavlink/MAVSDK-Proto/blob/main/protos/shell/shell.proto)

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
class ShellImpl;

/**
 * @brief Allow to communicate with the vehicle's system shell.
 */
class Shell : public PluginBase {
public:
    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto shell = Shell(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit Shell(System& system); // deprecated

    /**
     * @brief Constructor. Creates the plugin for a specific System.
     *
     * The plugin is typically created as shown below:
     *
     *     ```cpp
     *     auto shell = Shell(system);
     *     ```
     *
     * @param system The specific system associated with this plugin.
     */
    explicit Shell(std::shared_ptr<System> system); // new

    /**
     * @brief Destructor (internal use only).
     */
    ~Shell() override;

    /**
     * @brief Possible results returned for shell requests
     */
    enum class Result {
        Unknown, /**< @brief Unknown result. */
        Success, /**< @brief Request succeeded. */
        NoSystem, /**< @brief No system is connected. */
        ConnectionError, /**< @brief Connection error. */
        NoResponse, /**< @brief Response was not received. */
        Busy, /**< @brief Shell busy (transfer in progress). */
    };

    /**
     * @brief Stream operator to print information about a `Shell::Result`.
     *
     * @return A reference to the stream.
     */
    friend std::ostream& operator<<(std::ostream& str, Shell::Result const& result);

    /**
     * @brief Callback type for asynchronous Shell calls.
     */
    using ResultCallback = std::function<void(Result)>;

    /**
     * @brief Send a command line.
     *
     * This function is blocking.
     *

     * @return Result of request.

     */
    Result send(std::string command) const;

    /**
     * @brief Callback type for subscribe_receive.
     */
    using ReceiveCallback = std::function<void(std::string)>;

    /**
     * @brief Handle type for subscribe_receive.
     */
    using ReceiveHandle = Handle<std::string>;

    /**
     * @brief Receive feedback from a sent command line.
     *
     * This subscription needs to be made before a command line is sent, otherwise, no response will
     * be sent.
     */
    ReceiveHandle subscribe_receive(const ReceiveCallback& callback);

    /**
     * @brief Unsubscribe from subscribe_receive
     */
    void unsubscribe_receive(ReceiveHandle handle);

    /**
     * @brief Copy constructor.
     */
    Shell(const Shell& other);

    /**
     * @brief Equality operator (object is not copyable).
     */
    const Shell& operator=(const Shell&) = delete;

private:
    /** @private Underlying implementation, set at instantiation */
    std::unique_ptr<ShellImpl> _impl;
};

} // namespace mavsdk