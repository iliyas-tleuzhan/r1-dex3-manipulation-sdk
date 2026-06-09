# SDK Function Reference

This document summarizes the public functions exposed by this repository's Unitree SDK2 headers and the R1/DEX3 examples. It focuses on the SDK-facing APIs under `include/unitree` and the local example programs under `example/r1/low_level`.

Most robot client calls return `int32_t`: `0` means success, and non-zero values are SDK or robot-service errors. Call `Init()` on each client before using its service methods. Low-level DDS publishers and robot motion APIs can move real hardware; test with the robot supported, supervised, and clear of people and obstacles.

Generated IDL message headers under `include/unitree/idl` contain many small constructors, getters, setters, and serialization helpers for DDS message fields. They are intentionally summarized here as message data containers rather than listed one accessor at a time.

## Core RPC Client API

Defined in `include/unitree/robot/client`.

| Class / function | Description |
| --- | --- |
| `ClientBase(name)` | Creates the base request/response client for a named robot service. |
| `SetTimeout(int64_t timeout)` / `SetTimeout(float timeout)` | Sets the RPC wait timeout, in microseconds or seconds depending on overload. |
| `ClientBase::Call(...)` | Sends a raw RPC request with an API id, parameter payload, priority, lease id, and optional timeout. Overloads support string payloads, binary payloads, and binary responses. |
| `SetHeader(header, apiId, leaseId, priority, noReply)` | Fills a request header before sending a low-level RPC request. |
| `Client(name, enableLease)` | Higher-level RPC client with API registration and optional lease management. |
| `WaitLeaseApplied()` | Blocks until a lease-enabled client has obtained its lease. |
| `GetLeaseId()` | Returns the active lease id used for lease-protected APIs. |
| `GetServerApiVersion()` | Reads the remote service API version. |
| `SetApiVersion(apiVersion)` | Sets the expected service API version for compatibility checks. |
| `Noop()` | Sends a no-op request to verify service reachability. |
| `Client::Call(...)` | Sends registered API calls without manually providing priority or lease id. |
| `RegistApi(apiId, priority)` | Registers an API id and its priority with the client. |
| `CheckApi(apiId, priority, leaseId)` | Looks up an API id and resolves the priority and lease id that should be sent. |
| `LeaseContext::Update(id, term)` | Stores the current lease id and term. |
| `LeaseContext::Reset()` | Clears the stored lease. |
| `LeaseContext::Valid()` | Returns whether the lease context currently has a valid lease. |
| `LeaseContext::GetId()` | Returns the current lease id. |
| `LeaseContext::GetTerm()` | Returns the current lease term. |
| `LeaseClient(name)` | Creates the client used internally to apply for and renew service leases. |
| `LeaseClient::Init()` | Registers lease APIs. |
| `WaitApplied()` | Blocks until lease application completes. |
| `LeaseClient::GetId()` | Returns the applied lease id. |
| `Applied()` | Returns whether a lease has been acquired. |
| `Apply()` | Requests a new lease from the service. |
| `Renewal()` | Renews the current lease. |
| `ThreadFunction()` | Background lease renewal loop. |
| `GetWaitMicrosec()` | Returns the wait interval used between lease-renewal attempts. |

## Request Futures

Defined in `include/unitree/robot/future/request_future.hpp`.

| Function | Description |
| --- | --- |
| `RequestFuture()` / `RequestFuture(requestId)` | Creates an async response placeholder, optionally tied to a request id. |
| `SetRequestId(requestId)` | Assigns the request id tracked by the future. |
| `GetRequestId()` | Returns the tracked request id. |
| `SetQueue(futureQueuePtr)` | Connects the future to the queue that owns pending requests. |
| `Ready(responsePtr)` | Marks the future ready and stores the response. |
| `IsDeferred()` | Returns whether the future is waiting for a response. |
| `IsReady()` | Returns whether a response has arrived. |
| `RequestFutureQueue::Get(requestId)` | Finds a pending future by request id. |
| `Put(requestId, futurePtr)` | Inserts a pending future. |
| `Remove(requestId)` | Removes a future from the queue. |
| `Size()` | Returns the number of queued futures. |

## Server API

Defined in `include/unitree/robot/server`.

| Class / function | Description |
| --- | --- |
| `ServerBase(name)` | Creates the base response-side object for a named service. |
| `SendResponse(response)` | Sends a response to a client request. |
| `Server(name)` | Creates an RPC service server. |
| `StartLease(leaseTerm)` | Starts a lease server for APIs that require client ownership. |
| `GetCurrentApiId()` | Returns the API id currently being handled. |
| `SetApiVersion(version)` | Sets the server API version reported to clients. |
| `ServerRequestHandler(request)` | Dispatches an incoming request to the registered handler. |
| `RegistHandler(apiId, handler, checkLease)` | Registers a string request handler. |
| `RegistBinaryHandler(apiId, handler, checkLease)` | Registers a binary request handler. |
| `IsBinary(apiId)` | Returns whether the API id is registered as binary. |
| `GetHandler(apiId, ignoreLease)` | Looks up a string handler and whether it bypasses lease checks. |
| `GetBinaryHandler(apiId, ignoreLease)` | Looks up a binary handler and whether it bypasses lease checks. |
| `CheckLeaseDenied(leaseId)` | Returns whether the request lease is invalid for lease-protected APIs. |
| `LeaseCache::Set(id, name, lastModified)` | Stores the active lease. |
| `LeaseCache::Renewal(lastModified)` | Updates the active lease timestamp. |
| `LeaseCache::Clear()` | Clears the active lease. |
| `LeaseCache::GetLastModified()` | Returns the timestamp of the last lease update. |
| `LeaseCache::GetId()` | Returns the active lease id. |
| `LeaseServer(name, term)` | Creates a lease server with a lease term. |
| `LeaseServer::Init()` | Registers lease apply and renewal handlers. |
| `CheckRequestLeaseDenied(leaseId)` | Checks whether a request should be denied because of lease state. |
| `LeaseServer::ServerRequestHandler(request)` | Handles lease-service requests. |
| `Apply(parameter, data)` | Applies for a lease and writes the response data. |
| `Renewal(leaseId)` | Renews the lease matching `leaseId`. |
| `GenerateId(name)` | Generates a lease id for a client name. |

## DDS Channel API

Defined in `include/unitree/robot/channel`.

| Class / function | Description |
| --- | --- |
| `ChannelFactory::Init(domainId, networkInterface)` | Initializes DDS with a domain id and optional network interface. |
| `ChannelFactory::Init(configFileName)` | Initializes DDS from a configuration file. |
| `ChannelFactory::Init(jsonMap)` | Initializes DDS from a JSON configuration map. |
| `ChannelFactory::Release()` | Releases DDS resources held by the factory. |
| `CreateSendChannel<MSG>(name)` | Creates a typed DDS sending channel. |
| `CreateRecvChannel<MSG>(name, callback, queuelen)` | Creates a typed DDS receiving channel with a callback and optional queue length. |
| `ChannelLabor::InitChannel(name, callback, queuelen)` | Initializes a lower-level channel worker for receiving messages. |
| `ChannelLabor::Send(msg, waitTimeout)` | Sends a DDS message, optionally waiting for write completion. |
| `ChannelLabor::GetLastDataAvailableTime()` | Returns the timestamp of the last received data. |
| `ChannelPublisher(channelName)` | Creates a typed publisher for a DDS topic. |
| `ChannelPublisher::InitChannel()` | Opens the publisher channel. |
| `Write(msg, waitMicrosec)` | Publishes a typed DDS message. |
| `CloseChannel()` | Closes the publisher or subscriber channel. |
| `ChannelSubscriber(channelName)` | Creates a typed subscriber. |
| `ChannelSubscriber(channelName, handler, queuelen)` | Creates and configures a typed subscriber with a callback. |
| `ChannelSubscriber::InitChannel(handler, queuelen)` | Opens the subscriber with a callback. |
| `ChannelSubscriber::InitChannel()` | Opens the subscriber without replacing the handler. |
| `GetLastDataAvailableTime()` | Returns the last receive timestamp for the subscriber. |
| `GetSendChannelName(name)` | Converts a logical service name to the request/send DDS channel name. |
| `GetRecvChannelName(name)` | Converts a logical service name to the response/receive DDS channel name. |

## GO2 Clients

Defined in `include/unitree/robot/go2`.

### `go2::SportClient`

| Function | Description |
| --- | --- |
| `SportClient(enableLease)` | Creates the GO2 sport-mode client, optionally lease-protected. |
| `Init()` | Registers GO2 sport APIs. |
| `Damp()` | Enters damping mode. |
| `BalanceStand()` | Commands balanced standing. |
| `StopMove()` | Stops commanded body velocity. |
| `StandUp()` | Stands up. |
| `StandDown()` | Stands down. |
| `RecoveryStand()` | Attempts recovery to a standing posture. |
| `Euler(roll, pitch, yaw)` | Commands body attitude offsets. |
| `Move(vx, vy, vyaw)` | Commands body velocity in x, y, and yaw. |
| `Sit()` | Sits down. |
| `RiseSit()` | Rises from sitting. |
| `SpeedLevel(level)` | Selects a speed level. |
| `Hello()` | Runs the hello action. |
| `Stretch()` | Runs the stretch action. |
| `SwitchJoystick(flag)` | Enables or disables joystick control. |
| `Content()` | Runs the content action. |
| `Heart()` | Runs the heart action. |
| `Pose(flag)` | Enables or disables pose mode/action. |
| `Scrape()` | Runs the scrape action. |
| `FrontFlip()` / `LeftFlip()` / `BackFlip()` | Runs flip actions. Use only in a safe test area. |
| `FrontJump()` / `FrontPounce()` | Runs jump/pounce actions. |
| `Dance1()` / `Dance2()` | Runs built-in dance actions. |
| `HandStand(flag)` | Enables or disables handstand mode/action. |
| `FreeWalk()` | Enters free-walk mode. |
| `FreeBound(flag)` | Enables or disables free-bound mode. |
| `FreeJump(flag)` | Enables or disables free-jump mode. |
| `FreeAvoid(flag)` | Enables or disables free obstacle avoidance. |
| `ClassicWalk(flag)` | Enables or disables classic walk. |
| `WalkUpright(flag)` | Enables or disables upright walking. |
| `CrossStep(flag)` | Enables or disables cross-step gait. |
| `AutoRecoverSet(flag)` | Sets automatic recovery. |
| `AutoRecoverGet(flag)` | Reads automatic recovery state into `flag`. |
| `StaticWalk()` | Selects static walking. |
| `TrotRun()` | Selects trot running. |
| `EconomicGait()` | Selects economic gait. |
| `SwitchAvoidMode()` | Toggles obstacle-avoidance mode. |

### Other GO2 Clients

| Client / function | Description |
| --- | --- |
| `VideoClient::Init()` | Registers video APIs. |
| `VideoClient::GetImageSample(data)` | Reads one compressed image sample into `data`. |
| `ConfigClient::Set(name, content)` | Stores a named configuration. |
| `ConfigClient::Get(name, content)` | Reads a named configuration. |
| `ConfigClient::Del(name)` | Deletes a named configuration. |
| `ConfigClient::Meta(name, meta)` | Reads configuration metadata into a `ConfigMeta` object or JSON string. |
| `SubscribeChangeStatus(name, callback)` | Subscribes to configuration change status for a name. |
| `ChangeStatusMessageHandler(message)` | Internal callback adapter for config change messages. |
| `RobotStateClient::ServiceList(list)` | Lists robot services and their states. |
| `ServiceSwitch(name, swit, status)` | Enables/disables a named service and returns status. |
| `SetReportFreq(interval, duration)` | Sets robot-state report frequency for a duration. |
| `ObstaclesAvoidClient::SwitchSet(enable)` | Enables or disables obstacle avoidance. |
| `SwitchGet(enable)` | Reads obstacle-avoidance enable state. |
| `ObstaclesAvoidClient::Move(x, y, yaw)` | Commands obstacle-avoidance movement. |
| `UseRemoteCommandFromApi(flag)` | Selects whether remote commands come from the API. |
| `MoveToAbsolutePosition(x, y, yaw)` | Commands navigation to an absolute pose. |
| `MoveToIncrementPosition(x, y, yaw)` | Commands navigation by a relative pose increment. |
| `UtrackClient::SwitchSet(enable)` | Enables or disables UTrack tracking. |
| `UtrackClient::SwitchGet(enable)` | Reads UTrack switch state. |
| `UtrackClient::IsTracking(enable)` | Reads whether tracking is active. |
| `VuiClient::SetSwitch(enable)` / `GetSwitch(value)` | Sets or reads the voice UI switch. |
| `VuiClient::SetVolume(level)` / `GetVolume(value)` | Sets or reads voice UI volume. |
| `VuiClient::SetBrightness(level)` / `GetBrightness(value)` | Sets or reads voice UI brightness. |

## B2 Clients

Defined in `include/unitree/robot/b2`.

| Client / function | Description |
| --- | --- |
| `b2::SportClient::Init()` | Registers B2 sport APIs. |
| `Damp()` | Enters damping mode. |
| `BalanceStand()` | Commands balanced standing. |
| `StopMove()` | Stops velocity command. |
| `StandUp()` / `StandDown()` | Commands stand-up or stand-down posture. |
| `RecoveryStand()` | Attempts recovery to standing. |
| `Move(vx, vy, vyaw)` | Commands body velocity. |
| `SwitchGait(d)` | Selects gait by id. |
| `BodyHeight(height)` | Sets body height. |
| `SpeedLevel(level)` | Sets speed level. |
| `TrajectoryFollow(path)` | Follows a vector of `PathPoint` trajectory points. |
| `ContinuousGait(flag)` | Enables/disables continuous gait. |
| `MoveToPos(x, y, yaw)` | Commands movement to a target pose. |
| `SwitchMoveMode(flag)` | Toggles the movement mode. |
| `VisionWalk(flag)` | Enables/disables vision-assisted walk. |
| `HandStand(flag)` | Enables/disables handstand action. |
| `AutoRecoverySet(flag)` | Sets automatic recovery. |
| `FreeWalk()` | Enters free-walk mode. |
| `ClassicWalk(flag)` | Enables/disables classic walk. |
| `FastWalk(flag)` | Enables/disables fast walk. |
| `Euler(roll, pitch, yaw)` | Sets body attitude offsets. |
| `FreeHeight(flag)` | Enables/disables free-height mode. |
| `GaitHeight(flag)` | Enables/disables gait-height mode. |
| `FrontVideoClient::GetImageSample(data)` | Reads one front-camera image sample. |
| `BackVideoClient::GetImageSample(data)` | Reads one rear-camera image sample. |
| `ConfigClient` functions | Same semantics as GO2 `ConfigClient`. |
| `MotionSwitcherClient::CheckMode(form, name)` | Reads the currently selected motion mode. |
| `SelectMode(nameOrAlias)` | Selects a named motion mode. |
| `ReleaseMode()` | Releases the current motion mode. |
| `SetSilent(silent)` / `GetSilent(silent)` | Sets or reads silent mode. |
| `RobotStateClient::ServiceList(list)` | Lists robot services. |
| `ServiceSwitch(name, swit, status)` | Switches a service and returns status. |
| `SetReportFreq(interval, duration)` | Sets report frequency. |
| `LowPowerSwitch(swit)` / `LowPowerStatus(status)` | Sets or reads low-power state. |
| `GetPkgVersion(packageVersion, moduleVersionMap)` | Reads package and module version strings. |

## A2 and AS2 Clients

Defined in `include/unitree/robot/a2` and `include/unitree/robot/as2`.

| Function | Description |
| --- | --- |
| `SportClient::Init()` | Registers sport APIs. |
| `Damp()` | Enters damping mode. |
| `BalanceStand()` | Commands balanced standing. |
| `StopMove()` | Stops velocity command. |
| `StandUp()` / `StandDown()` | Commands stand-up or stand-down posture. |
| `RecoveryStand()` | Attempts recovery to standing. |
| `Euler(roll, pitch, yaw)` | Commands body attitude offsets. |
| `Move(vx, vy, vyaw)` | Commands body velocity. |
| `SwitchGait(gait_type)` | Selects gait by id. |
| `BodyHeight(height)` | Sets body height. |
| `SpeedLevel(level)` | Sets speed level. |
| `BodyPosition(x, y, z, yaw)` | Commands body position offsets. |
| `LeftSideGait(enter)` / `RightSideGait(enter)` | Enters or exits side gait. |
| `HandStand(enter)` | Enters or exits handstand. |
| `BipedStand(enter)` | Enters or exits biped stand. |
| `FrontFlip()` / `BackFlip()` | Runs flip actions. Use only in a safe test area. |
| `SetAutoRecovery(switch_on)` | Enables/disables automatic recovery. |
| `as2::SportClient::SwitchJoystick(switch_on)` | Enables/disables joystick control on AS2. |
| `GetState(state_map)` | Reads sport service state values into a string map. |
| `a2::AudioClient::TtsMaker(text, speaker_id)` | Sends text-to-speech request. |
| `GetVolume(volume)` | Reads audio volume. |
| `SetVolume(volume)` | Sets audio volume. |
| `PlayStream(app_name, stream_id, pcm_data)` | Sends PCM audio stream data for playback. |
| `PlayStop(app_name)` | Stops playback for an app. |
| `LedControl(R, G, B)` | Sets RGB LED color. |

## G1 Clients

Defined in `include/unitree/robot/g1`.

| Client / function | Description |
| --- | --- |
| `AgvClient::Init()` | Registers AGV APIs. |
| `AgvClient::Move(vx, vy, vyaw)` | Commands AGV velocity. |
| `AgvClient::HeightAdjust(vz)` | Commands vertical height adjustment velocity. |
| `AudioClient` functions | Same audio semantics as `a2::AudioClient`. |
| `G1ArmActionClient::Init()` | Registers arm action APIs. |
| `ExecuteAction(action_id)` | Executes a built-in arm action by numeric id. |
| `ExecuteAction(action_name)` | Executes a custom taught arm action by name. |
| `StopCustomAction()` | Stops the current custom action. |
| `GetActionList(data)` | Reads the available action list as a JSON/string payload. |
| `LocoClient::Init()` | Registers locomotion APIs. |
| `GetFsmId(fsm_id)` | Reads current finite-state-machine id. |
| `GetFsmMode(fsm_mode)` | Reads current FSM mode. |
| `GetBalanceMode(balance_mode)` | Reads balance mode. |
| `GetSwingHeight(swing_height)` | Reads swing height. |
| `GetStandHeight(stand_height)` | Reads stand height. |
| `GetPhase(phase)` | Reads gait phase vector. |
| `SetFsmId(fsm_id)` | Sets locomotion FSM id. |
| `SetBalanceMode(balance_mode)` | Sets balance mode. |
| `SetSwingHeight(swing_height)` | Sets swing height. |
| `SetStandHeight(stand_height)` | Sets standing height. |
| `SetVelocity(vx, vy, omega, duration)` | Commands body velocity for a duration. |
| `SetTaskId(task_id)` | Starts a locomotion/arm task by id. |
| `SwitchToUserCtrl()` | Switches locomotion control to user control. |
| `SwitchToInternalCtrl(mode)` | Switches back to an internal controller mode. |
| `Damp()` | Sets damping FSM. |
| `Start()` | Starts the main locomotion FSM. |
| `Squat()` / `Sit()` / `StandUp()` | Commands posture FSMs. |
| `ZeroTorque()` | Sets zero-torque FSM. |
| `StopMove()` | Sends zero velocity. |
| `HighStand()` / `LowStand()` | Requests high or low stand height. |
| `Move(vx, vy, vyaw, continuous_move)` | Commands velocity with explicit continuous/short duration. |
| `Move(vx, vy, vyaw)` | Commands velocity using the client's current continuous-move setting. |
| `BalanceStand()` | Selects balance stand mode. |
| `ContinuousGait(flag)` | Enables/disables continuous gait. |
| `SwitchMoveMode(flag)` | Stores and applies the continuous move mode. |
| `WaveHand(turn_flag)` | Starts the wave-hand task. |
| `ShakeHand(stage)` | Starts or advances a shake-hand task stage. |
| `SetSpeedMode(speed_mode)` | Sets locomotion speed mode. |

## H1, H2, and R1 Locomotion Clients

Defined in `include/unitree/robot/h1`, `include/unitree/robot/h2`, and `include/unitree/robot/r1`.

| Client / function | Description |
| --- | --- |
| `h1::LocoClient::Init()` / `h2::LocoClient::Init()` / `r1::LocoClient::Init()` | Registers locomotion APIs for that robot. |
| `GetFsmId(fsm_id)` | Reads current FSM id. |
| `GetFsmMode(fsm_mode)` | Reads current FSM mode. |
| `GetBalanceMode(balance_mode)` | Reads balance mode. H1/H2 only. |
| `GetSwingHeight(swing_height)` | Reads swing height. H1/H2 only. |
| `GetStandHeight(stand_height)` | Reads stand height. H1/H2 only. |
| `GetPhase(phase)` | Reads gait phase. H1/H2 only. |
| `h1::EnableOdom()` / `h1::DisableOdom()` | Enables or disables H1 odometry reporting. |
| `h1::GetOdom(x, y, yaw)` | Reads H1 odometry. |
| `h2::GetArmSdkStatus(status)` | Reads whether H2 arm SDK control is enabled. |
| `h2::GetAvailableFsmIds(ids, names)` | Reads available H2 FSM ids and names. |
| `SetFsmId(fsm_id)` | Sets locomotion FSM id. |
| `SetBalanceMode(balance_mode)` | Sets balance mode. H1/H2 only. |
| `h2::SetPunchApi(punch_api)` | Sends H2 punch API vector. |
| `SetSwingHeight(swing_height)` | Sets swing height. H1/H2 only. |
| `SetStandHeight(stand_height)` | Sets stand height. H1/H2 only. |
| `SetVelocity(vx, vy, omega, duration)` | Commands velocity for a duration. |
| `h1::SetPhase(phase)` | Sets H1 gait phase. |
| `h1::SetTargetPos(x, y, yaw, relative)` | Commands H1 target position. |
| `SetTaskId(task_id)` | Starts task id. H1/H2/G1-style locomotion clients. |
| `h2::SetArmSdkStatus(status)` | Enables or disables H2 arm SDK control. |
| `Damp()` | Sets damping FSM. |
| `Start()` | Starts the main locomotion FSM. R1 uses FSM `811`; H1 uses `204`; H2 uses `500`. |
| `Squat()` / `Sit()` | Commands squat/sit where supported. |
| `StandUp()` | Commands standing FSM. |
| `ZeroTorque()` | Sets zero torque FSM. |
| `StopMove()` | Sends zero velocity. |
| `HighStand()` / `LowStand()` | Requests high/low stand height where supported. |
| `Move(vx, vy, vyaw, continuous_move)` | Commands velocity with explicit duration behavior. |
| `Move(vx, vy, vyaw)` | Commands velocity using current move-mode state. |
| `BalanceStand()` | Selects balance mode. H1/H2 only. |
| `ContinuousGait(flag)` | Enables/disables continuous gait. H1/H2 only. |
| `SwitchMoveMode(flag)` | Stores continuous movement mode and changes command duration behavior. |
| `h1::SetNextFoot(foot)` | Chooses next H1 foot phase. |
| `WaveHand(...)` | Starts the wave-hand task where supported. |
| `ShakeHand(stage)` | Starts or advances a shake-hand task where supported. |
| `SetSpeedMode(speed_mode)` | Sets speed mode. G1/H2/R1 clients. |
| `h2::EnableArmSDK()` / `h2::DisableArmSDK()` | Convenience wrappers around `SetArmSdkStatus(true/false)`. |

## DDS Wrapper Helpers

Defined in `include/unitree/dds_wrapper`.

| Class / function | Description |
| --- | --- |
| `SubscriptionBase(topic)` | Creates a typed subscriber wrapper and stores the latest message. |
| `set_timeout_ms(timeout_ms)` | Sets subscriber timeout threshold. |
| `isTimeout()` | Returns whether no new message has arrived within the timeout. |
| `wait_for_connection()` | Blocks until the subscriber receives data. |
| Robot-specific `subscription::*::update()` | Copies the latest DDS message into wrapper fields and updates derived joystick timeout state when present. |
| Robot-specific `subscription::*::isJoystickTimeout()` | Returns whether joystick data has timed out. |
| `PublisherBase(topic)` | Creates a typed DDS publisher wrapper. |
| `RealTimePublisher(publisher)` / `RealTimePublisher(topic)` | Creates a publisher helper for real-time command loops. |
| `stop()` | Stops the publishing loop. |
| `trylock()` | Attempts to lock the command buffer without blocking. |
| `lock()` / `unlock()` | Locks or unlocks the command buffer. |
| `unlockAndPublish()` | Unlocks the command buffer and publishes the current message. |
| `is_running()` | Returns whether the publishing loop is active. |
| `publishingLoop()` | Internal loop that repeatedly publishes messages. |
| Robot-specific `publisher::*::pre_communication()` | Prepares messages before publishing, commonly by filling CRC fields or mode fields. |
| `g1::publisher::LowCmd::check_mode_machine(lowstate)` | Checks whether the G1 mode machine allows low-command publishing. |
| `g1::publisher::ArmSdk::weight(coe)` | Sets the arm SDK command weight. |
| `g1::publisher::ArmSdk::weight()` | Reads the arm SDK command weight. |
| `KeyBase::update(is_pressed)` | Updates button/axis pressed state, edge triggers, and hold counters. |
| `Button::operator()(data)` | Updates a button from raw joystick data. |
| `Axis::operator()(data)` | Updates an axis from raw joystick data. |
| `UnitreeJoystick::extract(key)` | Decodes raw Unitree remote data into button and axis states. |
| `UnitreeJoystick::combine()` | Encodes button and axis states back into raw remote data. |

## Common Utilities

Defined in `include/unitree/common`.

| Area | Functions |
| --- | --- |
| Time | `GetCurrentTimeval`, `GetCurrentTimespec`, `GetCurrentTime`, `GetCurrentMillisecond`, `GetCurrentMicrosecond`, `GetCurrentNanosecond`, `GetCurrentSecond`, `Sleep`, `Msleep`, `Usleep`, `Nsleep` provide timestamp and sleep helpers. |
| JSON | `ToJsonString`, `FromJsonString`, `Jsonize::fromJson`, `Jsonize::toJson`, and `JsonMap` helpers serialize command parameters and parse service replies. |
| Any | `Any`, `AnyCast`, `AnyNumberCast`, and type-check helpers provide a small type-erased value container. |
| BlockQueue | `Put`, `Get`, `Empty`, `Size`, and `Interrupt` implement a thread-safe blocking queue. |
| Locks | `Lock`, `Unlock`, `Trylock`, `Wait`, `Notify`, `NotifyAll`, and guard classes wrap mutexes, conditions, spinlocks, rwlocks, and file locks. |
| Threads | `Thread`, `ThreadPool`, `ThreadTask`, `Future`, and `RecurrentThread` helpers start background work and recurrent tasks. |
| Filesystem | File and directory helpers wrap existence checks, creation, removal, traversal, and path operations. |
| Logging | Logger, writer, store, policy, buffer, keeper, and init helpers implement SDK logging. |
| DDS | DDS parameter, QoS, entity, callback, topic-channel, and factory-model helpers wrap CycloneDDS setup and message transport. |
| Service framework | `ServiceApplication`, `ServiceBase`, `ServiceConfig`, and `DdsService` implement reusable long-running service processes. |

## Generated IDL Messages

Headers under `include/unitree/idl` and `include/unitree/robot/internal/internal_idl_decl` define DDS data structures such as low-level commands, states, IMU data, motor commands, motor states, hand commands, hand states, audio data, video data, ROS2-compatible geometry messages, and internal request/response messages.

Each generated message class generally provides:

| Function type | Description |
| --- | --- |
| Constructors and assignment operators | Create, copy, and move message objects. |
| Field getters and setters | Read or write each DDS field. Setters usually have const-reference and rvalue overloads. |
| Serialization helpers | Serialize, deserialize, and report serialized size/alignment for DDS transport. |
| Key helpers | Report whether the type has a DDS key and serialize the key when applicable. |

## R1 / DEX3 Example Functions

Defined in `example/r1/low_level`.

| Example | Important functions and behavior |
| --- | --- |
| `dex3_hand_position_monitor.cpp` | Initializes DDS on the selected network interface, subscribes to left or right `HandState_`, prints DEX3 motor positions, and does not publish commands. |
| `r1_arm_dex3_grasp_example.cpp` | Confirms safety, initializes R1 arm and selected DEX3 hand DDS channels, moves the selected arm up, opens/closes/opens the hand, then lowers the arm. |
| `dual_dex3_arm_hand_motion_example.cpp` | Confirms safety, initializes both arm and hand command channels, raises both arms, opens/closes both hands twice, and lowers both arms. |
| `r1_arm_wave_hand_example.cpp` | Confirms safety, initializes selected arm and hand channels, opens the hand, waves using shoulder/wrist joints, then returns to a lowered pose. |
| Shared helper patterns | Example helpers build `LowCmd_` and `HandCmd_` messages, set motor mode/position/velocity/torque fields, publish commands repeatedly for short hold windows, and compute CRCs where required. |

## Safety Notes

- `Move`, `SetVelocity`, `SetFsmId`, `StandUp`, `ZeroTorque`, arm action, hand command, flip, jump, and posture functions can immediately affect real robot motion.
- Lease-enabled clients should call `WaitLeaseApplied()` before commanding lease-protected APIs.
- Low-level DDS command loops should publish at a stable rate and should send neutral/stop commands before exiting when the robot mode requires it.
- Generated message setters do not validate robot-safe ranges; range limits must be enforced by the calling controller.
