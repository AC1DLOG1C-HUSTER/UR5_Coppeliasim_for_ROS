-- CoppeliaSim UR5 child script.
-- This file is Lua, not Python, even though it is named ur5.py here.

sim=require'sim'
simOMPL=require'simOMPL'
simIK=require'simIK'

if math.mod == nil then
    math.mod = math.fmod
end

-- Ubuntu project directory. Change this only if your project folder is different.
local PROJECT_DIR = '/home/ljh/ur5_coppelia_ws/src/UR5-Path-Planning-and-Grasping-using-Coppeliasim-'

-- Runtime settings. These values match the original slower, more stable behavior.
local ENABLE_FIRST_OBJECT_FALLBACK = true
local ENABLE_PATH_VISUALIZATION = true
local ENABLE_JOINT_DATA_RECORDING = true
local GRIPPER_WAIT_TIME = 1.25
local FIRST_PICK_SETTLE_TIME = 1.0
local FIRST_GRIPPER_WAIT_TIME = 2.5
local FIRST_RELEASE_WAIT_TIME = 1.0
local MOTION_SPEED_SCALE = 1.0
local FIRST_OBJECT_NAME = '/Cuboid_1'
local GRIPPER_SIGNAL_NAME = 'RG2_open'
local UR5_MODEL_OBJECT_NAME = '/UR5'

-- ==============================================
-- ?? ?Lua JSON?????????????json???
-- ==============================================
local json = {}
local function kind_of(obj)
    if type(obj) ~= 'table' then return type(obj) end
    local i = 1
    for _ in pairs(obj) do
        if obj[i] ~= nil then i = i + 1 else return 'table' end
    end
    if i == 1 then return 'table' else return 'array' end
end

local function escape_str(s)
    local in_char  = {'\\', '"', '/', '\b', '\f', '\n', '\r', '\t'}
    local out_char = {'\\', '"', '/',  'b',  'f',  'n',  'r',  't'}
    for i, c in ipairs(in_char) do
        s = s:gsub(c, '\\' .. out_char[i])
    end
    return s
end

function json.encode(obj)
    local kind = kind_of(obj)
    if kind == 'array' then
        local contents = ''
        for _, v in ipairs(obj) do
            contents = contents .. json.encode(v) .. ','
        end
        return '[' .. contents:sub(1, -2) .. ']'
    elseif kind == 'table' then
        local contents = ''
        for k, v in pairs(obj) do
            contents = contents .. json.encode(tostring(k)) .. ':' .. json.encode(v) .. ','
        end
        return '{' .. contents:sub(1, -2) .. '}'
    elseif kind == 'string' then
        return '"' .. escape_str(obj) .. '"'
    elseif kind == 'number' then
        return tostring(obj)
    elseif kind == 'boolean' then
        return tostring(obj)
    elseif obj == nil then
        return 'null'
    else
        error('Cannot encode ' .. kind)
    end
end

local function skip_space(str, pos)
    while pos <= #str and str:match('%s', pos) do
        pos = pos + 1
    end
    return pos
end

local function parse_str(str, pos)
    pos = pos + 1
    local end_pos = pos
    while end_pos <= #str and str:sub(end_pos, end_pos) ~= '"' do
        if str:sub(end_pos, end_pos) == '\\' then
            end_pos = end_pos + 1
        end
        end_pos = end_pos + 1
    end
    local s = str:sub(pos, end_pos - 1)
    s = s:gsub('\\(["\\/bfnrt])', {['"']='"', ['\\']='\\', ['/']='/', ['b']='\b', ['f']='\f', ['n']='\n', ['r']='\r', ['t']='\t'})
    return s, end_pos + 1
end

local function parse_num(str, pos)
    local end_pos = pos
    while end_pos <= #str and str:sub(end_pos, end_pos):match('[%d%.%-+eE]') do
        end_pos = end_pos + 1
    end
    return tonumber(str:sub(pos, end_pos - 1)), end_pos
end

local function parse_val(str, pos)
    pos = skip_space(str, pos)
    local c = str:sub(pos, pos)
    if c == '"' then
        return parse_str(str, pos)
    elseif c == '{' then
        return parse_obj(str, pos)
    elseif c == '[' then
        return parse_arr(str, pos)
    elseif c == 't' or c == 'f' then
        local val = str:sub(pos, pos + 3)
        return val == 'true', pos + 4
    elseif c == 'n' then
        return nil, pos + 4
    else
        return parse_num(str, pos)
    end
end

function parse_arr(str, pos)
    pos = pos + 1
    local arr = {}
    pos = skip_space(str, pos)
    if str:sub(pos, pos) == ']' then
        return arr, pos + 1
    end
    while true do
        local val
        val, pos = parse_val(str, pos)
        table.insert(arr, val)
        pos = skip_space(str, pos)
        if str:sub(pos, pos) == ']' then
            return arr, pos + 1
        end
        pos = pos + 1
    end
end

function parse_obj(str, pos)
    pos = pos + 1
    local obj = {}
    pos = skip_space(str, pos)
    if str:sub(pos, pos) == '}' then
        return obj, pos + 1
    end
    while true do
        local key, val
        key, pos = parse_val(str, pos)
        pos = skip_space(str, pos)
        pos = pos + 1
        val, pos = parse_val(str, pos)
        obj[key] = val
        pos = skip_space(str, pos)
        if str:sub(pos, pos) == '}' then
            return obj, pos + 1
        end
        pos = pos + 1
    end
end

function json.decode(str)
    local val, pos = parse_val(str, 1)
    return val
end

local function getTableLength(value)
    if type(value) == "table" then
        return #value
    end
    return "not-table"
end

local function decodePathJson(content)
    local luaExpression = content:gsub("%[", "{"):gsub("%]", "}")
    local chunk, err = load("return " .. luaExpression)
    if not chunk then
        error(err)
    end
    return chunk()
end
-- ==============================================
-- ?? JSON????
-- ==============================================

local gripperHandle = sim.getObject('./RG2')

function loadJsonFile(filePath)
    print("Attempting to load JSON file: " .. filePath)
    local file, err = io.open(filePath, "r")
    if not file then
        print("ERROR: Failed to open file: " .. tostring(err))
        return nil
    end
    local content = file:read("*a")
    file:close()
    if not content or #content == 0 then
        print("ERROR: JSON file is empty: " .. filePath)
        return nil
    end

    local ok = false
    local data = nil

    -- Prefer CoppeliaSim's built-in JSON decoder when available.
    if sim and sim.json and sim.json.decode then
        ok, data = pcall(sim.json.decode, content)
    end

    -- Fallback to the local Lua JSON decoder.
    if (not ok) or type(data) ~= "table" then
        ok, data = pcall(json.decode, content)
    end

    -- The path files contain only nested arrays of numbers, so they can be
    -- safely converted from JSON brackets to Lua table braces if generic JSON
    -- decoders are unavailable or incompatible.
    if (not ok) or type(data) ~= "table" then
        ok, data = pcall(decodePathJson, content)
    end

    if not ok then
        print("ERROR: Failed to decode JSON file: " .. filePath)
        print(tostring(data))
        return nil
    end
    print("JSON file loaded successfully. Lua type: " .. tostring(type(data)) .. ", length: " .. tostring(getTableLength(data)))
    return data
end

function ensureRuntimeHandles()
    if jh and #jh == 6 then
        return true
    end

    jh = {}
    jt = {}
    for i = 1, 6 do
        jh[i] = sim.getObject("/joint"..i)
        jt[i] = sim.getJointType(jh[i])
    end
    simBase = sim.getObject(".")
    simTarget = sim.getObject("./ikTarget")
    simTip = sim.getObject("./UR5_Gripper_Tip")

    ikEnv=simIK.createEnvironment()
    ikGroup=simIK.createGroup(ikEnv)
    local _,simToIkMap=simIK.addElementFromScene(ikEnv,ikGroup,simBase,simTip,simTarget,simIK.constraint_pose)
    ikJoints={}
    for i=1,#jh,1 do
        ikJoints[i]=simToIkMap[jh[i]]
    end
    ikTip=simToIkMap[simTip]
    metric={0.2,1,0.8,0.1,0.1,0.1}
    previousVelocities = previousVelocities or {}
    for i = 1, 6 do
        previousVelocities[i] = previousVelocities[i] or 0
    end
    return true
end

-- ?? ????????????????????
function visualizePath(path)
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,3,0,-1,99999,{1,0,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local lb=sim.setStepping(true)
        local initConfig=getConfig()
        local l=#jh
        local pc=#path/l
        for i=1,pc-1,1 do
            local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6]}
            local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6]}
            setConfig(config1)
            local lineDat=sim.getObjectPosition(simTip)
            setConfig(config2)
            local p=sim.getObjectPosition(simTip)
            lineDat[4]=p[1]
            lineDat[5]=p[2]
            lineDat[6]=p[3]
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
        setConfig(initConfig)
        sim.setStepping(lb)
    end
    sim.step()
end

function _getJointPosDifference(startValue,goalValue,isRevolute)
    local dx=goalValue-startValue
    if (isRevolute) then
        if (dx>=0) then
            dx=math.mod(dx+math.pi,2*math.pi)-math.pi
        else
            dx=math.mod(dx-math.pi,2*math.pi)+math.pi
        end
    end
    return(dx)
end

function _applyJoints(jointHandles,joints)
    for i=1,#jointHandles,1 do
        sim.setJointTargetPosition(jointHandles[i],joints[i])
    end
end

function generatePathLengths(path)
    local d=0
    local l=#jh
    local pc=#path/l
    local retLengths={0}
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6],path[(i-1)*l+7]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6],path[i*l+7]}
        d=d+getConfigConfigDistance(config1,config2)
        retLengths[i+1]=d
    end
    return retLengths
end

function getConfig()
    local config={}
    for i=1,#jh,1 do
        config[i]=sim.getJointPosition(jh[i])
    end
    return config
end

function setConfig(config)
    if config then
        for i=1,#jh,1 do
            sim.setJointPosition(jh[i],config[i])
        end
    end
end

function getConfigConfigDistance(config1,config2)
    local d=0
    for i=1,#jh,1 do
        local dx=(config1[i]-config2[i])*metric[i]
        d=d+dx*dx
    end
    return math.sqrt(d)
end

function getPathLength(path)
    local d=0
    local l=#jh
    local pc=#path/l
    for i=1,pc-1,1 do
        local config1={path[(i-1)*l+1],path[(i-1)*l+2],path[(i-1)*l+3],path[(i-1)*l+4],path[(i-1)*l+5],path[(i-1)*l+6]}
        local config2={path[i*l+1],path[i*l+2],path[i*l+3],path[i*l+4],path[i*l+5],path[i*l+6]}
        d=d+getConfigConfigDistance(config1,config2)
    end
    return d
end

function writeToFile(filename, data)
    local file = io.open(filename, "w")
    if file then
        file:write(data)
        file:close()
        print("Wrote file: " .. filename)
    else
        print("ERROR: Error writing to file: " .. filename)
    end
end

function recordJointData(dataCollection, pathIndex, dt)
    if not ENABLE_JOINT_DATA_RECORDING then
        return
    end
    local positions = {}
    local velocities = {}
    local accelerations = {}
    local torques = {}
    for i = 1, 6 do
        positions[i] = sim.getJointPosition(jh[i])
        local velocity = sim.getJointVelocity(jh[i])
        torque = sim.getJointForce(jh[i])
        velocities[i] = velocity
        torques[i] = torque
        accelerations[i] = (velocity - previousVelocities[i]) / dt
        previousVelocities[i] = velocity
    end
    table.insert(dataCollection, {index = pathIndex, positions = positions, velocities = velocities, accelerations = accelerations, torques = torques, time_step = dt})
end

function insertEmptyRow(dataCollection)
    if not ENABLE_JOINT_DATA_RECORDING then
        return
    end
    table.insert(dataCollection, {index = "separator", positions = {}, velocities = {}, torques = {}})
end

function executeMotion(path,lengths,maxVel,maxAccel,maxJerk, dataCollection, pathIndex)
    dt=sim.getSimulationTimeStep()
    jointsUpperVelocityLimits={}
    for j=1,6,1 do
        jointsUpperVelocityLimits[j]=sim.getObjectFloatParam(jh[j],sim.jointfloatparam_maxvel)
    end
    velCorrection=1
    sim.setAutoYieldDelay(0.2)
    while true do
        posVelAccel={0,0,0}
        targetPosVel={lengths[#lengths],0}
        pos=0
        res=0
        previousQ={path[1],path[2],path[3],path[4],path[5],path[6]}
        local rMax=0
        rmlHandle=sim.ruckigPos(1,0.0001,-1,posVelAccel,{maxVel*velCorrection,maxAccel,maxJerk},{1},targetPosVel)
        while res==0 do
            res,posVelAccel,sync=sim.ruckigStep(rmlHandle,dt)
            if (res>=0) then
                l=posVelAccel[1]
                for i=1,#lengths-1,1 do
                    l1=lengths[i]
                    l2=lengths[i+1]
                    if (l>=l1)and(l<=l2) then
                        t=(l-l1)/(l2-l1)
                        for j=1,6,1 do
                            q=path[6*(i-1)+j]+_getJointPosDifference(path[6*(i-1)+j],path[6*i+j],jt[j]==sim.joint_revolute_subtype)*t
                            dq=_getJointPosDifference(previousQ[j],q,jt[j]==sim.joint_revolute_subtype)
                            previousQ[j]=q
                            r=math.abs(dq/dt)/jointsUpperVelocityLimits[j]
                            if (r>rMax) then
                                rMax=r
                            end
                        end
                        break
                    end
                end
            end
        end
        sim.ruckigRemove(rmlHandle)
        if rMax>1.001 then
            velCorrection=velCorrection/rMax
        else
            break
        end
    end
    sim.setAutoYieldDelay(0.002)
    posVelAccel = {0, 0, 0}
    targetPosVel = {lengths[#lengths], 0}
    pos = 0
    res = 0
    jointPos = {}
    rmlHandle = sim.ruckigPos(1, 0.0001, -1, posVelAccel, {maxVel * velCorrection, maxAccel, maxJerk}, {1}, targetPosVel)
    while res == 0 do
        dt = sim.getSimulationTimeStep()
        res, posVelAccel, sync = sim.ruckigStep(rmlHandle, dt)
        if (res >= 0) then
            l = posVelAccel[1]
            for i = 1, #lengths - 1, 1 do
                l1 = lengths[i]
                l2 = lengths[i + 1]
                if (l >= l1) and (l <= l2) then
                    t = (l - l1) / (l2 - l1)
                    for j = 1, 6 do
                        jointPos[j] = path[6 * (i - 1) + j] + _getJointPosDifference(path[6 * (i - 1) + j], path[6 * i + j], jt[j] == sim.joint_revolute_subtype) * t
                    end
                    _applyJoints(jh, jointPos)
                    recordJointData(dataCollection, pathIndex, dt)
                    break
                end
            end
        end
        sim.step()
    end
    sim.ruckigRemove(rmlHandle)
end

function callback(pose,velocity,accel,auxData)
    sim.setObjectPose(auxData.target,pose)
    simIK.handleGroup(ikEnv,auxData.ikGroup,{syncWorlds=true})
end

function moveToPose(maxVelocity,maxAcceleration,maxJerk,targetPose,auxData)
    print("Hi, going down")
    local currentPose=sim.getObjectPose(auxData.target)
    local q_prev = currentPose
    return sim.moveToPose(-1,currentPose,maxVelocity,maxAcceleration,maxJerk,targetPose,callback,auxData,{1,1,1,0.1})
end

function generateIkPath(startConfig,goalPose,steps,ignoreCollisions, auxData)
    local lb=sim.setStepping(true)
    local currentConfig=getConfig()
    setConfig(startConfig)
    sim.setObjectPose(simTarget,goalPose)
    local val=validationCb
    if ignoreCollisions then
        val=nil
    end
    simIK.syncFromSim(ikEnv,{ikGroup})    
    local c=simIK.generatePath(ikEnv,ikGroup,ikJoints,ikTip,steps,val)
    setConfig(currentConfig)
    sim.setStepping(lb)
    if #c/6>0 then
        local d={}
        for i=1,#c/6,1 do
            for j=1,6,1 do
                d[(i-1)*6+j]=c[(i-1)*6+j]
            end
        end
        return d, generatePathLengths(d)
    end
end

function reversePath(path)
    local reversedPath = {}
    local numConfigs = #path / 6
    for i = numConfigs, 1, -1 do
        for j = 1, 6 do
            table.insert(reversedPath, path[(i-1)*6 + j])
        end
    end
    return reversedPath
end

function flattenPath(path)
    local flatPath = {}
    if not path then
        return flatPath
    end
    for i = 1, #path do
        for j = 1, #path[i] do
            table.insert(flatPath, path[i][j])
        end
    end
    return flatPath
end

function printObjectTipDistance(objectName, tipHandle, label)
    local ok, objectHandle = pcall(sim.getObject, objectName)
    if not ok or not objectHandle then
        print(label .. ": object not found: " .. tostring(objectName))
        return
    end
    local tipPos = sim.getObjectPosition(tipHandle, sim.handle_world)
    local objPos = sim.getObjectPosition(objectHandle, sim.handle_world)
    local dx = tipPos[1] - objPos[1]
    local dy = tipPos[2] - objPos[2]
    local dz = tipPos[3] - objPos[3]
    local dist = math.sqrt(dx * dx + dy * dy + dz * dz)
    print(label .. ": " .. tostring(objectName) ..
        " tip=(" .. tostring(tipPos[1]) .. "," .. tostring(tipPos[2]) .. "," .. tostring(tipPos[3]) .. ")" ..
        " obj=(" .. tostring(objPos[1]) .. "," .. tostring(objPos[2]) .. "," .. tostring(objPos[3]) .. ")" ..
        " delta=(" .. tostring(dx) .. "," .. tostring(dy) .. "," .. tostring(dz) .. ")" ..
        " dist=" .. tostring(dist))
end

local firstAttachedObjectHandle = nil
local firstAttachedStaticState = nil
local firstAttachedRespondableState = nil

function setObjectIntParamIfAvailable(handle, param, value)
    if param then
        pcall(sim.setObjectInt32Param, handle, param, value)
    end
end

function getObjectIntParamIfAvailable(handle, param)
    if param then
        local ok, value = pcall(sim.getObjectInt32Param, handle, param)
        if ok then
            return value
        end
    end
    return nil
end

function attachFirstObjectToGripper(parentHandle)
    if not ENABLE_FIRST_OBJECT_FALLBACK then
        return
    end

    local ok, objectHandle = pcall(sim.getObject, FIRST_OBJECT_NAME)
    if not ok or not objectHandle then
        print("FIRST OBJECT ATTACH ERROR: cannot find " .. tostring(FIRST_OBJECT_NAME))
        return
    end

    firstAttachedStaticState = getObjectIntParamIfAvailable(objectHandle, sim.shapeintparam_static)
    firstAttachedRespondableState = getObjectIntParamIfAvailable(objectHandle, sim.shapeintparam_respondable)

    -- Freeze physics while the object is carried, otherwise it can jitter against the gripper links.
    setObjectIntParamIfAvailable(objectHandle, sim.shapeintparam_static, 1)
    setObjectIntParamIfAvailable(objectHandle, sim.shapeintparam_respondable, 0)
    pcall(sim.resetDynamicObject, objectHandle)
    sim.setObjectParent(objectHandle, parentHandle, true)
    firstAttachedObjectHandle = objectHandle
    print("FIRST OBJECT ATTACH: attached " .. tostring(FIRST_OBJECT_NAME) .. " to gripper.")
end

function detachFirstObjectFromGripper(restorePhysics)
    if not firstAttachedObjectHandle then
        return
    end

    sim.setObjectParent(firstAttachedObjectHandle, -1, true)
    if restorePhysics then
        if firstAttachedStaticState ~= nil then
            setObjectIntParamIfAvailable(firstAttachedObjectHandle, sim.shapeintparam_static, firstAttachedStaticState)
        end
        if firstAttachedRespondableState ~= nil then
            setObjectIntParamIfAvailable(firstAttachedObjectHandle, sim.shapeintparam_respondable, firstAttachedRespondableState)
        end
    else
        -- Keep the manually attached first object frozen at the placement pose.
        -- This avoids a contact impulse from the closed/opening gripper or table.
        setObjectIntParamIfAvailable(firstAttachedObjectHandle, sim.shapeintparam_static, 1)
        setObjectIntParamIfAvailable(firstAttachedObjectHandle, sim.shapeintparam_respondable, 0)
    end
    pcall(sim.resetDynamicObject, firstAttachedObjectHandle)
    print("FIRST OBJECT ATTACH: detached first object.")
    firstAttachedObjectHandle = nil
end

function sysCall_thread()
    print("Coroutine has started")
    print("UR5_SCRIPT_ROS_BRIDGE_MODE")
    print("ROS should call rosExecutePath() on this child script.")
    print("The gripper signal is controlled by ROS through " .. GRIPPER_SIGNAL_NAME .. ".")

    ensureRuntimeHandles()

    sim.setInt32Signal(GRIPPER_SIGNAL_NAME, 1)
    sim.wait(0.1)

    local lastGripState = 1
    while sim.getSimulationState() ~= sim.simulation_stopping do
        local gripState = sim.getInt32Signal(GRIPPER_SIGNAL_NAME)
        if gripState ~= nil and gripState ~= lastGripState then
            if gripState == 0 then
                print("GRIPPER: close request received.")
                attachFirstObjectToGripper(simTip)
            elseif gripState == 1 then
                print("GRIPPER: open request received.")
                detachFirstObjectFromGripper(false)
            end
            lastGripState = gripState
        end
        sim.wait(0.05)
    end
end

function rosPing()
    return true
end

function rosExecutePath(pathJson, pathLabel)
    ensureRuntimeHandles()
    local path = pathJson
    if type(pathJson) == "string" then
        local ok, decoded = pcall(json.decode, pathJson)
        if ok and type(decoded) == "table" then
            path = decoded
        else
            local ok2, decoded2 = pcall(decodePathJson, pathJson)
            if ok2 and type(decoded2) == "table" then
                path = decoded2
            end
        end
    end

    if type(path) ~= "table" then
        error("rosExecutePath expects a table or JSON array of waypoints, got " .. tostring(type(pathJson)))
    end

    local flatPath = flattenPath(path)
    if #flatPath == 0 then
        error("rosExecutePath received an empty path")
    end

    local lengths = generatePathLengths(flatPath)
    local maxVel = (2*math.pi -0.1) * MOTION_SPEED_SCALE
    local maxAccel = (2*math.pi -0.1) * MOTION_SPEED_SCALE
    local maxJerk = 80 * MOTION_SPEED_SCALE

    if pathLabel == "close" then
        sim.setInt32Signal(GRIPPER_SIGNAL_NAME, 0)
    else
        sim.setInt32Signal(GRIPPER_SIGNAL_NAME, 1)
    end

    -- Visualization is disabled in ROS mode to avoid state transitions that
    -- can conflict with the running simulation state.

    local dataCollection = {}
    executeMotion(flatPath, lengths, maxVel, maxAccel, maxJerk, dataCollection, 1)
    return true
end
