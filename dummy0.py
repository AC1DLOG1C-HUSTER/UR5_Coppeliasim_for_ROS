function sysCall_init()
    -- Keep compatibility across CoppeliaSim versions.
    json = sim.json or simJSON
    if not json then
        json = {}

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
    end
end


function getMeshDataWithPath(handle)
    local vertices, indices = sim.getShapeMesh(handle)
    local objectPath = sim.getObjectAlias(handle, -1)
    
    -- Ensure the object path starts with '/'
    objectPath = '/' .. objectPath

    return objectPath, vertices, indices
end

-- Function to collect all mesh data
function getAllMeshData()
    local allMeshData = {}
    local shapeIndex = 0
    while true do
        local h = sim.getObjects(shapeIndex, sim.object_shape_type)
        if h < 0 then break end
        shapeIndex = shapeIndex + 1
        local objectPath, vertices, indices = getMeshDataWithPath(h)
        allMeshData[objectPath] = {vertices=vertices, indices=indices}
    end
    return allMeshData
end

-- Function to save the mesh data to a JSON file
function saveMeshDataToJSON(allMeshData, filename)
    local file = io.open(filename, "w")
    if file then
        local jsonString = json.encode(allMeshData)
        file:write(jsonString)
        file:close()
        print("Mesh data saved to " .. filename)
    else
        print("Could not open " .. filename .. " for writing.")
    end
end

-- Exposed callable function for the remote API
function getAndSaveMeshData(filename)
    local allMeshData = getAllMeshData()
    saveMeshDataToJSON(allMeshData, filename)
    return true
end
