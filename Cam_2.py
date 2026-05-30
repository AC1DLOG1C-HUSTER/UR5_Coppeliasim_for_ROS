function sysCall_init()

    -- Get the handle for the vision sensor

    visionSensorHandle = sim.getObject('.')



    -- Get handles and original visibility layers for the objects of interest

    objectsOfInterest = {'Cuboid_1', 'Cuboid_2', 'Cuboid_3', 'Cylinder_4', 'Sphere_5', 'Sphere_6'}

    objectsOfInterestHandles = {}

    originalLayers = {}



    -- Get all objects in the scene

    allObjects = sim.getObjectsInTree(sim.handle_scene, sim.object_shape_type, 0)



    for i = 1, #allObjects do

        local objName = sim.getObjectAlias(allObjects[i], -1)

        originalLayers[allObjects[i]] = sim.getObjectInt32Param(allObjects[i], sim.objintparam_visibility_layer)

        if isObjectOfInterest(objName) then

            table.insert(objectsOfInterestHandles, allObjects[i])

        end

    end

end



function isObjectOfInterest(name)

    for i = 1, #objectsOfInterest do

        if objectsOfInterest[i] == name then

            return true

        end

    end

    return false

end



function take_image_and_save_it()

    -- Make non-interest objects invisible

    for i = 1, #allObjects do

        if not isObjectOfInterest(sim.getObjectAlias(allObjects[i], -1)) then

            sim.setObjectInt32Param(allObjects[i], sim.objintparam_visibility_layer, 256)

        end

    end



    -- Handle the vision sensor to update its image

    sim.handleVisionSensor(visionSensorHandle)



    -- Capture an image from the vision sensor

    local image, resolution = sim.getVisionSensorImg(visionSensorHandle, 0, 0.0, {0, 0}, {0, 0})

    

    -- Restore the visibility of all objects

    for i = 1, #allObjects do

        sim.setObjectInt32Param(allObjects[i], sim.objintparam_visibility_layer, originalLayers[allObjects[i]])

    end



    -- Process and save the image

    local timestamp = os.date("%Y%m%d%H%M%S")

    local filePath = 'C:/Users/youssef/UR5_robot_pose_est_and_coppeliasim_connection/Saved_images_from_coppeliasim_scene/vision_sensor_image_'.. timestamp .. '.png'

    sim.saveImage(image, resolution, 0, filePath, 100)

end



function sysCall_sensing()

    -- Check for the signal from an external trigger

    local signalValue = sim.getInt32Signal('capture_signal')



    if signalValue then

        take_image_and_save_it()



        -- Reset the signal to wait for the next trigger

        sim.clearStringSignal('capture_signal')

    end

end



function sysCall_cleanup()

    -- Restore the visibility of all objects to their original state when the simulation ends

    for i = 1, #allObjects do

        sim.setObjectInt32Param(allObjects[i], sim.objintparam_visibility_layer, originalLayers[allObjects[i]])

    end

end

