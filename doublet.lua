-- This script will preform a control surface doublet
-- Charles Johnson, OSU 2020

local DOUBLET_ACTION_CHANNEL = 6 -- RCIN channel to start a doublet when high (>1700)
local DOUBLET_CHOICE_CHANNEL1 = 7 -- RCIN channel to choose elevator (low) or rudder (medium) or other (high)
local DOUBLET_CHOICE_CHANNEL2 = 5 -- RCIN channel to choose aileron (low) or throttle (medium) or other (high)
local DOUBLET_FUNCTION1 = 77 -- which control surface (SERVOx_FUNCTION) number will have a doublet happen
local DOUBLET_FUNCTION2 = 78 -- which control surface (SERVOx_FUNCTION) number will have a doublet happen
local DOUBLET_MAGNITUDE = 6 -- defined out of 45 deg used for set_output_scaled
local DOUBLET_TIME = 500 -- period of doublet signal in ms
local OBSERVATION_TIME = 5 -- multiple of the doublet time to hold other deflections constant


-- flight mode numbers for plane https://mavlink.io/en/messages/ardupilotmega.html
local MODE_MANUAL = 0
local MODE_FBWA = 5
local MODE_FBWB = 6
local MODE_RTL = 11
local K_AILERON = 4
local K_ELEVATOR = 19
local K_THROTTLE = 70
local K_RUDDER = 21
local K_ELEVONLEFT = 77
local K_ELEVONRIGHT = 78

-- elevon direction setup
local opposite_elevon_motion = 1;

-- store the info between callbacks
-- set at the start of each doublet
local start_time = -1
local end_time = -1
local now = -1

-- store information about the vehicle
local doublet_srv_chan1 = SRV_Channels:find_channel(DOUBLET_FUNCTION1)
local doublet_srv_min1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_MIN")
local doublet_srv_max1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_MAX")
local doublet_srv_trim1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_TRIM")
local pre_doublet_mode = vehicle:get_mode()
local current_hagl = ahrs:get_hagl()

function retry_set_mode(mode)
    if vehicle:set_mode(mode) then
        -- if the mode was set successfully, carry on as normal
        return doublet, 1
    else
        -- if the mode was not set successfully, try again ASAP
        return retry_set_mode, 1
    end
end

function doublet()
    local callback_time = 100
    if arming:is_armed() == true and rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 and end_time ==-1 then
        callback_time = DOUBLET_TIME / 10
        -- start a quick doublet based on some math/logic
        now = millis()
        if start_time == -1 then
            -- this is the time that we started
            -- stop time will be start_time + DOUBLET_TIME
            start_time = now
            
            -- where are we doing the doublet? set the other controls to trim
            local doublet_choice_pwm1 = rc:get_pwm(DOUBLET_CHOICE_CHANNEL1)
            local doublet_choice_pwm2 = rc:get_pwm(DOUBLET_CHOICE_CHANNEL2)
            local trim_funcs = {}
            local pre_doublet_elevator1 = SRV_Channels:get_output_pwm(K_ELEVONLEFT)
            local pre_doublet_elevator2 = SRV_Channels:get_output_pwm(K_ELEVONRIGHT)
            local pre_doublet_throttle = SRV_Channels:get_output_pwm(K_THROTTLE)
            opposite_elevon_motion = 1; -- setting initial rotation here

            if doublet_choice_pwm1 < 1300 then
                -- doublet on elevator
                opposite_elevon_motion = opposite_elevon_motion; -- choosing elevon rotation
                DOUBLET_FUNCTION1 = K_ELEVONLEFT
                DOUBLET_FUNCTION2 = K_ELEVONRIGHT
                trim_funcs = {K_RUDDER}
                DOUBLET_MAGNITUDE = 12
                doublet_srv_trim1 = pre_doublet_elevator1
                doublet_srv_trim2 = pre_doublet_elevator2
            elseif doublet_choice_pwm1 > 1300 and doublet_choice_pwm1 < 1700 then
                -- doublet on rudder
                DOUBLET_FUNCTION1 = K_RUDDER
                trim_funcs = {}
                DOUBLET_MAGNITUDE = 15
                -- pin elevator to current position. This is most likely different than the _TRIM value
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONLEFT), pre_doublet_elevator1, DOUBLET_TIME * 4)
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONRIGHT), pre_doublet_elevator2, DOUBLET_TIME * 4)
            elseif doublet_choice_pwm1 > 1700 and doublet_choice_pwm2 < 1300 then
                -- doublet on aileron
                opposite_elevon_motion = 0 - opposite_elevon_motion; -- choosing elevon rotation
                DOUBLET_FUNCTION1 = K_ELEVONLEFT
                DOUBLET_FUNCTION2 = K_ELEVONRIGHT
                trim_funcs = {K_RUDDER}
                DOUBLET_MAGNITUDE = 5
                -- pin elevator to current position. This is most likely different than the _TRIM value
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONLEFT), pre_doublet_elevator1, DOUBLET_TIME * 4)
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONRIGHT), pre_doublet_elevator2, DOUBLET_TIME * 4)
            elseif doublet_choice_pwm1 > 1700 and doublet_choice_pwm2 > 1300 and doublet_choice_pwm2 < 1700 then
                -- doublet on thrust
                DOUBLET_FUNCTION1 = K_THROTTLE
                trim_funcs = {K_RUDDER}
                DOUBLET_MAGNITUDE = 5
                -- pin elevator to current position. This is most likely different than the _TRIM value
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONLEFT), pre_doublet_elevator1, DOUBLET_TIME * 4)
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONRIGHT), pre_doublet_elevator2, DOUBLET_TIME * 4)
            end
            -- notify the gcs that we are starting a doublet
            gcs:send_text(6, "STARTING DOUBLET " .. DOUBLET_FUNCTION1)

            -- get info about the doublet channel
            doublet_srv_chan1 = SRV_Channels:find_channel(DOUBLET_FUNCTION1)
            doublet_srv_min1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_MIN")
            doublet_srv_max1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_MAX")
            doublet_srv_trim1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_TRIM")
            pre_doublet_mode = vehicle:get_mode()

            -- set the channels that need to be still to trim until the doublet is done
            for i = 1, #trim_funcs do
                local trim_chan = SRV_Channels:find_channel(trim_funcs[i])
                local trim_pwm = param:get("SERVO" .. trim_chan + 1 .. "_TRIM")
                SRV_Channels:set_output_pwm_chan_timeout(trim_chan, trim_pwm, DOUBLET_TIME * OBSERVATION_TIME)
            end

            if doublet_choice_pwm1 < 1700 or doublet_choice_pwm2 < 1300 or doublet_choice_pwm2 > 1700 then 
                -- get the current throttle PWM and pin it there until the doublet is done
                local pre_doublet_throttle = SRV_Channels:get_output_pwm(K_THROTTLE)
                SRV_Channels:set_output_pwm_chan_timeout(
                    SRV_Channels:find_channel(K_THROTTLE),
                    pre_doublet_throttle,
                    DOUBLET_TIME * OBSERVATION_TIME
                )
            elseif doublet_choice_pwm1 > 1700 and doublet_choice_pwm2 > 1300 and doublet_choice_pwm2 < 1700 then
                -- when the throttle is the one having the doublet execute this
                doublet_srv_chan1 = SRV_Channels:find_channel(DOUBLET_FUNCTION1)
                doublet_srv_trim1 = pre_doublet_throttle
            end
            -- enter manual mode
            retry_set_mode(MODE_MANUAL)
        end
        
        if doublet_choice_pwm1 < 1300 or (doublet_choice_pwm1 > 1700 and doublet_choice_pwm2 < 1300) then
            -- elevator or aileron doublet, setting up the other elevon
            doublet_srv_chan2 = SRV_Channels:find_channel(DOUBLET_FUNCTION2)
            doublet_srv_min2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_MIN")
            doublet_srv_max2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_MAX")
            doublet_srv_trim2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_TRIM")

            -- split time evenly between high and low signal
            if now < start_time + (DOUBLET_TIME / 2) then
                down = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45))
                up = doublet_srv_trim2 + (math.floor((doublet_srv_max2 - doublet_srv_trim2) * (DOUBLET_MAGNITUDE / 45)) * opposite_elevon_motion)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, DOUBLET_TIME / 2 + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, up, DOUBLET_TIME / 2 + 100)
            elseif now < start_time + DOUBLET_TIME then
                up = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45))
                down = doublet_srv_trim2 - (math.floor((doublet_srv_trim2 - doublet_srv_min2) * (DOUBLET_MAGNITUDE / 45)) * opposite_elevon_motion)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, DOUBLET_TIME / 2 + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, down, DOUBLET_TIME / 2 + 100)
            elseif now < start_time + (DOUBLET_TIME * 2) then
                -- stick fixed at pre doublet trim position
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, doublet_srv_trim1, DOUBLET_TIME * 2)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, doublet_srv_trim2, DOUBLET_TIME * 2)
            elseif (now > start_time + (DOUBLET_TIME * 2)) and (now < start_time + (DOUBLET_TIME * 2) + callback_time) then
                -- notify GCS
                gcs:send_text(6, "DOUBLET FINISHED")
            elseif (now > start_time + (DOUBLET_TIME * 2) + callback_time) and (now < start_time + (DOUBLET_TIME * OBSERVATION_TIME)) then
                -- do nothing until recording is complete
            elseif now > start_time + (DOUBLET_TIME * OBSERVATION_TIME) then
                -- wait for RC input channel to go low
                end_time = now
                gcs:send_text(6, "DOUBLET OBSERVATION FINISHED")
            else
                gcs:send_text(6, "this should not be reached")
            end
        else
            -- split time evenly between high and low signal
            if now < start_time + (DOUBLET_TIME / 2) then
                down = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, DOUBLET_TIME / 2 + 100)
            elseif now < start_time + DOUBLET_TIME then
                up = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, DOUBLET_TIME / 2 + 100)
            elseif now < start_time + (DOUBLET_TIME * 2) then
                -- stick fixed at pre doublet trim position
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, doublet_srv_trim1, DOUBLET_TIME * 2)
            elseif (now > start_time + (DOUBLET_TIME * 2)) and (now < start_time + (DOUBLET_TIME * 2) + callback_time) then
                -- notify GCS
                gcs:send_text(6, "DOUBLET FINISHED")
            elseif (now > start_time + (DOUBLET_TIME * 2) + callback_time) and (now < start_time + (DOUBLET_TIME * OBSERVATION_TIME)) then
                -- do nothing until recording is complete
            elseif now > start_time + (DOUBLET_TIME * OBSERVATION_TIME) then
                -- wait for RC input channel to go low
                end_time = now
                gcs:send_text(6, "DOUBLET OBSERVATION FINISHED")
            else
                gcs:send_text(6, "this should not be reached")
            end
        end

        
    
    elseif end_time ~= -1 and rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 then -- interlude time
        -- wait for RC input channel to go low
        gcs:send_text(6, "RC" .. DOUBLET_ACTION_CHANNEL .. " still high")
        callback_time = 1000 -- prevents spamming messages to the GCS
    
    elseif now ~= -1 and end_time ~= -1 then -- normal exit
        gcs:send_text(6, "RETURN TO PREVIOUS FLIGHT MODE")
        now = -1
        end_time = -1
        start_time = -1
        -- clear all of the timeouts
        control_functions = {K_ELEVONLEFT, K_ELEVONRIGHT, K_THROTTLE, K_RUDDER}
        for i = 1, 4 do
            local control_chan = SRV_Channels:find_channel(control_functions[i])
            SRV_Channels:set_output_pwm_chan_timeout(control_chan, param:get("SERVO" .. control_chan + 1 .. "_TRIM"), 0)
        end
        retry_set_mode(pre_doublet_mode)
        callback_time = 100 -- don't need to rerun for a little while
    
    elseif now ~= -1 then -- emergency recovery
        -- stopped before finishing. recover to level attitude
        gcs:send_text(4, "FBWA RECOVER")
        now = -1
        end_time = -1
        start_time = -1
        -- clear all of the timeouts
        control_functions = {K_ELEVONLEFT, K_ELEVONRIGHT, K_THROTTLE, K_RUDDER}
        for i = 1, 4 do
            local control_chan = SRV_Channels:find_channel(control_functions[i])
            SRV_Channels:set_output_pwm_chan_timeout(control_chan, param:get("SERVO" .. control_chan + 1 .. "_TRIM"), 0)
        end
        retry_set_mode(MODE_FBWA)
        callback_time = 100
    end
    return doublet, callback_time
end

gcs:send_text(6, "doublet.lua is running")
return doublet(), 1000