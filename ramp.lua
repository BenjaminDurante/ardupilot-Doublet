---@diagnostic disable: undefined-global
-- This script will preform a control surface doublet
-- Charles Johnson, OSU 2020
-- Benjamin Durante, UofC 2021

local MANEUVER_ACTION_CHANNEL = 9 -- RCIN channel to start a doublet when high (low) or ramp (medium) or step (high)
local DOUBLET_ACTION_CHANNEL = 6 -- RCIN channel to start a doublet when high (>1700)
local DOUBLET_CHOICE_CHANNEL1 = 7 -- RCIN channel to choose elevator (low) or rudder (medium) or alternative channel (high)
local DOUBLET_CHOICE_CHANNEL2 = 5 -- RCIN channel to choose aileron (low) or throttle (medium) or other (high)
local DOUBLET_FUNCTION1 = 77 -- which control surface (SERVOx_FUNCTION) number will have a doublet happen
local DOUBLET_FUNCTION2 = 78 -- which control surface (SERVOx_FUNCTION) number will have a doublet happen

-- Doublet parameters
local DOUBLET_TIME = 1000 -- period of doublet signal in ms
local OBSERVATION_TIME = 1 -- multiple of the doublet time to hold other deflections constant
local DOUBLET_MAGNITUDE = 6 -- defined out of 45 deg used for set_output_scaled
local DOUBLET_MAGNITUDE_ELEVATOR = 12 -- elevator deflection magnitude 
local DOUBLET_MAGNITUDE_AILERON = 5 -- aileron deflection magnitude
local DOUBLET_MAGNITUDE_RUDDER = 15 -- rudder deflection magnitude
local DOUBLET_MAGNITUDE_THROTTLE = 5 -- throttle deflection magnitude

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
local opposite_elevon_motion_master = 1; -- 1 or -1 to correct for elevator/aileron mix-up
local opposite_elevon_motion = opposite_elevon_motion_master;

-- elevon doublet active
local ACTIVE_AILERON = false
local ACTIVE_ELEVATOR = false
local ACTIVE_RUDDER = false
local ACTIVE_THROTTLE = false

-- store the info between callbacks
-- set at the start of each doublet
local start_time = -1
local end_time = -1
local now = -1
local ramp_start_time = -1

-- store information about the vehicle
local doublet_srv_chan1 = SRV_Channels:find_channel(DOUBLET_FUNCTION1)
local doublet_srv_min1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_MIN")
local doublet_srv_max1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_MAX")
local doublet_srv_trim1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_TRIM")
local pre_doublet_mode = vehicle:get_mode()
local current_hagl = ahrs:get_hagl()
local doublet_srv_chan2 = SRV_Channels:find_channel(DOUBLET_FUNCTION2)
local doublet_srv_min2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_MIN")
local doublet_srv_max2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_MAX")
local doublet_srv_trim2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_TRIM")

function retry_set_mode(mode) -- sets flight modes demanded from the main code
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
    if arming:is_armed() == true and rc:get_pwm(DOUBLET_ACTION_CHANNEL) > 1700 and end_time ==-1 and rc:get_pwm(MANEUVER_ACTION_CHANNEL) > 1300 and rc:get_pwm(MANEUVER_ACTION_CHANNEL) < 1700 then
        callback_time = DOUBLET_TIME / 10
        -- start a quick doublet based on some math/logic
        now = millis()
        if start_time == -1 then
            -- reseting the doublet variable to zero
            ACTIVE_AILERON = false
            ACTIVE_ELEVATOR = false
            ACTIVE_RUDDER = false
            ACTIVE_THROTTLE = false
            
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
        
            if doublet_choice_pwm1 < 1300 then
                -- doublet on elevator
                opposite_elevon_motion = opposite_elevon_motion_master; -- choosing elevon rotation
                ACTIVE_ELEVATOR = true
                DOUBLET_FUNCTION1 = K_ELEVONLEFT
                DOUBLET_FUNCTION2 = K_ELEVONRIGHT
                trim_funcs = {K_RUDDER}
                DOUBLET_MAGNITUDE = DOUBLET_MAGNITUDE_ELEVATOR
                doublet_srv_trim1 = pre_doublet_elevator1
                doublet_srv_trim2 = pre_doublet_elevator2
            elseif doublet_choice_pwm1 > 1300 and doublet_choice_pwm1 < 1700 then
                -- doublet on rudder
                ACTIVE_RUDDER = true
                DOUBLET_FUNCTION1 = K_RUDDER
                trim_funcs = {}
                DOUBLET_MAGNITUDE = DOUBLET_MAGNITUDE_RUDDER
                -- pin elevator to current position. This is most likely different than the _TRIM value
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONLEFT), pre_doublet_elevator1, DOUBLET_TIME * OBSERVATION_TIME)
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONRIGHT), pre_doublet_elevator2, DOUBLET_TIME * OBSERVATION_TIME)
            elseif doublet_choice_pwm1 > 1700 and doublet_choice_pwm2 < 1300 then
                -- doublet on aileron
                opposite_elevon_motion = 0 - opposite_elevon_motion_master; -- choosing elevon rotation
                ACTIVE_AILERON = true
                DOUBLET_FUNCTION1 = K_ELEVONLEFT
                DOUBLET_FUNCTION2 = K_ELEVONRIGHT
                trim_funcs = {K_RUDDER}
                DOUBLET_MAGNITUDE = DOUBLET_MAGNITUDE_AILERON
                -- pin elevator to current position. This is most likely different than the _TRIM value
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONLEFT), pre_doublet_elevator1, DOUBLET_TIME * OBSERVATION_TIME)
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONRIGHT), pre_doublet_elevator2, DOUBLET_TIME * OBSERVATION_TIME)
            elseif doublet_choice_pwm1 > 1700 and doublet_choice_pwm2 > 1300 and doublet_choice_pwm2 < 1700 then
                -- doublet on thrust
                ACTIVE_THROTTLE = true
                DOUBLET_FUNCTION1 = K_THROTTLE
                trim_funcs = {K_RUDDER}
                DOUBLET_MAGNITUDE = DOUBLET_MAGNITUDE_THROTTLE
                -- pin elevator to current position. This is most likely different than the _TRIM value
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONLEFT), pre_doublet_elevator1, DOUBLET_TIME * OBSERVATION_TIME)
                SRV_Channels:set_output_pwm_chan_timeout(SRV_Channels:find_channel(K_ELEVONRIGHT), pre_doublet_elevator2, DOUBLET_TIME * OBSERVATION_TIME)
            end
            
            -- notify the gcs that we are starting a doublet
            gcs:send_text(6, "STARTING DOUBLET " .. DOUBLET_FUNCTION1)

            
            -- get info about the doublet channel
            doublet_srv_chan1 = SRV_Channels:find_channel(DOUBLET_FUNCTION1)
            doublet_srv_min1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_MIN")
            doublet_srv_max1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_MAX")
            doublet_srv_trim1 = param:get("SERVO" .. doublet_srv_chan1 + 1 .. "_TRIM")
            pre_doublet_mode = vehicle:get_mode()

            if ACTIVE_AILERON == true or ACTIVE_ELEVATOR == true then
                -- elevator or aileron doublet, setting up the other elevon
                doublet_srv_chan2 = SRV_Channels:find_channel(DOUBLET_FUNCTION2)
                doublet_srv_min2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_MIN")
                doublet_srv_max2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_MAX")
                doublet_srv_trim2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_TRIM")
            elseif ACTIVE_RUDDER == true then 
                doublet_srv_chan2 = doublet_srv_chan1 + 1
                doublet_srv_min2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_MIN")
                doublet_srv_max2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_MAX")
                doublet_srv_trim2 = param:get("SERVO" .. doublet_srv_chan2 + 1 .. "_TRIM")
            end

            -- set the rudder channels that need to be still to trim until the doublet is done
            if ACTIVE_RUDDER ~= true then 
                for i = 1,2 do
                    local trim_chan = SRV_Channels:find_channel(K_RUDDER)
                    local trim_pwm = param:get("SERVO" .. trim_chan + i .. "_TRIM")
                    SRV_Channels:set_output_pwm_chan_timeout((trim_chan + (i-1)), trim_pwm, DOUBLET_TIME * OBSERVATION_TIME)
                end
            end

            if ACTIVE_THROTTLE ~= true then 
                -- get the current throttle PWM and pin it there until the doublet is done
                local pre_doublet_throttle = SRV_Channels:get_output_pwm(K_THROTTLE)
                SRV_Channels:set_output_pwm_chan_timeout(
                    SRV_Channels:find_channel(K_THROTTLE),
                    pre_doublet_throttle,
                    DOUBLET_TIME * OBSERVATION_TIME
                )
            elseif ACTIVE_THROTTLE == true then
                -- when the throttle is the one having the doublet execute this
                doublet_srv_chan1 = SRV_Channels:find_channel(DOUBLET_FUNCTION1)
                doublet_srv_trim1 = pre_doublet_throttle
            end
            -- enter manual mode
            retry_set_mode(MODE_MANUAL)
            ramp_start_time = tonumber(tostring(now))
        end
        
        if ACTIVE_AILERON == true or ACTIVE_ELEVATOR == true then
            -- elevator or aileron doublet, setting up the other elevon
            -- split time evenly between high and low signal
            if now < start_time + (DOUBLET_TIME * 1/6) then
                down = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45) * (tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 1/6) - ramp_start_time))
                up = doublet_srv_trim2 + (math.floor((doublet_srv_max2 - doublet_srv_trim2) * (DOUBLET_MAGNITUDE / 45)) * opposite_elevon_motion * (tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 1/6) - ramp_start_time))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, up, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif now < start_time + (DOUBLET_TIME * 2/6) then 
                down = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45))
                up = doublet_srv_trim2 + (math.floor((doublet_srv_max2 - doublet_srv_trim2) * (DOUBLET_MAGNITUDE / 45)) * opposite_elevon_motion)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, up, math.floor(DOUBLET_TIME * 1/6) + 100)
                ramp_start_time = tonumber(tostring(now))
           elseif now < start_time + (DOUBLET_TIME * 3/6) then
                down = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45) * (1 - (tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 1/6) - ramp_start_time)))
                up = doublet_srv_trim2 + (math.floor((doublet_srv_max2 - doublet_srv_trim2) * (DOUBLET_MAGNITUDE / 45)) * opposite_elevon_motion * (1 - (tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 1/6) - ramp_start_time)))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, up, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif now < start_time + (DOUBLET_TIME * 4/6) then
                up = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45) * ((tonumber(tostring(now)) - (ramp_start_time + DOUBLET_TIME * 1/6)) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 4/6) - (ramp_start_time + DOUBLET_TIME * 1/6))))
                down = doublet_srv_trim2 - (math.floor((doublet_srv_trim2 - doublet_srv_min2) * (DOUBLET_MAGNITUDE / 45)) * opposite_elevon_motion * ((tonumber(tostring(now)) - (ramp_start_time + DOUBLET_TIME * 1/6)) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 4/6) - (ramp_start_time + DOUBLET_TIME * 1/6))))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, down, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif now < start_time + (DOUBLET_TIME * 5/6) then
                up = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45))
                down = doublet_srv_trim2 - (math.floor((doublet_srv_trim2 - doublet_srv_min2) * (DOUBLET_MAGNITUDE / 45)) * opposite_elevon_motion)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, down, math.floor(DOUBLET_TIME * 1/6) + 100)
                ramp_start_time = tonumber(tostring(now))
            elseif now < start_time + (DOUBLET_TIME * 6/6) then 
                up = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45) * (1 - ((tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 6/6) - ramp_start_time))))
                down = doublet_srv_trim2 - (math.floor((doublet_srv_trim2 - doublet_srv_min2) * (DOUBLET_MAGNITUDE / 45)) * (1 - ((tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 3/6) - ramp_start_time))))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, down, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif (now > (start_time + DOUBLET_TIME)) and (now < (start_time + DOUBLET_TIME + callback_time)) then
                -- notify GCS
                gcs:send_text(6, "DOUBLET FINISHED")
                -- stick fixed at pre doublet trim position
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, doublet_srv_trim1, DOUBLET_TIME * (OBSERVATION_TIME - 1))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, doublet_srv_trim2, DOUBLET_TIME * (OBSERVATION_TIME - 1))
            elseif (now > start_time + DOUBLET_TIME + callback_time) and (now < start_time + (DOUBLET_TIME * OBSERVATION_TIME)) then
                -- do nothing until recording is complete
            elseif now > start_time + (DOUBLET_TIME * OBSERVATION_TIME) then
                -- wait for RC input channel to go low
                end_time = now
                gcs:send_text(6, "DOUBLET OBSERVATION FINISHED")
            else
                gcs:send_text(6, "this should not be reached")
            end
        elseif ACTIVE_RUDDER == true then
            -- split time evenly between high and low signal
            if now < start_time + (DOUBLET_TIME * 1/6) then
                down = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45) * (tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 1/6) - ramp_start_time))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, down, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif now < start_time + (DOUBLET_TIME * 2/6) then 
                down = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, down, math.floor(DOUBLET_TIME * 1/6) + 100)
                ramp_start_time = tonumber(tostring(now))
           elseif now < start_time + (DOUBLET_TIME * 3/6) then
                up = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45) * (1 - ((tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 3/6) - ramp_start_time))))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, up, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif now < start_time + (DOUBLET_TIME * 4/6) then
                up = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45) * ((tonumber(tostring(now)) - (ramp_start_time + DOUBLET_TIME * 1/6)) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 4/6) - (ramp_start_time + DOUBLET_TIME * 1/6))))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, up, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif now < start_time + (DOUBLET_TIME * 5/6) then
                up = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, up, math.floor(DOUBLET_TIME * 1/6) + 100)
                ramp_start_time = tonumber(tostring(now))
            elseif now < start_time + (DOUBLET_TIME * 6/6) then 
                down = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45) * (1 - ((tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 6/6) - ramp_start_time))))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, math.floor(DOUBLET_TIME * 1/6) + 100)
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, down, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif (now > (start_time + DOUBLET_TIME)) and (now < (start_time + DOUBLET_TIME + callback_time)) then
                -- notify GCS
                gcs:send_text(6, "DOUBLET FINISHED")
                -- stick fixed at pre doublet trim position
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, doublet_srv_trim1, DOUBLET_TIME * (OBSERVATION_TIME - 1))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan2, doublet_srv_trim2, DOUBLET_TIME * (OBSERVATION_TIME - 1))         
            elseif (now > start_time + DOUBLET_TIME + callback_time) and (now < start_time + (DOUBLET_TIME * OBSERVATION_TIME)) then
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
            if now < start_time + (DOUBLET_TIME * 1/6) then
                down = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45) * (tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 1/6) - ramp_start_time))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif now < start_time + (DOUBLET_TIME * 2/6) then 
                down = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, math.floor(DOUBLET_TIME * 1/6) + 100)
                ramp_start_time = tonumber(tostring(now))
           elseif now < start_time + (DOUBLET_TIME * 3/6) then
                up = doublet_srv_trim1 - math.floor((doublet_srv_trim1 - doublet_srv_min1) * (DOUBLET_MAGNITUDE / 45) * (1 - ((tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 3/6) - ramp_start_time))))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif now < start_time + (DOUBLET_TIME * 4/6) then
                up = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45) * ((tonumber(tostring(now)) - (ramp_start_time + DOUBLET_TIME * 1/6)) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 4/6) - (ramp_start_time + DOUBLET_TIME * 1/6))))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif now < start_time + (DOUBLET_TIME * 5/6) then
                up = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, up, math.floor(DOUBLET_TIME * 1/6) + 100)
                ramp_start_time = tonumber(tostring(now))
            elseif now < start_time + (DOUBLET_TIME * 6/6) then 
                down = doublet_srv_trim1 + math.floor((doublet_srv_max1 - doublet_srv_trim1) * (DOUBLET_MAGNITUDE / 45) * (1 - ((tonumber(tostring(now)) - ramp_start_time) / (tonumber(tostring(start_time)) + (DOUBLET_TIME * 6/6) - ramp_start_time))))
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, down, math.floor(DOUBLET_TIME * 1/6) + 100)
            elseif (now > (start_time + DOUBLET_TIME)) and (now < (start_time + DOUBLET_TIME + callback_time)) then
                -- notify GCS
                gcs:send_text(6, "DOUBLET FINISHED")
                -- stick fixed at pre doublet trim position
                SRV_Channels:set_output_pwm_chan_timeout(doublet_srv_chan1, doublet_srv_trim1, DOUBLET_TIME * (OBSERVATION_TIME - 1))
            elseif (now > start_time + DOUBLET_TIME + callback_time) and (now < start_time + (DOUBLET_TIME * OBSERVATION_TIME)) then
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