import numpy as np
import random

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
start = True

def mean_angle(angles):
    return np.clip(np.mean(angles * 180/np.pi), -15, 15)
def decision_step(Rover):

    # Implement conditionals to decide what to do given perception data
    # Here you're all set up with some basic functionality but you'll need to
    # improve on this decision tree to do a good job of navigating autonomously!
    
    # near_sample property seems to stay asserted for a short while even after pickup is completed
    # this can cause this loop to be selected for ever and get stuck
    # so add in a timeout after pickup has occured until next one is allowed to happen
    
    print("."*30)
    #Record starting point of rover so we can return there after all samples collected
    global start
    if start:
        Rover.start_pos = Rover.pos
        start = False
    if Rover.prev_pos is None: Rover.prev_pos = Rover.pos        
    
    #check if the rover is stuck, if it's stuck turn it.
    if Rover.counter % 300 == 0 and Rover.mode != "turn":
        dist = np.linalg.norm(np.array(Rover.pos)-np.array(Rover.prev_pos))
        print("DIST", dist)
        if dist<1:#stuck
            print("UNTUCKING "*100)
            Rover.mode = "turn"
            return Rover
        Rover.prev_pos = Rover.pos
        
    Rover.counter += 1
    print("Counter",Rover.counter)
    if Rover.mode == 'turn':
        Rover.throttle = 0
        Rover.brake = 0
        Rover.steer = -15
        Rover.turn_counter += 1
        if Rover.turn_counter>60:
            Rover.mode = 'forward'
            Rover.turn_counter = 0
        return Rover
        
       
    #stop the rover and pick the sample
    if Rover.near_sample and Rover.mode !='picking':
        print("Pick sample", "*"*30)
        Rover.do_stop()
        #pick up the rock
        Rover.send_pickup = True
        Rover.samples_collected += 1
        Rover.mode = "picking"
        return Rover
    
    #start moving when sample picked
    if not Rover.near_sample and Rover.mode == 'picking':
        Rover.mode = "forward"
        return Rover
    
    #move to the sample
    if len(Rover.rocks_angles) > 0:
        print("Go to sample")
        Rover.throttle = 0.2
        Rover.steer = mean_angle(Rover.rocks_angles)
        return Rover
    
    #not initialized, wait
    if Rover.nav_angles is None:
        Rover.do_stop()
        return Rover
    
    #forward mode, advance through navigable terrain
    if Rover.mode == 'forward': 
        # Check the extent of navigable terrain
        if len(Rover.nav_angles) >= Rover.stop_forward:  
            # If mode is forward, navigable terrain looks good 
            # and velocity is below max, then throttle 
            if Rover.vel < Rover.max_vel:
                # Set throttle value to throttle setting
                Rover.throttle = Rover.throttle_set
            else: # Else coast
                Rover.throttle = 0
            Rover.brake = 0
            # add randomness to the movement to wideexploring region
            if random.random() > 0.001:
                Rover.steer = mean_angle(Rover.nav_angles)
            else:
                Rover.mode = 'random'
        else:
            Rover.do_stop()
        return Rover
    
    if Rover.mode == 'stop':
        # If still moving, then stop
        if Rover.vel > 0.2:
            Rover.do_stop()
        # turn
        elif Rover.vel <= 0.2:
            # turn until there is terrain to advance
            if len(Rover.nav_angles) < Rover.go_forward:
                Rover.do_stop(steer=-10)
            else: #free terrain
                Rover.do_stop(steer = mean_angle(Rover.nav_angles) )
                Rover.mode = 'forward'
        return Rover
    
    if Rover.mode == 'random':
        if random.random() > 0.97:
            Rover.mode = 'forward'
        else:
            Rover.do_stop(steer=-10)
        return Rover
                
   
    return Rover
