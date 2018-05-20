'''An agent with Seek, Flee, Arrive, Pursuit behaviours

Created for COS30002 AI for Games by Clinton Woodward cwoodward@swin.edu.au

'''

from vector2d import Vector2D
from vector2d import Point2D
from graphics import egi, KEY
from math import sin, cos, radians
from random import random, randrange, uniform
from path import Path
from datetime import timedelta

AGENT_MODES = {
}

class Agent(object):

    # NOTE: Class Object (not *instance*) variables!
    DECELERATION_SPEEDS = {
        'slow': 0.9,
        ### ADD 'normal' and 'fast' speeds here
        'normal': 1.3,
        'fast': 1.6,
    }


    def __init__(self, world=None, scale=5.0, mass=1.0, mode='hide'):


        # keep a reference to the world object
        self.world = world
        self.mode = mode
        
        # where am i and where am i going? random start pos
        dir = radians(random()*360)
        self.pos = Vector2D(randrange(world.cx), randrange(world.cy))
        self.vel = Vector2D()
        self.heading = Vector2D(sin(dir), cos(dir))
        self.side = self.heading.perp()
        self.scale = Vector2D(scale, scale)  # easy scaling of agent size
        self.force = Vector2D()  # current steering force
        self.accel = Vector2D() # current acceleration due to force
        self.mass = mass

        # data for drawing this agent
        self.color = 'ORANGE'
        self.vehicle_shape = [
            Point2D(-1.0,  0.6),
            Point2D( 1.0,  0.0),
            Point2D(-1.0, -0.6)
        ]

        self.path = Path()
        self.waypoint_threshold = 9.0
        self.randomise_path()

        ### wander details

        self.wander_target = Vector2D(1, 0)
        self.wander_dist = 1.0 * scale
        self.wander_radius = 1.0 * scale
        self.wander_jitter = 10.0 * scale
        self.bRadius = scale

        self.tagged = False
        self.botList = self.world.agents
        self.overlapping = False
        self.neighbour_radius = 6 * scale

        # Force and speed limiting code
        self.max_speed = self.world.botMaxSpeed
        self.max_force = 500.0

        # debug draw info?
        self.show_info = False

    def calculate(self):
        # calculate the current steering force
        mode = self.mode
        if mode == 'wander':
            force = self.wander(self)
        elif mode == 'hide':
            force = self.hide(self.world.hunter, self.world.obstacles)
        elif mode == 'evade':
            force = self.evade(self.world.hunter)
        else:
            force = Vector2D()
        self.force = force
        return force

    def randomise_path(self):
        cx = self.world.cx
        cy = self.world.cy
        margin = min(cx,cy)*(1/6)
        self.path.create_random_path(8, 100, 100, 300, 300, True)

    def update(self, delta):
        ''' update vehicle position and orientation '''


        # calculate and set self.force to be applied
        force = self.calculate()
        #force = self.calculate(delta)  # <-- delta needed for wander
        #force.truncate(self.max_force)
        
        self.accel = force / self.mass  # not needed if mass = 1.0
        # new velocity
        self.vel += self.accel * delta
        # check for limits of new velocity
        self.vel.truncate(self.max_speed)
        # update position
        self.pos += self.vel * delta
        # update heading is non-zero velocity (moving)
        if self.vel.length_sq() > 0.00000001:
            self.heading = self.vel.get_normalised()
            self.side = self.heading.perp()
        # treat world as continuous space - wrap new position if needed
        self.world.wrap_around(self.pos)

    def render(self, color=None):
        ''' Draw the triangle agent with color'''
        # draw the path if it exists and the mode is follow
        if self.mode == 'follow_path':
            self.path.render()

        # draw the ship
        egi.set_pen_color(name=self.color)
        pts = self.world.transform_points(self.vehicle_shape, self.pos,
                                          self.heading, self.side, self.scale)
        # draw it!
        egi.closed_shape(pts)

        # draw wander info?
        if self.mode == 'wander':
            ## ...
            pass

        #draw neighbour radius
        cntr_pos = Vector2D()
        cntrwld_pos = self.world.transform_point(cntr_pos,self.pos,self.heading,self.side)

        egi.white_pen()
        egi.circle(cntrwld_pos, self.neighbour_radius)
        

        # add some handy debug drawing info lines - force and velocity
        if self.show_info:
            s = 0.5 # <-- scaling factor
            # force
            egi.red_pen()
            egi.line_with_arrow(self.pos, self.pos + self.force * s, 5)
            # velocity
            egi.grey_pen()
            egi.line_with_arrow(self.pos, self.pos + self.vel * s, 5)
            # net (desired) change
            egi.white_pen()
            egi.line_with_arrow(self.pos+self.vel * s, self.pos+ (self.force+self.vel) * s, 5)
            egi.line_with_arrow(self.pos, self.pos+ (self.force+self.vel) * s, 5)

        if(self.mode == 'wander'):
            #calculate center of the wander circle in front of agent
            wnd_pos = Vector2D(self.wander_dist, 0)
            wld_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
            #draw the wander circle
            egi.green_pen()
            egi.circle(wld_pos, self.wander_radius)
            #draw the wander target (little circle on big circle)
            egi.red_pen()
            wnd_pos = (self.wander_target + Vector2D(self.wander_dist, 0))
            wnd_pos = self.world.transform_point(wnd_pos, self.pos, self.heading, self.side)
            egi.circle(wld_pos, 3)

    def speed(self):
        return self.vel.length()

    #--------------------------------------------------------------------------

    def seek(self, target_pos):
        ''' move towards target position '''
        desired_vel = (target_pos - self.pos).normalise() * self.max_speed
        return (desired_vel - self.vel)

    def flee(self, target):
        # returns a vector that is pointing away from the target
        # opposite of Seek()
        # Panic sensor?
        # for range 10, squared to save computation
        panic_range_sq = 62500 #250 units
        if self.pos.distance_sq(target) > panic_range_sq:
            return Vector2D() # (0,0)
        
        desired_vel = (self.pos - target).normalise()* self.max_speed

        return desired_vel - self.vel

    def arrive(self, target_pos, speed):
        ''' this behaviour is similar to seek() but it attempts to arrive at
            the target position with a zero velocity'''
        decel_rate = self.DECELERATION_SPEEDS[speed]
        to_target = target_pos - self.pos
        dist = to_target.length()
        if dist > 0:
            # calculate the speed required to reach the target given the
            # desired deceleration rate
            speed = dist / decel_rate
            # make sure the velocity does not exceed the max
            speed = min(speed, self.max_speed)
            # from here proceed just like Seek except we don't need to
            # normalize the to_target vector because we have already gone to the
            # trouble of calculating its length for dist.
            desired_vel = to_target * (speed / dist)
            return (desired_vel - self.vel)
        return Vector2D(0, 0)

    def pursuit(self, evader):
        ''' this behaviour predicts where an agent will be in time T and seeks
            towards that point to intercept it. '''
        ## OPTIONAL EXTRA... pursuit (you'll need something to pursue!)
        return Vector2D()

    def wander(self, delta):
        ''' random wandering using a projected jitter circle '''
        wt = self.wander_target
        # this behaviour is dependent on the update rate, so this line must
        # be included when using time independent framerate.
        #jitter_tts = self.wander_jitter * delta # this time slice
        jitter_tts = self.wander_jitter# * delta # this time slice
        # first, add a small random vector to the target's position
        wt += Vector2D(uniform(-1,1) * jitter_tts, uniform(-1,1) * jitter_tts)
        # re-project this new vector back on to a unit circle
        wt.normalise()
        # increase the length of the vector to the same as the radius
        # of the wander circle
        wt *= self.wander_radius
        # move the target into a position WanderDist in front of the agent
        target = wt + Vector2D(self.wander_dist, 0)
        # project the target into world space
        wld_target = self.world.transform_point(target, self.pos, self.heading, self.side)
        # and steer towards it
        return self.seek(wld_target) 

    def follow_path(self, path):
        to_target = path.current_pt() - self.pos
        dist = to_target.length()
        ##print(dist)
        if(self.path.is_finished == True):
            return Arrive(self.path.current_pt, 'fast')
        else:
            if(dist<=self.waypoint_threshold):
                self.path.inc_current_pt()
            return self.seek(path.current_pt())

    #--------------------------------------------------------------------------

    def evade(self, pursuer):

        #flee from the estimated future position
        toPursuer = pursuer.pos - self.pos

        #proportional to distance, inversely proportional to sum of velocities
        lookAheadTime = toPursuer.length() / (self.max_speed + pursuer.speed())

        #go in the opposite predicted position
        return self.flee(pursuer.pos + pursuer.vel * lookAheadTime)

    def GetHidingPosition(self, hunter, obj):
        # set the distance between the object and the hiding point
        DistFromBoundary = 30.0 # system setting
        DistAway = obj.radius + DistFromBoundary
        
        # get the normal vector from the hunter to the hiding point
        ToObj = (obj.pos - hunter.pos).normalise()
        
        # scale size past the object to the hiding location
        return (ToObj*DistAway)+obj.pos 

    def hide(self,hunter,objs):
        DistToClosest = 500000000
        
        
        # check for possible hiding spots
        #for obj in objs:
        for idx, obj in enumerate(objs):
            HidingSpot = self.GetHidingPosition(hunter,obj)
            self.world.hidingSpots[idx] = HidingSpot
            HidingDist = HidingSpot.distance_sq(self.pos)
            if HidingDist < DistToClosest:
                DistToClosest = HidingDist
                self.world.bestHidingSpot = HidingSpot

        # if we have a best hiding spot, use it
        if self.world.bestHidingSpot:
            return self.arrive(self.world.bestHidingSpot, 'fast') # speed = fast?
        # default - run away!
        return self.evade(hunter) 



    def pursuit(self,evader):
     bot = self.bot
     # assumes that evader is a Vehicle
     toEvader = evader.pos - bot.pos
     relativeHeading = bot.heading.dot(evader.heading)
     
     # simple out: if target is ahead and facing us, head straight to it
     if (toEvader.dot(bot.heading)>0) and (relativeHeading < 0.95):
         # acos(0.95)=18 degrees
         return self._Seek(evader.pos)
    
     # time proportional to distance, inversely proportional to sum of velocities
     lookAheadTime = toEvader.length() / (bot.MaxSpeed + evader.Speed())
     
     # turn rate delay? dot product = 1 if ahead, -1 if behind.
     lookAheadTime += (1 - bot.heading.dot(evader))*- bot.turnRate
     
     # Seek the predicted location (using look-ahead time)
     lookAheadPos = evader.pos + evader.vel * lookAheadTime
     return self.seek(lookAheadPos)    
