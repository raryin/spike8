'''Autonomous Agent Movement: Seek, Arrive and Flee

Created for COS30002 AI for Games, Lab 05
By Clinton Woodward cwoodward@swin.edu.au

'''
from graphics import egi, KEY
from pyglet import window, clock
from pyglet.gl import *

from vector2d import Vector2D
from world import World
from obstacle import Obstacle
from agent import Agent, AGENT_MODES  # Agent with wander, alignment,
                                      #cohesion & separation


def on_mouse_press(x, y, button, modifiers):
    if button == 1:  # left
        world.target = Vector2D(x, y)


def on_key_press(symbol, modifiers):
    if symbol == KEY.P:
        world.paused = not world.paused

    #reset path
    elif symbol == KEY.R:
        for agent in world.agents:
            agent.path.clear()
    #add new agent
    elif symbol == KEY.N:
        world.agents.append(Agent(world))
        world.agents[-1].mode = world.agents[1].mode
        world.agents[-1].mass = world.agents[0].mass
        world.agents[-1].max_speed = world.agents[0].max_speed
        world.agents[-1].max_force = world.agents[0].max_force

    #mass change
    elif symbol == KEY.Q:
        for agent in world.agents:
            agent.mass += 1
    elif symbol == KEY.W:
        for agent in world.agents:
            agent.mass -= 1

    #change speed
    elif symbol == KEY.Z:
        for agent in world.agents:
            agent.max_speed = agent.max_speed + 10
    elif symbol == KEY.X:
        for agent in world.agents:
            agent.max_speed = agent.max_speed - 10

    #change turn force
    elif symbol == KEY.A:
        for agent in world.agents:
            agent.max_force = agent.max_force + 10
    elif symbol == KEY.S:
        for agent in world.agents:
            agent.max_force = agent.max_force - 10


    # Toggle debug force line info on the agent
    elif symbol == KEY.I:
        for agent in world.agents:
            agent.show_info = not agent.show_info



def on_resize(cx, cy):
    world.cx = cx
    world.cy = cy


if __name__ == '__main__':

    # create a pyglet window and set glOptions
    win = window.Window(width=500, height=500, vsync=True, resizable=True)
    glEnable(GL_BLEND)
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
    # needed so that egi knows where to draw
    egi.InitWithPyglet(win)
    # prep the fps display
    fps_display = clock.ClockDisplay()
    # register key and mouse event handlers
    win.push_handlers(on_key_press)
    win.push_handlers(on_mouse_press)
    win.push_handlers(on_resize)

    # create a world for agents
    world = World(500, 500)
    # add two agents
    world.agents.append(Agent(world))
    world.agents.append(Agent(world))
    # unpause the world ready for movement
    world.paused = False

    world.agents[0].color = 'RED'
    world.agents[0].mode = 'wander'
    world.hunter = world.agents[0]

    world.obstacles.append(Obstacle(world, 100, 100, 40))
    world.obstacles.append(Obstacle(world, 150, 380, 60))
    world.obstacles.append(Obstacle(world, 350, 200, 75))

    while not win.has_exit:
        win.dispatch_events()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # show nice FPS bottom right (default)
        delta = clock.tick()
        world.update(delta)
        world.render()
        fps_display.draw()
        # swap the double buffer
        win.flip()

