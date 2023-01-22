from graphics import *
from os.path import exists
import json
import sys
import os

def tx(x):
    return x*100-offX

def ty(y):
    return hRcm-(y*100-offY)

def draw_obstacle(win, obstacle):
    vertices = [Point(tx(v['x']), ty(v['y'])) for v in obstacle['inflated']]
    c = Polygon(vertices)
    c.setFill(color_rgb(255,168,168))
    c.setOutline(color_rgb(255,168,168))
    c.draw(win)
    vertices = [Point(tx(v['x']), ty(v['y'])) for v in obstacle['normal']]
    c = Polygon(vertices)
    c.setFill("red")
    c.draw(win)

def draw_exit(win, exit):
    # draw regular exit
    c = Circle(Point(tx(exit['normal']['x']), ty(exit['normal']['y'])), 8)
    c.setFill('blue')
    c.setOutline("blue")
    c.draw(win)
    # draw inflated exit
    c = Circle(Point(tx(exit['inflated']['x']), ty(exit['inflated']['y'])), 8)
    c.setFill('blue')
    c.setOutline("blue")
    c.draw(win)
    # draw regular-inflated exit link
    l = Line(Point(tx(exit['normal']['x']), ty(exit['normal']['y'])), Point(tx(exit['inflated']['x']), ty(exit['inflated']['y'])))
    l.setWidth(1)
    l.setFill('blue')
    l.draw(win)

def draw_node(win, node):
    c = Circle(Point(tx(node['x']), ty(node['y'])), 5)
    c.setFill("black")
    c.draw(win)

def draw_link(win, link):
    node1 = Point(tx(link['node1']['x']), ty(link['node1']['y']))
    node2 = Point(tx(link['node2']['x']), ty(link['node2']['y']))
    c = Line(node1, node2)
    c.setWidth(2)
    c.draw(win)

def draw_room(win, vertexes, vertexes_deflated):
    vert = [Point(tx(v['x']), ty(v['y'])) for v in vertexes]
    c = Polygon(vert)
    c.setFill(color_rgb(255,168,168))
    c.setOutline("black")
    c.setWidth(3)
    c.draw(win)
    vert = [Point(tx(v['x']), ty(v['y'])) for v in vertexes_deflated]
    c = Polygon(vert)
    c.setFill("white")
    c.setOutline("white")
    c.draw(win)

def draw_map(win, data):
    
    # setup room borders
    room_vertexes = data['room']['vertexes']
    room_vertexes_deflated = data['room']['vertexes_deflated']
    draw_room(win, room_vertexes, room_vertexes_deflated)

    # setup roadmap
    roadmap = data['roadmap']

    # setup obstacles
    obstacles = roadmap['obstacles']
    for obstacle in obstacles:
        draw_obstacle(win, obstacle)  

    # setup exits
    exits = roadmap['exits']
    for ex in exits:
        draw_exit(win, ex)

    # setup nodes
    nodes = roadmap['nodes']
    for node in nodes:
        draw_node(win, node)

    # setup links
    links = roadmap['links']
    for link in links:
        draw_link(win, link)

def draw_moves(win, moves, color):
    lines=[]
    index=1
    lst_x = tx(moves[0]["x"])
    lst_y = ty(moves[0]["y"])
    t = Text(Point(lst_x, lst_y-10), "0")
    t.setSize(14)
    t.setFill("black")
    t.draw(win)
    for move in moves[1:]:
        act_x = tx(move["x"])
        act_y = ty(move["y"])
        l = Line(Point(lst_x, lst_y), Point(act_x, act_y))
        l.setFill(color)
        l.setWidth(2)
        lines.append(l)
        l.draw(win)
        if(move["marked"]):
            t = Text(Point(act_x, act_y-10), str(index))
            t.setSize(14)
            t.setFill("black")
            t.draw(win)
            index += 1
        lst_x=act_x
        lst_y=act_y


def main():

    script_dir = os.path.dirname(__file__)

    if len(sys.argv) < 2:
        print("not enough arguments")
        return

    abs_path = os.path.join(script_dir, sys.argv[1])
    if not os.path.exists(abs_path):
        print("map file missing")
        return    
    f = open(abs_path)
    data = json.load(f)
    f.close()

    # setup window
    global offX, offY
    offX = data['room']['min_x']*100
    offY = data['room']['min_y']*100
    global hRcm
    hRcm = data['room']['h']*100
    wRcm = data['room']['w']*100
    win = GraphWin("roadmap", hRcm, wRcm)

    # Map
    draw_map(win, data)

    if len(sys.argv) > 2:

        abs_path = os.path.join(script_dir, sys.argv[2])

        if os.path.exists(abs_path):

            f = open(abs_path)
            data = json.load(f)
            f.close()

            moves_purser = data["moves_pursuer"]
            moves_evader = data["moves_evader"]

            draw_moves(win, moves_purser, "green2")
            draw_moves(win, moves_evader, "orange")    
                
    keyString = ""
    while keyString!="q":
        keyString = win.getKey() # Pause to view result
    win.close()    # Close window when done

main()