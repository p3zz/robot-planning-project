from graphics import *
from os.path import exists
import json

def tx(x):
    return x*100-offX

def ty(y):
    return hRcm-(y*100-offY)

def main():

    ## Map & Roadmap ##

    f = open('map.json')
    data = json.load(f)
    f.close()

    global offX, offY
    offX=data['room']['min_x']*100
    offY=data['room']['min_y']*100
    global hRcm
    hRcm=data['room']['h']*100
    wRcm=data['room']['w']*100
    vertexes=data['room']['vertexes']
    vertexes_deflated=data['room']['vertexes_deflated']
    win = GraphWin("roadmap", hRcm, wRcm)

    roadmap = data['roadmap']
    nodes = roadmap['nodes']
    links = roadmap['links']
    exits = roadmap['exits']
    obstacles = roadmap['obstacles']

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

    for obstacle in obstacles:
        vertices = [Point(tx(v['x']), ty(v['y'])) for v in obstacle['inflated']]
        c = Polygon(vertices)
        c.setFill(color_rgb(255,168,168))
        c.setOutline(color_rgb(255,168,168))
        c.draw(win)
        vertices = [Point(tx(v['x']), ty(v['y'])) for v in obstacle['normal']]
        c = Polygon(vertices)
        c.setFill("red")
        c.draw(win)  

    for ex in exits:
        c = Circle(Point(tx(ex['normal']['x']), ty(ex['normal']['y'])), 8)
        c.setFill('blue')
        c.setOutline("blue")
        c.draw(win)
        c = Circle(Point(tx(ex['inflated']['x']), ty(ex['inflated']['y'])), 8)
        c.setFill('blue')
        c.setOutline("blue")
        c.draw(win)
        l = Line(Point(tx(ex['normal']['x']), ty(ex['normal']['y'])), Point(tx(ex['inflated']['x']), ty(ex['inflated']['y'])))
        l.setWidth(1)
        l.setFill('blue')
        l.draw(win)

    for point in nodes:
        c = Circle(Point(tx(point['x']), ty(point['y'])), 5)
        c.setFill("black")
        c.draw(win)

    for link in links:
        node1 = Point(tx(link['node1']['x']), ty(link['node1']['y']))
        node2 = Point(tx(link['node2']['x']), ty(link['node2']['y']))
        c = Line(node1, node2)
        c.setWidth(2)
        c.draw(win)

    ## Moves ##

    if exists('moves.json'):
        f2 = open('moves.json')
        data = json.load(f2)
        f2.close()
        moves_purser = data["moves_pursuer"]
        moves_evader = data["moves_evader"]
        lines=[]
        
        index=1
        lst_x=tx(moves_purser[0]["x"])
        lst_y=ty(moves_purser[0]["y"])
        t=Text(Point(lst_x, lst_y-10), "0")
        t.setSize(14)
        t.setFill("green3")
        t.draw(win)
        for move in moves_purser[1:]:
            act_x=tx(move["x"])
            act_y=ty(move["y"])
            l = Line(Point(lst_x, lst_y), Point(act_x, act_y))
            l.setFill("green1")
            l.setWidth(2)
            lines.append(l)
            #l.draw(win)
            if(move["marked"]):
                t=Text(Point(act_x, act_y-10), str(index))
                t.setSize(14)
                t.setFill("green3")
                t.draw(win)
                index+=1
            lst_x=act_x
            lst_y=act_y

        index=1
        lst_x=tx(moves_evader[0]["x"])
        lst_y=ty(moves_evader[0]["y"])
        t=Text(Point(lst_x, lst_y-10), "0")
        t.setSize(14)
        t.setFill("orange3")
        t.draw(win)
        for move in moves_evader[1:]:
            act_x=tx(move["x"])
            act_y=ty(move["y"])
            l = Line(Point(lst_x, lst_y), Point(act_x, act_y))
            l.setFill("orange")
            l.setWidth(2)
            lines.append(l)
            #l.draw(win)
            if(move["marked"]):
                t=Text(Point(act_x, act_y-10), str(index))
                t.setSize(14)
                t.setFill("orange3")
                t.draw(win)
                index+=1
            lst_x=act_x
            lst_y=act_y
        
        for l in lines:
            l.draw(win)
            
        
    keyString = ""
    while keyString!="q":
        keyString = win.getKey() # Pause to view result
    win.close()    # Close window when done

main()