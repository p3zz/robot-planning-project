from graphics import *
from os.path import exists
import json

def main():

    ## Map & Roadmap ##

    f = open('map.json')
    data = json.load(f)
    f.close()

    hRcm=1000
    wRcm=1000
    win = GraphWin("roadmap", hRcm, wRcm)

    roadmap = data['roadmap']
    nodes = roadmap['nodes']
    links = roadmap['links']
    exits = roadmap['exits']
    obstacles = roadmap['obstacles']

    for obstacle in obstacles:
        vertices = [Point(v['x']*100, hRcm-v['y']*100) for v in obstacle['inflated']]
        c = Polygon(vertices)
        c.setFill("green3")
        c.setOutline("green3")
        c.draw(win)
        vertices = [Point(v['x']*100, hRcm-v['y']*100) for v in obstacle['normal']]
        c = Polygon(vertices)
        c.setFill("green")
        c.draw(win)  

    for ex in exits:
        c = Circle(Point(ex['normal']['x']*100, hRcm-ex['normal']['y']*100), 8)
        c.setFill('blue')
        c.setOutline("blue")
        c.draw(win)
        c = Circle(Point(ex['inflated']['x']*100, hRcm-ex['inflated']['y']*100), 8)
        c.setFill('blue')
        c.setOutline("blue")
        c.draw(win)
        l = Line(Point(ex['normal']['x']*100, hRcm-ex['normal']['y']*100), Point(ex['inflated']['x']*100, hRcm-ex['inflated']['y']*100))
        l.setWidth(1)
        l.setFill('blue')
        l.draw(win)

    for point in nodes:
        c = Circle(Point(point['x']*100, hRcm-point['y']*100), 5)
        c.setFill("black")
        c.draw(win)

    for link in links:
        node1 = Point(link['node1']['x']*100, hRcm-link['node1']['y']*100)
        node2 = Point(link['node2']['x']*100, hRcm-link['node2']['y']*100)
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
        lst_x=moves_purser[0]["x"]*100
        lst_y=hRcm-moves_purser[0]["y"]*100
        t=Text(Point(lst_x, lst_y-10), "0")
        t.setSize(14)
        t.setFill("red3")
        t.draw(win)
        for move in moves_purser[1:]:
            act_x=move["x"]*100
            act_y=hRcm-move["y"]*100
            l = Line(Point(lst_x, lst_y), Point(act_x, act_y))
            l.setFill("red")
            l.setWidth(2)
            lines.append(l)
            #l.draw(win)
            if(move["marked"]):
                t=Text(Point(act_x, act_y-10), str(index))
                t.setSize(14)
                t.setFill("red3")
                t.draw(win)
                index+=1
            lst_x=act_x
            lst_y=act_y

        index=1
        lst_x=moves_evader[0]["x"]*100
        lst_y=hRcm-moves_evader[0]["y"]*100
        t=Text(Point(lst_x, lst_y-10), "0")
        t.setSize(14)
        t.setFill("orange3")
        t.draw(win)
        for move in moves_evader[1:]:
            act_x=move["x"]*100
            act_y=hRcm-move["y"]*100
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