from graphics import *
from os.path import exists
import json

def main():
    f = open('map.json')
    data = json.load(f)
    f.close()

    win = GraphWin("roadmap", 1000, 1000)

    roadmap = data['roadmap']
    nodes = roadmap['nodes']
    links = roadmap['links']
    obstacles = roadmap['obstacles']

    index=0
    for obstacle in obstacles:
        vertices = [Point(v['x']*100, v['y']*100) for v in obstacle['inflated']]
        c = Polygon(vertices)
        c.setFill("green3")
        c.setOutline("green3")
        c.draw(win)
        vertices = [Point(v['x']*100, v['y']*100) for v in obstacle['normal']]
        c = Polygon(vertices)
        c.setFill("green")
        c.draw(win)   
        t=Text(Point(obstacle['normal'][0]['x']*100, obstacle['normal'][0]['y']*100), str(index))
        t.setTextColor("blue")
        t.setSize(18)
        #t.draw(win)
        index+=1

    for point in nodes:
        if point['x'] == 0 or point['x'] == 10:
            r = Rectangle(Point(point['x']*100-10, point['y']*100-20), Point(point['x']*100+10, point['y']*100+20))
            r.setFill("blue")
            r.draw(win)
        elif point['y'] == 0 or point['y'] == 10:
            r = Rectangle(Point(point['x']*100-20, point['y']*100-10), Point(point['x']*100+20, point['y']*100+10))
            r.setFill("blue")
            r.draw(win)
        c = Circle(Point(point['x']*100,point['y']*100), 5)
        c.setFill("black")
        t=Text(Point(point['x']*100,point['y']*100), "{:02}".format(point['x'])+" "+"{:02}".format(point['y']))
        t.setTextColor("green")
        t.setSize(12)
        c.draw(win)
        #t.draw(win)

    for link in links:
        node1 = Point(link['node1']['x']*100,link['node1']['y']*100)
        node2 = Point(link['node2']['x']*100,link['node2']['y']*100)
        c = Line(node1, node2)
        c.setWidth(2)
        c.draw(win)

    if exists('moves.json'):
        f2 = open('moves.json')
        data = json.load(f2)
        f2.close()
        moves = data["moves"]
        lst_pursuer_x=moves[0]["pursuer"]["x"]*100
        lst_pursuer_y=moves[0]["pursuer"]["y"]*100
        c = Circle(Point(lst_pursuer_x, lst_pursuer_y), 10)
        c.setFill("yellow")
        c.draw(win)
        t=Text(Point(lst_pursuer_x, lst_pursuer_y+3), "0")
        t.setSize(14)
        t.draw(win)
        lst_evader_x=moves[0]["evader"]["x"]*100
        lst_evader_y=moves[0]["evader"]["y"]*100
        c = Circle(Point(lst_evader_x, lst_evader_y), 10)
        c.setFill("red")
        c.draw(win)
        t=Text(Point(lst_evader_x, lst_evader_y+3), "0")
        t.setSize(14)
        t.draw(win)
        index=1
        for move in moves[1:]:

            act_pursuer_x=move["pursuer"]["x"]*100
            act_pursuer_y=move["pursuer"]["y"]*100
            c = Circle(Point(act_pursuer_x, act_pursuer_y), 8)
            c.setFill("yellow")
            c.draw(win)
            l = Line(Point(lst_pursuer_x, lst_pursuer_y), Point(act_pursuer_x, act_pursuer_y))
            l.setArrow("last")
            l.setFill("yellow4")
            l.draw(win)
            t=Text(Point(act_pursuer_x, act_pursuer_y+2), str(index))
            t.setSize(12)
            t.draw(win)
            lst_pursuer_x=act_pursuer_x
            lst_pursuer_y=act_pursuer_y

            act_evader_x=move["evader"]["x"]*100
            act_evader_y=move["evader"]["y"]*100
            c = Circle(Point(act_evader_x, act_evader_y), 8)
            c.setFill("red")
            c.draw(win)
            l = Line(Point(lst_evader_x, lst_evader_y), Point(act_evader_x, act_evader_y))
            l.setArrow("last")
            l.setFill("red4")
            l.draw(win)
            t=Text(Point(act_evader_x, act_evader_y+2), str(index))
            t.setSize(12)
            t.draw(win)
            lst_evader_x=act_evader_x
            lst_evader_y=act_evader_y

            index+=1
        
    keyString = ""
    while keyString!="q":
        keyString = win.getKey() # Pause to view result
    win.close()    # Close window when done

main()