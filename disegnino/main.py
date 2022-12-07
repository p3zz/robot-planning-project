from graphics import *
import json

def main():
    f = open('map.json')
    data = json.load(f)

    win = GraphWin("roadmap", 1000, 1000)

    roadmap = data['roadmap']
    nodes = roadmap['nodes']
    links = roadmap['links']
    obstacles = roadmap['obstacles']

    for point in nodes:
        c = Circle(Point(point['x']*100,point['y']*100), 5)
        c.draw(win)

    for link in links:
        source = Point(link['source']['x']*100,link['source']['y']*100)
        dest = Point(link['dest']['x']*100,link['dest']['y']*100) 
        c = Line(source, dest)
        c.draw(win)

    for obstacle in obstacles:
        vertices = [Point(v['x']*100, v['y']*100) for v in obstacle]
        c = Polygon(vertices)
        c.draw(win)    

    win.getMouse() # Pause to view result
    win.close()    # Close window when done

    f.close()

main()