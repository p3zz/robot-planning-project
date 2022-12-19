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
        node1 = Point(link['node1']['x']*100,link['node1']['y']*100)
        node2 = Point(link['node2']['x']*100,link['node2']['y']*100)
        c = Line(node1, node2)
        c.draw(win)

    for obstacle in obstacles:
        vertices = [Point(v['x']*100, v['y']*100) for v in obstacle]
        c = Polygon(vertices)
        c.setFill("red")
        c.draw(win)    

    win.getMouse() # Pause to view result
    win.close()    # Close window when done

    f.close()

main()