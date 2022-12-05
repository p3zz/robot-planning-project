from graphics import *
import json

def main():
    f = open('map.json')
    data = json.load(f)

    win = GraphWin("roadmap", 500, 500)

    roadmap = data['roadmap']
    nodes = roadmap['nodes']
    links = roadmap['links']
    obstacles = roadmap['obstacles']

    for point in nodes:
        c = Circle(Point(point['x'],point['y']), 5)
        c.draw(win)

    for link in links:
        source = Point(link['source']['x'],link['source']['y'])
        dest = Point(link['dest']['x'],link['dest']['y']) 
        c = Line(source, dest)
        c.draw(win)

    for obstacle in obstacles:
        vertices = [Point(v['x'], v['y']) for v in obstacle]
        c = Polygon(vertices)
        c.draw(win)    

    win.getMouse() # Pause to view result
    win.close()    # Close window when done

    f.close()

main()